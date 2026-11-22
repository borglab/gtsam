/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    BatchedBlockTridiagonalLevenbergMarquardt.cpp
 * @brief   Levenberg-Marquardt over B independent, equal-shape nonlinear
 *          chain graphs, batching a block-tridiagonal-exact GPU solve
 * @author  Shaji
 * @date    Nov 20, 2026
 */

#include <gtsam/nonlinear/cuda/BatchedBlockTridiagonalLevenbergMarquardt.h>

#include <gtsam/base/cuda/Errors.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/linear/cuda/internal/BatchedBlockHessianAssembler.h>
#include <gtsam/nonlinear/internal/LevenbergMarquardtPolicy.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>

namespace gtsam::cuda {
namespace {

constexpr size_t kMaxAttemptsPerOuterIteration = 32;

// Where one factor's (fixed, graph-structure-only) row range and each of
// its keys' column slots land in the dense per-item Jacobian -- computed
// once per item from the first linearization, then reused every outer
// iteration to copy each factor's freshly-linearized block directly into
// place, bypassing GaussianFactorGraph::jacobian(ordering)'s generic
// VariableSlots/ordering-inversion machinery (the real cost for a graph
// whose structure never changes across outer iterations, only its values).
struct ColumnSlot {
  size_t colOffset = 0;
  size_t colWidth = 0;
};
struct FactorAssemblyPlan {
  size_t rowOffset = 0;
  size_t rowCount = 0;
  std::vector<ColumnSlot> columns;  // one per key, in factor->keys() order
};

}  // namespace

struct BatchedBlockTridiagonalLevenbergMarquardtOptimizer::Impl {
  std::vector<NonlinearFactorGraph> graphs;
  std::vector<Values> currentValues;
  std::vector<KeyInfo> keyInfos;
  // [item][factorIndex] -- see FactorAssemblyPlan.
  std::vector<std::vector<FactorAssemblyPlan>> itemAssemblyPlan;
  // Primed with every active item's Values and computed as one cross-item
  // batch before each outer iteration's per-item linearize() below; see
  // BatchedFactorGroupHook.h.
  std::vector<BatchedFactorGroupHook> factorGroupHooks;
  LevenbergMarquardtParams params;
  BatchedBlockTridiagonalCholeskySolver solver;
  // Forms diag/offDiag/rhs on GPU via batched GEMM; see
  // BatchedBlockHessianAssembler.h.
  BatchedBlockHessianAssembler hessianAssembler;
  DeviceArray<double> deviceJacobianA;
  DeviceArray<double> deviceJacobianB;
  DeviceArray<double> deviceDiagRaw;
  DeviceArray<double> deviceOffDiagRaw;
  DeviceArray<double> deviceRhsRaw;
  DeviceArray<double> deviceDiag;
  DeviceArray<double> deviceOffDiag;
  DeviceArray<double> deviceRhs;
  // factorize()/solve() destructively overwrite deviceOffDiag/deviceRhs
  // (they become Z_t and the solution), so each attempt needs a fresh
  // copy; kept here untouched and refreshed via a cheap on-device copy
  // every attempt, only re-marshaled/re-uploaded when participating changes.
  DeviceArray<double> pristineOffDiag;
  DeviceArray<double> pristineRhs;
  std::vector<bool> previousParticipating;
  bool pristineValid = false;
  LinearSolveStats stats;
  std::vector<double> hostDiag;
  std::vector<double> hostOffDiag;
  std::vector<double> hostRhs;
  std::vector<bool> participating;
  // Undamped per-block H_tt/H_{t,t+1}/rhs_t, cached once per outer
  // iteration and reused by every lambda-retry attempt within it.
  std::vector<std::vector<Matrix>> diagBase;     // [item][block]
  std::vector<std::vector<Matrix>> offDiagBase;  // [item][block], size numBlocks-1
  std::vector<std::vector<Vector>> rhsBase;      // [item][block]

  int batchSize = 0;
  int numBlocks = 0;
  int blockDim = 0;
  int dimension = 0;  // numBlocks * blockDim, for delta/retract bookkeeping
  std::vector<double> currentError;
  std::vector<double> lambda;
  std::vector<double> currentFactor;
  std::vector<bool> terminated;
  std::vector<BatchedBlockTridiagonalLevenbergMarquardtItemResult> itemResults;
  bool optimized = false;
  // -1 means "whatever device is already current" (see
  // BatchedDenseSolverOptions). optimize() and polish() each re-assert this
  // before their own direct DeviceArray upload/download, not just relying
  // on solver's/hessianAssembler's construction-time cudaSetDevice --
  // cudaSetDevice is thread-global, so a second same-thread instance
  // constructed on a different device would otherwise leave this
  // instance's later calls running against the wrong device's memory.
  int deviceId = -1;

  Impl(const std::vector<NonlinearFactorGraph>& inputGraphs,
       const std::vector<Values>& initialValues, int inputNumBlocks,
       int inputBlockDim, const LevenbergMarquardtParams& inputParams,
       const BatchedDenseSolverOptions& deviceOptions,
       const std::vector<BatchedFactorGroupHook>& inputFactorGroupHooks)
      : graphs(inputGraphs),
        currentValues(initialValues),
        factorGroupHooks(inputFactorGroupHooks),
        params(inputParams),
        solver(deviceOptions),
        hessianAssembler(deviceOptions),
        numBlocks(inputNumBlocks),
        blockDim(inputBlockDim),
        deviceId(deviceOptions.deviceId) {
    if (inputGraphs.empty() || inputGraphs.size() != initialValues.size()) {
      throw std::invalid_argument(
          "BatchedBlockTridiagonalLevenbergMarquardtOptimizer requires "
          "equally-sized, nonempty graphs and initialValues batches");
    }
    if (numBlocks <= 1 || blockDim <= 0) {
      throw std::invalid_argument(
          "BatchedBlockTridiagonalLevenbergMarquardtOptimizer requires "
          "numBlocks > 1 and blockDim > 0");
    }
    batchSize = static_cast<int>(inputGraphs.size());
    dimension = numBlocks * blockDim;
    keyInfos.reserve(static_cast<size_t>(batchSize));
    itemAssemblyPlan.resize(static_cast<size_t>(batchSize));
    for (int b = 0; b < batchSize; ++b) {
      const auto linearized = graphs[static_cast<size_t>(b)].linearize(
          currentValues[static_cast<size_t>(b)]);
      KeyInfo keyInfo(*linearized);
      if (static_cast<int>(keyInfo.numCols()) != dimension) {
        std::ostringstream os;
        os << "BatchedBlockTridiagonalLevenbergMarquardtOptimizer requires "
           << "every graph to linearize to numBlocks*blockDim=" << dimension
           << " scalar columns; graph " << b << " has " << keyInfo.numCols();
        throw std::invalid_argument(os.str());
      }

      // Build this item's fixed row/column assembly plan from the same
      // linearization, before keyInfo is moved into keyInfos below.
      std::vector<FactorAssemblyPlan>& plan =
          itemAssemblyPlan[static_cast<size_t>(b)];
      plan.resize(linearized->size());
      size_t rowCursor = 0;
      for (size_t f = 0; f < linearized->size(); ++f) {
        const auto& factor = linearized->at(f);
        auto jf = std::dynamic_pointer_cast<JacobianFactor>(factor);
        if (factor && !jf) {
          throw std::runtime_error(
              "BatchedBlockTridiagonalLevenbergMarquardtOptimizer's "
              "specialized Jacobian assembly requires every linearized "
              "factor to be a JacobianFactor");
        }
        FactorAssemblyPlan& factorPlan = plan[f];
        factorPlan.rowOffset = rowCursor;
        factorPlan.rowCount = jf ? static_cast<size_t>(jf->rows()) : 0;
        rowCursor += factorPlan.rowCount;
        if (!jf) continue;
        factorPlan.columns.resize(jf->size());
        int minBlock = std::numeric_limits<int>::max();
        int maxBlock = std::numeric_limits<int>::min();
        for (size_t k = 0; k < jf->size(); ++k) {
          const Key key = jf->keys()[k];
          const auto entryIt = keyInfo.find(key);
          if (entryIt == keyInfo.end()) {
            throw std::runtime_error(
                "BatchedBlockTridiagonalLevenbergMarquardtOptimizer: "
                "linearized factor references a key absent from this "
                "item's KeyInfo");
          }
          const size_t colOffset = entryIt->second.start;
          const size_t colWidth = entryIt->second.dim;
          factorPlan.columns[k].colOffset = colOffset;
          factorPlan.columns[k].colWidth = colWidth;

          // Enforce the documented block-tridiagonal precondition (each
          // factor touches at most two adjacent numBlocks*blockDim groups,
          // and no key straddles a group boundary): silently violating
          // this would make BatchedBlockHessianAssembler (which only ever
          // forms diag[t]=At^T At and offDiag[t]=At^T At+1) drop this
          // factor's contribution to any non-adjacent block pair, producing
          // a wrong (incomplete) normal-equations system with no runtime
          // signal. See BatchedBlockTridiagonalLevenbergMarquardtOptimizer's
          // class doc ("each factor touching only one or two adjacent
          // groups").
          const int firstBlock = static_cast<int>(colOffset / blockDim);
          const int lastBlock =
              static_cast<int>((colOffset + colWidth - 1) / blockDim);
          if (firstBlock != lastBlock) {
            throw std::invalid_argument(
                "BatchedBlockTridiagonalLevenbergMarquardtOptimizer requires "
                "every key to lie within a single numBlocks*blockDim group; "
                "factor " + std::to_string(f) + " (graph " +
                std::to_string(b) + ") has a key straddling a group "
                "boundary at column " + std::to_string(colOffset));
          }
          minBlock = std::min(minBlock, firstBlock);
          maxBlock = std::max(maxBlock, firstBlock);
        }
        if (!factorPlan.columns.empty() && maxBlock - minBlock > 1) {
          throw std::invalid_argument(
              "BatchedBlockTridiagonalLevenbergMarquardtOptimizer requires "
              "every factor to touch at most two adjacent blocks (chain "
              "structure); factor " + std::to_string(f) + " (graph " +
              std::to_string(b) + ") spans blocks " +
              std::to_string(minBlock) + ".." + std::to_string(maxBlock) +
              " -- BatchedBlockHessianAssembler only forms diagonal and "
              "immediate off-diagonal Hessian blocks and would silently "
              "drop this factor's contribution to non-adjacent blocks");
        }
      }
      keyInfos.push_back(std::move(keyInfo));
    }

    currentError.resize(static_cast<size_t>(batchSize));
    lambda.assign(static_cast<size_t>(batchSize), params.lambdaInitial);
    currentFactor.assign(static_cast<size_t>(batchSize), params.lambdaFactor);
    terminated.assign(static_cast<size_t>(batchSize), false);
    itemResults.resize(static_cast<size_t>(batchSize));
    hostDiag.resize(static_cast<size_t>(batchSize) *
                    static_cast<size_t>(numBlocks) *
                    static_cast<size_t>(blockDim) *
                    static_cast<size_t>(blockDim));
    hostOffDiag.resize(static_cast<size_t>(batchSize) *
                       static_cast<size_t>(numBlocks - 1) *
                       static_cast<size_t>(blockDim) *
                       static_cast<size_t>(blockDim));
    hostRhs.resize(static_cast<size_t>(batchSize) *
                  static_cast<size_t>(numBlocks) *
                  static_cast<size_t>(blockDim));
    participating.resize(static_cast<size_t>(batchSize));
    diagBase.resize(static_cast<size_t>(batchSize));
    offDiagBase.resize(static_cast<size_t>(batchSize));
    rhsBase.resize(static_cast<size_t>(batchSize));
    for (int b = 0; b < batchSize; ++b) {
      currentError[static_cast<size_t>(b)] =
          graphs[static_cast<size_t>(b)].error(
              currentValues[static_cast<size_t>(b)]);
      itemResults[static_cast<size_t>(b)].initialError =
          currentError[static_cast<size_t>(b)];
      if (currentError[static_cast<size_t>(b)] <= params.errorTol) {
        terminated[static_cast<size_t>(b)] = true;
        itemResults[static_cast<size_t>(b)].converged = true;
      }
    }
  }

  bool allTerminated() const {
    return std::all_of(terminated.begin(), terminated.end(),
                       [](bool t) { return t; });
  }

  // Stages+batches every registered factor group's net evaluation, then
  // linearizes every item in activeItems and assembles diag/offDiag/rhs via
  // the GPU Hessian assembler, leaving the result in deviceDiagRaw/
  // deviceOffDiagRaw/deviceRhsRaw (still on-device, in activeItems order).
  // Shared by optimize()'s per-outer-iteration assembly (which additionally
  // downloads to host via downloadAssembledBatch(), below, since it needs
  // per-item lambda damping applied on the CPU before re-uploading) and
  // polish()'s per-step assembly (which consumes the device buffers
  // directly -- no lambda, no retry, so no host round trip is needed).
  void assembleBatchToDevice(const std::vector<int>& activeItems) {
    // Cross-item-batch every registered Python-backed factor group's net
    // evaluation once, before the per-item linearize() loop below reads
    // the (now cached) results back through the unchanged CustomFactor
    // callback contract.
    for (const auto& hook : factorGroupHooks) {
      for (int item : activeItems) {
        hook.stage(item, currentValues[static_cast<size_t>(item)]);
      }
      hook.computeBatch();
    }
    const size_t numActive = activeItems.size();
    std::vector<Matrix> itemA(numActive);
    std::vector<Vector> itemB(numActive);
    int jacobianRows = -1;
    for (size_t idx = 0; idx < numActive; ++idx) {
      const size_t i = static_cast<size_t>(activeItems[idx]);
      const auto linearized = graphs[i].linearize(currentValues[i]);
      const std::vector<FactorAssemblyPlan>& plan = itemAssemblyPlan[i];
      const size_t totalRows =
          plan.empty() ? 0 : plan.back().rowOffset + plan.back().rowCount;
      itemA[idx] = Matrix::Zero(static_cast<Eigen::Index>(totalRows),
                                dimension);
      itemB[idx] = Vector::Zero(static_cast<Eigen::Index>(totalRows));
      for (size_t f = 0; f < plan.size(); ++f) {
        const FactorAssemblyPlan& factorPlan = plan[f];
        if (factorPlan.rowCount == 0) continue;
        auto jf = std::dynamic_pointer_cast<JacobianFactor>(
            linearized->at(f));
        if (!jf) {
          throw std::runtime_error(
              "BatchedBlockTridiagonalLevenbergMarquardtOptimizer: factor "
              "" + std::to_string(f) +
              " linearized to a non-JacobianFactor or null after "
              "producing rows at construction -- active()/type must stay "
              "stable across outer iterations for the cached assembly "
              "plan to remain valid");
        }
        itemB[idx].segment(static_cast<Eigen::Index>(factorPlan.rowOffset),
                           static_cast<Eigen::Index>(factorPlan.rowCount)) =
            jf->getb();
        for (size_t k = 0; k < factorPlan.columns.size(); ++k) {
          const ColumnSlot& slot = factorPlan.columns[k];
          itemA[idx].block(
              static_cast<Eigen::Index>(factorPlan.rowOffset),
              static_cast<Eigen::Index>(slot.colOffset),
              static_cast<Eigen::Index>(factorPlan.rowCount),
              static_cast<Eigen::Index>(slot.colWidth)) =
              jf->getA(jf->begin() + static_cast<std::ptrdiff_t>(k));
        }
      }
      if (jacobianRows < 0) {
        jacobianRows = static_cast<int>(itemA[idx].rows());
      } else if (static_cast<int>(itemA[idx].rows()) != jacobianRows) {
        throw std::runtime_error(
            "BatchedBlockTridiagonalLevenbergMarquardtOptimizer requires "
            "every active item to linearize to the same residual row "
            "count for the batched GPU Hessian assembly");
      }
    }

    // Stack active items' Jacobians/RHS into one contiguous buffer,
    // then assemble diag/offDiag/rhs for the whole batch via GPU GEMM.
    std::vector<double> hostJacobianA(
        numActive * static_cast<size_t>(jacobianRows) *
        static_cast<size_t>(dimension));
    std::vector<double> hostJacobianB(numActive *
                                      static_cast<size_t>(jacobianRows));
    for (size_t idx = 0; idx < numActive; ++idx) {
      Eigen::Map<Matrix>(hostJacobianA.data() +
                            idx * static_cast<size_t>(jacobianRows) *
                                static_cast<size_t>(dimension),
                        jacobianRows, dimension) = itemA[idx];
      Eigen::Map<Vector>(hostJacobianB.data() +
                            idx * static_cast<size_t>(jacobianRows),
                        jacobianRows) = itemB[idx];
    }
    deviceJacobianA.upload(hostJacobianA);
    deviceJacobianB.upload(hostJacobianB);
    hessianAssembler.assemble(static_cast<int>(numActive), numBlocks,
                             blockDim, jacobianRows, &deviceJacobianA,
                             &deviceJacobianB, &deviceDiagRaw,
                             &deviceOffDiagRaw, &deviceRhsRaw);
  }

  // Downloads deviceDiagRaw/deviceOffDiagRaw/deviceRhsRaw (as left by the
  // most recent assembleBatchToDevice(activeItems) call) into diagBase/
  // offDiagBase/rhsBase at those items' indices -- only optimize()'s
  // runAttemptRound needs this (per-item lambda damping is CPU-side
  // bookkeeping), so it is a separate step from assembleBatchToDevice.
  void downloadAssembledBatch(const std::vector<int>& activeItems) {
    const size_t blockElems =
        static_cast<size_t>(blockDim) * static_cast<size_t>(blockDim);
    const size_t numActive = activeItems.size();
    std::vector<double> hostDiagRaw, hostOffDiagRaw, hostRhsRaw;
    deviceDiagRaw.download(&hostDiagRaw);
    deviceOffDiagRaw.download(&hostOffDiagRaw);
    deviceRhsRaw.download(&hostRhsRaw);
    GTSAM_CUDA_CHECK(cudaStreamSynchronize(nullptr));

    for (size_t idx = 0; idx < numActive; ++idx) {
      const size_t i = static_cast<size_t>(activeItems[idx]);
      diagBase[i].resize(static_cast<size_t>(numBlocks));
      rhsBase[i].resize(static_cast<size_t>(numBlocks));
      offDiagBase[i].resize(static_cast<size_t>(numBlocks - 1));
      const size_t diagOffset = idx * numBlocks * blockElems;
      const size_t offDiagOffset = idx * (numBlocks - 1) * blockElems;
      const size_t rhsOffset = idx * static_cast<size_t>(dimension);
      for (int t = 0; t < numBlocks; ++t) {
        diagBase[i][static_cast<size_t>(t)] = Eigen::Map<const Matrix>(
            hostDiagRaw.data() + diagOffset +
                static_cast<size_t>(t) * blockElems,
            blockDim, blockDim);
        rhsBase[i][static_cast<size_t>(t)] = Eigen::Map<const Vector>(
            hostRhsRaw.data() + rhsOffset +
                static_cast<size_t>(t) * static_cast<size_t>(blockDim),
            blockDim);
        if (t + 1 < numBlocks) {
          offDiagBase[i][static_cast<size_t>(t)] = Eigen::Map<const Matrix>(
              hostOffDiagRaw.data() + offDiagOffset +
                  static_cast<size_t>(t) * blockElems,
              blockDim, blockDim);
        }
      }
    }
  }

  // Plain (undamped) Gauss-Newton polish: every item participates, every
  // step is taken unconditionally (no lambda, no retry, no accept/reject,
  // no termination/convergence bookkeeping) -- unlike optimize()'s
  // runAttemptRound, which is why this does not reuse it despite the
  // similar shape.
  const std::vector<Values>& polish(int numSteps) {
    if (deviceId >= 0) {
      GTSAM_CUDA_CHECK(cudaSetDevice(deviceId));
    }
    if (numSteps <= 0) return currentValues;

    std::vector<int> allItems(static_cast<size_t>(batchSize));
    for (int item = 0; item < batchSize; ++item) {
      allItems[static_cast<size_t>(item)] = item;
    }
    const size_t rhsItemStride =
        static_cast<size_t>(numBlocks) * static_cast<size_t>(blockDim);

    for (int step = 0; step < numSteps; ++step) {
      // Every item always participates and no lambda damping is added, so
      // (unlike optimize()'s runAttemptRound) there is nothing CPU-side to
      // do to deviceDiagRaw/deviceOffDiagRaw/deviceRhsRaw before solving --
      // feed them to the solver directly, skipping the
      // download-to-host-then-reupload round trip downloadAssembledBatch()
      // exists for. factorize()/solve() are destructive, but these buffers
      // are already fully consumed by the end of this step, so that's fine.
      assembleBatchToDevice(allItems);
      solver.factorize(batchSize, numBlocks, blockDim, &deviceDiagRaw,
                       &deviceOffDiagRaw, nullptr, &stats);
      solver.solve(1, &deviceRhsRaw, nullptr, &stats);
      std::vector<double> hostDelta;
      deviceRhsRaw.download(&hostDelta);
      GTSAM_CUDA_CHECK(cudaStreamSynchronize(nullptr));

      for (int item = 0; item < batchSize; ++item) {
        const size_t i = static_cast<size_t>(item);
        Vector delta(dimension);
        std::copy(hostDelta.begin() + i * rhsItemStride,
                 hostDelta.begin() + (i + 1) * rhsItemStride, delta.data());
        if (!delta.allFinite()) {
          throw std::runtime_error(
              "BatchedBlockTridiagonalLevenbergMarquardtOptimizer::polish "
              "produced a non-finite delta for item " +
              std::to_string(item));
        }
        VectorValues deltaValues = buildVectorValues(delta, keyInfos[i]);
        currentValues[i] = currentValues[i].retract(deltaValues);
      }
      // Any subsequent optimize() attempt round must not reuse offDiag/rhs
      // cached before this polish step changed currentValues.
      pristineValid = false;
    }

    for (int item = 0; item < batchSize; ++item) {
      const size_t i = static_cast<size_t>(item);
      currentError[i] = graphs[i].error(currentValues[i]);
      itemResults[i].finalError = currentError[i];
    }
    return currentValues;
  }

  const std::vector<Values>& optimize() {
    if (deviceId >= 0) {
      GTSAM_CUDA_CHECK(cudaSetDevice(deviceId));
    }
    if (optimized) return currentValues;
    if (params.maxIterations == 0 || allTerminated()) {
      finish();
      return currentValues;
    }

    const int maxOuterIterations = static_cast<int>(params.maxIterations);
    for (int outer = 0; outer < maxOuterIterations && !allTerminated();
        ++outer) {
      std::vector<int> activeItems;
      for (int item = 0; item < batchSize; ++item) {
        if (!terminated[static_cast<size_t>(item)]) activeItems.push_back(item);
      }
      assembleBatchToDevice(activeItems);
      downloadAssembledBatch(activeItems);

      // diagBase/offDiagBase/rhsBase were just rebuilt from a fresh
      // linearization, so the pristine copy is stale and must be rebuilt
      // on attempt 1.
      pristineValid = false;
      std::vector<bool> acceptedThisOuter(static_cast<size_t>(batchSize),
                                         false);
      for (size_t attempt = 0;
          attempt < kMaxAttemptsPerOuterIteration &&
          !std::all_of(acceptedThisOuter.begin(), acceptedThisOuter.end(),
                      [](bool a) { return a; }) &&
          !allTerminated();
          ++attempt) {
        runAttemptRound(&acceptedThisOuter);
      }
    }

    finish();
    return currentValues;
  }

  void runAttemptRound(std::vector<bool>* acceptedThisOuter) {
    std::fill(hostDiag.begin(), hostDiag.end(), 0.0);
    std::fill(participating.begin(), participating.end(), false);

    const size_t diagItemStride = static_cast<size_t>(numBlocks) *
                                 static_cast<size_t>(blockDim) *
                                 static_cast<size_t>(blockDim);
    const size_t offDiagItemStride = static_cast<size_t>(numBlocks - 1) *
                                    static_cast<size_t>(blockDim) *
                                    static_cast<size_t>(blockDim);
    const size_t rhsItemStride =
        static_cast<size_t>(numBlocks) * static_cast<size_t>(blockDim);
    const size_t blockElems =
        static_cast<size_t>(blockDim) * static_cast<size_t>(blockDim);

    // hostDiag genuinely changes every attempt (lambda damping), so it is
    // always rebuilt and uploaded directly into the working buffer below.
    for (int item = 0; item < batchSize; ++item) {
      const size_t i = static_cast<size_t>(item);
      const size_t diagItemOffset = i * diagItemStride;
      if (terminated[i] || (*acceptedThisOuter)[i]) {
        for (int t = 0; t < numBlocks; ++t) {
          const size_t blockOffset =
              diagItemOffset + static_cast<size_t>(t) * blockElems;
          for (int d = 0; d < blockDim; ++d) {
            hostDiag[blockOffset + static_cast<size_t>(d) *
                                       static_cast<size_t>(blockDim) +
                    static_cast<size_t>(d)] = 1.0;
          }
        }
        continue;
      }
      participating[i] = true;
      for (int t = 0; t < numBlocks; ++t) {
        const size_t blockOffset =
            diagItemOffset + static_cast<size_t>(t) * blockElems;
        const Matrix& block = diagBase[i][static_cast<size_t>(t)];
        std::copy(block.data(), block.data() + block.size(),
                 hostDiag.begin() + blockOffset);
        // Matches LevenbergMarquardtOptimizer::buildDampedSystem exactly
        // (see BatchedDenseLevenbergMarquardt.cpp's identical fix): honor
        // diagonalDamping instead of always applying flat identity damping
        // (PR #2937 review, "nonlinear optimization" finding).
        for (int d = 0; d < blockDim; ++d) {
          const size_t idx = blockOffset + static_cast<size_t>(d) *
                                               static_cast<size_t>(blockDim) +
                             static_cast<size_t>(d);
          const double dampingTerm =
              params.dampingParams.diagonalDamping
                  ? lambda[i] * std::clamp(hostDiag[idx],
                                          params.dampingParams.minDiagonal,
                                          params.dampingParams.maxDiagonal)
                  : lambda[i];
          hostDiag[idx] += dampingTerm;
        }
      }
    }
    deviceDiag.upload(hostDiag);

    // offDiag/rhs only change when the participating set itself changes
    // (an item accepted or terminated since the last attempt): rebuild and
    // upload into the pristine buffers only then, otherwise reuse what is
    // already there.
    if (!pristineValid || participating != previousParticipating) {
      std::fill(hostOffDiag.begin(), hostOffDiag.end(), 0.0);
      for (int item = 0; item < batchSize; ++item) {
        const size_t i = static_cast<size_t>(item);
        if (!participating[i]) continue;
        const size_t offDiagItemOffset = i * offDiagItemStride;
        const size_t rhsItemOffset = i * rhsItemStride;
        for (int t = 0; t < numBlocks; ++t) {
          const Vector& rhsBlock = rhsBase[i][static_cast<size_t>(t)];
          std::copy(rhsBlock.data(), rhsBlock.data() + rhsBlock.size(),
                   hostRhs.begin() + rhsItemOffset +
                       static_cast<size_t>(t) * static_cast<size_t>(blockDim));
          if (t + 1 < numBlocks) {
            const size_t offBlockOffset =
                offDiagItemOffset + static_cast<size_t>(t) * blockElems;
            const Matrix& offBlock = offDiagBase[i][static_cast<size_t>(t)];
            std::copy(offBlock.data(), offBlock.data() + offBlock.size(),
                     hostOffDiag.begin() + offBlockOffset);
          }
        }
      }
      pristineOffDiag.upload(hostOffDiag);
      pristineRhs.upload(hostRhs);
      previousParticipating = participating;
      pristineValid = true;
    }

    // Refresh working buffers from the untouched pristine copies every
    // attempt (cheap, on-device), since factorize()/solve() destroy them.
    deviceOffDiag.copyFrom(pristineOffDiag);
    solver.factorize(batchSize, numBlocks, blockDim, &deviceDiag,
                     &deviceOffDiag, nullptr, &stats);
    deviceRhs.copyFrom(pristineRhs);
    solver.solve(1, &deviceRhs, nullptr, &stats);
    std::vector<double> hostDelta;
    deviceRhs.download(&hostDelta);
    GTSAM_CUDA_CHECK(cudaStreamSynchronize(nullptr));

    for (int item = 0; item < batchSize; ++item) {
      const size_t i = static_cast<size_t>(item);
      if (!participating[i]) continue;
      ++itemResults[i].lambdaAttempts;

      Vector delta(dimension);
      std::copy(hostDelta.begin() + i * rhsItemStride,
               hostDelta.begin() + (i + 1) * rhsItemStride, delta.data());
      if (!delta.allFinite()) {
        throw std::runtime_error(
            "BatchedBlockTridiagonalLevenbergMarquardtOptimizer produced a "
            "non-finite delta for item " +
            std::to_string(item));
      }

      // rhsBase is per-block here, so flatten it to match delta's layout.
      Vector rhsFlat(dimension);
      for (int t = 0; t < numBlocks; ++t) {
        rhsFlat.segment(t * blockDim, blockDim) =
            rhsBase[i][static_cast<size_t>(t)];
      }
      const double linearizedChange =
          0.5 * delta.dot(rhsFlat + lambda[i] * delta);

      // Matches real LevenbergMarquardtOptimizer: skip retract()/error()
      // when the linear model already predicts a rejected step.
      double costChange = 0.0;
      double modelFidelity = 0.0;
      bool stepSuccessful = false;
      Values trialValues;
      double trialError = 0.0;
      if (linearizedChange >= 0.0) {
        VectorValues deltaValues = buildVectorValues(delta, keyInfos[i]);
        trialValues = currentValues[i].retract(deltaValues);
        trialError = graphs[i].error(trialValues);
        if (!std::isfinite(trialError)) {
          throw std::runtime_error(
              "BatchedBlockTridiagonalLevenbergMarquardtOptimizer produced a "
              "non-finite trial error for item " +
              std::to_string(item));
        }
        costChange = currentError[i] - trialError;
        if (linearizedChange > std::numeric_limits<double>::epsilon() *
                                   std::abs(currentError[i])) {
          modelFidelity = costChange / linearizedChange;
          stepSuccessful = modelFidelity > params.minModelFidelity;
        }
      }

      if (stepSuccessful) {
        currentValues[i] = trialValues;
        currentError[i] = trialError;
        gtsam::internal::decreaseLevenbergMarquardtLambda(
            params, modelFidelity, &lambda[i], &currentFactor[i]);
        ++itemResults[i].acceptedIterations;
        (*acceptedThisOuter)[i] = true;
        if (costChange < params.relativeErrorTol * currentError[i] ||
            costChange < params.absoluteErrorTol) {
          terminated[i] = true;
          itemResults[i].converged = true;
        }
      } else if (linearizedChange >= 0.0 &&
                std::abs(costChange) <
                    params.relativeErrorTol * currentError[i]) {
        // Matches tryLambda's stopSearchingLambda: cost barely changed even
        // on a valid trial, not worth a bigger lambda; no step is taken.
        (*acceptedThisOuter)[i] = true;
        terminated[i] = true;
        itemResults[i].converged = true;
      } else {
        gtsam::internal::increaseLevenbergMarquardtLambda(
            params, &lambda[i], &currentFactor[i]);
        if (lambda[i] >= params.lambdaUpperBound) {
          terminated[i] = true;
          (*acceptedThisOuter)[i] = true;
        }
      }
    }
  }

  void finish() {
    for (int b = 0; b < batchSize; ++b) {
      itemResults[static_cast<size_t>(b)].finalError =
          currentError[static_cast<size_t>(b)];
      itemResults[static_cast<size_t>(b)].finalLambda =
          lambda[static_cast<size_t>(b)];
    }
    optimized = true;
  }
};

BatchedBlockTridiagonalLevenbergMarquardtOptimizer::
    BatchedBlockTridiagonalLevenbergMarquardtOptimizer(
        const std::vector<NonlinearFactorGraph>& graphs,
        const std::vector<Values>& initialValues, int numBlocks, int blockDim,
        const LevenbergMarquardtParams& params,
        const BatchedDenseSolverOptions& deviceOptions,
        const std::vector<BatchedFactorGroupHook>& factorGroupHooks)
    : impl_(std::make_unique<Impl>(graphs, initialValues, numBlocks, blockDim,
                                  params, deviceOptions, factorGroupHooks)) {}

BatchedBlockTridiagonalLevenbergMarquardtOptimizer::
    ~BatchedBlockTridiagonalLevenbergMarquardtOptimizer() = default;

const std::vector<Values>&
BatchedBlockTridiagonalLevenbergMarquardtOptimizer::optimize() {
  return impl_->optimize();
}

const std::vector<Values>&
BatchedBlockTridiagonalLevenbergMarquardtOptimizer::polish(int numSteps) {
  return impl_->polish(numSteps);
}

const std::vector<Values>&
BatchedBlockTridiagonalLevenbergMarquardtOptimizer::values() const {
  return impl_->currentValues;
}

const std::vector<BatchedBlockTridiagonalLevenbergMarquardtItemResult>&
BatchedBlockTridiagonalLevenbergMarquardtOptimizer::results() const {
  return impl_->itemResults;
}

const LinearSolveStats&
BatchedBlockTridiagonalLevenbergMarquardtOptimizer::linearSolveStats() const {
  return impl_->stats;
}

}  // namespace gtsam::cuda

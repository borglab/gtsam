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
#include <gtsam/linear/VectorValues.h>
#include <gtsam/linear/cuda/internal/BatchedBlockHessianAssembler.h>
#include <gtsam/nonlinear/internal/LevenbergMarquardtPolicy.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>

namespace gtsam::cuda {
namespace {

constexpr size_t kMaxAttemptsPerOuterIteration = 32;

using Clock = std::chrono::steady_clock;

}  // namespace

struct BatchedBlockTridiagonalLevenbergMarquardtOptimizer::Impl {
  std::vector<NonlinearFactorGraph> graphs;
  std::vector<Values> currentValues;
  std::vector<KeyInfo> keyInfos;
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

  // TEMPORARY diagnostics -- to be reverted once the current real-workload
  // breakdown is measured. See [diag-forward] print in optimize().
  double diagHookStageSeconds = 0.0;
  double diagHookComputeBatchSeconds = 0.0;
  double diagLinearizeSeconds = 0.0;
  double diagAssemblySeconds = 0.0;
  // Throwaway CPU recomputation of the exact same diag/offDiag/rhs this
  // outer iteration's GPU assembly just produced, timed only -- answers
  // "would CPU have been faster for this exact op on this exact data,"
  // independent of cross-item batching.
  double diagCpuAssemblyEquivalentSeconds = 0.0;
  double diagAttemptRoundSeconds = 0.0;
  double diagAttemptErrorSeconds = 0.0;
  size_t diagAttemptErrorCalls = 0;

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
        blockDim(inputBlockDim) {
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

  const std::vector<Values>& optimize() {
    if (optimized) return currentValues;
    if (params.maxIterations == 0 || allTerminated()) {
      finish();
      return currentValues;
    }

    const int maxOuterIterations = static_cast<int>(params.maxIterations);
    const size_t blockElems =
        static_cast<size_t>(blockDim) * static_cast<size_t>(blockDim);
    for (int outer = 0; outer < maxOuterIterations && !allTerminated();
        ++outer) {
      // linearize() stays on CPU; only the At^T*At/At^T*b/At^T*At1
      // formation below moves to the GPU.
      std::vector<int> activeItems;
      for (int item = 0; item < batchSize; ++item) {
        if (!terminated[static_cast<size_t>(item)]) activeItems.push_back(item);
      }
      // Cross-item-batch every registered Python-backed factor group's net
      // evaluation once, before the per-item linearize() loop below reads
      // the (now cached) results back through the unchanged CustomFactor
      // callback contract.
      for (const auto& hook : factorGroupHooks) {
        auto stageStart = Clock::now();
        for (int item : activeItems) {
          hook.stage(item, currentValues[static_cast<size_t>(item)]);
        }
        diagHookStageSeconds +=
            std::chrono::duration<double>(Clock::now() - stageStart).count();
        auto computeStart = Clock::now();
        hook.computeBatch();
        diagHookComputeBatchSeconds +=
            std::chrono::duration<double>(Clock::now() - computeStart)
                .count();
      }
      const size_t numActive = activeItems.size();
      std::vector<Matrix> itemA(numActive);
      std::vector<Vector> itemB(numActive);
      int jacobianRows = -1;
      auto linearizeStart = Clock::now();
      for (size_t idx = 0; idx < numActive; ++idx) {
        const size_t i = static_cast<size_t>(activeItems[idx]);
        const auto linearized = graphs[i].linearize(currentValues[i]);
        std::tie(itemA[idx], itemB[idx]) =
            linearized->jacobian(keyInfos[i].ordering());
        if (jacobianRows < 0) {
          jacobianRows = static_cast<int>(itemA[idx].rows());
        } else if (static_cast<int>(itemA[idx].rows()) != jacobianRows) {
          throw std::runtime_error(
              "BatchedBlockTridiagonalLevenbergMarquardtOptimizer requires "
              "every active item to linearize to the same residual row "
              "count for the batched GPU Hessian assembly");
        }
      }
      diagLinearizeSeconds +=
          std::chrono::duration<double>(Clock::now() - linearizeStart)
              .count();

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
      auto assemblyStart = Clock::now();
      deviceJacobianA.upload(hostJacobianA);
      deviceJacobianB.upload(hostJacobianB);
      hessianAssembler.assemble(static_cast<int>(numActive), numBlocks,
                               blockDim, jacobianRows, &deviceJacobianA,
                               &deviceJacobianB, &deviceDiagRaw,
                               &deviceOffDiagRaw, &deviceRhsRaw);
      std::vector<double> hostDiagRaw, hostOffDiagRaw, hostRhsRaw;
      deviceDiagRaw.download(&hostDiagRaw);
      deviceOffDiagRaw.download(&hostOffDiagRaw);
      deviceRhsRaw.download(&hostRhsRaw);
      GTSAM_CUDA_CHECK(cudaStreamSynchronize(nullptr));
      diagAssemblySeconds +=
          std::chrono::duration<double>(Clock::now() - assemblyStart).count();

      // Throwaway: recompute the same diag/offDiag/rhs via plain CPU Eigen
      // matmul on the exact same itemA/itemB data, timed only, result
      // discarded -- isolates "is this op itself faster on CPU" from
      // "is cross-item batching faster than per-item."
      {
        auto cpuStart = Clock::now();
        volatile double sink = 0.0;
        for (size_t idx = 0; idx < numActive; ++idx) {
          const Matrix& A = itemA[idx];
          const Vector& b = itemB[idx];
          for (int t = 0; t < numBlocks; ++t) {
            const auto colsT = A.middleCols(t * blockDim, blockDim);
            Matrix diagBlock = colsT.transpose() * colsT;
            Vector rhsBlock = colsT.transpose() * b;
            sink += diagBlock(0, 0) + rhsBlock(0);
            if (t + 1 < numBlocks) {
              const auto colsT1 = A.middleCols((t + 1) * blockDim, blockDim);
              Matrix offDiagBlock = colsT.transpose() * colsT1;
              sink += offDiagBlock(0, 0);
            }
          }
        }
        diagCpuAssemblyEquivalentSeconds +=
            std::chrono::duration<double>(Clock::now() - cpuStart).count();
        (void)sink;
      }

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

      // offDiagBase/rhsBase were just rebuilt from a fresh linearization,
      // so the pristine copy is stale and must be rebuilt on attempt 1.
      pristineValid = false;
      std::vector<bool> acceptedThisOuter(static_cast<size_t>(batchSize),
                                         false);
      for (size_t attempt = 0;
          attempt < kMaxAttemptsPerOuterIteration &&
          !std::all_of(acceptedThisOuter.begin(), acceptedThisOuter.end(),
                      [](bool a) { return a; }) &&
          !allTerminated();
          ++attempt) {
        auto attemptStart = Clock::now();
        runAttemptRound(&acceptedThisOuter);
        diagAttemptRoundSeconds +=
            std::chrono::duration<double>(Clock::now() - attemptStart)
                .count();
      }
    }

    fprintf(stderr,
            "[diag-forward] hookStageSeconds=%.3f "
            "hookComputeBatchSeconds=%.3f linearizeSeconds=%.3f "
            "assemblySeconds=%.3f cpuAssemblyEquivalentSeconds=%.3f "
            "attemptRoundSeconds=%.3f attemptErrorSeconds=%.3f "
            "attemptErrorCalls=%zu factorizationSeconds=%.3f "
            "solveSeconds=%.3f factorizationCount=%zu solveCount=%zu\n",
            diagHookStageSeconds, diagHookComputeBatchSeconds,
            diagLinearizeSeconds, diagAssemblySeconds,
            diagCpuAssemblyEquivalentSeconds, diagAttemptRoundSeconds,
            diagAttemptErrorSeconds, diagAttemptErrorCalls,
            stats.factorizationSeconds, stats.solveSeconds,
            stats.factorizationCount, stats.solveCount);

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
        for (int d = 0; d < blockDim; ++d) {
          hostDiag[blockOffset + static_cast<size_t>(d) *
                                     static_cast<size_t>(blockDim) +
                  static_cast<size_t>(d)] += lambda[i];
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
        auto errorStart = Clock::now();
        VectorValues deltaValues = buildVectorValues(delta, keyInfos[i]);
        trialValues = currentValues[i].retract(deltaValues);
        trialError = graphs[i].error(trialValues);
        diagAttemptErrorSeconds +=
            std::chrono::duration<double>(Clock::now() - errorStart).count();
        ++diagAttemptErrorCalls;
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

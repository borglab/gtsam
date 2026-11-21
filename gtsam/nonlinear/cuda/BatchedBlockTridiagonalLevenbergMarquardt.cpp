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
  LevenbergMarquardtParams params;
  BatchedBlockTridiagonalCholeskySolver solver;
  DeviceArray<double> deviceDiag;
  DeviceArray<double> deviceOffDiag;
  DeviceArray<double> deviceRhs;
  // factorize()/solve() destructively overwrite deviceOffDiag/deviceRhs in
  // place (documented in BatchedBlockTridiagonalCholeskySolver.h -- they
  // become Z_t and the solution, respectively), so each attempt needs a
  // fresh copy of the original values before calling them again. Unlike
  // deviceDiag (genuinely different every attempt: lambda damping
  // changes), offDiag/rhs only change when the participating set changes
  // (an item accepts or terminates), so instead of re-marshaling host-side
  // and re-uploading over PCIe every attempt, the untouched original is
  // kept here and refreshed into deviceOffDiag/deviceRhs via a cheap
  // on-device copy (no PCIe) every attempt; the comparatively expensive
  // host-side rebuild + upload into these two only happens when the
  // participating set actually changed.
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

  Impl(const std::vector<NonlinearFactorGraph>& inputGraphs,
       const std::vector<Values>& initialValues, int inputNumBlocks,
       int inputBlockDim, const LevenbergMarquardtParams& inputParams,
       const BatchedDenseSolverOptions& deviceOptions)
      : graphs(inputGraphs),
        currentValues(initialValues),
        params(inputParams),
        solver(deviceOptions),
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
    for (int outer = 0; outer < maxOuterIterations && !allTerminated();
        ++outer) {
      // Same GIL-release precondition as BatchedDenseLevenbergMarquardt's
      // identical loop (see cuda_nonlinear.h). Also needs -fopenmp
      // registered for this file in gtsam/CMakeLists.txt, or the pragma
      // below silently runs serially (cost ~8x here once, before being caught).
#pragma omp parallel for num_threads(batchSize) schedule(static)
      for (int item = 0; item < batchSize; ++item) {
        const size_t i = static_cast<size_t>(item);
        if (terminated[i]) continue;
        const auto linearized = graphs[i].linearize(currentValues[i]);
        Matrix A;
        Vector b;
        std::tie(A, b) = linearized->jacobian(keyInfos[i].ordering());

        // Only the block-tridiagonal pieces (see
        // BatchedBlockTridiagonalCholeskySolver.h for why exact, not
        // approximate, on a chain graph), never the full dense A^T A.
        diagBase[i].resize(static_cast<size_t>(numBlocks));
        rhsBase[i].resize(static_cast<size_t>(numBlocks));
        offDiagBase[i].resize(static_cast<size_t>(numBlocks - 1));
        // Copy each block into contiguous memory first, since A.block(...)
        // is a strided view and multiplying through it directly is slow.
        std::vector<Matrix> blockCopies(static_cast<size_t>(numBlocks));
        for (int t = 0; t < numBlocks; ++t) {
          blockCopies[static_cast<size_t>(t)] =
              A.block(0, t * blockDim, A.rows(), blockDim);
        }
        for (int t = 0; t < numBlocks; ++t) {
          const Matrix& At = blockCopies[static_cast<size_t>(t)];
          diagBase[i][static_cast<size_t>(t)] = At.transpose() * At;
          rhsBase[i][static_cast<size_t>(t)] = At.transpose() * b;
          if (t + 1 < numBlocks) {
            const Matrix& At1 = blockCopies[static_cast<size_t>(t + 1)];
            offDiagBase[i][static_cast<size_t>(t)] = At.transpose() * At1;
          }
        }
      }

      // diagBase/offDiagBase/rhsBase were just rebuilt above from a fresh
      // linearization, so any pristine offDiag/rhs from the previous outer
      // iteration is now stale and must be rebuilt on this iteration's
      // first attempt, regardless of whether the participating pattern
      // happens to match.
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

    // factorize()/solve() destructively overwrite their offDiag/rhs
    // arguments, so the working buffers need a fresh copy from the
    // (untouched) pristine ones before every call -- cheap, on-device,
    // no PCIe transfer, regardless of whether the pristine rebuild above
    // ran this attempt.
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

      const VectorValues deltaValues = buildVectorValues(delta, keyInfos[i]);
      const Values trialValues = currentValues[i].retract(deltaValues);
      const double trialError = graphs[i].error(trialValues);
      if (!std::isfinite(trialError)) {
        throw std::runtime_error(
            "BatchedBlockTridiagonalLevenbergMarquardtOptimizer produced a "
            "non-finite trial error for item " +
            std::to_string(item));
      }

      // Same GTSAM real trust-region model-fidelity rule as
      // BatchedDenseLevenbergMarquardt; rhsBase is per-block here, so flatten
      // it to match delta's layout before the dot product.
      Vector rhsFlat(dimension);
      for (int t = 0; t < numBlocks; ++t) {
        rhsFlat.segment(t * blockDim, blockDim) =
            rhsBase[i][static_cast<size_t>(t)];
      }
      const double linearizedChange =
          0.5 * delta.dot(rhsFlat + lambda[i] * delta);
      const double costChange = currentError[i] - trialError;
      double modelFidelity = 0.0;
      bool stepSuccessful = false;
      if (linearizedChange >
          std::numeric_limits<double>::epsilon() * std::abs(currentError[i])) {
        modelFidelity = costChange / linearizedChange;
        stepSuccessful = modelFidelity > params.minModelFidelity;
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
        const BatchedDenseSolverOptions& deviceOptions)
    : impl_(std::make_unique<Impl>(graphs, initialValues, numBlocks, blockDim,
                                  params, deviceOptions)) {}

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

/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    BatchedDenseLevenbergMarquardt.cpp
 * @brief   Levenberg-Marquardt over B independent, equal-dimension nonlinear
 *          graphs, batching only the dense linear solve on one GPU
 * @author  Shaji
 * @date    Nov 20, 2026
 */

#include <gtsam/nonlinear/cuda/BatchedDenseLevenbergMarquardt.h>

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

/// Maximum lambda-retry attempts per outer iteration before an item is left
/// unchanged for this iteration and retried fresh (new linearization) next
/// iteration, rather than looping forever on a single bad step.
constexpr size_t kMaxAttemptsPerOuterIteration = 32;

using Clock = std::chrono::steady_clock;

}  // namespace

struct BatchedDenseLevenbergMarquardtOptimizer::Impl {
  std::vector<NonlinearFactorGraph> graphs;
  std::vector<Values> currentValues;
  std::vector<KeyInfo> keyInfos;
  // Primed with every active item's Values and computed as one cross-item
  // batch before each outer iteration's per-item linearize() below; see
  // BatchedFactorGroupHook.h.
  std::vector<BatchedFactorGroupHook> factorGroupHooks;
  LevenbergMarquardtParams params;
  BatchedDenseCholeskySolver solver;
  // Persistent across every round so upload() never reallocates; a fresh
  // DeviceArray per round was previously the dominant real cost here.
  DeviceArray<double> deviceH;
  DeviceArray<double> deviceRhs;
  LinearSolveStats stats;
  BatchedDenseLevenbergMarquardtHostTimings hostTimings;
  // Host-side counterpart to the same persistent-buffer fix.
  std::vector<double> hostH;
  std::vector<double> hostRhs;
  std::vector<bool> participating;
  // Undamped A^T A / A^T b, cached once per outer iteration (see optimize())
  // and reused by every lambda-retry attempt within it.
  std::vector<Matrix> hBase;
  std::vector<Vector> rhsBase;

  int batchSize = 0;
  int dimension = 0;
  std::vector<double> currentError;
  std::vector<double> lambda;
  // Per-item rejection multiplier, one per item instead of one globally.
  std::vector<double> currentFactor;
  std::vector<bool> terminated;  // converged or hit lambdaUpperBound
  std::vector<BatchedDenseLevenbergMarquardtItemResult> itemResults;
  bool optimized = false;
  // Re-asserted by optimize() before its own device uploads/downloads:
  // cudaSetDevice is thread-global and must stay current per call.
  int deviceId = -1;

  Impl(const std::vector<NonlinearFactorGraph>& inputGraphs,
       const std::vector<Values>& initialValues,
       const LevenbergMarquardtParams& inputParams,
       const BatchedDenseSolverOptions& deviceOptions,
       const std::vector<BatchedFactorGroupHook>& inputFactorGroupHooks)
      : graphs(inputGraphs),
        currentValues(initialValues),
        factorGroupHooks(inputFactorGroupHooks),
        params(inputParams),
        solver(deviceOptions),
        deviceId(deviceOptions.deviceId) {
    if (inputGraphs.empty() || inputGraphs.size() != initialValues.size()) {
      throw std::invalid_argument(
          "BatchedDenseLevenbergMarquardtOptimizer requires equally-sized, "
          "nonempty graphs and initialValues batches");
    }
    batchSize = static_cast<int>(inputGraphs.size());
    keyInfos.reserve(static_cast<size_t>(batchSize));
    for (int b = 0; b < batchSize; ++b) {
      const auto linearized = graphs[static_cast<size_t>(b)].linearize(
          currentValues[static_cast<size_t>(b)]);
      KeyInfo keyInfo(*linearized);
      if (b == 0) {
        dimension = static_cast<int>(keyInfo.numCols());
        if (dimension <= 0) {
          throw std::invalid_argument(
              "BatchedDenseLevenbergMarquardtOptimizer requires a positive "
              "scalar dimension");
        }
      } else if (static_cast<int>(keyInfo.numCols()) != dimension) {
        std::ostringstream os;
        os << "BatchedDenseLevenbergMarquardtOptimizer requires every graph "
           << "to linearize to the same scalar dimension; graph 0 has "
           << dimension << ", graph " << b << " has " << keyInfo.numCols();
        throw std::invalid_argument(os.str());
      }
      keyInfos.push_back(std::move(keyInfo));
    }

    currentError.resize(static_cast<size_t>(batchSize));
    lambda.assign(static_cast<size_t>(batchSize), params.lambdaInitial);
    currentFactor.assign(static_cast<size_t>(batchSize), params.lambdaFactor);
    terminated.assign(static_cast<size_t>(batchSize), false);
    itemResults.resize(static_cast<size_t>(batchSize));
    hostH.resize(static_cast<size_t>(batchSize) *
                static_cast<size_t>(dimension) *
                static_cast<size_t>(dimension));
    hostRhs.resize(static_cast<size_t>(batchSize) *
                  static_cast<size_t>(dimension));
    participating.resize(static_cast<size_t>(batchSize));
    hBase.resize(static_cast<size_t>(batchSize));
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
      // H_base/rhsBase formed once per outer iteration, not per retry:
      // only the O(dimension) damping term changes per attempt, so
      // recomputing the full matrix product on every retry (as an earlier
      // version did) was the dominant real cost of this optimizer.
      {
        const auto linearizeStart = Clock::now();
        // Batch every registered factor group's net evaluation once,
        // before the per-item linearize() loop reads the cached results.
        for (const auto& hook : factorGroupHooks) {
          for (int item = 0; item < batchSize; ++item) {
            if (terminated[static_cast<size_t>(item)]) continue;
            hook.stage(item, currentValues[static_cast<size_t>(item)]);
          }
          hook.computeBatch();
        }
        for (int item = 0; item < batchSize; ++item) {
          const size_t i = static_cast<size_t>(item);
          if (terminated[i]) continue;
          const auto linearized = graphs[i].linearize(currentValues[i]);
          Matrix A;
          Vector b;
          std::tie(A, b) = linearized->jacobian(keyInfos[i].ordering());
          hBase[i] = A.transpose() * A;
          rhsBase[i] = A.transpose() * b;
        }
        hostTimings.linearizeAndFormSeconds +=
            std::chrono::duration<double>(Clock::now() - linearizeStart)
                .count();
      }

      std::vector<bool> acceptedThisOuter(static_cast<size_t>(batchSize),
                                         false);
      for (size_t attempt = 0;
          attempt < kMaxAttemptsPerOuterIteration &&
          !std::all_of(acceptedThisOuter.begin(), acceptedThisOuter.end(),
                      [](bool a) { return a; }) &&
          !allTerminated();
          ++attempt) {
        runAttemptRound(outer, &acceptedThisOuter);
      }
    }

    finish();
    return currentValues;
  }

  /// Builds and solves one batched damped system across every active item
  /// (others get an identity placeholder); reuses cached hBase/rhsBase.
  void runAttemptRound(int outer, std::vector<bool>* acceptedThisOuter) {
    const auto marshalStart = Clock::now();
    std::fill(hostH.begin(), hostH.end(), 0.0);
    std::fill(participating.begin(), participating.end(), false);

    for (int item = 0; item < batchSize; ++item) {
      const size_t i = static_cast<size_t>(item);
      const size_t hOffset =
          i * static_cast<size_t>(dimension) * static_cast<size_t>(dimension);
      const size_t rhsOffset = i * static_cast<size_t>(dimension);
      if (terminated[i] || (*acceptedThisOuter)[i]) {
        for (int d = 0; d < dimension; ++d) {
          hostH[hOffset + static_cast<size_t>(d) *
                              static_cast<size_t>(dimension) +
                static_cast<size_t>(d)] = 1.0;
        }
        continue;
      }
      participating[i] = true;
      std::copy(hBase[i].data(), hBase[i].data() + hBase[i].size(),
               hostH.begin() + hOffset);
      // Matches LevenbergMarquardtOptimizer::buildDampedSystem: scale
      // damping by each entry's own magnitude when diagonalDamping is
      // set, instead of always applying flat identity damping.
      for (int d = 0; d < dimension; ++d) {
        const size_t idx = hOffset + static_cast<size_t>(d) *
                                         static_cast<size_t>(dimension) +
                           static_cast<size_t>(d);
        const double dampingTerm =
            params.dampingParams.diagonalDamping
                ? lambda[i] * std::clamp(hostH[idx],
                                        params.dampingParams.minDiagonal,
                                        params.dampingParams.maxDiagonal)
                : lambda[i];
        hostH[idx] += dampingTerm;
      }
      std::copy(rhsBase[i].data(), rhsBase[i].data() + rhsBase[i].size(),
               hostRhs.begin() + rhsOffset);
    }
    hostTimings.marshalSeconds +=
        std::chrono::duration<double>(Clock::now() - marshalStart).count();

    // Reuses the persistent deviceH/deviceRhs allocations (see their
    // declaration above): upload() only re-copies these bytes, it does not
    // reallocate once the batch's size is established on the first round.
    const auto gpuRoundTripStart = Clock::now();
    deviceH.upload(hostH);
    solver.factorize(batchSize, dimension, &deviceH, nullptr, &stats);
    deviceRhs.upload(hostRhs);
    solver.solve(1, &deviceRhs, nullptr, &stats);
    std::vector<double> hostDelta;
    deviceRhs.download(&hostDelta);
    // download() does not synchronize (see DeviceArray's documented
    // contract); this is the one wait that actually has to happen before
    // hostDelta below is safe to read.
    GTSAM_CUDA_CHECK(cudaStreamSynchronize(nullptr));
    hostTimings.gpuRoundTripSeconds +=
        std::chrono::duration<double>(Clock::now() - gpuRoundTripStart)
            .count();

    const auto retractErrorStart = Clock::now();
    for (int item = 0; item < batchSize; ++item) {
      const size_t i = static_cast<size_t>(item);
      if (!participating[i]) continue;
      ++itemResults[i].lambdaAttempts;

      Vector delta(dimension);
      std::copy(hostDelta.begin() + i * static_cast<size_t>(dimension),
               hostDelta.begin() + (i + 1) * static_cast<size_t>(dimension),
               delta.data());
      if (!delta.allFinite()) {
        throw std::runtime_error(
            "BatchedDenseLevenbergMarquardtOptimizer produced a non-finite "
            "delta for item " +
            std::to_string(item));
      }

      // GTSAM's real trust-region model-fidelity rule (standard LM/Nielsen
      // derivation), not a simplified monotonic-decrease rule.
      const double linearizedChange =
          0.5 * delta.dot(rhsBase[i] + lambda[i] * delta);

      // Matches real LevenbergMarquardtOptimizer: skip retract()/error()
      // when the linear model already predicts a rejected step.
      double costChange = 0.0;
      double modelFidelity = 0.0;
      bool stepSuccessful = false;
      Values trialValues;
      double trialError = 0.0;
      if (linearizedChange >= 0.0) {
        const VectorValues deltaValues = buildVectorValues(delta, keyInfos[i]);
        trialValues = currentValues[i].retract(deltaValues);
        trialError = graphs[i].error(trialValues);
        if (!std::isfinite(trialError)) {
          throw std::runtime_error(
              "BatchedDenseLevenbergMarquardtOptimizer produced a non-finite "
              "trial error for item " +
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
          (*acceptedThisOuter)[i] = true;  // stop retrying this outer iter
        }
      }
    }
    hostTimings.retractAndErrorSeconds +=
        std::chrono::duration<double>(Clock::now() - retractErrorStart)
            .count();
    (void)outer;
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

BatchedDenseLevenbergMarquardtOptimizer::
    BatchedDenseLevenbergMarquardtOptimizer(
        const std::vector<NonlinearFactorGraph>& graphs,
        const std::vector<Values>& initialValues,
        const LevenbergMarquardtParams& params,
        const BatchedDenseSolverOptions& deviceOptions,
        const std::vector<BatchedFactorGroupHook>& factorGroupHooks)
    : impl_(std::make_unique<Impl>(graphs, initialValues, params,
                                  deviceOptions, factorGroupHooks)) {}

BatchedDenseLevenbergMarquardtOptimizer::
    ~BatchedDenseLevenbergMarquardtOptimizer() = default;

const std::vector<Values>& BatchedDenseLevenbergMarquardtOptimizer::optimize() {
  return impl_->optimize();
}

const std::vector<Values>& BatchedDenseLevenbergMarquardtOptimizer::values()
    const {
  return impl_->currentValues;
}

const std::vector<BatchedDenseLevenbergMarquardtItemResult>&
BatchedDenseLevenbergMarquardtOptimizer::results() const {
  return impl_->itemResults;
}

const LinearSolveStats& BatchedDenseLevenbergMarquardtOptimizer::
    linearSolveStats() const {
  return impl_->stats;
}

const BatchedDenseLevenbergMarquardtHostTimings&
BatchedDenseLevenbergMarquardtOptimizer::hostTimings() const {
  return impl_->hostTimings;
}

}  // namespace gtsam::cuda

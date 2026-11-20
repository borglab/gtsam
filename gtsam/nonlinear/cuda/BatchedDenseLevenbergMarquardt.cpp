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
  LevenbergMarquardtParams params;
  BatchedDenseCholeskySolver solver;
  // Persistent across every attempt round and outer iteration so the inner
  // loop never cudaMalloc/cudaFree on the steady-state path: an earlier
  // version allocated a fresh DeviceArray per round, and that repeated
  // allocation -- not the actual cuSOLVER compute -- turned out to be the
  // dominant real cost at DFG-TAMP's problem size. resize() inside upload()
  // is a no-op once the size matches, which it always does here since
  // batchSize/dimension are fixed for the optimizer's lifetime.
  DeviceArray<double> deviceH;
  DeviceArray<double> deviceRhs;
  LinearSolveStats stats;
  BatchedDenseLevenbergMarquardtHostTimings hostTimings;
  // Host-side counterparts to the same persistent-buffer fix: a fresh
  // ~batchSize*dimension^2 std::vector<double> allocated per attempt round
  // (134 rounds observed in one real training step) is itself a real,
  // avoidable cost, separate from and on top of the H_base recomputation
  // this class also used to redo on every retry.
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
  // Per-item rejection multiplier for gtsam::internal::increase/decrease
  // LevenbergMarquardtLambda -- the same state LevenbergMarquardtOptimizer's
  // own internal loop keeps (there called currentFactor), just one per item
  // instead of one for the whole (single) problem.
  std::vector<double> currentFactor;
  std::vector<bool> terminated;  // converged or hit lambdaUpperBound
  std::vector<BatchedDenseLevenbergMarquardtItemResult> itemResults;
  bool optimized = false;

  Impl(const std::vector<NonlinearFactorGraph>& inputGraphs,
       const std::vector<Values>& initialValues,
       const LevenbergMarquardtParams& inputParams,
       const BatchedDenseSolverOptions& deviceOptions)
      : graphs(inputGraphs),
        currentValues(initialValues),
        params(inputParams),
        solver(deviceOptions) {
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
    if (optimized) return currentValues;
    if (params.maxIterations == 0 || allTerminated()) {
      finish();
      return currentValues;
    }

    const int maxOuterIterations = static_cast<int>(params.maxIterations);
    for (int outer = 0; outer < maxOuterIterations && !allTerminated();
        ++outer) {
      // Linearize once per outer iteration (the same convention
      // SparseLevenbergMarquardtOptimizer uses: lambda attempts within one
      // outer iteration retry against this one linearization). H_base/
      // rhsBase (the undamped A^T A / A^T b) are also formed only once here,
      // not inside the attempt-round loop below: within one outer iteration,
      // every lambda-retry attempt reuses the SAME linearization, so only
      // the cheap O(dimension) diagonal damping term actually needs to
      // change per attempt, not the O(dimension^2 * rows) matrix product
      // that built H_base. Recomputing that product on every retry (as an
      // earlier version of this loop did) turned out to be the dominant
      // real cost of this optimizer -- orders of magnitude more host time
      // than the batched GPU solve itself.
      {
        const auto linearizeStart = Clock::now();
        // graphs[i].linearize() calls every factor's own linearize()/
        // evaluateError, which for a CustomFactor means calling back into
        // an arbitrary, possibly Python, callback. Running that from
        // multiple OpenMP worker threads is only safe if the GIL has
        // already been released by the caller -- pybind11's own
        // std::function trampoline then lets each worker thread safely
        // (if serially, since Python itself still only runs one thread at
        // a time) re-acquire it just for the callback. None of GTSAM's
        // wrap-generated .def() bindings release the GIL by default, so
        // this specific method is hand-bound instead, in
        // python/gtsam/specializations/cuda_nonlinear.h, with
        // py::call_guard<py::gil_scoped_release>() -- see that file for
        // why, and for the deadlock this is a fix for (reproduced directly
        // with a Python CustomFactor before that binding existed). A pure
        // C++ caller (this project's own benchmark and unit tests, for
        // example) never goes through that binding at all, so this loop is
        // unconditionally safe there regardless.
#pragma omp parallel for num_threads(batchSize) schedule(static)
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

  /// Builds and solves one batched damped system across every item that is
  /// neither terminated nor has already accepted a step this outer
  /// iteration; items outside that set get a harmless identity placeholder
  /// so the batch stays uniformly sized at `batchSize`. Reuses this outer
  /// iteration's cached hBase/rhsBase (see optimize()): only the diagonal
  /// damping term is rebuilt per attempt.
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
      for (int d = 0; d < dimension; ++d) {
        hostH[hOffset + static_cast<size_t>(d) *
                            static_cast<size_t>(dimension) +
              static_cast<size_t>(d)] += lambda[i];
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

      const VectorValues deltaValues =
          buildVectorValues(delta, keyInfos[i]);
      const Values trialValues =
          currentValues[i].retract(deltaValues);
      const double trialError = graphs[i].error(trialValues);
      if (!std::isfinite(trialError)) {
        throw std::runtime_error(
            "BatchedDenseLevenbergMarquardtOptimizer produced a non-finite "
            "trial error for item " +
            std::to_string(item));
      }

      // GTSAM's real trust-region model-fidelity rule (same policy
      // LevenbergMarquardtOptimizer/SparseLevenbergMarquardtOptimizer use),
      // not a simplified monotonic-decrease rule: predicted reduction for
      // (H+lambda*I)delta=rhsBase is linearizedChange = 0.5*delta . (rhsBase
      // + lambda*delta) (standard LM/Nielsen derivation: since
      // H*delta = rhsBase - lambda*delta, cost reduction rhsBase.delta -
      // 0.5*delta.H.delta simplifies to this). Accepting only when the
      // model actually predicted the observed improvement well (not just
      // "any improvement") rejects far fewer/more appropriately than the
      // earlier rule, directly cutting the retry-round count that was
      // inflating both GPU dispatch and host marshaling cost.
      const double linearizedChange =
          0.5 * delta.dot(rhsBase[i] + lambda[i] * delta);
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
        const BatchedDenseSolverOptions& deviceOptions)
    : impl_(std::make_unique<Impl>(graphs, initialValues, params,
                                  deviceOptions)) {}

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

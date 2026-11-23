/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    BatchedDenseLevenbergMarquardt.h
 * @brief   Levenberg-Marquardt over B independent, equal-dimension nonlinear
 *          graphs, batching only the dense linear solve on one GPU
 * @author  Shaji
 * @date    Nov 20, 2026
 */

#pragma once

#include <gtsam/dllexport.h>
#include <gtsam/linear/KeyInfo.h>
#include <gtsam/linear/cuda/internal/BatchedDenseCholeskySolver.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/cuda/BatchedFactorGroupHook.h>

#include <cstddef>
#include <memory>
#include <vector>

namespace gtsam::cuda {

/// Host-side timing breakdown for one optimize() call, separate from
/// linearSolveStats()'s pure GPU time.
struct BatchedDenseLevenbergMarquardtHostTimings {
  /// linearize() + forming H_base/rhs_base, once per outer iteration.
  double linearizeAndFormSeconds = 0.0;
  /// retract() + graph.error(), per participating item per attempt round.
  double retractAndErrorSeconds = 0.0;
  /// Copying H_base/rhs_base plus damping into host buffers, per round.
  double marshalSeconds = 0.0;
  /// Upload + factorize/solve + sync + download wall-clock, per round.
  double gpuRoundTripSeconds = 0.0;
};

/// Per-item outcome after a BatchedDenseLevenbergMarquardtOptimizer run.
struct BatchedDenseLevenbergMarquardtItemResult {
  double initialError = 0.0;
  double finalError = 0.0;
  double finalLambda = 0.0;
  size_t acceptedIterations = 0;
  size_t lambdaAttempts = 0;
  bool converged = false;
};

/**
 * Batch Levenberg-Marquardt for B independent, equal-dimension nonlinear
 * graphs, batching only the dense Cholesky factorize+solve over all B
 * damped normal equations at once on one GPU. Per-item lambda/acceptance/
 * convergence are fully independent, so trajectories match
 * LevenbergMarquardtOptimizer's outcome. Dense, not sparse, since
 * DFG-TAMP's window graphs (~256 columns) are small enough for this to
 * substitute for a sparse solve. optimize() may be called once.
 */
class GTSAM_EXPORT BatchedDenseLevenbergMarquardtOptimizer {
 public:
  /// Linearizes every graph once to fix each item's key ordering. Throws
  /// on a batch-size mismatch or differing scalar dimensions.
  /// factorGroupHooks, if given, batch each item's net evaluation.
  BatchedDenseLevenbergMarquardtOptimizer(
      const std::vector<NonlinearFactorGraph>& graphs,
      const std::vector<Values>& initialValues,
      const LevenbergMarquardtParams& params = LevenbergMarquardtParams(),
      const BatchedDenseSolverOptions& deviceOptions = {},
      const std::vector<BatchedFactorGroupHook>& factorGroupHooks = {});
  ~BatchedDenseLevenbergMarquardtOptimizer();

  BatchedDenseLevenbergMarquardtOptimizer(
      const BatchedDenseLevenbergMarquardtOptimizer&) = delete;
  BatchedDenseLevenbergMarquardtOptimizer& operator=(
      const BatchedDenseLevenbergMarquardtOptimizer&) = delete;

  /// Runs every item's LM loop up to params.maxIterations outer iterations
  /// (converged/capped items become cheap no-ops). Returns final values.
  const std::vector<Values>& optimize();

  /// Returns the current (post-optimize(), if called) per-graph values.
  const std::vector<Values>& values() const;
  /// Returns per-item diagnostics after optimize() has run.
  const std::vector<BatchedDenseLevenbergMarquardtItemResult>& results() const;
  /// Returns cumulative batched-solver call counts and GPU seconds.
  const LinearSolveStats& linearSolveStats() const;
  /// Returns the host-side timing breakdown accumulated during optimize().
  const BatchedDenseLevenbergMarquardtHostTimings& hostTimings() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace gtsam::cuda

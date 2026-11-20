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
 * @author  FIXME: fill in your name
 * @date    Nov 20, 2026
 */

#pragma once

#include <gtsam/dllexport.h>
#include <gtsam/linear/KeyInfo.h>
#include <gtsam/linear/cuda/internal/BatchedDenseCholeskySolver.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <cstddef>
#include <memory>
#include <vector>

namespace gtsam::cuda {

/**
 * Host-side timing breakdown for one optimize() call, separate from the
 * actual GPU solver time (see linearSolveStats()) -- isolates per-item
 * bookkeeping cost (linearization, trial-point retract/error) from the
 * batched linear-algebra step, to find where host time actually goes.
 */
struct BatchedDenseLevenbergMarquardtHostTimings {
  /// Outer-loop: linearize() + forming H_base = A^T A / rhs_base = A^T b,
  /// once per outer iteration per active item (not repeated across retries).
  double linearizeAndFormSeconds = 0.0;
  /// Per attempt round: retract() + graph.error(trialValues) for every
  /// participating item -- scales with the number of lambda-retry rounds.
  double retractAndErrorSeconds = 0.0;
  /// Per attempt round: copying H_base/rhs_base plus damping into the flat
  /// host buffers uploaded to the device.
  double marshalSeconds = 0.0;
  /// Per attempt round: upload + factorize() + solve() + sync + download
  /// call overhead, wall-clock, as seen by this class -- includes the pure
  /// cuSOLVER kernel time counted separately in linearSolveStats(), plus
  /// whatever upload/dispatch/sync/download overhead surrounds it.
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
 * Batch Levenberg-Marquardt optimizer that solves B independent, identically
 * scalar-dimensioned nonlinear graphs together, batching only the expensive
 * linear-algebra step -- dense Cholesky factorize and solve over all B
 * damped normal equations at once -- on one GPU. Per-item lambda tracking,
 * acceptance, and convergence are fully independent, so each graph's
 * trust-region schedule proceeds exactly as it would running alone; batching
 * changes only where the arithmetic runs, not the optimization trajectory
 * (acceptance here uses a classic monotonic-decrease Levenberg rule, not
 * GTSAM's linearized/nonlinear model-fidelity ratio, so trajectories match
 * LevenbergMarquardtOptimizer in outcome, not attempt-for-attempt).
 *
 * This is the dense-batched counterpart to SparseLevenbergMarquardtOptimizer.
 * That class amortizes CUDA setup over one graph's own outer iterations,
 * which loses to CPU at DFG-TAMP's problem size and calling pattern -- one
 * fresh ~256-column optimizer per (batch_item, timestep), never reused
 * (CLAUDE.md's GPU-accelerated GTSAM TODO measured this at ~50% slower than
 * CPU). This class instead amortizes setup over DFG-TAMP's actual
 * B=8-graphs-per-training-step batch (JointWindowTrainer.training_step),
 * which is the real lever: batching many independent small problems is how
 * every comparable GPU win in the batched-optimization literature works.
 * See the scratchpad prototype this implements (recorded in
 * i-want-you-to-dapper-torvalds.md) for the validated speedup this is based
 * on: 2.1x forward over CPU at this exact B=8, n=256, 40-iteration shape.
 *
 * Dense, not sparse, for the same reason as BatchedDenseCholeskySolver:
 * DFG-TAMP's window graphs are small (~256 columns), so dense batched
 * Cholesky is a legitimate, already-linked substitute for a batched sparse
 * solve (cuDSS has no batched multi-system API and is not installed on this
 * machine either way).
 *
 * The graph topology, each graph's key ordering, and the total batch size
 * are fixed at construction. optimize() may be called once.
 */
class GTSAM_EXPORT BatchedDenseLevenbergMarquardtOptimizer {
 public:
  /**
   * Linearizes every graph once to fix each item's key ordering (reused for
   * every later outer iteration, so retract/jacobian stay consistent even if
   * recomputing an ordering from scratch each time would differ).
   *
   * Throws std::invalid_argument if graphs and initialValues differ in size,
   * the batch is empty, or any two graphs linearize to a different total
   * scalar dimension.
   */
  BatchedDenseLevenbergMarquardtOptimizer(
      const std::vector<NonlinearFactorGraph>& graphs,
      const std::vector<Values>& initialValues,
      const LevenbergMarquardtParams& params = LevenbergMarquardtParams(),
      const BatchedDenseSolverOptions& deviceOptions = {});
  ~BatchedDenseLevenbergMarquardtOptimizer();

  BatchedDenseLevenbergMarquardtOptimizer(
      const BatchedDenseLevenbergMarquardtOptimizer&) = delete;
  BatchedDenseLevenbergMarquardtOptimizer& operator=(
      const BatchedDenseLevenbergMarquardtOptimizer&) = delete;

  /**
   * Runs every item's LM loop for up to params.maxIterations accepted outer
   * iterations (an item that converges or hits params.lambdaUpperBound stops
   * early and is carried as a cheap no-op for the rest of the batch), each
   * outer iteration batching one dense factorize+solve per lambda attempt
   * across every still-active item. Returns the final per-graph values.
   */
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

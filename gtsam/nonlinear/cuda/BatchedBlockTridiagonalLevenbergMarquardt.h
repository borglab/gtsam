/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    BatchedBlockTridiagonalLevenbergMarquardt.h
 * @brief   Levenberg-Marquardt over B independent, equal-shape nonlinear
 *          chain graphs, batching a block-tridiagonal-exact GPU solve
 * @author  Shaji
 * @date    Nov 20, 2026
 */

#pragma once

#include <gtsam/dllexport.h>
#include <gtsam/linear/KeyInfo.h>
#include <gtsam/linear/cuda/internal/BatchedBlockTridiagonalCholeskySolver.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/cuda/BatchedFactorGroupHook.h>

#include <cstddef>
#include <memory>
#include <vector>

namespace gtsam::cuda {

/// Per-item outcome after a BatchedBlockTridiagonalLevenbergMarquardtOptimizer
/// run.
struct BatchedBlockTridiagonalLevenbergMarquardtItemResult {
  double initialError = 0.0;
  double finalError = 0.0;
  double finalLambda = 0.0;
  size_t acceptedIterations = 0;
  size_t lambdaAttempts = 0;
  bool converged = false;
};

/**
 * Batch Levenberg-Marquardt for B independent *chain* nonlinear graphs
 * (scalar unknowns partition into numBlocks blockDim-wide groups, each
 * factor touching only one or two adjacent groups -- DFG-TAMP's
 * build_window_graph shape). Assembles and factorizes only the block-
 * tridiagonal pieces via BatchedBlockTridiagonalCholeskySolver, exploiting
 * the chain sparsity instead of forming a dense n x n Hessian. optimize()
 * may be called once.
 */
class GTSAM_EXPORT BatchedBlockTridiagonalLevenbergMarquardtOptimizer {
 public:
  /// numBlocks/blockDim describe the shared chain structure (column
  /// t*blockDim.. must be chain step t's natural-key-order columns).
  /// Throws on a batch-size mismatch or a graph of the wrong dimension.
  /// factorGroupHooks, if given, are primed with every active item's
  /// current Values (and computed as one cross-item batch) before each
  /// outer iteration's per-item linearize() call -- see
  /// BatchedFactorGroupHook.h. Each item's graph must still contain the
  /// same Python-backed factors as always; the hooks only change when
  /// those factors' underlying net evaluation actually runs.
  BatchedBlockTridiagonalLevenbergMarquardtOptimizer(
      const std::vector<NonlinearFactorGraph>& graphs,
      const std::vector<Values>& initialValues, int numBlocks, int blockDim,
      const LevenbergMarquardtParams& params = LevenbergMarquardtParams(),
      const BatchedSolverOptions& deviceOptions = {},
      const std::vector<BatchedFactorGroupHook>& factorGroupHooks = {});
  ~BatchedBlockTridiagonalLevenbergMarquardtOptimizer();

  BatchedBlockTridiagonalLevenbergMarquardtOptimizer(
      const BatchedBlockTridiagonalLevenbergMarquardtOptimizer&) = delete;
  BatchedBlockTridiagonalLevenbergMarquardtOptimizer& operator=(
      const BatchedBlockTridiagonalLevenbergMarquardtOptimizer&) = delete;

  /**
   * Runs every item's LM loop for up to params.maxIterations accepted outer
   * iterations, each outer iteration batching one block-tridiagonal
   * factorize+solve per lambda attempt across every still-active item.
   * Returns the final per-graph values.
   */
  const std::vector<Values>& optimize();

  /**
   * Runs numSteps plain (undamped) Gauss-Newton steps across every item,
   * batched on the same block-tridiagonal GPU path optimize() uses --
   * reuses the same factorGroupHooks, assembly plan, Cholesky solver, and
   * device buffers, but with no lambda damping, no per-attempt retry, and
   * no accept/reject test: every step is taken unconditionally for every
   * item. Intended as a cheap refinement pass after optimize() converges
   * (matching callers that previously ran extra plain Gauss-Newton steps
   * per item on CPU via graph.linearize(values).optimize()), not as a
   * general-purpose solver -- it does not check or update convergence,
   * lambda, or terminated state. May be called whether or not optimize()
   * has been called, and may be called more than once.
   */
  const std::vector<Values>& polish(int numSteps);

  /// Returns the current (post-optimize()/polish(), if called) per-graph values.
  const std::vector<Values>& values() const;
  /// Returns per-item diagnostics after optimize() has run.
  const std::vector<BatchedBlockTridiagonalLevenbergMarquardtItemResult>&
  results() const;
  /// Returns cumulative batched-solver call counts and GPU seconds.
  const LinearSolveStats& linearSolveStats() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace gtsam::cuda

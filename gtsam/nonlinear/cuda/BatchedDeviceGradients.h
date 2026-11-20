/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    BatchedDeviceGradients.h
 * @brief   GPU-resident differentiable backward for a batch of independent,
 *          converged nonlinear graphs of identical scalar dimension
 * @author  Shaji
 * @date    Nov 20, 2026
 */

#pragma once

#include <gtsam/dllexport.h>
#include <gtsam/linear/KeyInfo.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/linear/cuda/internal/BatchedDenseCholeskySolver.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <memory>
#include <vector>

namespace gtsam::cuda {

/**
 * GPU-resident differentiable backward for a batch of independent, converged
 * nonlinear graphs, mirroring gtsam::Gradients::adjoint but solving every
 * graph's cotangent query in one batched GPU call instead of one CPU
 * Bayes-tree back-substitution per graph.
 *
 * gtsam::Gradients::adjoint solves H * lambda = stateCotangents, with the
 * undamped H = J^T J built once at construction (Gradients.h's documented
 * Gauss-Newton model) and reused for every subsequent query -- confirmed by
 * direct comparison against this exact solve in the scratchpad prototype
 * this class is built from (see i-want-you-to-dapper-torvalds.md:
 * gtsam.Gradients.adjoint(c) matched np.linalg.solve(H, c) to ~1e-15). This
 * class performs the same undamped solve for B graphs at once: each graph's
 * whitened Jacobian comes from the ordinary, public
 * NonlinearFactorGraph::linearize() + GaussianFactorGraph::jacobian() path
 * (the same whitened A that LevenbergMarquardtOptimizer's own normal
 * equations use), forms H_b = A_b^T A_b, stacks the batch, and factorizes it
 * once via BatchedDenseCholeskySolver. Every later adjoint() call reuses that
 * one factorization, so the N per-tag parameterGradient-style queries a
 * training step issues become N cheap batched solves instead of N*B small
 * CPU solves.
 *
 * All graphs in the batch must linearize to the same total scalar dimension;
 * DFG-TAMP's identical-topology window graphs satisfy this, and it is
 * checked at construction. Dense, not sparse, for the same reason as
 * BatchedDenseCholeskySolver: DFG-TAMP's graphs are small (~256 columns), so
 * a dense batched factorization is a legitimate, already-available
 * substitute for a batched sparse direct solve.
 */
class GTSAM_EXPORT BatchedDeviceGradients {
 public:
  /**
   * Linearizes each (graph, solution) pair without damping, forms each
   * item's dense H = A^T A, and factorizes the whole batch once.
   *
   * Throws std::invalid_argument if graphs and solutions differ in size, the
   * batch is empty, or any two graphs linearize to a different total scalar
   * dimension.
   */
  BatchedDeviceGradients(const std::vector<NonlinearFactorGraph>& graphs,
                         const std::vector<Values>& solutions,
                         const BatchedDenseSolverOptions& options = {});
  ~BatchedDeviceGradients();

  BatchedDeviceGradients(const BatchedDeviceGradients&) = delete;
  BatchedDeviceGradients& operator=(const BatchedDeviceGradients&) = delete;

  /**
   * Returns the intermediate vector lambda_b satisfying H_b * lambda_b =
   * stateCotangents[b] for every graph in the batch, in one batched GPU
   * solve reusing the cached factorization. stateCotangents must have one
   * entry per graph in the batch.
   */
  std::vector<VectorValues> adjoint(
      const std::vector<VectorValues>& stateCotangents) const;

  /// Number of graphs in the batch.
  int batchSize() const;
  /// Total scalar dimension shared by every graph in the batch.
  int dimension() const;
  /// Key layout used for graph index i's flattened vectors.
  const KeyInfo& keyInfo(size_t index) const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace gtsam::cuda

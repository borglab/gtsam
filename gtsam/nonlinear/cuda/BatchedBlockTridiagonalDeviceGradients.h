/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    BatchedBlockTridiagonalDeviceGradients.h
 * @brief   GPU-resident differentiable backward for a batch of independent,
 *          converged *chain* nonlinear graphs of identical shape
 * @author  Shaji
 * @date    Nov 20, 2026
 */

#pragma once

#include <gtsam/dllexport.h>
#include <gtsam/linear/KeyInfo.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/linear/cuda/internal/BatchedBlockTridiagonalCholeskySolver.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/ParameterTag.h>
#include <gtsam/nonlinear/Values.h>

#include <memory>
#include <optional>
#include <vector>

namespace gtsam::cuda {

/**
 * Block-tridiagonal-exact counterpart to BatchedDeviceGradients, for the
 * same chain-graph shape BatchedBlockTridiagonalLevenbergMarquardtOptimizer
 * targets. adjoint()/pullback()/parameterGradient() match
 * BatchedDeviceGradients exactly; only the one-time H factorization at
 * construction uses the block solver instead of a dense n x n one.
 */
class GTSAM_EXPORT BatchedBlockTridiagonalDeviceGradients {
 public:
  /// Linearizes each (graph, solution) pair without damping and factorizes
  /// the whole batch once. numBlocks/blockDim describe the shared chain
  /// structure. Throws on a batch-size mismatch or wrong-dimension graph.
  BatchedBlockTridiagonalDeviceGradients(
      const std::vector<NonlinearFactorGraph>& graphs,
      const std::vector<Values>& solutions, int numBlocks, int blockDim,
      const BatchedDenseSolverOptions& options = {});
  ~BatchedBlockTridiagonalDeviceGradients();

  BatchedBlockTridiagonalDeviceGradients(
      const BatchedBlockTridiagonalDeviceGradients&) = delete;
  BatchedBlockTridiagonalDeviceGradients& operator=(
      const BatchedBlockTridiagonalDeviceGradients&) = delete;

  /// Same semantics as BatchedDeviceGradients::adjoint.
  std::vector<VectorValues> adjoint(
      const std::vector<VectorValues>& stateCotangents) const;

  /// Same semantics as BatchedDeviceGradients::pullback.
  std::vector<std::vector<Vector>> pullback(
      const std::vector<VectorValues>& stateCotangents) const;

  /// Same semantics as BatchedDeviceGradients::parameterGradient.
  std::vector<std::optional<Vector>> parameterGradient(
      const std::vector<VectorValues>& stateCotangents,
      const ParameterTag& parameter) const;

  /// Number of graphs in the batch.
  int batchSize() const;
  /// Total scalar dimension shared by every graph in the batch (numBlocks *
  /// blockDim).
  int dimension() const;
  /// Key layout used for graph index i's flattened vectors.
  const KeyInfo& keyInfo(size_t index) const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace gtsam::cuda

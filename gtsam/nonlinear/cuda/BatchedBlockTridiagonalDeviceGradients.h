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
#include <gtsam/linear/NoiseModel.h>
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
 * GPU-resident differentiable backward for a batch of independent,
 * converged *chain* nonlinear graphs, mirroring gtsam::Gradients but
 * solving every graph's cotangent query in one batched GPU call via the
 * block-tridiagonal Cholesky solver, for the same chain-graph shape
 * BatchedBlockTridiagonalLevenbergMarquardtOptimizer targets. Every active
 * factor must be a NoiseModelFactor with a dimension-matched, unconstrained
 * noiseModel::Gaussian; robust and constrained noise models are rejected.
 */
class GTSAM_EXPORT BatchedBlockTridiagonalDeviceGradients {
 public:
  /// Linearizes each (graph, solution) pair without damping and factorizes
  /// the whole batch once. numBlocks/blockDim describe the shared chain
  /// structure; throws on a batch/dimension mismatch or unsupported input.
  BatchedBlockTridiagonalDeviceGradients(
      const std::vector<NonlinearFactorGraph>& graphs,
      const std::vector<Values>& solutions, int numBlocks, int blockDim,
      const BatchedSolverOptions& options = {});
  ~BatchedBlockTridiagonalDeviceGradients();

  BatchedBlockTridiagonalDeviceGradients(
      const BatchedBlockTridiagonalDeviceGradients&) = delete;
  BatchedBlockTridiagonalDeviceGradients& operator=(
      const BatchedBlockTridiagonalDeviceGradients&) = delete;

  /// Returns lambda_b satisfying H_b*lambda_b = stateCotangents[b] per
  /// graph. Omitted known keys mean zero; an unknown key or a present key
  /// with the wrong (or nonfinite) value throws std::invalid_argument.
  std::vector<VectorValues> adjoint(
      const std::vector<VectorValues>& stateCotangents) const;

  /// Returns, per graph, one whitened factor-residual cotangent per
  /// factor slot, mirroring gtsam::Gradients::pullback: -(A_b * adjoint_b).
  std::vector<std::vector<Vector>> pullback(
      const std::vector<VectorValues>& stateCotangents) const;

  /// Returns, per graph, the accumulated gradient for `parameter`,
  /// mirroring gtsam::Gradients::parameterGradient; std::nullopt if no
  /// factor depends on it.
  std::vector<std::optional<Vector>> parameterGradient(
      const std::vector<VectorValues>& stateCotangents,
      const ParameterTag& parameter) const;

  /// Returns, per graph, -H_b^{-1} * A_b^T * factorResidualDirections[b].
  /// Supply one finite vector per factor slot, matching that factor's row
  /// dimension; null/inactive slots take an empty vector.
  std::vector<VectorValues> pushforward(
      const std::vector<std::vector<Vector>>& factorResidualDirections) const;

  /// Returns, per graph, the gradient w.r.t. model's sigmas (Diagonal or
  /// Isotropic only, matched by object identity); std::nullopt if no
  /// active factor holds model.
  std::vector<std::optional<Vector>> noiseGradient(
      const std::vector<VectorValues>& stateCotangents,
      const SharedNoiseModel model) const;

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

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
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/linear/cuda/internal/BatchedDenseCholeskySolver.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/ParameterTag.h>
#include <gtsam/nonlinear/Values.h>

#include <memory>
#include <optional>
#include <vector>

namespace gtsam::cuda {

/**
 * GPU-resident differentiable backward for a batch of independent,
 * converged nonlinear graphs, mirroring gtsam::Gradients::adjoint but
 * solving every graph's cotangent query in one batched GPU call. Forms
 * the undamped H_b = A_b^T A_b per graph and factorizes the batch once
 * (BatchedDenseCholeskySolver), so N per-tag queries become N cheap
 * batched solves instead of N*B small CPU solves. Dense, not sparse, for
 * the same reason as BatchedDenseCholeskySolver.
 *
 * Matches gtsam::Gradients' supported-noise convention exactly: every
 * active factor must be a NoiseModelFactor whose noise model is null or a
 * dimension-matched, unconstrained noiseModel::Gaussian with finite,
 * positive sigmas (or a finite positive-definite covariance for a full
 * Gaussian); robust losses and hard constraints are rejected at
 * construction, not silently folded into A.
 */
class GTSAM_EXPORT BatchedDeviceGradients {
 public:
  /// Linearizes each (graph, solution) pair without damping and
  /// factorizes the batch once. Throws on a batch-size or dimension
  /// mismatch, a factor without a NoiseModelFactor residual Jacobian, or an
  /// unsupported noise model (robust, constrained, or otherwise not a
  /// dimension-matched unconstrained Gaussian).
  BatchedDeviceGradients(const std::vector<NonlinearFactorGraph>& graphs,
                         const std::vector<Values>& solutions,
                         const BatchedDenseSolverOptions& options = {});
  ~BatchedDeviceGradients();

  BatchedDeviceGradients(const BatchedDeviceGradients&) = delete;
  BatchedDeviceGradients& operator=(const BatchedDeviceGradients&) = delete;

  /// Returns lambda_b satisfying H_b*lambda_b = stateCotangents[b] per
  /// graph, in one batched GPU solve reusing the cached factorization.
  /// Each cotangent may be partial: an omitted known key means zero, as in
  /// gtsam::Gradients::adjoint. A key absent from graph b's keys, or a
  /// present key with the wrong (or nonfinite) value, throws
  /// std::invalid_argument.
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

  /// Returns, per graph, -H_b^{-1} * A_b^T * factorResidualDirections[b],
  /// mirroring gtsam::Gradients::pushforward exactly: supply one finite
  /// vector per factor slot (graph b's factorResidualDirections[b][f]),
  /// with that factor's row dimension; null/inactive slots require empty
  /// vectors. Reuses adjoint()'s batched solve, same as the CPU class does.
  std::vector<VectorValues> pushforward(
      const std::vector<std::vector<Vector>>& factorResidualDirections) const;

  /// Returns, per graph, the gradient through the solved states with
  /// respect to model's sigmas, mirroring gtsam::Gradients::noiseGradient
  /// exactly (including its object-identity factor matching and Diagonal/
  /// Isotropic-only support); std::nullopt if no active factor holds
  /// model.
  std::vector<std::optional<Vector>> noiseGradient(
      const std::vector<VectorValues>& stateCotangents,
      const SharedNoiseModel model) const;

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

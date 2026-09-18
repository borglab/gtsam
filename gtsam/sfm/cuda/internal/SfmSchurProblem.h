/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SfmSchurProblem.h
 * @brief   Device SFM normal equations and point elimination by Schur complement
 * @author  Ruogu Li
 * @date    Aug 18, 2026
 */

#pragma once

#include <gtsam/base/cuda/DeviceArray.h>
#include <gtsam/dllexport.h>
#include <gtsam/linear/cuda/DeviceSparseSpdSystem.h>
#include <gtsam/linear/cuda/LinearSystem.h>
#include <gtsam/nonlinear/cuda/internal/DeviceValues.h>
#include <gtsam/sfm/cuda/internal/SfmProjectionBatch.h>
#include <gtsam/sfm/cuda/internal/SfmProjectionLinearization.h>

#include <cuda_runtime_api.h>

#include <cstddef>
#include <memory>

namespace gtsam::cuda {

class SfmReducedCsrPlan;

/**
 * Non-owning matrix-free reduced-camera Schur system.
 *
 * Pointers are owned by the producing SfmSchurProblem and remain valid only
 * until the next initialize, linearize, or prepare operation, move, or
 * destruction. Use the producing stream or establish an explicit dependency.
 */
struct SfmImplicitSchurView {
  /// Matrix-free damped reduced-camera operator.
  const LinearOperator* linearOperator = nullptr;
  /// Camera block-Jacobi preconditioner.
  const Preconditioner* preconditioner = nullptr;
  /// Device pointer to the condensed camera right-hand side.
  const double* rhs = nullptr;
  /// Scalar dimension of the reduced camera system.
  int dimension = 0;
};

/// Undamped normal-equation blocks retained for one SFM linearization.
struct SfmSchurBlocks {
  /// Per-camera U = Jc'Jc blocks, row-major 9-by-9.
  DeviceArray<double> cameraNormalBlocks;
  /// Per-camera gc = -Jc'r vectors.
  DeviceArray<double> cameraGradient;
  /// Per-point V = Jp'Jp blocks, row-major 3-by-3.
  DeviceArray<double> pointNormalBlocks;
  /// Per-point gp = -Jp'r vectors.
  DeviceArray<double> pointGradient;
  /// Per-observation W = Jc'Jp blocks, row-major 9-by-3.
  DeviceArray<double> cameraPointBlocks;
};

/**
 * Persistent SFM-specific producer for reduced camera Schur systems.
 *
 * Projection residuals and Jacobians are linearized once per LM outer
 * iteration. A backend may then prepare a fresh lambda-dependent system for
 * every damping attempt and use the same state to recover point increments.
 */
class GTSAM_EXPORT SfmSchurProblem {
 public:
  /// Constructs an empty producer; call initialize before linearize.
  SfmSchurProblem();
  ~SfmSchurProblem();

  SfmSchurProblem(const SfmSchurProblem&) = delete;
  SfmSchurProblem& operator=(const SfmSchurProblem&) = delete;
  SfmSchurProblem(SfmSchurProblem&&) noexcept;
  SfmSchurProblem& operator=(SfmSchurProblem&&) noexcept;

  /**
   * Initializes dimensions and borrows batch for subsequent operations.
   * batch must outlive this object or the next initialization.
   */
  void initialize(const SfmProjectionBatch& batch, int numCameras);
  /**
   * Linearizes values and builds undamped Schur blocks asynchronously.
   * values need only remain valid until the queued work on stream completes.
   */
  void linearize(const DeviceValues& values, cudaStream_t stream = nullptr);

  /// Returns a borrowed dense reduced system with scalar damping.
  DenseSpdSystemView prepareDense(double lambda,
                                  cudaStream_t stream = nullptr);
  /**
   * Returns a borrowed dense reduced system with full-system diagonal damping.
   */
  DenseSpdSystemView prepareDense(
      double lambda, const DeviceArray<double>& dampingDiagonal,
      cudaStream_t stream = nullptr);

  /**
   * Assembles scalar-damped upper CSR according to plan and returns
   * producer-owned storage.
   */
  DeviceSparseSpdSystem& prepareSparse(
      double lambda, const SfmReducedCsrPlan& plan,
      cudaStream_t stream = nullptr);
  /// Assembles diagonally damped upper CSR in producer-owned storage.
  DeviceSparseSpdSystem& prepareSparse(
      double lambda, const DeviceArray<double>& dampingDiagonal,
      const SfmReducedCsrPlan& plan, cudaStream_t stream = nullptr);

  /// Returns a borrowed matrix-free reduced system with scalar damping.
  SfmImplicitSchurView prepareImplicit(
      double lambda, cudaStream_t stream = nullptr);
  /// Returns a borrowed matrix-free system with full-system diagonal damping.
  SfmImplicitSchurView prepareImplicit(
      double lambda, const DeviceArray<double>& dampingDiagonal,
      cudaStream_t stream = nullptr);

  /**
   * Recovers point increments from a scalar-damped camera solution.
   * fullDelta is resized to totalDimension(); all device work is queued on
   * stream.
   */
  void recoverPoints(double lambda,
                     const DeviceArray<double>& cameraDelta,
                     DeviceArray<double>* fullDelta,
                     cudaStream_t stream = nullptr);
  /// Recovers point increments using the supplied damping diagonal.
  void recoverPoints(double lambda,
                     const DeviceArray<double>& dampingDiagonal,
                     const DeviceArray<double>& cameraDelta,
                     DeviceArray<double>* fullDelta,
                     cudaStream_t stream = nullptr);

  /// Returns the scalar dimension of the reduced camera system.
  int cameraDimension() const;
  /// Returns the scalar dimension of cameras and points together.
  int totalDimension() const;
  /// Returns the number of successful linearizations.
  size_t linearizationCount() const;
  /// Returns the number of undamped Schur block builds.
  size_t blockBuildCount() const;
  /// Returns the number of dense reduced-system assemblies.
  size_t denseAssemblyCount() const;
  /// Borrows the current owned projection linearization.
  const SfmProjectionLinearization& linearization() const;
  /// Borrows the current owned undamped Schur blocks.
  const SfmSchurBlocks& blocks() const;

  /// Borrow the current condensed RHS (overwritten in-place by dense solve).
  const DeviceArray<double>& cameraRhs() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace gtsam::cuda

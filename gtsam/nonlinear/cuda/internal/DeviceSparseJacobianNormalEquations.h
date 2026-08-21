/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    DeviceSparseJacobianNormalEquations.h
 * @brief   Device pipeline from a packed sparse Jacobian to normal equations
 * @author  Ruogu Li
 * @date    Jul 14, 2026
 */

#pragma once

#include <cuda_runtime_api.h>
#include <gtsam/base/Vector.h>
#include <gtsam/dllexport.h>
#include <gtsam/linear/cuda/DeviceSparseSpdSystem.h>
#include <gtsam/nonlinear/cuda/internal/PcgOperatorBuilder.h>
#include <gtsam/nonlinear/cuda/internal/HostSparseJacobian.h>
#include <gtsam/nonlinear/cuda/internal/SparseJacobianPlan.h>

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace gtsam::cuda {

/// Runtime availability of the requested persistent sparse-Jacobian path.
struct DeviceSparseJacobianCapability {
  bool supported = false;
  std::string detail;
};

/// Numerical representation produced for the shared solver session.
enum class DeviceNormalSolverBackend { Cudss, Pcg };

/**
 * Linear-solver selection for the persistent device pipeline. In Pcg mode
 * the normal matrix H is never formed: SpGEMM pattern discovery, stable-H
 * storage, and cuDSS are all skipped, and columnBlockOffsets must hold the
 * variable-block boundaries (numBlocks+1 ascending scalar-column offsets
 * ending at the plan's column count) for the block-Jacobi preconditioner.
 */
struct DeviceNormalSolverOptions {
  DeviceNormalSolverBackend backend = DeviceNormalSolverBackend::Cudss;
  DevicePcgOptions pcg;
  std::vector<int> columnBlockOffsets;
  std::vector<int> scalarPermutation;
};

/// Old and trial linearized-model errors for one LM attempt.
struct LinearizedModelErrors {
  double oldError = 0.0;
  double newError = 0.0;

  double change() const { return oldError - newError; }
};

/// Host-owned result downloaded after evaluating a solved device delta.
struct DeviceSparseJacobianAttemptResult {
  Vector delta;
  LinearizedModelErrors model;
  int pcgIterations = 0;
  bool pcgConverged = true;
};

/**
 * Cumulative device-stage profile for one persistent sparse-Jacobian system.
 *
 * CUDA durations are measured with events on the fixed stream. Pending
 * iteration/attempt event spans are harvested by downloadAttemptResult()
 * after its existing stream synchronization. Solver lifecycle timing belongs
 * to LinearSolverSession. Transfer counters report logical bytes even when
 * timing collection is disabled.
 */
struct DeviceSparseJacobianProfile {
  // One-time setup.
  double initializeWall = 0.0;
  double patternH2d = 0.0;
  double structureSetup = 0.0;
  double setupD2h = 0.0;
  // Per outer linearization.
  double numericH2d = 0.0;
  double transposeUpdate = 0.0;
  double normalJtJ = 0.0;
  double normalJtb = 0.0;
  double diagonalExtraction = 0.0;
  double oldModelError = 0.0;
  double dampingPreparation = 0.0;

  // Per lambda attempt.
  double dampingApplication = 0.0;
  double newModelError = 0.0;
  double attemptD2h = 0.0;
  double attemptHostBuild = 0.0;

  // PCG producer setup only; solve timing belongs to LinearSolverSession.
  double pcgPreconditionerBuild = 0.0;

  size_t patternH2dBytes = 0;
  size_t numericH2dBytes = 0;
  size_t setupD2hBytes = 0;
  size_t attemptD2hBytes = 0;

  size_t totalH2dBytes() const { return patternH2dBytes + numericH2dBytes; }
  size_t totalD2hBytes() const { return setupD2hBytes + attemptD2hBytes; }
};

/**
 * Persistent producer for sparse-Jacobian normal-equation solves.
 *
 * This object owns matrix storage and CUDA descriptors but not the numerical
 * solver lifecycle. A `LinearSolverSession` must analyze and solve the
 * representation exposed by this producer.
 */
class GTSAM_EXPORT DeviceSparseJacobianNormalEquations {
 public:
  DeviceSparseJacobianNormalEquations();
  ~DeviceSparseJacobianNormalEquations();

  DeviceSparseJacobianNormalEquations(
      const DeviceSparseJacobianNormalEquations&) = delete;
  DeviceSparseJacobianNormalEquations& operator=(
      const DeviceSparseJacobianNormalEquations&) = delete;
  DeviceSparseJacobianNormalEquations(
      DeviceSparseJacobianNormalEquations&&) noexcept;
  DeviceSparseJacobianNormalEquations& operator=(
      DeviceSparseJacobianNormalEquations&&) noexcept;

  /// Check availability of the default cuDSS representation.
  static DeviceSparseJacobianCapability preflightCapability();
  /// Check runtime/toolkit availability of a requested representation.
  static DeviceSparseJacobianCapability preflightCapability(
      DeviceNormalSolverBackend backend);

  /**
   * Allocate persistent storage for a fixed sparse plan.
   *
   * The borrowed stream must outlive this object; destruction waits for it
   * before releasing descriptors, workspaces, and device allocations.
   */
  void initialize(const SparseJacobianPlan& plan, cudaStream_t stream = nullptr,
                  bool collectProfile = false,
                  const DeviceNormalSolverOptions& solverOptions = {});
  /**
   * Asynchronously upload refreshed numerical values.
   *
   * The pinned host storage must remain alive and unmodified until the fixed
   * stream reaches the queued copies.
   */
  void uploadNumerics(const HostSparseJacobian& host,
                      cudaStream_t stream = nullptr);
  /// Form the undamped direct-solver system or PCG preconditioner.
  void formUndampedSystem(cudaStream_t stream = nullptr);

  // Every stream-taking operation after initialize() must receive the fixed
  // stream. Consumers of system() must use that stream, or establish ordering
  // with an event, before reading or modifying its storage.
  // Producer operations are asynchronous.
  // downloadAttemptResult() performs the one synchronization needed to return
  // host-owned values.
  void prepareDamping(bool diagonalDamping, double minDiagonal,
                      double maxDiagonal, cudaStream_t stream = nullptr);
  /// Apply one lambda to a materialized direct-solver system.
  void applyExplicitDamping(double lambda, cudaStream_t stream = nullptr);
  /// Prepare the matrix-free operator and preconditioner for one lambda.
  void prepareOperatorSystem(double lambda,
                             cudaStream_t stream = nullptr);
  /// Return the prepared matrix-free operator.
  const LinearOperator& linearOperator() const;
  /// Return the prepared matrix-free preconditioner.
  const Preconditioner& preconditioner() const;
  /// Return the device right-hand side for the current linearization.
  const double* deviceRhs() const;
  /// Evaluate the linearized trial error for the current device delta.
  void evaluateSolvedDelta(cudaStream_t stream = nullptr);
  /// Synchronize and download the delta and linearized-model errors.
  DeviceSparseJacobianAttemptResult downloadAttemptResult(
      cudaStream_t stream = nullptr) const;

  /// Return cumulative producer timings and logical transfer counts.
  const DeviceSparseJacobianProfile& profile() const;

  // True only in cuDSS mode, where the normal matrix H is materialized;
  // system() throws std::logic_error otherwise.
  bool hasNormalMatrix() const;
  /// Return the mutable materialized SPD system; throws in PCG mode.
  DeviceSparseSpdSystem& mutableSystem();
  /// Return the materialized SPD system; throws in PCG mode.
  const DeviceSparseSpdSystem& system() const;
  /// Return storage for the current solution vector.
  DeviceArray<double>& deviceDelta();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace gtsam::cuda

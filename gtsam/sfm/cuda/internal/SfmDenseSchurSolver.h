/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SfmDenseSchurSolver.h
 * @brief   Dense Cholesky solve of the reduced camera Schur system
 * @author  Ruogu Li
 * @date    Aug 18, 2026
 */

#pragma once

#include <gtsam/base/cuda/DeviceArray.h>
#include <gtsam/dllexport.h>
#include <gtsam/linear/cuda/LinearSolver.h>
#include <gtsam/nonlinear/cuda/internal/DeviceValues.h>
#include <gtsam/sfm/cuda/internal/SfmProjectionBatch.h>
#include <gtsam/sfm/cuda/internal/SfmProjectionLinearization.h>

#include <cuda_runtime_api.h>

#include <cstddef>
#include <memory>

namespace gtsam::cuda {

/**
 * Solves an SFM step by eliminating the landmarks and factoring the resulting
 * dense reduced camera system with Cholesky.
 *
 * Landmarks outnumber cameras by orders of magnitude in bundle adjustment but
 * each touches only its own observations, so eliminating them leaves a system in
 * the cameras alone, of side 9 per camera. That is small enough to hold and
 * factor densely for typical problems, which makes this the reference against
 * which the sparse cuDSS and matrix-free PCG paths are checked, and the default
 * backend. Once the reduced solution is in hand each landmark's increment is
 * recovered by back-substitution, so the delta this produces spans cameras and
 * landmarks both.
 *
 * The lifecycle is split because LM tries several damping values against one
 * linearization: linearize() once per nonlinear iteration, then solveLinearized()
 * per attempt. The reduced system's dimension does not change between attempts,
 * so the underlying dense factorization's setup is done once and reused. The
 * single-call solve() overloads do both and exist for tests and for callers with
 * no attempt loop. Everything is queued on the caller's stream; nothing here
 * synchronizes.
 *
 * State is held behind a pointer so this header does not have to be compiled by
 * nvcc.
 */
class GTSAM_EXPORT SfmDenseSchurSolver {
 public:
  /// Creates a solver holding no problem yet; the first linearize() sizes it.
  SfmDenseSchurSolver();
  ~SfmDenseSchurSolver();

  SfmDenseSchurSolver(const SfmDenseSchurSolver&) = delete;
  SfmDenseSchurSolver& operator=(const SfmDenseSchurSolver&) = delete;
  SfmDenseSchurSolver(SfmDenseSchurSolver&&) noexcept;
  SfmDenseSchurSolver& operator=(SfmDenseSchurSolver&&) noexcept;

  /// Explicit outer-iteration lifecycle used by the SFM optimizer.
  /// Evaluates the residuals and Jacobians of every observation at `values`,
  /// which is the once-per-iteration half of the work.
  void linearize(const DeviceValues& values,
                 const SfmProjectionBatch& batch, int numCameras,
                 cudaStream_t stream = nullptr);
  /// Eliminates the landmarks, solves the reduced system damped by lambda times
  /// the identity, and back-substitutes, writing the full camera-and-landmark
  /// increment to `delta`, which is resized as needed. Requires a preceding
  /// linearize(), and may be called repeatedly with different lambdas.
  void solveLinearized(double lambda, DeviceArray<double>* delta,
                       cudaStream_t stream = nullptr);
  /// As above, but damping by lambda times the given per-scalar diagonal, which
  /// is what LM's scale-invariant damping needs.
  void solveLinearized(double lambda,
                       const DeviceArray<double>& dampingDiagonal,
                       DeviceArray<double>* delta,
                       cudaStream_t stream = nullptr);

  /// How many times linearize() has run, for checking that a solve loop reuses
  /// its linearization rather than redoing it per attempt.
  size_t linearizationCount() const;
  /// How many reduced systems have been assembled, one per solveLinearized().
  size_t denseAssemblyCount() const;
  /// The device-resident residuals and Jacobians from the last linearize().
  const SfmProjectionLinearization& linearization() const;
  /// Counters and device timings of the dense Cholesky solves.
  const LinearSolveStats& linearSolveStats() const;

  /// Convenience: linearize() then solveLinearized() in one call.
  void solve(const DeviceValues& values, const SfmProjectionBatch& batch,
             int numCameras, double lambda, DeviceArray<double>* delta,
             cudaStream_t stream = nullptr);
  /// Convenience: as above, with an explicit damping diagonal.
  void solve(const DeviceValues& values, const SfmProjectionBatch& batch,
             int numCameras, double lambda,
             const DeviceArray<double>& dampingDiagonal,
             DeviceArray<double>* delta, cudaStream_t stream = nullptr);

 private:
  struct Impl;
  /// Holds the Schur problem, the dense solver session, and its cached setup.
  std::unique_ptr<Impl> impl_;
};

/// One-shot dense Schur solve, for callers that keep no solver across
/// iterations. Rebuilds all solver state on every call.
GTSAM_EXPORT void solveSfmDenseSchur(
    const DeviceValues& values, const SfmProjectionBatch& batch,
    int numCameras, double lambda, DeviceArray<double>* delta,
    cudaStream_t stream = nullptr);

/// As above, with an explicit damping diagonal.
GTSAM_EXPORT void solveSfmDenseSchur(
    const DeviceValues& values, const SfmProjectionBatch& batch,
    int numCameras, double lambda,
    const DeviceArray<double>& dampingDiagonal,
    DeviceArray<double>* delta, cudaStream_t stream = nullptr);

}  // namespace gtsam::cuda

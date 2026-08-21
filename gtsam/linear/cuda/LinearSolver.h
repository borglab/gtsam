/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    LinearSolver.h
 * @brief   Shared CUDA linear-solver session and backend selection
 * @author  Ruogu Li
 * @date    Aug 15, 2026
 */

#pragma once

#include <gtsam/dllexport.h>
#include <gtsam/base/cuda/DeviceArray.h>
#include <gtsam/linear/cuda/LinearSystem.h>
#include <gtsam/linear/cuda/DeviceSparseSpdSystem.h>

#include <cstddef>
#include <memory>
#include <vector>

namespace gtsam::cuda {

/// Backend-independent convergence and warm-start policy for CUDA PCG.
struct PcgOptions {
  int maxIterations = 250;
  double relativeTolerance = 1e-6;
  bool warmStart = true;
  int convergenceCheckInterval = 10;
};

/// Numerical backends owned by a shared CUDA linear-solver session.
enum class LinearSolverType { DenseCholesky, Cudss, Pcg };

/// Numerical backend selection.
struct LinearSolverOptions {
  /// Numerical backend used by the session.
  LinearSolverType backend = LinearSolverType::Cudss;
};

/// Cumulative lifecycle, convergence, and timing data; defined below.
struct LinearSolveStats;

/**
 * Reusable numerical backend with one analyze/solve lifecycle.
 *
 * Frontends retain ownership of dense, sparse, or operator system storage.
 * A session validates that representation against its selected backend and
 * caches all backend analysis/workspace state across numerical solves.
 */
class GTSAM_EXPORT LinearSolverSession {
 public:
  /// Construct an uninitialized session for the selected backend.
  explicit LinearSolverSession(const LinearSolverOptions& options);
  ~LinearSolverSession();

  LinearSolverSession(const LinearSolverSession&) = delete;
  LinearSolverSession& operator=(const LinearSolverSession&) = delete;
  LinearSolverSession(LinearSolverSession&&) noexcept;
  LinearSolverSession& operator=(LinearSolverSession&&) noexcept;

  /// Return whether a backend consumes the requested representation.
  static bool supports(LinearSolverType backend,
                       LinearSystemKind systemKind);
  /// Throw when backend, representation, or ordering policy is incompatible.
  static void validate(const LinearSolverOptions& options,
                       LinearSystemKind systemKind);

  /// Analyze a dense SPD system of fixed dimension.
  void analyze(int denseDimension, cudaStream_t stream = nullptr);
  /// Analyze a sparse SPD pattern using backend-managed ordering.
  void analyze(const DeviceSparseSpdSystem& system,
               DeviceArray<double>* solution,
               cudaStream_t stream = nullptr);
  /// Analyze a sparse SPD pattern using a scalar permutation.
  void analyze(const DeviceSparseSpdSystem& system,
               DeviceArray<double>* solution,
               const std::vector<int>& scalarPermutation,
               cudaStream_t stream = nullptr);
  /// Initialize an iterative solve for a fixed operator dimension.
  void analyze(int operatorDimension, const PcgOptions& pcgOptions,
               cudaStream_t stream = nullptr,
               bool collectProfile = false);

  /// Factor and solve one analyzed dense SPD system.
  void solve(DenseSpdSystemView system,
             DeviceArray<double>* solution,
             cudaStream_t stream = nullptr);
  /// Factor and solve one analyzed sparse SPD system.
  void solve(const DeviceSparseSpdSystem& system,
             DeviceArray<double>* solution,
             cudaStream_t stream = nullptr);
  /// Solve one analyzed matrix-free system with PCG.
  void solve(const LinearOperator& linearOperator,
             const Preconditioner& preconditioner, const double* rhs,
             DeviceArray<double>* solution,
             cudaStream_t stream = nullptr);

  /** Discard an iterative warm start after the operator's numerics change.
   * Direct backends have no warm-start state and treat this as a no-op. */
  void invalidateWarmStart();

  /// Harvest completed backend work and return cumulative statistics.
  const LinearSolveStats& stats() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

/**
 * Cumulative, backend-independent lifecycle, convergence, and timing data.
 *
 * Counts and durations accumulate over a session's whole lifetime, while the
 * lastPcg* fields describe only the most recent solve. Timings come from CUDA
 * events, so they measure device time and are only meaningful once
 * LinearSolverSession::stats() has harvested the completed work. Fields that
 * name a backend are zero for the others.
 */
struct LinearSolveStats {
  /// Backend the session was constructed with.
  LinearSolverType backend = LinearSolverType::Cudss;
  /// Whether a caller-supplied ordering reached the backend's analysis.
  bool userOrderingApplied = false;
  /// Number of analyze() calls, which is the count of symbolic analyses.
  size_t analysisCount = 0;
  /// Number of numerical factorizations, one per accepted damping attempt.
  size_t factorizationCount = 0;
  /// Number of solves, including those that reused a factorization.
  size_t solveCount = 0;
  /// Total conjugate gradient iterations over all PCG solves.
  size_t pcgIterationsTotal = 0;
  /// Number of PCG solves that stopped at maxIterations without converging.
  size_t pcgMaxIterationHits = 0;
  /// Number of PCG solves that ended on a near-zero curvature denominator.
  size_t pcgBreakdownCount = 0;
  /// Iterations used by the most recent PCG solve.
  size_t lastPcgIterations = 0;
  /// Number of times a residual norm was read back to test convergence.
  size_t pcgHostConvergenceChecks = 0;
  /// Bytes copied device to host for those convergence checks.
  size_t pcgD2hBytes = 0;
  /// Whether the most recent PCG solve met its relative tolerance.
  bool lastPcgConverged = false;
  /// Whether the most recent PCG solve ended in breakdown.
  bool lastPcgBreakdown = false;
  /// Squared residual norm the most recent PCG solve finished at.
  double lastPcgResidualNormSquared = 0.0;
  /// Squared right-hand-side norm the tolerance was measured against.
  double lastPcgRhsNormSquared = 0.0;
  /// Device seconds in symbolic analysis.
  double analysisSeconds = 0.0;
  /// Device seconds in numerical factorization.
  double factorizationSeconds = 0.0;
  /// Device seconds in triangular solves or CG iterations.
  double solveSeconds = 0.0;
  /// Device seconds building preconditioners.
  double preconditionerSeconds = 0.0;
  /// Device seconds in the convergence-check copies counted by pcgD2hBytes.
  double pcgD2hSeconds = 0.0;
  /// Host seconds blocked reading cuDSS status, which forces a sync.
  double dataInfoBoundarySeconds = 0.0;
};

}  // namespace gtsam::cuda

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

/// Cumulative, backend-independent lifecycle, convergence, and timing data.
struct LinearSolveStats {
  LinearSolverType backend = LinearSolverType::Cudss;
  bool userOrderingApplied = false;
  size_t analysisCount = 0;
  size_t factorizationCount = 0;
  size_t solveCount = 0;
  size_t pcgIterationsTotal = 0;
  size_t pcgMaxIterationHits = 0;
  size_t pcgBreakdownCount = 0;
  size_t lastPcgIterations = 0;
  size_t pcgHostConvergenceChecks = 0;
  size_t pcgD2hBytes = 0;
  bool lastPcgConverged = false;
  bool lastPcgBreakdown = false;
  double lastPcgResidualNormSquared = 0.0;
  double lastPcgRhsNormSquared = 0.0;
  double analysisSeconds = 0.0;
  double factorizationSeconds = 0.0;
  double solveSeconds = 0.0;
  double preconditionerSeconds = 0.0;
  double pcgD2hSeconds = 0.0;
  double dataInfoBoundarySeconds = 0.0;
};

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

}  // namespace gtsam::cuda

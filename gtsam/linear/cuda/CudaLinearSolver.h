#pragma once

#include <gtsam/dllexport.h>
#include <gtsam/base/cuda/CudaDeviceArray.h>
#include <gtsam/linear/cuda/CudaLinearSystem.h>
#include <gtsam/linear/cuda/DeviceSparseSpdSystem.h>

#include <cstddef>
#include <memory>
#include <vector>

namespace gtsam::cuda {

struct CudaPcgOptions;

enum class CudaLinearSolverType { DenseCholesky, Cudss, Pcg };

struct CudaLinearSolverOptions {
  CudaLinearSolverType backend = CudaLinearSolverType::Cudss;
  bool useUserOrdering = false;
};

struct CudaLinearSolveStats {
  CudaLinearSolverType backend = CudaLinearSolverType::Cudss;
  bool userOrderingApplied = false;
  size_t analysisCount = 0;
  size_t factorizationCount = 0;
  size_t solveCount = 0;
  size_t pcgIterationsTotal = 0;
  size_t pcgMaxIterationHits = 0;
  size_t lastPcgIterations = 0;
  size_t pcgHostConvergenceChecks = 0;
  bool lastPcgConverged = false;
  bool lastPcgBreakdown = false;
  double lastPcgResidualNormSquared = 0.0;
  double lastPcgRhsNormSquared = 0.0;
  double analysisSeconds = 0.0;
  double factorizationSeconds = 0.0;
  double solveSeconds = 0.0;
  double preconditionerSeconds = 0.0;
};

class GTSAM_EXPORT CudaLinearSolverSession {
 public:
  explicit CudaLinearSolverSession(const CudaLinearSolverOptions& options);
  ~CudaLinearSolverSession();

  CudaLinearSolverSession(const CudaLinearSolverSession&) = delete;
  CudaLinearSolverSession& operator=(const CudaLinearSolverSession&) = delete;
  CudaLinearSolverSession(CudaLinearSolverSession&&) noexcept;
  CudaLinearSolverSession& operator=(CudaLinearSolverSession&&) noexcept;

  static bool Supports(CudaLinearSolverType backend,
                       CudaLinearSystemKind systemKind);
  static void Validate(const CudaLinearSolverOptions& options,
                       CudaLinearSystemKind systemKind);

  void analyze(int denseDimension, cudaStream_t stream = nullptr);
  void analyze(const DeviceSparseSpdSystem& system,
               CudaDeviceArray<double>* solution,
               cudaStream_t stream = nullptr);
  void analyze(const DeviceSparseSpdSystem& system,
               CudaDeviceArray<double>* solution,
               const std::vector<int>& scalarPermutation,
               cudaStream_t stream = nullptr);
  void analyze(int operatorDimension, const CudaPcgOptions& pcgOptions,
               cudaStream_t stream = nullptr,
               bool collectProfile = false);

  void solve(CudaDenseSpdSystemView system,
             CudaDeviceArray<double>* solution,
             cudaStream_t stream = nullptr);
  void solve(const DeviceSparseSpdSystem& system,
             CudaDeviceArray<double>* solution,
             cudaStream_t stream = nullptr);
  void solve(const CudaLinearOperator& linearOperator,
             const CudaPreconditioner& preconditioner, const double* rhs,
             CudaDeviceArray<double>* solution,
             cudaStream_t stream = nullptr);

  /** Discard an iterative warm start after the operator's numerics change.
   * Direct backends have no warm-start state and treat this as a no-op. */
  void invalidateWarmStart();

  const CudaLinearSolveStats& stats() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace gtsam::cuda

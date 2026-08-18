#pragma once

#include <gtsam/base/cuda/DeviceArray.h>
#include <gtsam/dllexport.h>
#include <gtsam/linear/cuda/LinearSolver.h>

#include <memory>

namespace gtsam::cuda {

/** Backend-independent convergence and warm-start policy for CUDA PCG. */
struct PcgOptions {
  int maxIterations = 250;
  double relativeTolerance = 1e-6;
  bool warmStart = true;
  int convergenceCheckInterval = 10;
};

/** Device-resident PCG recurrence over frontend-supplied operator objects. */
class GTSAM_EXPORT PcgSolver {
 public:
  PcgSolver();
  ~PcgSolver();

  PcgSolver(const PcgSolver&) = delete;
  PcgSolver& operator=(const PcgSolver&) = delete;
  PcgSolver(PcgSolver&&) noexcept;
  PcgSolver& operator=(PcgSolver&&) noexcept;

  void initialize(int dimension, const PcgOptions& options,
                  cudaStream_t stream = nullptr,
                  bool collectProfile = false);
  void solve(const LinearOperator& linearOperator,
             const Preconditioner& preconditioner, const double* rhs,
             DeviceArray<double>* solution,
             cudaStream_t stream = nullptr);

  const LinearSolveStats& stats() const;
  void invalidateWarmStart();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace gtsam::cuda

#pragma once

#include <gtsam/base/cuda/DeviceArray.h>
#include <gtsam/linear/cuda/LinearSolver.h>

#include <memory>

namespace gtsam::cuda {

/** Device-resident PCG recurrence over frontend-supplied operator objects. */
class PcgSolver {
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

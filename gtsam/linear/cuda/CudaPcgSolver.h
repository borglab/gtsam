#pragma once

#include <gtsam/base/cuda/CudaDeviceArray.h>
#include <gtsam/dllexport.h>
#include <gtsam/linear/cuda/CudaLinearSolver.h>

#include <memory>

namespace gtsam::cuda {

struct CudaPcgOptions {
  int maxIterations = 250;
  double relativeTolerance = 1e-6;
  bool warmStart = true;
  int convergenceCheckInterval = 10;
};

class GTSAM_EXPORT CudaPcgSolver {
 public:
  CudaPcgSolver();
  ~CudaPcgSolver();

  CudaPcgSolver(const CudaPcgSolver&) = delete;
  CudaPcgSolver& operator=(const CudaPcgSolver&) = delete;
  CudaPcgSolver(CudaPcgSolver&&) noexcept;
  CudaPcgSolver& operator=(CudaPcgSolver&&) noexcept;

  void initialize(int dimension, const CudaPcgOptions& options,
                  cudaStream_t stream = nullptr,
                  bool collectProfile = false);
  void solve(const CudaLinearOperator& linearOperator,
             const CudaPreconditioner& preconditioner, const double* rhs,
             CudaDeviceArray<double>* solution,
             cudaStream_t stream = nullptr);

  const CudaLinearSolveStats& stats() const;
  void invalidateWarmStart();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace gtsam::cuda

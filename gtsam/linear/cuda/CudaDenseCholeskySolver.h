#pragma once

#include <gtsam/base/cuda/CudaDeviceArray.h>
#include <gtsam/dllexport.h>
#include <gtsam/linear/cuda/CudaLinearSolver.h>

#include <memory>

namespace gtsam::cuda {

/** Persistent cuSOLVER DN Cholesky backend for column-major SPD systems. */
class GTSAM_EXPORT CudaDenseCholeskySolver {
 public:
  CudaDenseCholeskySolver();
  ~CudaDenseCholeskySolver();

  CudaDenseCholeskySolver(const CudaDenseCholeskySolver&) = delete;
  CudaDenseCholeskySolver& operator=(const CudaDenseCholeskySolver&) = delete;
  CudaDenseCholeskySolver(CudaDenseCholeskySolver&&) noexcept;
  CudaDenseCholeskySolver& operator=(CudaDenseCholeskySolver&&) noexcept;

  void analyze(int maximumDimension, cudaStream_t stream = nullptr);
  void solveInPlace(CudaDenseSpdSystemView system,
                    cudaStream_t stream = nullptr,
                    CudaLinearSolveStats* stats = nullptr);
  void solve(CudaDenseSpdSystemView system,
             CudaDeviceArray<double>* solution,
             cudaStream_t stream = nullptr,
             CudaLinearSolveStats* stats = nullptr);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace gtsam::cuda

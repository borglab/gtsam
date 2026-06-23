#pragma once

#include <gtsam/dllexport.h>
#include <gtsam/base/cuda/CudaDeviceArray.h>
#include <gtsam/nonlinear/cuda/DeviceSparseNormalEquations.h>

#include <memory>

namespace gtsam::cuda {

class GTSAM_EXPORT CudssSpdSolver {
 public:
  CudssSpdSolver();
  ~CudssSpdSolver();

  CudssSpdSolver(const CudssSpdSolver&) = delete;
  CudssSpdSolver& operator=(const CudssSpdSolver&) = delete;
  CudssSpdSolver(CudssSpdSolver&&) noexcept;
  CudssSpdSolver& operator=(CudssSpdSolver&&) noexcept;

  void analyze(const DeviceSparseNormalEquations& system,
               CudaDeviceArray<double>* solution,
               cudaStream_t stream = nullptr);
  void solve(const DeviceSparseNormalEquations& system,
             CudaDeviceArray<double>* solution,
             cudaStream_t stream = nullptr);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

class GTSAM_EXPORT CudssLinearSolver {
 public:
  void solveSpd(const DeviceSparseNormalEquations& system,
                CudaDeviceArray<double>* solution,
                cudaStream_t stream = nullptr) const;
};

}  // namespace gtsam::cuda

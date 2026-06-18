#pragma once

#include <gtsam/base/cuda/CudaDeviceArray.h>
#include <gtsam/nonlinear/cuda/DeviceValues.h>
#include <gtsam/slam/cuda/CudaSfmProjectionBatch.h>

#include <cuda_runtime_api.h>

#include <memory>

namespace gtsam::cuda {

class CudaSfmDenseSchurSolver {
 public:
  CudaSfmDenseSchurSolver();
  ~CudaSfmDenseSchurSolver();

  CudaSfmDenseSchurSolver(const CudaSfmDenseSchurSolver&) = delete;
  CudaSfmDenseSchurSolver& operator=(const CudaSfmDenseSchurSolver&) = delete;
  CudaSfmDenseSchurSolver(CudaSfmDenseSchurSolver&&) noexcept;
  CudaSfmDenseSchurSolver& operator=(CudaSfmDenseSchurSolver&&) noexcept;

  void solve(const DeviceValues& values, const CudaSfmProjectionBatch& batch,
             int numCameras, double lambda, CudaDeviceArray<double>* delta,
             cudaStream_t stream = nullptr);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

void SolveCudaSfmDenseSchur(const DeviceValues& values,
                            const CudaSfmProjectionBatch& batch,
                            int numCameras, double lambda,
                            CudaDeviceArray<double>* delta,
                            cudaStream_t stream = nullptr);

}  // namespace gtsam::cuda

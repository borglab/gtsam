#pragma once

#include <gtsam/dllexport.h>
#include <gtsam/base/cuda/CudaDeviceArray.h>
#include <gtsam/nonlinear/cuda/DeviceValues.h>
#include <gtsam/slam/cuda/CudaSfmProjectionBatch.h>

#include <cuda_runtime_api.h>

#include <cstddef>
#include <memory>

namespace gtsam::cuda {

class GTSAM_EXPORT CudaSfmDenseSchurSolver {
 public:
  CudaSfmDenseSchurSolver();
  ~CudaSfmDenseSchurSolver();

  CudaSfmDenseSchurSolver(const CudaSfmDenseSchurSolver&) = delete;
  CudaSfmDenseSchurSolver& operator=(const CudaSfmDenseSchurSolver&) = delete;
  CudaSfmDenseSchurSolver(CudaSfmDenseSchurSolver&&) noexcept;
  CudaSfmDenseSchurSolver& operator=(CudaSfmDenseSchurSolver&&) noexcept;

  /** Explicit outer-iteration lifecycle used by the SFM optimizer. */
  void linearize(const DeviceValues& values,
                 const CudaSfmProjectionBatch& batch, int numCameras,
                 cudaStream_t stream = nullptr);
  void solveLinearized(double lambda, CudaDeviceArray<double>* delta,
                       cudaStream_t stream = nullptr);
  void solveLinearized(double lambda,
                       const CudaDeviceArray<double>& dampingDiagonal,
                       CudaDeviceArray<double>* delta,
                       cudaStream_t stream = nullptr);

  size_t linearizationCount() const;
  size_t denseAssemblyCount() const;

  void solve(const DeviceValues& values, const CudaSfmProjectionBatch& batch,
             int numCameras, double lambda, CudaDeviceArray<double>* delta,
             cudaStream_t stream = nullptr);
  void solve(const DeviceValues& values, const CudaSfmProjectionBatch& batch,
             int numCameras, double lambda,
             const CudaDeviceArray<double>& dampingDiagonal,
             CudaDeviceArray<double>* delta, cudaStream_t stream = nullptr);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

GTSAM_EXPORT void SolveCudaSfmDenseSchur(
    const DeviceValues& values, const CudaSfmProjectionBatch& batch,
    int numCameras, double lambda, CudaDeviceArray<double>* delta,
    cudaStream_t stream = nullptr);

GTSAM_EXPORT void SolveCudaSfmDenseSchur(
    const DeviceValues& values, const CudaSfmProjectionBatch& batch,
    int numCameras, double lambda,
    const CudaDeviceArray<double>& dampingDiagonal,
    CudaDeviceArray<double>* delta, cudaStream_t stream = nullptr);

}  // namespace gtsam::cuda

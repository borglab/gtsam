#pragma once

#include <gtsam/base/cuda/CudaDeviceArray.h>
#include <gtsam/nonlinear/cuda/DeviceValues.h>
#include <gtsam/slam/cuda/CudaSfmProjectionBatch.h>

#include <cuda_runtime_api.h>

namespace gtsam::cuda {

struct CudaSfmProjectionLinearization {
  CudaDeviceArray<double> residuals;
  CudaDeviceArray<double> cameraJacobians;
  CudaDeviceArray<double> pointJacobians;
};

void LinearizeCudaSfmProjectionBatch(
    const DeviceValues& values, const CudaSfmProjectionBatch& batch,
    CudaSfmProjectionLinearization* linearization,
    cudaStream_t stream = nullptr);

double ComputeCudaSfmProjectionError(const DeviceValues& values,
                                     const CudaSfmProjectionBatch& batch,
                                     cudaStream_t stream = nullptr);

}  // namespace gtsam::cuda

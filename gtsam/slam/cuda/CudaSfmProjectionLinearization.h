#pragma once

#include <gtsam/base/cuda/CudaDeviceArray.h>
#include <gtsam/nonlinear/cuda/DeviceSparseNormalEquations.h>
#include <gtsam/nonlinear/cuda/DeviceValues.h>
#include <gtsam/slam/cuda/CudaSfmProjectionBatch.h>

#include <cuda_runtime_api.h>

#include <cstddef>

namespace gtsam::cuda {

struct CudaSfmProjectionLinearization {
  CudaDeviceArray<double> residuals;
  CudaDeviceArray<double> cameraJacobians;
  CudaDeviceArray<double> pointJacobians;
};

/** Logical device-to-host traffic caused by a scalar reduction boundary. */
struct CudaSfmReductionTransferProfile {
  size_t d2hBytes = 0;
  double d2hElapsed = 0.0;
};

void LinearizeCudaSfmProjectionBatch(
    const DeviceValues& values, const CudaSfmProjectionBatch& batch,
    CudaSfmProjectionLinearization* linearization,
    cudaStream_t stream = nullptr);

double ComputeCudaSfmProjectionError(const DeviceValues& values,
                                     const CudaSfmProjectionBatch& batch,
                                     cudaStream_t stream = nullptr,
                                     CudaSfmReductionTransferProfile* profile =
                                         nullptr);

void ComputeCudaSfmHessianDiagonal(
    const DeviceValues& values, const CudaSfmProjectionBatch& batch,
    int numCameras, double minDiagonal, double maxDiagonal,
    CudaDeviceArray<double>* diagonal, cudaStream_t stream = nullptr);

double ComputeCudaSfmLinearizedErrorChange(
    const DeviceValues& values, const CudaSfmProjectionBatch& batch,
    int numCameras, const CudaDeviceArray<double>& delta,
    double* oldLinearizedError = nullptr,
    double* newLinearizedError = nullptr, cudaStream_t stream = nullptr);

double ComputeCudaSfmLinearizedErrorChange(
    const CudaSfmProjectionLinearization& linearization,
    const CudaSfmProjectionBatch& batch, int numCameras,
    const CudaDeviceArray<double>& delta,
    double* oldLinearizedError = nullptr,
    double* newLinearizedError = nullptr, cudaStream_t stream = nullptr,
    CudaSfmReductionTransferProfile* profile = nullptr);

void AccumulateCudaSfmNormalEquations(
    const DeviceValues& values, const CudaSfmProjectionBatch& batch,
    int numCameras, DeviceSparseNormalEquations* system,
    cudaStream_t stream = nullptr);

void AccumulateCudaSfmNormalEquations(
    const CudaSfmProjectionLinearization& linearization,
    const CudaSfmProjectionBatch& batch, int numCameras,
    DeviceSparseNormalEquations* system, cudaStream_t stream = nullptr);

}  // namespace gtsam::cuda

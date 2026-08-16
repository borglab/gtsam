#pragma once

#include <gtsam/base/cuda/CudaDeviceArray.h>
#include <gtsam/dllexport.h>
#include <gtsam/linear/cuda/DeviceSparseSpdSystem.h>
#include <gtsam/slam/cuda/CudaSfmProjectionBatch.h>
#include <gtsam/slam/cuda/CudaSfmProjectionLinearization.h>

#include <cuda_runtime_api.h>

namespace gtsam::cuda {

/** Numerically assemble a camera-only upper-CSR Schur complement in-place. */
GTSAM_EXPORT void AssembleCudaSfmSparseSchur(
    const CudaSfmProjectionBatch& batch,
    const CudaSfmProjectionLinearization& linearization, int numCameras,
    double lambda, const CudaDeviceArray<double>* dampingDiagonal,
    DeviceSparseSpdSystem* system, CudaDeviceArray<int>* singularPointBlocks,
    cudaStream_t stream = nullptr);

}  // namespace gtsam::cuda

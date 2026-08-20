#pragma once

#include <gtsam/base/cuda/DeviceArray.h>
#include <gtsam/dllexport.h>
#include <gtsam/linear/cuda/DeviceSparseSpdSystem.h>
#include <gtsam/slam/cuda/internal/SfmProjectionBatch.h>
#include <gtsam/slam/cuda/internal/SfmProjectionLinearization.h>

#include <cuda_runtime_api.h>

namespace gtsam::cuda {

struct SfmSchurBlocks;

/// Numerically assemble a camera-only upper-CSR Schur complement in-place.
GTSAM_EXPORT void assembleSfmSparseSchur(
    const SfmProjectionBatch& batch,
    const SfmSchurBlocks& blocks, int numCameras,
    double lambda, const DeviceArray<double>* dampingDiagonal,
    DeviceSparseSpdSystem* system, DeviceArray<int>* singularPointBlocks,
    cudaStream_t stream = nullptr);

}  // namespace gtsam::cuda

/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SfmSparseSchur.h
 * @brief   Numerical assembly of the camera-only upper-CSR Schur complement
 * @author  Ruogu Li
 * @date    Aug 18, 2026
 */

#pragma once

#include <gtsam/base/cuda/DeviceArray.h>
#include <gtsam/dllexport.h>
#include <gtsam/linear/cuda/DeviceSparseSpdSystem.h>
#include <gtsam/sfm/cuda/internal/SfmProjectionBatch.h>
#include <gtsam/sfm/cuda/internal/SfmProjectionLinearization.h>

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

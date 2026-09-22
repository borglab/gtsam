/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SfmProjectionLinearization.h
 * @brief   Linearizes a projection batch and stages the scalar reductions
 * @author  Ruogu Li
 * @date    Aug 18, 2026
 */

#pragma once

#include <gtsam/base/cuda/DeviceArray.h>
#include <gtsam/base/cuda/PinnedHostArray.h>
#include <gtsam/dllexport.h>
#include <gtsam/nonlinear/cuda/internal/DeviceValues.h>
#include <gtsam/sfm/cuda/internal/SfmProjectionBatch.h>

#include <cuda_runtime_api.h>

#include <cstddef>

namespace gtsam::cuda {

/// Device buffers produced by linearizing a projection batch.
struct SfmProjectionLinearization {
  DeviceArray<double> residuals;
  DeviceArray<double> cameraJacobians;
  DeviceArray<double> pointJacobians;
};

/// Logical device-to-host traffic caused by a scalar reduction boundary.
struct SfmReductionTransferProfile {
  size_t d2hBytes = 0;
  double d2hElapsed = 0.0;
};

/**
 * Reusable staging for the scalar reductions at the end of an LM attempt.
 *
 * The projection-error and linearized-error-change reductions each run once
 * per damping attempt and each ends by moving one block-sum array to the host.
 * Holding those arrays here removes a cudaMalloc/cudaFree pair and a host
 * allocation from every attempt, and pinning the host side lets the download
 * proceed as a plain DMA instead of a staged pageable copy. Block sums are
 * still accumulated on the host in launch order, so reductions produce exactly
 * the same values as they do without a scratch.
 *
 * One scratch serves one stream at a time; concurrent reductions need one
 * scratch each.
 */
class GTSAM_EXPORT SfmReductionScratch {
 public:
  /// Independent buffer pairs, one per concurrent block-sum array.
  enum Slot {
    kProjectionError = 0,
    kOldLinearizedError = 1,
    kNewLinearizedError = 2,
    kSlotCount = 3,
  };

  /// Returns device storage for exactly `blocks` sums, reallocating on change.
  DeviceArray<double>& device(Slot slot, size_t blocks);
  /// Returns pinned host storage for exactly `blocks` sums.
  double* host(Slot slot, size_t blocks);

 private:
  DeviceArray<double> device_[kSlotCount];
  PinnedHostArray<double> host_[kSlotCount];
};

/// Linearizes all observations in a projection batch.
GTSAM_EXPORT void linearizeSfmProjectionBatch(
    const DeviceValues& values, const SfmProjectionBatch& batch,
    SfmProjectionLinearization* linearization,
    cudaStream_t stream = nullptr);

/**
 * Computes the nonlinear projection objective on the device.
 * Pass scratch to reuse reduction staging across calls on the same stream.
 */
GTSAM_EXPORT double computeSfmProjectionError(
    const DeviceValues& values, const SfmProjectionBatch& batch,
    cudaStream_t stream = nullptr,
    SfmReductionTransferProfile* profile = nullptr,
    SfmReductionScratch* scratch = nullptr);

/// Computes and clamps the Hessian diagonal used for LM damping.
GTSAM_EXPORT void computeSfmHessianDiagonal(
    const DeviceValues& values, const SfmProjectionBatch& batch,
    int numCameras, double minDiagonal, double maxDiagonal,
    DeviceArray<double>* diagonal, cudaStream_t stream = nullptr);

/// Computes predicted objective change from current values and a delta.
GTSAM_EXPORT double computeSfmLinearizedErrorChange(
    const DeviceValues& values, const SfmProjectionBatch& batch,
    int numCameras, const DeviceArray<double>& delta,
    double* oldLinearizedError = nullptr,
    double* newLinearizedError = nullptr, cudaStream_t stream = nullptr);

/**
 * Computes predicted objective change from a cached linearization.
 * Pass scratch to reuse reduction staging across calls on the same stream.
 */
GTSAM_EXPORT double computeSfmLinearizedErrorChange(
    const SfmProjectionLinearization& linearization,
    const SfmProjectionBatch& batch, int numCameras,
    const DeviceArray<double>& delta,
    double* oldLinearizedError = nullptr,
    double* newLinearizedError = nullptr, cudaStream_t stream = nullptr,
    SfmReductionTransferProfile* profile = nullptr,
    SfmReductionScratch* scratch = nullptr);

}  // namespace gtsam::cuda

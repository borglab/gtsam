#pragma once

#include <gtsam/base/cuda/DeviceArray.h>
#include <gtsam/dllexport.h>
#include <gtsam/nonlinear/cuda/DeviceSparseNormalEquations.h>
#include <gtsam/nonlinear/cuda/DeviceValues.h>
#include <gtsam/slam/cuda/SfmProjectionBatch.h>

#include <cuda_runtime_api.h>

#include <cstddef>

namespace gtsam::cuda {

/// Device buffers produced by linearizing a projection batch.
struct SfmProjectionLinearization {
  DeviceArray<double> residuals;
  DeviceArray<double> cameraJacobians;
  DeviceArray<double> pointJacobians;
};

/** Logical device-to-host traffic caused by a scalar reduction boundary. */
struct SfmReductionTransferProfile {
  size_t d2hBytes = 0;
  double d2hElapsed = 0.0;
};

/** Linearizes all observations in a projection batch. */
GTSAM_EXPORT void linearizeSfmProjectionBatch(
    const DeviceValues& values, const SfmProjectionBatch& batch,
    SfmProjectionLinearization* linearization,
    cudaStream_t stream = nullptr);

/** Computes the nonlinear projection objective on the device. */
GTSAM_EXPORT double computeSfmProjectionError(
    const DeviceValues& values, const SfmProjectionBatch& batch,
    cudaStream_t stream = nullptr,
    SfmReductionTransferProfile* profile = nullptr);

/** Computes and clamps the Hessian diagonal used for LM damping. */
GTSAM_EXPORT void computeSfmHessianDiagonal(
    const DeviceValues& values, const SfmProjectionBatch& batch,
    int numCameras, double minDiagonal, double maxDiagonal,
    DeviceArray<double>* diagonal, cudaStream_t stream = nullptr);

/** Computes predicted objective change from current values and a delta. */
GTSAM_EXPORT double computeSfmLinearizedErrorChange(
    const DeviceValues& values, const SfmProjectionBatch& batch,
    int numCameras, const DeviceArray<double>& delta,
    double* oldLinearizedError = nullptr,
    double* newLinearizedError = nullptr, cudaStream_t stream = nullptr);

/** Computes predicted objective change from a cached linearization. */
GTSAM_EXPORT double computeSfmLinearizedErrorChange(
    const SfmProjectionLinearization& linearization,
    const SfmProjectionBatch& batch, int numCameras,
    const DeviceArray<double>& delta,
    double* oldLinearizedError = nullptr,
    double* newLinearizedError = nullptr, cudaStream_t stream = nullptr,
    SfmReductionTransferProfile* profile = nullptr);

/** Accumulates projection factors into a sparse normal-equation system. */
GTSAM_EXPORT void accumulateSfmNormalEquations(
    const DeviceValues& values, const SfmProjectionBatch& batch,
    int numCameras, DeviceSparseNormalEquations* system,
    cudaStream_t stream = nullptr);

/** Accumulates a cached projection linearization into normal equations. */
GTSAM_EXPORT void accumulateSfmNormalEquations(
    const SfmProjectionLinearization& linearization,
    const SfmProjectionBatch& batch, int numCameras,
    DeviceSparseNormalEquations* system, cudaStream_t stream = nullptr);

}  // namespace gtsam::cuda

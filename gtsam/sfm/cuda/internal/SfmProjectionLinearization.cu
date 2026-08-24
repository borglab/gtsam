/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SfmProjectionLinearization.cu
 * @brief   Linearizes a projection batch and stages the scalar reductions
 * @author  Ruogu Li
 * @date    Jun 17, 2026
 */

#include <gtsam/base/cuda/Errors.h>
#include <gtsam/sfm/cuda/internal/DeviceGeometryKernels.h>
#include <gtsam/sfm/cuda/internal/SfmProjectionLinearization.h>

#include <algorithm>
#include <chrono>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace gtsam::cuda {
namespace {

constexpr int kProjectionLinearizationBlockSize = 256;
constexpr int kProjectionErrorBlockSize = 256;
constexpr int kHessianDiagonalBlockSize = 256;

const SfmSqrtInfo2* sqrtInfoDataOrNull(
    const SfmProjectionBatch& batch, size_t numObservations,
    const char* caller) {
  if (batch.noiseMode() == SfmProjectionNoiseMode::Unit) {
    return nullptr;
  }
  if (batch.sqrtInfos().size() != numObservations) {
    throw std::invalid_argument(std::string(caller) +
                                " sqrt-info/observation size mismatch");
  }
  return batch.sqrtInfos().data();
}

const SfmRobustModel* robustModelDataOrNull(
    const SfmProjectionBatch& batch, size_t numObservations,
    const char* caller) {
  if (batch.noiseMode() != SfmProjectionNoiseMode::Robust) {
    return nullptr;
  }
  if (batch.robustModels().size() != numObservations) {
    throw std::invalid_argument(std::string(caller) +
                                " robust-model/observation size mismatch");
  }
  return batch.robustModels().data();
}

__device__ double robustWeight(const SfmRobustModel& model,
                               double distance) {
  const double absDistance = fabs(distance);
  switch (model.kind) {
    case SfmRobustModelKind::None:
      return 1.0;
    case SfmRobustModelKind::Huber:
      return absDistance <= model.parameter ? 1.0
                                            : model.parameter / absDistance;
    case SfmRobustModelKind::Tukey: {
      if (absDistance > model.parameter) {
        return 0.0;
      }
      const double normalized = distance / model.parameter;
      const double oneMinusSquared = 1.0 - normalized * normalized;
      return oneMinusSquared * oneMinusSquared;
    }
  }
  return 1.0;
}

__device__ double robustLoss(const SfmRobustModel& model,
                             double distance) {
  const double absDistance = fabs(distance);
  switch (model.kind) {
    case SfmRobustModelKind::None:
      return 0.5 * distance * distance;
    case SfmRobustModelKind::Huber:
      if (absDistance <= model.parameter) {
        return 0.5 * distance * distance;
      }
      return model.parameter * (absDistance - 0.5 * model.parameter);
    case SfmRobustModelKind::Tukey: {
      const double c2 = model.parameter * model.parameter;
      if (absDistance > model.parameter) {
        return c2 / 6.0;
      }
      const double normalized = distance / model.parameter;
      const double oneMinusSquared = 1.0 - normalized * normalized;
      return c2 *
             (1.0 - oneMinusSquared * oneMinusSquared * oneMinusSquared) /
             6.0;
    }
  }
  return 0.5 * distance * distance;
}

__device__ void robustRowScales(const SfmRobustModel& model,
                                double residual0, double residual1,
                                double* row0Scale, double* row1Scale) {
  if (model.reweightScheme == SfmRobustReweightScheme::Scalar) {
    *row0Scale = sqrt(robustWeight(model, residual0));
    *row1Scale = sqrt(robustWeight(model, residual1));
  } else {
    const double distance =
        sqrt(residual0 * residual0 + residual1 * residual1);
    const double scale = sqrt(robustWeight(model, distance));
    *row0Scale = scale;
    *row1Scale = scale;
  }
}

template <bool kWhitened, bool kRobust>
__global__ void linearizeSfmProjectionKernel(
    const DevicePinholeCameraCal3Bundler* cameras, const DevicePoint3* points,
    const SfmObservation* observations, size_t numObservations,
    const SfmSqrtInfo2* sqrtInfos,
    const SfmRobustModel* robustModels,
    double* residuals, double* cameraJacobians, double* pointJacobians) {
  const size_t i = static_cast<size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (i >= numObservations) return;

  const SfmObservation observation = observations[i];
  const DeviceProjectionResult result = evaluatePinholeBundlerProjection(
      cameras[observation.cameraSlot], points[observation.pointSlot],
      observation);

  double r0 = result.residual[0];
  double r1 = result.residual[1];
  SfmSqrtInfo2 sqrtInfo{1.0, 0.0, 1.0};
  if constexpr (kWhitened) {
    sqrtInfo = sqrtInfos[i];
    const double rawR0 = r0;
    const double rawR1 = r1;
    r0 = sqrtInfo.r00 * rawR0 + sqrtInfo.r01 * rawR1;
    r1 = sqrtInfo.r11 * rawR1;
  }
  double row0Scale = 1.0;
  double row1Scale = 1.0;
  if constexpr (kRobust) {
    robustRowScales(robustModels[i], r0, r1, &row0Scale, &row1Scale);
    r0 *= row0Scale;
    r1 *= row1Scale;
  }

  residuals[2 * i] = r0;
  residuals[2 * i + 1] = r1;

  for (int col = 0; col < 9; ++col) {
    const double j0 = result.cameraJacobian[col];
    const double j1 = result.cameraJacobian[9 + col];
    double row0 = j0;
    double row1 = j1;
    if constexpr (kWhitened) {
      row0 = sqrtInfo.r00 * j0 + sqrtInfo.r01 * j1;
      row1 = sqrtInfo.r11 * j1;
    }
    cameraJacobians[18 * i + col] = row0Scale * row0;
    cameraJacobians[18 * i + 9 + col] = row1Scale * row1;
  }
  for (int col = 0; col < 3; ++col) {
    const double j0 = result.pointJacobian[col];
    const double j1 = result.pointJacobian[3 + col];
    double row0 = j0;
    double row1 = j1;
    if constexpr (kWhitened) {
      row0 = sqrtInfo.r00 * j0 + sqrtInfo.r01 * j1;
      row1 = sqrtInfo.r11 * j1;
    }
    pointJacobians[6 * i + col] = row0Scale * row0;
    pointJacobians[6 * i + 3 + col] = row1Scale * row1;
  }
}

template <bool kWhitened, bool kRobust>
__global__ void computeSfmProjectionErrorKernel(
    const DevicePinholeCameraCal3Bundler* cameras, const DevicePoint3* points,
    const SfmObservation* observations, size_t numObservations,
    const SfmSqrtInfo2* sqrtInfos,
    const SfmRobustModel* robustModels,
    double* blockSums) {
  __shared__ double shared[kProjectionErrorBlockSize];

  const int thread = threadIdx.x;
  const size_t stride = static_cast<size_t>(gridDim.x) * blockDim.x;
  size_t i = static_cast<size_t>(blockIdx.x) * blockDim.x + threadIdx.x;

  double sum = 0.0;
  while (i < numObservations) {
    const SfmObservation observation = observations[i];
    const DeviceProjectionResult result = evaluatePinholeBundlerProjection(
        cameras[observation.cameraSlot], points[observation.pointSlot],
        observation);
    double r0 = result.residual[0];
    double r1 = result.residual[1];
    if constexpr (kWhitened) {
      const SfmSqrtInfo2 sqrtInfo = sqrtInfos[i];
      const double rawR0 = r0;
      const double rawR1 = r1;
      r0 = sqrtInfo.r00 * rawR0 + sqrtInfo.r01 * rawR1;
      r1 = sqrtInfo.r11 * rawR1;
    }
    if constexpr (kRobust) {
      sum += robustLoss(robustModels[i], sqrt(r0 * r0 + r1 * r1));
    } else {
      sum += 0.5 * (r0 * r0 + r1 * r1);
    }
    i += stride;
  }

  shared[thread] = sum;
  __syncthreads();

  for (int offset = blockDim.x / 2; offset > 0; offset /= 2) {
    if (thread < offset) {
      shared[thread] += shared[thread + offset];
    }
    __syncthreads();
  }

  if (thread == 0) {
    blockSums[blockIdx.x] = shared[0];
  }
}

template <bool kWhitened, bool kRobust>
__global__ void accumulateSfmHessianDiagonalKernel(
    const DevicePinholeCameraCal3Bundler* cameras, const DevicePoint3* points,
    const SfmObservation* observations, size_t numObservations,
    const SfmSqrtInfo2* sqrtInfos,
    const SfmRobustModel* robustModels,
    int numCameras, double* diagonal) {
  const size_t i = static_cast<size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (i >= numObservations) return;

  const SfmObservation observation = observations[i];
  const DeviceProjectionResult result = evaluatePinholeBundlerProjection(
      cameras[observation.cameraSlot], points[observation.pointSlot],
      observation);
  SfmSqrtInfo2 sqrtInfo{1.0, 0.0, 1.0};
  if constexpr (kWhitened) {
    sqrtInfo = sqrtInfos[i];
  }
  double residual0 = result.residual[0];
  double residual1 = result.residual[1];
  if constexpr (kWhitened) {
    const double rawResidual0 = residual0;
    const double rawResidual1 = residual1;
    residual0 = sqrtInfo.r00 * rawResidual0 + sqrtInfo.r01 * rawResidual1;
    residual1 = sqrtInfo.r11 * rawResidual1;
  }
  double row0Scale = 1.0;
  double row1Scale = 1.0;
  if constexpr (kRobust) {
    robustRowScales(robustModels[i], residual0, residual1, &row0Scale,
                    &row1Scale);
  }

  const int cameraBase = 9 * observation.cameraSlot;
  const int pointBase = 9 * numCameras + 3 * observation.pointSlot;
  for (int col = 0; col < 9; ++col) {
    double j0 = result.cameraJacobian[col];
    double j1 = result.cameraJacobian[9 + col];
    if constexpr (kWhitened) {
      const double rawJ0 = j0;
      const double rawJ1 = j1;
      j0 = sqrtInfo.r00 * rawJ0 + sqrtInfo.r01 * rawJ1;
      j1 = sqrtInfo.r11 * rawJ1;
    }
    j0 *= row0Scale;
    j1 *= row1Scale;
    atomicAdd(&diagonal[cameraBase + col], j0 * j0 + j1 * j1);
  }
  for (int col = 0; col < 3; ++col) {
    double j0 = result.pointJacobian[col];
    double j1 = result.pointJacobian[3 + col];
    if constexpr (kWhitened) {
      const double rawJ0 = j0;
      const double rawJ1 = j1;
      j0 = sqrtInfo.r00 * rawJ0 + sqrtInfo.r01 * rawJ1;
      j1 = sqrtInfo.r11 * rawJ1;
    }
    j0 *= row0Scale;
    j1 *= row1Scale;
    atomicAdd(&diagonal[pointBase + col], j0 * j0 + j1 * j1);
  }
}

__global__ void clampHessianDiagonalKernel(size_t dimension,
                                           double minDiagonal,
                                           double maxDiagonal,
                                           double* diagonal) {
  const size_t i = static_cast<size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (i >= dimension) return;

  const double value = diagonal[i];
  diagonal[i] = fmin(maxDiagonal, fmax(minDiagonal, value));
}

__global__ void computeLinearizedErrorChangeKernel(
    const double* residuals, const double* cameraJacobians,
    const double* pointJacobians, const SfmObservation* observations,
    size_t numObservations, int numCameras, const double* delta,
    double* oldBlockSums, double* newBlockSums) {
  __shared__ double sharedOld[kProjectionErrorBlockSize];
  __shared__ double sharedNew[kProjectionErrorBlockSize];

  const int thread = threadIdx.x;
  const size_t stride = static_cast<size_t>(gridDim.x) * blockDim.x;
  size_t i = static_cast<size_t>(blockIdx.x) * blockDim.x + threadIdx.x;

  double oldSum = 0.0;
  double newSum = 0.0;
  while (i < numObservations) {
    const SfmObservation observation = observations[i];
    const int cameraBase = 9 * observation.cameraSlot;
    const int pointBase = 9 * numCameras + 3 * observation.pointSlot;

    double r0 = residuals[2 * i];
    double r1 = residuals[2 * i + 1];
    oldSum += 0.5 * (r0 * r0 + r1 * r1);

    const double* cameraJacobian = cameraJacobians + 18 * i;
    const double* pointJacobian = pointJacobians + 6 * i;
    for (int col = 0; col < 9; ++col) {
      const double d = delta[cameraBase + col];
      r0 += cameraJacobian[col] * d;
      r1 += cameraJacobian[9 + col] * d;
    }
    for (int col = 0; col < 3; ++col) {
      const double d = delta[pointBase + col];
      r0 += pointJacobian[col] * d;
      r1 += pointJacobian[3 + col] * d;
    }
    newSum += 0.5 * (r0 * r0 + r1 * r1);
    i += stride;
  }

  sharedOld[thread] = oldSum;
  sharedNew[thread] = newSum;
  __syncthreads();

  for (int offset = blockDim.x / 2; offset > 0; offset /= 2) {
    if (thread < offset) {
      sharedOld[thread] += sharedOld[thread + offset];
      sharedNew[thread] += sharedNew[thread + offset];
    }
    __syncthreads();
  }

  if (thread == 0) {
    oldBlockSums[blockIdx.x] = sharedOld[0];
    newBlockSums[blockIdx.x] = sharedNew[0];
  }
}

}  // namespace

void linearizeSfmProjectionBatch(
    const DeviceValues& values, const SfmProjectionBatch& batch,
    SfmProjectionLinearization* linearization, cudaStream_t stream) {
  if (!linearization) {
    throw std::invalid_argument(
        "linearizeSfmProjectionBatch requires output storage");
  }

  const auto& cameraBlock = values.block<DevicePinholeCameraCal3Bundler>(
      kDevicePinholeCameraCal3BundlerType);
  const auto& pointBlock = values.block<DevicePoint3>(kDevicePoint3Type);

  if (batch.numCameras() > cameraBlock.values.size()) {
    throw std::invalid_argument(
        "linearizeSfmProjectionBatch camera batch/value size mismatch");
  }
  if (batch.numPoints() > pointBlock.values.size()) {
    throw std::invalid_argument(
        "linearizeSfmProjectionBatch point batch/value size mismatch");
  }

  const size_t numObservations = batch.numObservations();
  linearization->residuals.resize(2 * numObservations);
  linearization->cameraJacobians.resize(18 * numObservations);
  linearization->pointJacobians.resize(6 * numObservations);
  if (numObservations == 0) {
    return;
  }

  const size_t gridSizeSize =
      (numObservations + kProjectionLinearizationBlockSize - 1) /
      kProjectionLinearizationBlockSize;
  if (gridSizeSize > static_cast<size_t>(std::numeric_limits<int>::max())) {
    throw std::invalid_argument(
        "linearizeSfmProjectionBatch grid size exceeds CUDA launch limit");
  }
  const int gridSize = static_cast<int>(gridSizeSize);
  const SfmSqrtInfo2* sqrtInfos = sqrtInfoDataOrNull(
      batch, numObservations, "linearizeSfmProjectionBatch");
  const SfmRobustModel* robustModels = robustModelDataOrNull(
      batch, numObservations, "linearizeSfmProjectionBatch");
  if (batch.noiseMode() == SfmProjectionNoiseMode::Robust) {
    linearizeSfmProjectionKernel<true, true>
        <<<gridSize, kProjectionLinearizationBlockSize, 0, stream>>>(
            cameraBlock.values.data(), pointBlock.values.data(),
            batch.observations().data(), numObservations, sqrtInfos,
            robustModels, linearization->residuals.data(),
            linearization->cameraJacobians.data(),
            linearization->pointJacobians.data());
  } else if (batch.noiseMode() == SfmProjectionNoiseMode::Whitened) {
    linearizeSfmProjectionKernel<true, false>
        <<<gridSize, kProjectionLinearizationBlockSize, 0, stream>>>(
            cameraBlock.values.data(), pointBlock.values.data(),
            batch.observations().data(), numObservations, sqrtInfos,
            nullptr, linearization->residuals.data(),
            linearization->cameraJacobians.data(),
            linearization->pointJacobians.data());
  } else {
    linearizeSfmProjectionKernel<false, false>
        <<<gridSize, kProjectionLinearizationBlockSize, 0, stream>>>(
            cameraBlock.values.data(), pointBlock.values.data(),
            batch.observations().data(), numObservations, nullptr,
            nullptr, linearization->residuals.data(),
            linearization->cameraJacobians.data(),
            linearization->pointJacobians.data());
  }
  GTSAM_CUDA_CHECK(cudaGetLastError());
}

DeviceArray<double>& SfmReductionScratch::device(Slot slot, size_t blocks) {
  DeviceArray<double>& buffer = device_[slot];
  buffer.resize(blocks);
  return buffer;
}

double* SfmReductionScratch::host(Slot slot, size_t blocks) {
  PinnedHostArray<double>& buffer = host_[slot];
  buffer.resize(blocks);
  return buffer.data();
}

namespace {

/// Queues a download of `blocks` device sums into pinned host storage.
void enqueueBlockDownload(const DeviceArray<double>& deviceSums,
                          double* hostSums, size_t blocks,
                          cudaStream_t stream) {
  GTSAM_CUDA_CHECK(cudaMemcpyAsync(hostSums, deviceSums.data(),
                                   sizeof(double) * blocks,
                                   cudaMemcpyDeviceToHost, stream));
}

/// Adds downloaded block sums in index order, as a whole-array sum would.
double sumBlocks(const double* hostSums, size_t blocks) {
  double sum = 0.0;
  for (size_t i = 0; i < blocks; ++i) sum += hostSums[i];
  return sum;
}

}  // namespace

double computeSfmProjectionError(const DeviceValues& values,
                                     const SfmProjectionBatch& batch,
                                     cudaStream_t stream,
                                     SfmReductionTransferProfile* profile,
                                     SfmReductionScratch* scratch) {
  const auto& cameraBlock = values.block<DevicePinholeCameraCal3Bundler>(
      kDevicePinholeCameraCal3BundlerType);
  const auto& pointBlock = values.block<DevicePoint3>(kDevicePoint3Type);

  if (batch.numCameras() > cameraBlock.values.size()) {
    throw std::invalid_argument(
        "computeSfmProjectionError camera batch/value size mismatch");
  }
  if (batch.numPoints() > pointBlock.values.size()) {
    throw std::invalid_argument(
        "computeSfmProjectionError point batch/value size mismatch");
  }

  const size_t numObservations = batch.numObservations();
  if (numObservations == 0) {
    return 0.0;
  }

  const int gridSize = static_cast<int>(std::min<size_t>(
      (numObservations + kProjectionErrorBlockSize - 1) /
          kProjectionErrorBlockSize,
      4096));
  const size_t numBlocks = static_cast<size_t>(gridSize);
  SfmReductionScratch localScratch;
  SfmReductionScratch& staging = scratch ? *scratch : localScratch;
  DeviceArray<double>& blockSums =
      staging.device(SfmReductionScratch::kProjectionError, numBlocks);
  double* hostBlockSums =
      staging.host(SfmReductionScratch::kProjectionError, numBlocks);
  const SfmSqrtInfo2* sqrtInfos = sqrtInfoDataOrNull(
      batch, numObservations, "computeSfmProjectionError");
  const SfmRobustModel* robustModels = robustModelDataOrNull(
      batch, numObservations, "computeSfmProjectionError");
  if (batch.noiseMode() == SfmProjectionNoiseMode::Robust) {
    computeSfmProjectionErrorKernel<true, true>
        <<<gridSize, kProjectionErrorBlockSize, 0, stream>>>(
            cameraBlock.values.data(), pointBlock.values.data(),
            batch.observations().data(), numObservations, sqrtInfos,
            robustModels, blockSums.data());
  } else if (batch.noiseMode() == SfmProjectionNoiseMode::Whitened) {
    computeSfmProjectionErrorKernel<true, false>
        <<<gridSize, kProjectionErrorBlockSize, 0, stream>>>(
            cameraBlock.values.data(), pointBlock.values.data(),
            batch.observations().data(), numObservations, sqrtInfos,
            nullptr, blockSums.data());
  } else {
    computeSfmProjectionErrorKernel<false, false>
        <<<gridSize, kProjectionErrorBlockSize, 0, stream>>>(
            cameraBlock.values.data(), pointBlock.values.data(),
            batch.observations().data(), numObservations, nullptr,
            nullptr, blockSums.data());
  }
  GTSAM_CUDA_CHECK(cudaGetLastError());

  const auto downloadStart = std::chrono::steady_clock::now();
  enqueueBlockDownload(blockSums, hostBlockSums, numBlocks, stream);
  GTSAM_CUDA_CHECK(cudaStreamSynchronize(stream));
  const double error = sumBlocks(hostBlockSums, numBlocks);
  if (profile) {
    profile->d2hBytes += sizeof(double) * numBlocks;
    profile->d2hElapsed +=
        std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                      downloadStart)
            .count();
  }
  return error;
}

void computeSfmHessianDiagonal(
    const DeviceValues& values, const SfmProjectionBatch& batch,
    int numCameras, double minDiagonal, double maxDiagonal,
    DeviceArray<double>* diagonal, cudaStream_t stream) {
  if (!diagonal) {
    throw std::invalid_argument(
        "computeSfmHessianDiagonal requires output storage");
  }
  if (numCameras < 0) {
    throw std::invalid_argument(
        "computeSfmHessianDiagonal requires nonnegative camera count");
  }
  if (minDiagonal < 0.0 || maxDiagonal < minDiagonal) {
    throw std::invalid_argument(
        "computeSfmHessianDiagonal invalid diagonal clamp bounds");
  }

  const auto& cameraBlock = values.block<DevicePinholeCameraCal3Bundler>(
      kDevicePinholeCameraCal3BundlerType);
  const auto& pointBlock = values.block<DevicePoint3>(kDevicePoint3Type);
  if (static_cast<size_t>(numCameras) < batch.numCameras() ||
      batch.numCameras() > cameraBlock.values.size()) {
    throw std::invalid_argument(
        "computeSfmHessianDiagonal camera batch/value size mismatch");
  }
  if (batch.numPoints() > pointBlock.values.size()) {
    throw std::invalid_argument(
        "computeSfmHessianDiagonal point batch/value size mismatch");
  }

  const size_t dimension =
      9 * static_cast<size_t>(numCameras) + 3 * batch.numPoints();
  diagonal->resize(dimension);
  diagonal->zero(stream);
  if (dimension == 0) {
    return;
  }

  const size_t numObservations = batch.numObservations();
  if (numObservations > 0) {
    const size_t gridSizeSize =
        (numObservations + kHessianDiagonalBlockSize - 1) /
        kHessianDiagonalBlockSize;
    if (gridSizeSize > static_cast<size_t>(std::numeric_limits<int>::max())) {
      throw std::invalid_argument(
          "computeSfmHessianDiagonal grid size exceeds CUDA launch limit");
    }
    const SfmSqrtInfo2* sqrtInfos = sqrtInfoDataOrNull(
        batch, numObservations, "computeSfmHessianDiagonal");
    const SfmRobustModel* robustModels = robustModelDataOrNull(
        batch, numObservations, "computeSfmHessianDiagonal");
    if (batch.noiseMode() == SfmProjectionNoiseMode::Robust) {
      accumulateSfmHessianDiagonalKernel<true, true>
          <<<static_cast<int>(gridSizeSize), kHessianDiagonalBlockSize, 0,
             stream>>>(cameraBlock.values.data(), pointBlock.values.data(),
                       batch.observations().data(), numObservations, sqrtInfos,
                       robustModels, numCameras, diagonal->data());
    } else if (batch.noiseMode() == SfmProjectionNoiseMode::Whitened) {
      accumulateSfmHessianDiagonalKernel<true, false>
          <<<static_cast<int>(gridSizeSize), kHessianDiagonalBlockSize, 0,
             stream>>>(cameraBlock.values.data(), pointBlock.values.data(),
                       batch.observations().data(), numObservations, sqrtInfos,
                       nullptr, numCameras, diagonal->data());
    } else {
      accumulateSfmHessianDiagonalKernel<false, false>
          <<<static_cast<int>(gridSizeSize), kHessianDiagonalBlockSize, 0,
             stream>>>(cameraBlock.values.data(), pointBlock.values.data(),
                       batch.observations().data(), numObservations, nullptr,
                       nullptr, numCameras, diagonal->data());
    }
    GTSAM_CUDA_CHECK(cudaGetLastError());
  }

  const size_t clampGridSize =
      (dimension + kHessianDiagonalBlockSize - 1) / kHessianDiagonalBlockSize;
  if (clampGridSize > static_cast<size_t>(std::numeric_limits<int>::max())) {
    throw std::invalid_argument(
        "computeSfmHessianDiagonal clamp grid size exceeds CUDA launch "
        "limit");
  }
  clampHessianDiagonalKernel<<<static_cast<int>(clampGridSize),
                               kHessianDiagonalBlockSize, 0, stream>>>(
      dimension, minDiagonal, maxDiagonal, diagonal->data());
  GTSAM_CUDA_CHECK(cudaGetLastError());
}

double computeSfmLinearizedErrorChange(
    const DeviceValues& values, const SfmProjectionBatch& batch,
    int numCameras, const DeviceArray<double>& delta,
    double* oldLinearizedError, double* newLinearizedError,
    cudaStream_t stream) {
  if (numCameras < 0) {
    throw std::invalid_argument(
        "computeSfmLinearizedErrorChange requires nonnegative camera "
        "count");
  }
  const auto& cameraBlock = values.block<DevicePinholeCameraCal3Bundler>(
      kDevicePinholeCameraCal3BundlerType);
  const auto& pointBlock = values.block<DevicePoint3>(kDevicePoint3Type);
  if (static_cast<size_t>(numCameras) < batch.numCameras() ||
      batch.numCameras() > cameraBlock.values.size()) {
    throw std::invalid_argument(
        "computeSfmLinearizedErrorChange camera batch/value size mismatch");
  }
  if (batch.numPoints() > pointBlock.values.size()) {
    throw std::invalid_argument(
        "computeSfmLinearizedErrorChange point batch/value size mismatch");
  }
  const size_t dimension =
      9 * static_cast<size_t>(numCameras) + 3 * batch.numPoints();
  if (delta.size() != dimension) {
    throw std::invalid_argument(
        "computeSfmLinearizedErrorChange delta size mismatch");
  }

  if (batch.numObservations() == 0) {
    if (oldLinearizedError) *oldLinearizedError = 0.0;
    if (newLinearizedError) *newLinearizedError = 0.0;
    return 0.0;
  }

  SfmProjectionLinearization linearization;
  linearizeSfmProjectionBatch(values, batch, &linearization, stream);

  return computeSfmLinearizedErrorChange(
      linearization, batch, numCameras, delta, oldLinearizedError,
      newLinearizedError, stream);
}

double computeSfmLinearizedErrorChange(
    const SfmProjectionLinearization& linearization,
    const SfmProjectionBatch& batch, int numCameras,
    const DeviceArray<double>& delta, double* oldLinearizedError,
    double* newLinearizedError, cudaStream_t stream,
    SfmReductionTransferProfile* profile, SfmReductionScratch* scratch) {
  if (numCameras < 0 ||
      static_cast<size_t>(numCameras) < batch.numCameras()) {
    throw std::invalid_argument(
        "computeSfmLinearizedErrorChange camera count mismatch");
  }
  const size_t observations = batch.numObservations();
  if (linearization.residuals.size() != 2 * observations ||
      linearization.cameraJacobians.size() != 18 * observations ||
      linearization.pointJacobians.size() != 6 * observations) {
    throw std::invalid_argument(
        "computeSfmLinearizedErrorChange linearization size mismatch");
  }
  const size_t dimension =
      9 * static_cast<size_t>(numCameras) + 3 * batch.numPoints();
  if (delta.size() != dimension) {
    throw std::invalid_argument(
        "computeSfmLinearizedErrorChange delta size mismatch");
  }
  if (observations == 0) {
    if (oldLinearizedError) *oldLinearizedError = 0.0;
    if (newLinearizedError) *newLinearizedError = 0.0;
    return 0.0;
  }

  const int gridSize = static_cast<int>(std::min<size_t>(
      (observations + kProjectionErrorBlockSize - 1) /
          kProjectionErrorBlockSize,
      4096));
  const size_t numBlocks = static_cast<size_t>(gridSize);
  SfmReductionScratch localScratch;
  SfmReductionScratch& staging = scratch ? *scratch : localScratch;
  DeviceArray<double>& oldBlockSums =
      staging.device(SfmReductionScratch::kOldLinearizedError, numBlocks);
  DeviceArray<double>& newBlockSums =
      staging.device(SfmReductionScratch::kNewLinearizedError, numBlocks);
  double* hostOldBlockSums =
      staging.host(SfmReductionScratch::kOldLinearizedError, numBlocks);
  double* hostNewBlockSums =
      staging.host(SfmReductionScratch::kNewLinearizedError, numBlocks);
  computeLinearizedErrorChangeKernel<<<gridSize, kProjectionErrorBlockSize, 0,
                                        stream>>>(
      linearization.residuals.data(), linearization.cameraJacobians.data(),
      linearization.pointJacobians.data(), batch.observations().data(),
      observations, numCameras, delta.data(), oldBlockSums.data(),
      newBlockSums.data());
  GTSAM_CUDA_CHECK(cudaGetLastError());

  const auto downloadStart = std::chrono::steady_clock::now();
  enqueueBlockDownload(oldBlockSums, hostOldBlockSums, numBlocks, stream);
  enqueueBlockDownload(newBlockSums, hostNewBlockSums, numBlocks, stream);
  GTSAM_CUDA_CHECK(cudaStreamSynchronize(stream));
  if (profile) {
    profile->d2hBytes += sizeof(double) * 2 * numBlocks;
    profile->d2hElapsed +=
        std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                      downloadStart)
            .count();
  }

  const double oldError = sumBlocks(hostOldBlockSums, numBlocks);
  const double newError = sumBlocks(hostNewBlockSums, numBlocks);
  if (oldLinearizedError) *oldLinearizedError = oldError;
  if (newLinearizedError) *newLinearizedError = newError;
  return oldError - newError;
}


}  // namespace gtsam::cuda

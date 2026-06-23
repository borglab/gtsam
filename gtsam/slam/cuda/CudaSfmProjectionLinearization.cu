#include <gtsam/base/cuda/CudaErrors.h>
#include <gtsam/nonlinear/cuda/DeviceGeometryKernels.h>
#include <gtsam/slam/cuda/CudaSfmProjectionLinearization.h>

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace gtsam::cuda {
namespace {

constexpr int kProjectionLinearizationBlockSize = 256;
constexpr int kProjectionTangentDim = 12;
constexpr int kProjectionErrorBlockSize = 256;
constexpr int kHessianDiagonalBlockSize = 256;

const CudaSfmSqrtInfo2* SqrtInfoDataOrNull(
    const CudaSfmProjectionBatch& batch, size_t numObservations,
    const char* caller) {
  if (batch.noiseMode() == CudaSfmProjectionNoiseMode::Unit) {
    return nullptr;
  }
  if (batch.sqrtInfos().size() != numObservations) {
    throw std::invalid_argument(std::string(caller) +
                                " sqrt-info/observation size mismatch");
  }
  return batch.sqrtInfos().data();
}

const CudaSfmRobustModel* RobustModelDataOrNull(
    const CudaSfmProjectionBatch& batch, size_t numObservations,
    const char* caller) {
  if (batch.noiseMode() != CudaSfmProjectionNoiseMode::Robust) {
    return nullptr;
  }
  if (batch.robustModels().size() != numObservations) {
    throw std::invalid_argument(std::string(caller) +
                                " robust-model/observation size mismatch");
  }
  return batch.robustModels().data();
}

__device__ double RobustWeight(const CudaSfmRobustModel& model,
                               double distance) {
  const double absDistance = fabs(distance);
  switch (model.kind) {
    case CudaSfmRobustModelKind::None:
      return 1.0;
    case CudaSfmRobustModelKind::Huber:
      return absDistance <= model.parameter ? 1.0
                                            : model.parameter / absDistance;
    case CudaSfmRobustModelKind::Tukey: {
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

__device__ double RobustLoss(const CudaSfmRobustModel& model,
                             double distance) {
  const double absDistance = fabs(distance);
  switch (model.kind) {
    case CudaSfmRobustModelKind::None:
      return 0.5 * distance * distance;
    case CudaSfmRobustModelKind::Huber:
      if (absDistance <= model.parameter) {
        return 0.5 * distance * distance;
      }
      return model.parameter * (absDistance - 0.5 * model.parameter);
    case CudaSfmRobustModelKind::Tukey: {
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

__device__ void RobustRowScales(const CudaSfmRobustModel& model,
                                double residual0, double residual1,
                                double* row0Scale, double* row1Scale) {
  if (model.reweightScheme == CudaSfmRobustReweightScheme::Scalar) {
    *row0Scale = sqrt(RobustWeight(model, residual0));
    *row1Scale = sqrt(RobustWeight(model, residual1));
  } else {
    const double distance =
        sqrt(residual0 * residual0 + residual1 * residual1);
    const double scale = sqrt(RobustWeight(model, distance));
    *row0Scale = scale;
    *row1Scale = scale;
  }
}

template <bool kWhitened, bool kRobust>
__global__ void LinearizeCudaSfmProjectionKernel(
    const DevicePinholeCameraCal3Bundler* cameras, const DevicePoint3* points,
    const CudaSfmObservation* observations, size_t numObservations,
    const CudaSfmSqrtInfo2* sqrtInfos,
    const CudaSfmRobustModel* robustModels,
    double* residuals, double* cameraJacobians, double* pointJacobians) {
  const size_t i = static_cast<size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (i >= numObservations) return;

  const CudaSfmObservation observation = observations[i];
  const DeviceProjectionResult result = EvaluatePinholeBundlerProjection(
      cameras[observation.cameraSlot], points[observation.pointSlot],
      observation);

  double r0 = result.residual[0];
  double r1 = result.residual[1];
  CudaSfmSqrtInfo2 sqrtInfo{1.0, 0.0, 1.0};
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
    RobustRowScales(robustModels[i], r0, r1, &row0Scale, &row1Scale);
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

__device__ int FindCsrEntry(const int* rowPointers, const int* colIndices,
                            int row, int col) {
  int begin = rowPointers[row];
  int end = rowPointers[row + 1];
  while (begin < end) {
    const int mid = begin + (end - begin) / 2;
    const int midCol = colIndices[mid];
    if (midCol < col) {
      begin = mid + 1;
    } else if (midCol > col) {
      end = mid;
    } else {
      return mid;
    }
  }
  return -1;
}

template <bool kWhitened, bool kRobust>
__global__ void AccumulateCudaSfmNormalEquationsKernel(
    const DevicePinholeCameraCal3Bundler* cameras, const DevicePoint3* points,
    const CudaSfmObservation* observations, size_t numObservations,
    const CudaSfmSqrtInfo2* sqrtInfos,
    const CudaSfmRobustModel* robustModels,
    int numCameras, const int* rowPointers, const int* colIndices,
    double* values, double* rhs, int* missingEntries) {
  const size_t i = static_cast<size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (i >= numObservations) return;

  const CudaSfmObservation observation = observations[i];
  const DeviceProjectionResult result = EvaluatePinholeBundlerProjection(
      cameras[observation.cameraSlot], points[observation.pointSlot],
      observation);
  CudaSfmSqrtInfo2 sqrtInfo{1.0, 0.0, 1.0};
  if constexpr (kWhitened) {
    sqrtInfo = sqrtInfos[i];
  }

  const int cameraBase = 9 * observation.cameraSlot;
  const int pointBase = 9 * numCameras + 3 * observation.pointSlot;
  int global[kProjectionTangentDim];
  double jacobian0[kProjectionTangentDim];
  double jacobian1[kProjectionTangentDim];

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
    RobustRowScales(robustModels[i], residual0, residual1, &row0Scale,
                    &row1Scale);
    residual0 *= row0Scale;
    residual1 *= row1Scale;
  }

  for (int col = 0; col < 9; ++col) {
    global[col] = cameraBase + col;
    const double j0 = result.cameraJacobian[col];
    const double j1 = result.cameraJacobian[9 + col];
    double row0 = j0;
    double row1 = j1;
    if constexpr (kWhitened) {
      row0 = sqrtInfo.r00 * j0 + sqrtInfo.r01 * j1;
      row1 = sqrtInfo.r11 * j1;
    }
    jacobian0[col] = row0Scale * row0;
    jacobian1[col] = row1Scale * row1;
  }
  for (int col = 0; col < 3; ++col) {
    global[9 + col] = pointBase + col;
    const double j0 = result.pointJacobian[col];
    const double j1 = result.pointJacobian[3 + col];
    double row0 = j0;
    double row1 = j1;
    if constexpr (kWhitened) {
      row0 = sqrtInfo.r00 * j0 + sqrtInfo.r01 * j1;
      row1 = sqrtInfo.r11 * j1;
    }
    jacobian0[9 + col] = row0Scale * row0;
    jacobian1[9 + col] = row1Scale * row1;
  }
  for (int a = 0; a < kProjectionTangentDim; ++a) {
    atomicAdd(&rhs[global[a]],
              -jacobian0[a] * residual0 - jacobian1[a] * residual1);
    for (int b = a; b < kProjectionTangentDim; ++b) {
      int row = global[a];
      int col = global[b];
      if (row > col) {
        const int tmp = row;
        row = col;
        col = tmp;
      }

      const int entry = FindCsrEntry(rowPointers, colIndices, row, col);
      if (entry >= 0) {
        atomicAdd(&values[entry], jacobian0[a] * jacobian0[b] +
                                      jacobian1[a] * jacobian1[b]);
      } else {
        atomicAdd(missingEntries, 1);
      }
    }
  }
}

template <bool kWhitened, bool kRobust>
__global__ void ComputeCudaSfmProjectionErrorKernel(
    const DevicePinholeCameraCal3Bundler* cameras, const DevicePoint3* points,
    const CudaSfmObservation* observations, size_t numObservations,
    const CudaSfmSqrtInfo2* sqrtInfos,
    const CudaSfmRobustModel* robustModels,
    double* blockSums) {
  __shared__ double shared[kProjectionErrorBlockSize];

  const int thread = threadIdx.x;
  const size_t stride = static_cast<size_t>(gridDim.x) * blockDim.x;
  size_t i = static_cast<size_t>(blockIdx.x) * blockDim.x + threadIdx.x;

  double sum = 0.0;
  while (i < numObservations) {
    const CudaSfmObservation observation = observations[i];
    const DeviceProjectionResult result = EvaluatePinholeBundlerProjection(
        cameras[observation.cameraSlot], points[observation.pointSlot],
        observation);
    double r0 = result.residual[0];
    double r1 = result.residual[1];
    if constexpr (kWhitened) {
      const CudaSfmSqrtInfo2 sqrtInfo = sqrtInfos[i];
      const double rawR0 = r0;
      const double rawR1 = r1;
      r0 = sqrtInfo.r00 * rawR0 + sqrtInfo.r01 * rawR1;
      r1 = sqrtInfo.r11 * rawR1;
    }
    if constexpr (kRobust) {
      sum += RobustLoss(robustModels[i], sqrt(r0 * r0 + r1 * r1));
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
__global__ void AccumulateCudaSfmHessianDiagonalKernel(
    const DevicePinholeCameraCal3Bundler* cameras, const DevicePoint3* points,
    const CudaSfmObservation* observations, size_t numObservations,
    const CudaSfmSqrtInfo2* sqrtInfos,
    const CudaSfmRobustModel* robustModels,
    int numCameras, double* diagonal) {
  const size_t i = static_cast<size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (i >= numObservations) return;

  const CudaSfmObservation observation = observations[i];
  const DeviceProjectionResult result = EvaluatePinholeBundlerProjection(
      cameras[observation.cameraSlot], points[observation.pointSlot],
      observation);
  CudaSfmSqrtInfo2 sqrtInfo{1.0, 0.0, 1.0};
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
    RobustRowScales(robustModels[i], residual0, residual1, &row0Scale,
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

__global__ void ClampHessianDiagonalKernel(size_t dimension,
                                           double minDiagonal,
                                           double maxDiagonal,
                                           double* diagonal) {
  const size_t i = static_cast<size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (i >= dimension) return;

  const double value = diagonal[i];
  diagonal[i] = fmin(maxDiagonal, fmax(minDiagonal, value));
}

__global__ void ComputeLinearizedErrorChangeKernel(
    const double* residuals, const double* cameraJacobians,
    const double* pointJacobians, const CudaSfmObservation* observations,
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
    const CudaSfmObservation observation = observations[i];
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

void LinearizeCudaSfmProjectionBatch(
    const DeviceValues& values, const CudaSfmProjectionBatch& batch,
    CudaSfmProjectionLinearization* linearization, cudaStream_t stream) {
  if (!linearization) {
    throw std::invalid_argument(
        "LinearizeCudaSfmProjectionBatch requires output storage");
  }

  const auto& cameraBlock = values.block<DevicePinholeCameraCal3Bundler>(
      kDevicePinholeCameraCal3BundlerType);
  const auto& pointBlock = values.block<DevicePoint3>(kDevicePoint3Type);

  if (batch.numCameras() > cameraBlock.values.size()) {
    throw std::invalid_argument(
        "LinearizeCudaSfmProjectionBatch camera batch/value size mismatch");
  }
  if (batch.numPoints() > pointBlock.values.size()) {
    throw std::invalid_argument(
        "LinearizeCudaSfmProjectionBatch point batch/value size mismatch");
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
        "LinearizeCudaSfmProjectionBatch grid size exceeds CUDA launch limit");
  }
  const int gridSize = static_cast<int>(gridSizeSize);
  const CudaSfmSqrtInfo2* sqrtInfos = SqrtInfoDataOrNull(
      batch, numObservations, "LinearizeCudaSfmProjectionBatch");
  const CudaSfmRobustModel* robustModels = RobustModelDataOrNull(
      batch, numObservations, "LinearizeCudaSfmProjectionBatch");
  if (batch.noiseMode() == CudaSfmProjectionNoiseMode::Robust) {
    LinearizeCudaSfmProjectionKernel<true, true>
        <<<gridSize, kProjectionLinearizationBlockSize, 0, stream>>>(
            cameraBlock.values.data(), pointBlock.values.data(),
            batch.observations().data(), numObservations, sqrtInfos,
            robustModels, linearization->residuals.data(),
            linearization->cameraJacobians.data(),
            linearization->pointJacobians.data());
  } else if (batch.noiseMode() == CudaSfmProjectionNoiseMode::Whitened) {
    LinearizeCudaSfmProjectionKernel<true, false>
        <<<gridSize, kProjectionLinearizationBlockSize, 0, stream>>>(
            cameraBlock.values.data(), pointBlock.values.data(),
            batch.observations().data(), numObservations, sqrtInfos,
            nullptr, linearization->residuals.data(),
            linearization->cameraJacobians.data(),
            linearization->pointJacobians.data());
  } else {
    LinearizeCudaSfmProjectionKernel<false, false>
        <<<gridSize, kProjectionLinearizationBlockSize, 0, stream>>>(
            cameraBlock.values.data(), pointBlock.values.data(),
            batch.observations().data(), numObservations, nullptr,
            nullptr, linearization->residuals.data(),
            linearization->cameraJacobians.data(),
            linearization->pointJacobians.data());
  }
  GTSAM_CUDA_CHECK(cudaGetLastError());
}

double ComputeCudaSfmProjectionError(const DeviceValues& values,
                                     const CudaSfmProjectionBatch& batch,
                                     cudaStream_t stream) {
  const auto& cameraBlock = values.block<DevicePinholeCameraCal3Bundler>(
      kDevicePinholeCameraCal3BundlerType);
  const auto& pointBlock = values.block<DevicePoint3>(kDevicePoint3Type);

  if (batch.numCameras() > cameraBlock.values.size()) {
    throw std::invalid_argument(
        "ComputeCudaSfmProjectionError camera batch/value size mismatch");
  }
  if (batch.numPoints() > pointBlock.values.size()) {
    throw std::invalid_argument(
        "ComputeCudaSfmProjectionError point batch/value size mismatch");
  }

  const size_t numObservations = batch.numObservations();
  if (numObservations == 0) {
    return 0.0;
  }

  const int gridSize = static_cast<int>(std::min<size_t>(
      (numObservations + kProjectionErrorBlockSize - 1) /
          kProjectionErrorBlockSize,
      4096));
  CudaDeviceArray<double> blockSums(static_cast<size_t>(gridSize));
  const CudaSfmSqrtInfo2* sqrtInfos = SqrtInfoDataOrNull(
      batch, numObservations, "ComputeCudaSfmProjectionError");
  const CudaSfmRobustModel* robustModels = RobustModelDataOrNull(
      batch, numObservations, "ComputeCudaSfmProjectionError");
  if (batch.noiseMode() == CudaSfmProjectionNoiseMode::Robust) {
    ComputeCudaSfmProjectionErrorKernel<true, true>
        <<<gridSize, kProjectionErrorBlockSize, 0, stream>>>(
            cameraBlock.values.data(), pointBlock.values.data(),
            batch.observations().data(), numObservations, sqrtInfos,
            robustModels, blockSums.data());
  } else if (batch.noiseMode() == CudaSfmProjectionNoiseMode::Whitened) {
    ComputeCudaSfmProjectionErrorKernel<true, false>
        <<<gridSize, kProjectionErrorBlockSize, 0, stream>>>(
            cameraBlock.values.data(), pointBlock.values.data(),
            batch.observations().data(), numObservations, sqrtInfos,
            nullptr, blockSums.data());
  } else {
    ComputeCudaSfmProjectionErrorKernel<false, false>
        <<<gridSize, kProjectionErrorBlockSize, 0, stream>>>(
            cameraBlock.values.data(), pointBlock.values.data(),
            batch.observations().data(), numObservations, nullptr,
            nullptr, blockSums.data());
  }
  GTSAM_CUDA_CHECK(cudaGetLastError());

  std::vector<double> hostBlockSums;
  blockSums.download(&hostBlockSums, stream);
  GTSAM_CUDA_CHECK(cudaStreamSynchronize(stream));

  double error = 0.0;
  for (double blockSum : hostBlockSums) {
    error += blockSum;
  }
  return error;
}

void ComputeCudaSfmHessianDiagonal(
    const DeviceValues& values, const CudaSfmProjectionBatch& batch,
    int numCameras, double minDiagonal, double maxDiagonal,
    CudaDeviceArray<double>* diagonal, cudaStream_t stream) {
  if (!diagonal) {
    throw std::invalid_argument(
        "ComputeCudaSfmHessianDiagonal requires output storage");
  }
  if (numCameras < 0) {
    throw std::invalid_argument(
        "ComputeCudaSfmHessianDiagonal requires nonnegative camera count");
  }
  if (minDiagonal < 0.0 || maxDiagonal < minDiagonal) {
    throw std::invalid_argument(
        "ComputeCudaSfmHessianDiagonal invalid diagonal clamp bounds");
  }

  const auto& cameraBlock = values.block<DevicePinholeCameraCal3Bundler>(
      kDevicePinholeCameraCal3BundlerType);
  const auto& pointBlock = values.block<DevicePoint3>(kDevicePoint3Type);
  if (static_cast<size_t>(numCameras) < batch.numCameras() ||
      batch.numCameras() > cameraBlock.values.size()) {
    throw std::invalid_argument(
        "ComputeCudaSfmHessianDiagonal camera batch/value size mismatch");
  }
  if (batch.numPoints() > pointBlock.values.size()) {
    throw std::invalid_argument(
        "ComputeCudaSfmHessianDiagonal point batch/value size mismatch");
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
          "ComputeCudaSfmHessianDiagonal grid size exceeds CUDA launch limit");
    }
    const CudaSfmSqrtInfo2* sqrtInfos = SqrtInfoDataOrNull(
        batch, numObservations, "ComputeCudaSfmHessianDiagonal");
    const CudaSfmRobustModel* robustModels = RobustModelDataOrNull(
        batch, numObservations, "ComputeCudaSfmHessianDiagonal");
    if (batch.noiseMode() == CudaSfmProjectionNoiseMode::Robust) {
      AccumulateCudaSfmHessianDiagonalKernel<true, true>
          <<<static_cast<int>(gridSizeSize), kHessianDiagonalBlockSize, 0,
             stream>>>(cameraBlock.values.data(), pointBlock.values.data(),
                       batch.observations().data(), numObservations, sqrtInfos,
                       robustModels, numCameras, diagonal->data());
    } else if (batch.noiseMode() == CudaSfmProjectionNoiseMode::Whitened) {
      AccumulateCudaSfmHessianDiagonalKernel<true, false>
          <<<static_cast<int>(gridSizeSize), kHessianDiagonalBlockSize, 0,
             stream>>>(cameraBlock.values.data(), pointBlock.values.data(),
                       batch.observations().data(), numObservations, sqrtInfos,
                       nullptr, numCameras, diagonal->data());
    } else {
      AccumulateCudaSfmHessianDiagonalKernel<false, false>
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
        "ComputeCudaSfmHessianDiagonal clamp grid size exceeds CUDA launch "
        "limit");
  }
  ClampHessianDiagonalKernel<<<static_cast<int>(clampGridSize),
                               kHessianDiagonalBlockSize, 0, stream>>>(
      dimension, minDiagonal, maxDiagonal, diagonal->data());
  GTSAM_CUDA_CHECK(cudaGetLastError());
}

double ComputeCudaSfmLinearizedErrorChange(
    const DeviceValues& values, const CudaSfmProjectionBatch& batch,
    int numCameras, const CudaDeviceArray<double>& delta,
    double* oldLinearizedError, double* newLinearizedError,
    cudaStream_t stream) {
  if (numCameras < 0) {
    throw std::invalid_argument(
        "ComputeCudaSfmLinearizedErrorChange requires nonnegative camera "
        "count");
  }
  const auto& cameraBlock = values.block<DevicePinholeCameraCal3Bundler>(
      kDevicePinholeCameraCal3BundlerType);
  const auto& pointBlock = values.block<DevicePoint3>(kDevicePoint3Type);
  if (static_cast<size_t>(numCameras) < batch.numCameras() ||
      batch.numCameras() > cameraBlock.values.size()) {
    throw std::invalid_argument(
        "ComputeCudaSfmLinearizedErrorChange camera batch/value size mismatch");
  }
  if (batch.numPoints() > pointBlock.values.size()) {
    throw std::invalid_argument(
        "ComputeCudaSfmLinearizedErrorChange point batch/value size mismatch");
  }
  const size_t dimension =
      9 * static_cast<size_t>(numCameras) + 3 * batch.numPoints();
  if (delta.size() != dimension) {
    throw std::invalid_argument(
        "ComputeCudaSfmLinearizedErrorChange delta size mismatch");
  }

  if (batch.numObservations() == 0) {
    if (oldLinearizedError) *oldLinearizedError = 0.0;
    if (newLinearizedError) *newLinearizedError = 0.0;
    return 0.0;
  }

  CudaSfmProjectionLinearization linearization;
  LinearizeCudaSfmProjectionBatch(values, batch, &linearization, stream);

  const int gridSize = static_cast<int>(std::min<size_t>(
      (batch.numObservations() + kProjectionErrorBlockSize - 1) /
          kProjectionErrorBlockSize,
      4096));
  CudaDeviceArray<double> oldBlockSums(static_cast<size_t>(gridSize));
  CudaDeviceArray<double> newBlockSums(static_cast<size_t>(gridSize));
  ComputeLinearizedErrorChangeKernel<<<gridSize, kProjectionErrorBlockSize, 0,
                                        stream>>>(
      linearization.residuals.data(), linearization.cameraJacobians.data(),
      linearization.pointJacobians.data(), batch.observations().data(),
      batch.numObservations(), numCameras, delta.data(), oldBlockSums.data(),
      newBlockSums.data());
  GTSAM_CUDA_CHECK(cudaGetLastError());

  std::vector<double> hostOldBlockSums;
  std::vector<double> hostNewBlockSums;
  oldBlockSums.download(&hostOldBlockSums, stream);
  newBlockSums.download(&hostNewBlockSums, stream);
  GTSAM_CUDA_CHECK(cudaStreamSynchronize(stream));

  double oldError = 0.0;
  double newError = 0.0;
  for (double blockSum : hostOldBlockSums) oldError += blockSum;
  for (double blockSum : hostNewBlockSums) newError += blockSum;
  if (oldLinearizedError) *oldLinearizedError = oldError;
  if (newLinearizedError) *newLinearizedError = newError;
  return oldError - newError;
}

void AccumulateCudaSfmNormalEquations(
    const DeviceValues& values, const CudaSfmProjectionBatch& batch,
    int numCameras, DeviceSparseNormalEquations* system, cudaStream_t stream) {
  if (!system) {
    throw std::invalid_argument(
        "AccumulateCudaSfmNormalEquations requires output system");
  }

  if (numCameras < 0) {
    throw std::invalid_argument(
        "AccumulateCudaSfmNormalEquations requires nonnegative camera count");
  }
  if (static_cast<size_t>(numCameras) < batch.numCameras()) {
    throw std::invalid_argument(
        "AccumulateCudaSfmNormalEquations camera count smaller than batch");
  }

  const auto& cameraBlock = values.block<DevicePinholeCameraCal3Bundler>(
      kDevicePinholeCameraCal3BundlerType);
  const auto& pointBlock = values.block<DevicePoint3>(kDevicePoint3Type);

  if (batch.numCameras() > cameraBlock.values.size()) {
    throw std::invalid_argument(
        "AccumulateCudaSfmNormalEquations camera batch/value size mismatch");
  }
  if (batch.numPoints() > pointBlock.values.size()) {
    throw std::invalid_argument(
        "AccumulateCudaSfmNormalEquations point batch/value size mismatch");
  }

  const size_t maxInt = static_cast<size_t>(std::numeric_limits<int>::max());
  if (static_cast<size_t>(numCameras) > maxInt / 9 ||
      batch.numPoints() > (maxInt - 9 * static_cast<size_t>(numCameras)) / 3) {
    throw std::invalid_argument(
        "AccumulateCudaSfmNormalEquations dimension exceeds int range");
  }
  const size_t requiredRows =
      9 * static_cast<size_t>(numCameras) + 3 * batch.numPoints();
  if (system->rows() != static_cast<int>(requiredRows)) {
    throw std::invalid_argument(
        "AccumulateCudaSfmNormalEquations system row count mismatch");
  }

  const size_t numObservations = batch.numObservations();
  const size_t gridSizeSize =
      (numObservations + kProjectionLinearizationBlockSize - 1) /
      kProjectionLinearizationBlockSize;
  if (gridSizeSize > static_cast<size_t>(std::numeric_limits<int>::max())) {
    throw std::invalid_argument(
        "AccumulateCudaSfmNormalEquations grid size exceeds CUDA launch limit");
  }

  system->zero(stream);
  if (numObservations == 0) {
    return;
  }

  CudaDeviceArray<int> missingEntries(1);
  missingEntries.zero(stream);

  const int gridSize = static_cast<int>(gridSizeSize);
  const CudaSfmSqrtInfo2* sqrtInfos = SqrtInfoDataOrNull(
      batch, numObservations, "AccumulateCudaSfmNormalEquations");
  const CudaSfmRobustModel* robustModels = RobustModelDataOrNull(
      batch, numObservations, "AccumulateCudaSfmNormalEquations");
  if (batch.noiseMode() == CudaSfmProjectionNoiseMode::Robust) {
    AccumulateCudaSfmNormalEquationsKernel<true, true>
        <<<gridSize, kProjectionLinearizationBlockSize, 0, stream>>>(
            cameraBlock.values.data(), pointBlock.values.data(),
            batch.observations().data(), numObservations, sqrtInfos,
            robustModels, numCameras, system->rowPointers().data(),
            system->colIndices().data(), system->values().data(),
            system->rhs().data(), missingEntries.data());
  } else if (batch.noiseMode() == CudaSfmProjectionNoiseMode::Whitened) {
    AccumulateCudaSfmNormalEquationsKernel<true, false>
        <<<gridSize, kProjectionLinearizationBlockSize, 0, stream>>>(
            cameraBlock.values.data(), pointBlock.values.data(),
            batch.observations().data(), numObservations, sqrtInfos,
            nullptr, numCameras, system->rowPointers().data(),
            system->colIndices().data(), system->values().data(),
            system->rhs().data(), missingEntries.data());
  } else {
    AccumulateCudaSfmNormalEquationsKernel<false, false>
        <<<gridSize, kProjectionLinearizationBlockSize, 0, stream>>>(
            cameraBlock.values.data(), pointBlock.values.data(),
            batch.observations().data(), numObservations, nullptr, nullptr,
            numCameras, system->rowPointers().data(),
            system->colIndices().data(), system->values().data(),
            system->rhs().data(), missingEntries.data());
  }
  GTSAM_CUDA_CHECK(cudaGetLastError());

  std::vector<int> hostMissingEntries;
  missingEntries.download(&hostMissingEntries, stream);
  GTSAM_CUDA_CHECK(cudaStreamSynchronize(stream));
  if (!hostMissingEntries.empty() && hostMissingEntries[0] != 0) {
    throw std::runtime_error(
        "AccumulateCudaSfmNormalEquations missing CSR entries");
  }
}

}  // namespace gtsam::cuda

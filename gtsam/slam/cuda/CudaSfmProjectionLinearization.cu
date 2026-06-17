#include <gtsam/base/cuda/CudaErrors.h>
#include <gtsam/nonlinear/cuda/DeviceGeometryKernels.h>
#include <gtsam/slam/cuda/CudaSfmProjectionLinearization.h>

#include <limits>
#include <stdexcept>
#include <vector>

namespace gtsam::cuda {
namespace {

constexpr int kProjectionLinearizationBlockSize = 256;
constexpr int kProjectionTangentDim = 12;

__global__ void LinearizeCudaSfmProjectionKernel(
    const DevicePinholeCameraCal3Bundler* cameras, const DevicePoint3* points,
    const CudaSfmObservation* observations, size_t numObservations,
    double* residuals, double* cameraJacobians, double* pointJacobians) {
  const size_t i = static_cast<size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (i >= numObservations) return;

  const CudaSfmObservation observation = observations[i];
  const DeviceProjectionResult result = EvaluatePinholeBundlerProjection(
      cameras[observation.cameraSlot], points[observation.pointSlot],
      observation);

  residuals[2 * i] = result.residual[0];
  residuals[2 * i + 1] = result.residual[1];

  for (int k = 0; k < 18; ++k) {
    cameraJacobians[18 * i + k] = result.cameraJacobian[k];
  }
  for (int k = 0; k < 6; ++k) {
    pointJacobians[6 * i + k] = result.pointJacobian[k];
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

__global__ void AccumulateCudaSfmNormalEquationsKernel(
    const DevicePinholeCameraCal3Bundler* cameras, const DevicePoint3* points,
    const CudaSfmObservation* observations, size_t numObservations,
    int numCameras, const int* rowPointers, const int* colIndices,
    double* values, double* rhs) {
  const size_t i = static_cast<size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (i >= numObservations) return;

  const CudaSfmObservation observation = observations[i];
  const DeviceProjectionResult result = EvaluatePinholeBundlerProjection(
      cameras[observation.cameraSlot], points[observation.pointSlot],
      observation);

  const int cameraBase = 9 * observation.cameraSlot;
  const int pointBase = 9 * numCameras + 3 * observation.pointSlot;
  int global[kProjectionTangentDim];
  double jacobian0[kProjectionTangentDim];
  double jacobian1[kProjectionTangentDim];

  for (int col = 0; col < 9; ++col) {
    global[col] = cameraBase + col;
    jacobian0[col] = result.cameraJacobian[col];
    jacobian1[col] = result.cameraJacobian[9 + col];
  }
  for (int col = 0; col < 3; ++col) {
    global[9 + col] = pointBase + col;
    jacobian0[9 + col] = result.pointJacobian[col];
    jacobian1[9 + col] = result.pointJacobian[3 + col];
  }

  const double residual0 = result.residual[0];
  const double residual1 = result.residual[1];
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
      }
    }
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
  LinearizeCudaSfmProjectionKernel<<<gridSize, kProjectionLinearizationBlockSize,
                                     0, stream>>>(
      cameraBlock.values.data(), pointBlock.values.data(),
      batch.observations().data(), numObservations,
      linearization->residuals.data(), linearization->cameraJacobians.data(),
      linearization->pointJacobians.data());
  GTSAM_CUDA_CHECK(cudaGetLastError());
}

double ComputeCudaSfmProjectionError(const DeviceValues& values,
                                     const CudaSfmProjectionBatch& batch,
                                     cudaStream_t stream) {
  CudaSfmProjectionLinearization linearization;
  LinearizeCudaSfmProjectionBatch(values, batch, &linearization, stream);

  std::vector<double> residuals;
  linearization.residuals.download(&residuals, stream);
  GTSAM_CUDA_CHECK(cudaStreamSynchronize(stream));

  double error = 0.0;
  for (double residual : residuals) {
    error += 0.5 * residual * residual;
  }
  return error;
}

void AccumulateCudaSfmNormalEquations(
    const DeviceValues& values, const CudaSfmProjectionBatch& batch,
    int numCameras, DeviceSparseNormalEquations* system, cudaStream_t stream) {
  if (!system) {
    throw std::invalid_argument(
        "AccumulateCudaSfmNormalEquations requires output system");
  }

  system->zero(stream);

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
  if (system->rows() < static_cast<int>(requiredRows)) {
    throw std::invalid_argument(
        "AccumulateCudaSfmNormalEquations system too small");
  }

  const size_t numObservations = batch.numObservations();
  if (numObservations == 0) {
    return;
  }

  const size_t gridSizeSize =
      (numObservations + kProjectionLinearizationBlockSize - 1) /
      kProjectionLinearizationBlockSize;
  if (gridSizeSize > static_cast<size_t>(std::numeric_limits<int>::max())) {
    throw std::invalid_argument(
        "AccumulateCudaSfmNormalEquations grid size exceeds CUDA launch limit");
  }
  const int gridSize = static_cast<int>(gridSizeSize);
  AccumulateCudaSfmNormalEquationsKernel<<<
      gridSize, kProjectionLinearizationBlockSize, 0, stream>>>(
      cameraBlock.values.data(), pointBlock.values.data(),
      batch.observations().data(), numObservations, numCameras,
      system->rowPointers().data(), system->colIndices().data(),
      system->values().data(), system->rhs().data());
  GTSAM_CUDA_CHECK(cudaGetLastError());
}

}  // namespace gtsam::cuda

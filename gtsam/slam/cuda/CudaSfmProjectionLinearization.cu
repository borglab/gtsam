#include <gtsam/base/cuda/CudaErrors.h>
#include <gtsam/nonlinear/cuda/DeviceGeometryKernels.h>
#include <gtsam/slam/cuda/CudaSfmProjectionLinearization.h>

#include <stdexcept>
#include <vector>

namespace gtsam::cuda {
namespace {

constexpr int kProjectionLinearizationBlockSize = 256;

__global__ void LinearizeCudaSfmProjectionKernel(
    const DevicePinholeCameraCal3Bundler* cameras, const DevicePoint3* points,
    const CudaSfmObservation* observations, size_t numObservations,
    double* residuals, double* cameraJacobians, double* pointJacobians) {
  const size_t i = blockIdx.x * blockDim.x + threadIdx.x;
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

}  // namespace

void LinearizeCudaSfmProjectionBatch(
    const DeviceValues& values, const CudaSfmProjectionBatch& batch,
    CudaSfmProjectionLinearization* linearization, cudaStream_t stream) {
  if (!linearization) {
    throw std::invalid_argument(
        "LinearizeCudaSfmProjectionBatch requires output storage");
  }

  const size_t numObservations = batch.numObservations();
  linearization->residuals.resize(2 * numObservations);
  linearization->cameraJacobians.resize(18 * numObservations);
  linearization->pointJacobians.resize(6 * numObservations);
  if (numObservations == 0) {
    return;
  }

  const auto& cameraBlock = values.block<DevicePinholeCameraCal3Bundler>(
      kDevicePinholeCameraCal3BundlerType);
  const auto& pointBlock = values.block<DevicePoint3>(kDevicePoint3Type);

  const int gridSize =
      static_cast<int>((numObservations + kProjectionLinearizationBlockSize -
                        1) /
                       kProjectionLinearizationBlockSize);
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

}  // namespace gtsam::cuda

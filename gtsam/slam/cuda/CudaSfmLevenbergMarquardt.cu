#include <gtsam/base/cuda/CudaContext.h>
#include <gtsam/base/cuda/CudaDeviceArray.h>
#include <gtsam/base/cuda/CudaErrors.h>
#include <gtsam/nonlinear/cuda/CudssLinearSolver.h>
#include <gtsam/nonlinear/cuda/DeviceGeometryKernels.h>
#include <gtsam/slam/cuda/CudaBalCsrStructure.h>
#include <gtsam/slam/cuda/CudaSfmLevenbergMarquardt.h>
#include <gtsam/slam/cuda/CudaSfmProjectionLinearization.h>
#include <gtsam/slam/cuda/CudaSfmValues.h>

#include <algorithm>
#include <stdexcept>

namespace gtsam::cuda {
namespace {

constexpr int kApplyDeltaBlockSize = 256;

__global__ void ApplyDeltaKernel(
    const DevicePinholeCameraCal3Bundler* cameras,
    const DevicePoint3* points, int numCameras, int numPoints,
    const double* delta, DevicePinholeCameraCal3Bundler* trialCameras,
    DevicePoint3* trialPoints) {
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < numCameras) {
    trialCameras[i] = RetractCamera(cameras[i], delta + 9 * i);
  }
  if (i < numPoints) {
    const int pointOffset = 9 * numCameras + 3 * i;
    trialPoints[i] = RetractPoint(points[i], delta + pointOffset);
  }
}

void ApplyDelta(const DeviceValues& current, const CudaDeviceArray<double>& delta,
                DeviceValues* trial, int numCameras, int numPoints,
                cudaStream_t stream) {
  const int maxVariables = std::max(numCameras, numPoints);
  if (maxVariables <= 0) {
    return;
  }

  const auto& currentCameras =
      current.block<DevicePinholeCameraCal3Bundler>(
          kDevicePinholeCameraCal3BundlerType)
          .values;
  const auto& currentPoints =
      current.block<DevicePoint3>(kDevicePoint3Type).values;
  auto& trialCameras =
      trial->block<DevicePinholeCameraCal3Bundler>(
               kDevicePinholeCameraCal3BundlerType)
          .values;
  auto& trialPoints = trial->block<DevicePoint3>(kDevicePoint3Type).values;

  const int gridSize =
      (maxVariables + kApplyDeltaBlockSize - 1) / kApplyDeltaBlockSize;
  ApplyDeltaKernel<<<gridSize, kApplyDeltaBlockSize, 0, stream>>>(
      currentCameras.data(), currentPoints.data(), numCameras, numPoints,
      delta.data(), trialCameras.data(), trialPoints.data());
  GTSAM_CUDA_CHECK(cudaGetLastError());
}

void AcceptTrial(const DeviceValues& trial, DeviceValues* current,
                 cudaStream_t stream) {
  const auto& trialCameras =
      trial.block<DevicePinholeCameraCal3Bundler>(
               kDevicePinholeCameraCal3BundlerType)
          .values;
  const auto& trialPoints =
      trial.block<DevicePoint3>(kDevicePoint3Type).values;
  auto& currentCameras =
      current->block<DevicePinholeCameraCal3Bundler>(
                 kDevicePinholeCameraCal3BundlerType)
          .values;
  auto& currentPoints = current->block<DevicePoint3>(kDevicePoint3Type).values;

  currentCameras.copyFrom(trialCameras, stream);
  currentPoints.copyFrom(trialPoints, stream);
}

}  // namespace

CudaSfmLevenbergMarquardtResult OptimizeCudaSfm(
    const SfmData& data, const CudaSfmLevenbergMarquardtParams& params) {
#if !GTSAM_ENABLE_CUDSS
  (void)data;
  (void)params;
  throw std::runtime_error("OptimizeCudaSfm requires GTSAM_ENABLE_CUDSS=ON");
#else
  CudaContext context;
  DeviceValues current = PackSfmValues(data, context.stream());
  DeviceValues trial = PackSfmValues(data, context.stream());
  const CudaSfmProjectionBatch batch =
      CudaSfmProjectionBatch::FromSfmData(data, context.stream());
  const CudaBalCsrStructure structure = CudaBalCsrStructure::FromSfmData(data);

  CudaSfmLevenbergMarquardtResult result;
  double currentError =
      ComputeCudaSfmProjectionError(current, batch, context.stream());
  result.initialError = currentError;

  if (params.maxIterations <= 0 || structure.dimension() == 0) {
    result.finalError = currentError;
    result.optimizedValues = DownloadSfmValues(current, context.stream());
    return result;
  }

  DeviceSparseNormalEquations system;
  system.uploadPattern(structure.dimension(), structure.rowPointers(),
                       structure.colIndices(), context.stream());
  CudaDeviceArray<double> delta;
  CudssSpdSolver solver;
  bool solverAnalyzed = false;

  double lambda = params.initialLambda;
  const int numCameras = static_cast<int>(structure.numCameras());
  const int numPoints = static_cast<int>(structure.numPoints());

  for (int iteration = 0; iteration < params.maxIterations; ++iteration) {
    AccumulateCudaSfmNormalEquations(current, batch, numCameras, &system,
                                     context.stream());
    system.addDiagonalDamping(lambda, context.stream());
    if (!solverAnalyzed) {
      solver.analyze(system, &delta, context.stream());
      solverAnalyzed = true;
    }
    solver.solve(system, &delta, context.stream());

    ApplyDelta(current, delta, &trial, numCameras, numPoints,
               context.stream());
    const double trialError =
        ComputeCudaSfmProjectionError(trial, batch, context.stream());
    ++result.iterations;

    if (trialError < currentError) {
      const double relativeDecrease =
          (currentError - trialError) / std::max(1.0, currentError);
      AcceptTrial(trial, &current, context.stream());
      currentError = trialError;
      ++result.acceptedSteps;
      lambda *= params.lambdaDownFactor;
      if (relativeDecrease < params.relativeErrorTol) {
        break;
      }
    } else {
      lambda *= params.lambdaUpFactor;
    }
  }

  result.finalError = currentError;
  result.optimizedValues = DownloadSfmValues(current, context.stream());
  return result;
#endif
}

}  // namespace gtsam::cuda

#include <gtsam/base/cuda/CudaContext.h>
#include <gtsam/base/cuda/CudaDeviceArray.h>
#include <gtsam/base/cuda/CudaErrors.h>
#include <gtsam/nonlinear/cuda/CudssLinearSolver.h>
#include <gtsam/nonlinear/cuda/DeviceGeometryKernels.h>
#include <gtsam/slam/cuda/CudaBalCsrStructure.h>
#include <gtsam/slam/cuda/CudaSfmDenseSchurSolver.h>
#include <gtsam/slam/cuda/CudaSfmLevenbergMarquardt.h>
#include <gtsam/slam/cuda/CudaSfmProjectionLinearization.h>
#include <gtsam/slam/cuda/CudaSfmValues.h>

#include <algorithm>
#include <chrono>
#include <stdexcept>

namespace gtsam::cuda {
namespace {

constexpr int kApplyDeltaBlockSize = 256;
using Clock = std::chrono::steady_clock;

double ElapsedSeconds(Clock::time_point start, Clock::time_point end) {
  return std::chrono::duration<double>(end - start).count();
}

double ElapsedSince(Clock::time_point start) {
  return ElapsedSeconds(start, Clock::now());
}

double ElapsedSinceAfterSync(Clock::time_point start, cudaStream_t stream) {
  GTSAM_CUDA_CHECK(cudaStreamSynchronize(stream));
  return ElapsedSince(start);
}

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
  CudaSfmLevenbergMarquardtResult result;
  const auto totalStart = Clock::now();

  auto stageStart = Clock::now();
  CudaContext context(nullptr);
  result.contextElapsed = ElapsedSince(stageStart);

  // TODO(perf): Avoid duplicate host-to-device transfers here. PackSfmValues()
  // uploads values, and CudaSfmProjectionBatch::FromSfmData() uploads overlapping
  // SFM data again. Consider constructing the projection batch from existing
  // device buffers or sharing the packed representation.
  stageStart = Clock::now();
  DeviceValues current = PackSfmValues(data, context.stream());
  result.packValuesElapsed =
      ElapsedSinceAfterSync(stageStart, context.stream());

  stageStart = Clock::now();
  DeviceValues trial = AllocateSfmValuesLike(current);
  result.allocateTrialElapsed = ElapsedSince(stageStart);

  stageStart = Clock::now();
  const CudaSfmProjectionBatch batch =
      CudaSfmProjectionBatch::FromSfmData(data, context.stream());
  result.projectionBatchElapsed =
      ElapsedSinceAfterSync(stageStart, context.stream());

  const int numCameras = static_cast<int>(data.numberCameras());
  const int numPoints = static_cast<int>(data.numberTracks());
  const int totalDimension = 9 * numCameras + 3 * numPoints;

  stageStart = Clock::now();
  double currentError =
      ComputeCudaSfmProjectionError(current, batch, context.stream());
  result.initialErrorElapsed = ElapsedSince(stageStart);
  result.initialError = currentError;

  if (params.maxIterations <= 0 || totalDimension == 0) {
    result.finalError = currentError;
    if (params.downloadOptimizedValues) {
      stageStart = Clock::now();
      result.optimizedValues = DownloadSfmValues(current, context.stream());
      result.downloadElapsed = ElapsedSince(stageStart);
    }
    result.totalMeasuredElapsed = ElapsedSince(totalStart);
    return result;
  }

  DeviceSparseNormalEquations system;
  CudaDeviceArray<double> delta;
  stageStart = Clock::now();
  CudssSpdSolver solver;
  result.cudssSolverConstructionElapsed = ElapsedSince(stageStart);
  stageStart = Clock::now();
  CudaSfmDenseSchurSolver denseSchurSolver;
  result.denseSchurSolverConstructionElapsed = ElapsedSince(stageStart);
  bool solverAnalyzed = false;
  if (params.linearSolver == CudaSfmLinearSolverType::CudssFullNormal) {
    stageStart = Clock::now();
    const CudaBalCsrStructure structure = CudaBalCsrStructure::FromSfmData(data);
    result.csrStructureElapsed = ElapsedSince(stageStart);

    stageStart = Clock::now();
    system.uploadPattern(structure.dimension(), structure.rowPointers(),
                         structure.colIndices(), context.stream());
    result.uploadPatternElapsed =
        ElapsedSinceAfterSync(stageStart, context.stream());
  }

  double lambda = params.initialLambda;

  result.setupElapsed = ElapsedSince(totalStart);
  const auto solveLoopStart = std::chrono::steady_clock::now();
  for (int iteration = 0; iteration < params.maxIterations; ++iteration) {
    if (params.linearSolver == CudaSfmLinearSolverType::DenseSchur) {
      denseSchurSolver.solve(current, batch, numCameras, lambda, &delta,
                             context.stream());
    } else {
      AccumulateCudaSfmNormalEquations(current, batch, numCameras, &system,
                                       context.stream());
      system.addDiagonalDamping(lambda, context.stream());
      if (!solverAnalyzed) {
        stageStart = Clock::now();
        solver.analyze(system, &delta, context.stream());
        result.firstCudssAnalyzeElapsed =
            ElapsedSinceAfterSync(stageStart, context.stream());
        solverAnalyzed = true;
      }
      solver.solve(system, &delta, context.stream());
    }

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
  GTSAM_CUDA_CHECK(cudaStreamSynchronize(context.stream()));
  const auto solveLoopEnd = std::chrono::steady_clock::now();
  result.solveLoopElapsed =
      std::chrono::duration<double>(solveLoopEnd - solveLoopStart).count();

  result.finalError = currentError;
  if (params.downloadOptimizedValues) {
    stageStart = Clock::now();
    result.optimizedValues = DownloadSfmValues(current, context.stream());
    result.downloadElapsed = ElapsedSince(stageStart);
  }
  result.totalMeasuredElapsed = ElapsedSince(totalStart);
  return result;
#endif
}

}  // namespace gtsam::cuda

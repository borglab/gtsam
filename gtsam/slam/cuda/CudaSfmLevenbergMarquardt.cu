#include <gtsam/base/cuda/CudaContext.h>
#include <gtsam/base/cuda/CudaDeviceArray.h>
#include <gtsam/base/cuda/CudaErrors.h>
#include <gtsam/nonlinear/internal/NonlinearOptimizerState.h>
#include <gtsam/nonlinear/cuda/CudssLinearSolver.h>
#include <gtsam/nonlinear/cuda/DeviceGeometryKernels.h>
#include <gtsam/slam/GeneralSFMFactor.h>
#include <gtsam/slam/cuda/CudaBalCsrStructure.h>
#include <gtsam/slam/cuda/CudaSfmDenseSchurSolver.h>
#include <gtsam/slam/cuda/CudaSfmLevenbergMarquardt.h>
#include <gtsam/slam/cuda/CudaSfmProjectionLinearization.h>
#include <gtsam/slam/cuda/CudaSfmValues.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>

namespace gtsam::cuda {
namespace {

constexpr int kApplyDeltaBlockSize = 256;
using Clock = std::chrono::steady_clock;
using BundlerProjectionFactor = GeneralSFMFactor<SfmCamera, Point3>;

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

std::vector<Key> DefaultCameraKeys(const SfmData& data) {
  std::vector<Key> keys;
  keys.reserve(data.numberCameras());
  for (size_t i = 0; i < data.numberCameras(); ++i) {
    keys.push_back(symbol_shorthand::C(i));
  }
  return keys;
}

std::vector<Key> DefaultPointKeys(const SfmData& data) {
  std::vector<Key> keys;
  keys.reserve(data.numberTracks());
  for (size_t i = 0; i < data.numberTracks(); ++i) {
    keys.push_back(symbol_shorthand::P(i));
  }
  return keys;
}

CudaSfmLevenbergMarquardtParams ConvertLmParams(
    const LevenbergMarquardtParams& params) {
  CudaSfmLevenbergMarquardtParams cudaParams;
  cudaParams.maxIterations = params.maxIterations;
  cudaParams.initialLambda = params.lambdaInitial;
  cudaParams.lambdaUpFactor = params.lambdaFactor;
  cudaParams.lambdaDownFactor =
      params.lambdaFactor == 0.0 ? 1.0 : 1.0 / params.lambdaFactor;
  cudaParams.lambdaUpperBound = params.lambdaUpperBound;
  cudaParams.lambdaLowerBound = params.lambdaLowerBound;
  cudaParams.relativeErrorTol = params.relativeErrorTol;
  cudaParams.absoluteErrorTol = params.absoluteErrorTol;
  cudaParams.errorTol = params.errorTol;
  cudaParams.minModelFidelity = params.minModelFidelity;
  cudaParams.useFixedLambdaFactor = params.useFixedLambdaFactor;
  cudaParams.diagonalDamping = params.dampingParams.diagonalDamping;
  cudaParams.minDiagonal = params.dampingParams.minDiagonal;
  cudaParams.maxDiagonal = params.dampingParams.maxDiagonal;
  cudaParams.downloadOptimizedValues = true;
  return cudaParams;
}

void IncreaseLambda(const CudaSfmLevenbergMarquardtParams& params,
                    double* lambda, double* currentFactor) {
  *lambda *= *currentFactor;
  if (!params.useFixedLambdaFactor) {
    *currentFactor *= 2.0;
  }
}

void DecreaseLambda(const CudaSfmLevenbergMarquardtParams& params,
                    double modelFidelity, double* lambda,
                    double* currentFactor) {
  if (params.useFixedLambdaFactor) {
    *lambda *= params.lambdaDownFactor;
  } else {
    *lambda *=
        std::max(1.0 / 3.0, 1.0 - std::pow(2.0 * modelFidelity - 1.0, 3));
    *currentFactor *= 2.0;
  }
  *lambda = std::max(params.lambdaLowerBound, *lambda);
}

bool CheckCudaLmConvergence(const CudaSfmLevenbergMarquardtParams& params,
                            double currentError, double newError) {
  if (newError <= params.errorTol) {
    return true;
  }
  const double absoluteDecrease = currentError - newError;
  const double relativeDecrease =
      currentError != 0.0 ? absoluteDecrease / currentError : 0.0;
  return (params.relativeErrorTol != 0.0 &&
          relativeDecrease <= params.relativeErrorTol) ||
         (absoluteDecrease <= params.absoluteErrorTol);
}

}  // namespace

CudaSfmFactorGraphData ConvertGeneralSfmGraphToCudaSfmData(
    const NonlinearFactorGraph& graph, const Values& initialValues) {
  CudaSfmFactorGraphData converted;

  std::map<Key, size_t> cameraSlots;
  for (const auto& keyCamera : initialValues.extract<SfmCamera>()) {
    cameraSlots.emplace(keyCamera.first, converted.cameraKeys.size());
    converted.cameraKeys.push_back(keyCamera.first);
    converted.data.cameras.push_back(keyCamera.second);
  }

  std::map<Key, size_t> pointSlots;
  for (const auto& keyPoint : initialValues.extract<Point3>()) {
    pointSlots.emplace(keyPoint.first, converted.pointKeys.size());
    converted.pointKeys.push_back(keyPoint.first);
    converted.data.tracks.emplace_back(keyPoint.second);
  }

  for (const auto& factor : graph) {
    if (!factor) {
      continue;
    }

    const auto sfmFactor =
        std::dynamic_pointer_cast<BundlerProjectionFactor>(factor);
    if (!sfmFactor) {
      throw std::invalid_argument(
          "CUDA SFM conversion only supports GeneralSFMFactor<SfmCamera, "
          "Point3>");
    }

    const SharedNoiseModel& model = sfmFactor->noiseModel();
    if (model && !model->isUnit()) {
      throw std::invalid_argument(
          "CUDA SFM conversion currently requires unit-noise projection "
          "factors");
    }

    const auto cameraSlot = cameraSlots.find(sfmFactor->key1());
    if (cameraSlot == cameraSlots.end()) {
      throw std::invalid_argument(
          "CUDA SFM conversion found a factor camera key missing from Values");
    }

    const auto pointSlot = pointSlots.find(sfmFactor->key2());
    if (pointSlot == pointSlots.end()) {
      throw std::invalid_argument(
          "CUDA SFM conversion found a factor point key missing from Values");
    }

    converted.data.tracks[pointSlot->second].measurements.emplace_back(
        cameraSlot->second, sfmFactor->measured());
  }

  return converted;
}

CudaSfmLevenbergMarquardtOptimizer::CudaSfmLevenbergMarquardtOptimizer(
    const NonlinearFactorGraph& graph, const Values& initialValues,
    const LevenbergMarquardtParams& params)
    : NonlinearOptimizer(
          graph, std::unique_ptr<gtsam::internal::NonlinearOptimizerState>(
                     new gtsam::internal::NonlinearOptimizerState(
                         initialValues, graph.error(initialValues)))),
      params_(params),
      cudaParams_(ConvertLmParams(params)) {}

CudaSfmLevenbergMarquardtOptimizer::CudaSfmLevenbergMarquardtOptimizer(
    const NonlinearFactorGraph& graph, const Values& initialValues,
    const LevenbergMarquardtParams& params,
    CudaSfmLinearSolverType linearSolver)
    : CudaSfmLevenbergMarquardtOptimizer(graph, initialValues, params) {
  cudaParams_.linearSolver = linearSolver;
}

const Values& CudaSfmLevenbergMarquardtOptimizer::optimize() {
  const CudaSfmFactorGraphData converted =
      ConvertGeneralSfmGraphToCudaSfmData(graph(), values());
  result_ = OptimizeCudaSfm(converted.data, converted.cameraKeys,
                            converted.pointKeys, cudaParams_);

  Values merged(values());
  for (Key key : result_.optimizedValues.keys()) {
    merged.update(key, result_.optimizedValues.at(key));
  }
  const double newError = graph().error(merged);
  state_ = std::unique_ptr<gtsam::internal::NonlinearOptimizerState>(
      new gtsam::internal::NonlinearOptimizerState(std::move(merged), newError,
                                                   result_.iterations));
  return values();
}

GaussianFactorGraph::shared_ptr CudaSfmLevenbergMarquardtOptimizer::iterate() {
  throw std::runtime_error(
      "CudaSfmLevenbergMarquardtOptimizer::iterate is not implemented; use "
      "optimize()");
}

CudaSfmLevenbergMarquardtResult OptimizeCudaSfm(
    const SfmData& data, const CudaSfmLevenbergMarquardtParams& params) {
  return OptimizeCudaSfm(data, DefaultCameraKeys(data), DefaultPointKeys(data),
                         params);
}

CudaSfmLevenbergMarquardtResult OptimizeCudaSfm(
    const SfmData& data, const std::vector<Key>& cameraKeys,
    const std::vector<Key>& pointKeys,
    const CudaSfmLevenbergMarquardtParams& params) {
#if !GTSAM_ENABLE_CUDSS
  (void)data;
  (void)cameraKeys;
  (void)pointKeys;
  (void)params;
  throw std::runtime_error("OptimizeCudaSfm requires GTSAM_ENABLE_CUDSS=ON");
#else
  if (cameraKeys.size() != data.numberCameras()) {
    throw std::invalid_argument(
        "OptimizeCudaSfm camera key count does not match SfmData");
  }
  if (pointKeys.size() != data.numberTracks()) {
    throw std::invalid_argument(
        "OptimizeCudaSfm point key count does not match SfmData");
  }

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
  DeviceValues current =
      PackSfmValues(data, cameraKeys, pointKeys, context.stream());
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

  if (params.maxIterations <= 0 || totalDimension == 0 ||
      currentError <= params.errorTol) {
    result.finalError = currentError;
    result.finalLambda = params.initialLambda;
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
  double currentFactor = params.lambdaUpFactor;
  CudaDeviceArray<double> dampingDiagonal;

  result.setupElapsed = ElapsedSince(totalStart);
  const auto solveLoopStart = std::chrono::steady_clock::now();
  bool terminate = false;
  while (result.iterations < params.maxIterations && std::isfinite(currentError) &&
         !terminate) {
    if (params.diagonalDamping) {
      ComputeCudaSfmHessianDiagonal(current, batch, numCameras,
                                    params.minDiagonal, params.maxDiagonal,
                                    &dampingDiagonal, context.stream());
    }

    bool acceptedOrDone = false;
    while (!acceptedOrDone) {
      if (params.linearSolver == CudaSfmLinearSolverType::DenseSchur) {
        if (params.diagonalDamping) {
          denseSchurSolver.solve(current, batch, numCameras, lambda,
                                 dampingDiagonal, &delta, context.stream());
        } else {
          denseSchurSolver.solve(current, batch, numCameras, lambda, &delta,
                                 context.stream());
        }
      } else {
        AccumulateCudaSfmNormalEquations(current, batch, numCameras, &system,
                                         context.stream());
        if (params.diagonalDamping) {
          system.addDiagonalDamping(lambda, dampingDiagonal, context.stream());
        } else {
          system.addDiagonalDamping(lambda, context.stream());
        }
        if (!solverAnalyzed) {
          stageStart = Clock::now();
          solver.analyze(system, &delta, context.stream());
          result.firstCudssAnalyzeElapsed =
              ElapsedSinceAfterSync(stageStart, context.stream());
          solverAnalyzed = true;
        }
        solver.solve(system, &delta, context.stream());
      }
      ++result.innerIterations;

      double oldLinearizedError = 0.0;
      double newLinearizedError = 0.0;
      const double linearizedCostChange =
          ComputeCudaSfmLinearizedErrorChange(
              current, batch, numCameras, delta, &oldLinearizedError,
              &newLinearizedError, context.stream());

      double trialError = std::numeric_limits<double>::infinity();
      double costChange = 0.0;
      double modelFidelity = 0.0;
      bool stepSuccessful = false;
      bool stopSearchingLambda = false;

      if (linearizedCostChange >= 0.0) {
        ApplyDelta(current, delta, &trial, numCameras, numPoints,
                   context.stream());
        trialError =
            ComputeCudaSfmProjectionError(trial, batch, context.stream());
        costChange = currentError - trialError;

        if (linearizedCostChange >
            std::numeric_limits<double>::epsilon() * oldLinearizedError) {
          modelFidelity = costChange / linearizedCostChange;
          stepSuccessful = modelFidelity > params.minModelFidelity;
        }

        const double minAbsoluteTolerance =
            params.relativeErrorTol * currentError;
        if (std::abs(costChange) < minAbsoluteTolerance) {
          stopSearchingLambda = true;
        }
      }

      if (stepSuccessful) {
        const double previousError = currentError;
        AcceptTrial(trial, &current, context.stream());
        currentError = trialError;
        ++result.iterations;
        ++result.acceptedSteps;
        DecreaseLambda(params, modelFidelity, &lambda, &currentFactor);
        acceptedOrDone = true;
        terminate =
            CheckCudaLmConvergence(params, previousError, currentError);
      } else if (!stopSearchingLambda) {
        IncreaseLambda(params, &lambda, &currentFactor);
        if (lambda >= params.lambdaUpperBound) {
          acceptedOrDone = true;
          terminate = true;
        }
      } else {
        acceptedOrDone = true;
        terminate = true;
      }
    }
  }
  GTSAM_CUDA_CHECK(cudaStreamSynchronize(context.stream()));
  const auto solveLoopEnd = std::chrono::steady_clock::now();
  result.solveLoopElapsed =
      std::chrono::duration<double>(solveLoopEnd - solveLoopStart).count();

  result.finalError = currentError;
  result.finalLambda = lambda;
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

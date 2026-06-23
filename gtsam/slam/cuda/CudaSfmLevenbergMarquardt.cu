#include <gtsam/base/cuda/CudaContext.h>
#include <gtsam/base/cuda/CudaDeviceArray.h>
#include <gtsam/base/cuda/CudaErrors.h>
#include <gtsam/linear/NoiseModel.h>
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
#include <cctype>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <iostream>
#include <stdexcept>
#include <unordered_map>

namespace gtsam::cuda {
namespace {

constexpr int kApplyDeltaBlockSize = 256;
using Clock = std::chrono::steady_clock;
using BundlerProjectionFactor = GeneralSFMFactor<SfmCamera, Point3>;

struct CudaSfmLmExecutionOptions {
  bool downloadOptimizedValues = true;
};

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

std::string NormalizeLinearSolverName(const std::string& solver) {
  std::string normalized;
  normalized.reserve(solver.size());
  for (const unsigned char c : solver) {
    normalized.push_back(
        c == '_' ? '-' : static_cast<char>(std::tolower(c)));
  }
  return normalized;
}

NonlinearOptimizerParams BaseParamsAdapter(
    const CudaSfmLevenbergMarquardtParams& params) {
  NonlinearOptimizerParams baseParams;
  baseParams.maxIterations = params.maxIterations;
  baseParams.relativeErrorTol = params.relativeErrorTol;
  baseParams.absoluteErrorTol = params.absoluteErrorTol;
  baseParams.errorTol = params.errorTol;
  return baseParams;
}

CudaSfmSqrtInfo2 ExtractProjectionSqrtInfo(
    const SharedNoiseModel& model, bool* isNonUnit) {
  if (isNonUnit) {
    *isNonUnit = false;
  }
  if (!model || model->isUnit()) {
    return CudaSfmProjectionBatch::UnitSqrtInfo();
  }
  if (model->isConstrained()) {
    throw std::invalid_argument(
        "CUDA SFM conversion does not support constrained projection noise "
        "models");
  }
  if (model->dim() != 2) {
    throw std::invalid_argument(
        "CUDA SFM conversion requires 2D projection noise models");
  }

  const SharedGaussian gaussian =
      std::dynamic_pointer_cast<noiseModel::Gaussian>(model);
  if (!gaussian) {
    throw std::invalid_argument(
        "CUDA SFM conversion only supports fixed Gaussian projection noise "
        "models");
  }

  const Matrix R = gaussian->R();
  if (R.rows() != 2 || R.cols() != 2) {
    throw std::invalid_argument(
        "CUDA SFM conversion requires 2D Gaussian sqrt-information");
  }
  for (int row = 0; row < 2; ++row) {
    for (int col = 0; col < 2; ++col) {
      if (!std::isfinite(R(row, col))) {
        throw std::invalid_argument(
            "CUDA SFM conversion found non-finite Gaussian "
            "sqrt-information");
      }
    }
  }
  if (std::abs(R(1, 0)) > 1e-12) {
    throw std::invalid_argument(
        "CUDA SFM conversion requires upper-triangular Gaussian "
        "sqrt-information");
  }

  const CudaSfmSqrtInfo2 sqrtInfo{R(0, 0), R(0, 1), R(1, 1)};
  if (!CudaSfmProjectionBatch::IsUsableSqrtInfo(sqrtInfo)) {
    throw std::invalid_argument(
        "CUDA SFM conversion found invalid Gaussian sqrt-information");
  }
  if (isNonUnit) {
    *isNonUnit = !CudaSfmProjectionBatch::IsUnitSqrtInfo(sqrtInfo);
  }
  return sqrtInfo;
}

CudaSfmRobustReweightScheme ExtractRobustReweightScheme(
    noiseModel::mEstimator::Base::ReweightScheme scheme) {
  switch (scheme) {
    case noiseModel::mEstimator::Base::Scalar:
      return CudaSfmRobustReweightScheme::Scalar;
    case noiseModel::mEstimator::Base::Block:
      return CudaSfmRobustReweightScheme::Block;
  }
  throw std::invalid_argument(
      "CUDA SFM conversion found unknown robust reweight scheme");
}

CudaSfmRobustModel ExtractProjectionRobustModel(
    const noiseModel::mEstimator::Base::shared_ptr& model) {
  if (!model) {
    throw std::invalid_argument(
        "CUDA SFM conversion found robust noise without an m-estimator");
  }

  const CudaSfmRobustReweightScheme reweightScheme =
      ExtractRobustReweightScheme(model->reweightScheme());

  if (const auto huber =
          std::dynamic_pointer_cast<noiseModel::mEstimator::Huber>(model)) {
    return CudaSfmRobustModel{CudaSfmRobustModelKind::Huber, reweightScheme,
                              huber->modelParameter()};
  }
  if (const auto tukey =
          std::dynamic_pointer_cast<noiseModel::mEstimator::Tukey>(model)) {
    return CudaSfmRobustModel{CudaSfmRobustModelKind::Tukey, reweightScheme,
                              tukey->modelParameter()};
  }

  throw std::invalid_argument(
      "CUDA SFM conversion only supports Huber and Tukey robust projection "
      "noise models");
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
    *lambda *= params.lambdaFactor == 0.0 ? 1.0 : 1.0 / params.lambdaFactor;
  } else {
    *lambda *=
        std::max(1.0 / 3.0, 1.0 - std::pow(2.0 * modelFidelity - 1.0, 3));
    *currentFactor *= 2.0;
  }
  *lambda = std::max(params.lambdaLowerBound, *lambda);
}

bool CheckCudaLmConvergence(
    const CudaSfmLevenbergMarquardtParams& params,
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

CudaSfmLevenbergMarquardtParams::CudaSfmLevenbergMarquardtParams()
    : maxIterations(100),
      lambdaInitial(1e-5),
      lambdaFactor(10.0),
      lambdaUpperBound(1e5),
      lambdaLowerBound(0.0),
      relativeErrorTol(1e-5),
      absoluteErrorTol(1e-5),
      errorTol(0.0),
      minModelFidelity(1e-3),
      useFixedLambdaFactor(true),
      diagonalDamping(false),
      minDiagonal(1e-6),
      maxDiagonal(1e32),
      linearSolver(CudaSfmLinearSolverType::DenseSchur) {}

CudaSfmLevenbergMarquardtParams
CudaSfmLevenbergMarquardtParams::LegacyDefaults() {
  CudaSfmLevenbergMarquardtParams p;
  return p;
}

CudaSfmLevenbergMarquardtParams
CudaSfmLevenbergMarquardtParams::CeresDefaults() {
  CudaSfmLevenbergMarquardtParams p;
  p.maxIterations = 50;
  p.absoluteErrorTol = 0.0;
  p.relativeErrorTol = 1e-6;
  p.errorTol = 0.0;
  p.lambdaUpperBound = 1e32;
  p.lambdaLowerBound = 1e-16;
  p.lambdaInitial = 1e-4;
  p.lambdaFactor = 2.0;
  p.minModelFidelity = 1e-3;
  p.useFixedLambdaFactor = false;
  p.diagonalDamping = true;
  p.minDiagonal = 1e-6;
  p.maxDiagonal = 1e32;
  p.linearSolver = CudaSfmLinearSolverType::DenseSchur;
  return p;
}

std::string CudaSfmLevenbergMarquardtParams::getLinearSolver() const {
  switch (linearSolver) {
    case CudaSfmLinearSolverType::DenseSchur:
      return "dense-schur";
    case CudaSfmLinearSolverType::CudssFullNormal:
      return "cudss-full-normal";
  }
  throw std::invalid_argument("Unknown CUDA SFM linear solver type");
}

void CudaSfmLevenbergMarquardtParams::setLinearSolver(
    const std::string& solver) {
  const std::string normalized = NormalizeLinearSolverName(solver);
  if (normalized == "dense-schur") {
    linearSolver = CudaSfmLinearSolverType::DenseSchur;
    return;
  }
  if (normalized == "cudss-full-normal") {
    linearSolver = CudaSfmLinearSolverType::CudssFullNormal;
    return;
  }
  throw std::invalid_argument("Unknown CUDA SFM linear solver: " + solver);
}

void CudaSfmLevenbergMarquardtParams::print(const std::string& str) const {
  std::cout << str << "\n";
  std::cout << "              maxIterations: " << maxIterations << "\n";
  std::cout << "               lambdaInitial: " << lambdaInitial << "\n";
  std::cout << "                lambdaFactor: " << lambdaFactor << "\n";
  std::cout << "            lambdaUpperBound: " << lambdaUpperBound << "\n";
  std::cout << "            lambdaLowerBound: " << lambdaLowerBound << "\n";
  std::cout << "          relativeErrorTol: " << relativeErrorTol << "\n";
  std::cout << "          absoluteErrorTol: " << absoluteErrorTol << "\n";
  std::cout << "                   errorTol: " << errorTol << "\n";
  std::cout << "           minModelFidelity: " << minModelFidelity << "\n";
  std::cout << "       useFixedLambdaFactor: " << useFixedLambdaFactor << "\n";
  std::cout << "            diagonalDamping: " << diagonalDamping << "\n";
  std::cout << "                minDiagonal: " << minDiagonal << "\n";
  std::cout << "                maxDiagonal: " << maxDiagonal << "\n";
  std::cout << "               linearSolver: " << getLinearSolver() << "\n";
}

CudaSfmFactorGraphData ConvertGeneralSfmGraphToCudaSfmData(
    const NonlinearFactorGraph& graph, const Values& initialValues) {
  CudaSfmFactorGraphData converted;

  const auto cameraValues = initialValues.extract<SfmCamera>();
  const auto pointValues = initialValues.extract<Point3>();

  converted.cameraKeys.reserve(cameraValues.size());
  converted.data.cameras.reserve(cameraValues.size());
  std::unordered_map<Key, size_t> cameraSlots;
  cameraSlots.reserve(cameraValues.size());
  for (const auto& keyCamera : cameraValues) {
    cameraSlots.emplace(keyCamera.first, converted.cameraKeys.size());
    converted.cameraKeys.push_back(keyCamera.first);
    converted.data.cameras.push_back(keyCamera.second);
  }

  converted.pointKeys.reserve(pointValues.size());
  converted.data.tracks.reserve(pointValues.size());
  std::unordered_map<Key, size_t> pointSlots;
  pointSlots.reserve(pointValues.size());
  for (const auto& keyPoint : pointValues) {
    pointSlots.emplace(keyPoint.first, converted.pointKeys.size());
    converted.pointKeys.push_back(keyPoint.first);
    converted.data.tracks.emplace_back(keyPoint.second);
  }
  converted.sqrtInfoByTrack.resize(converted.data.numberTracks());

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

    SharedNoiseModel model = sfmFactor->noiseModel();
    CudaSfmRobustModel robustModel =
        CudaSfmProjectionBatch::NoRobustModel();
    if (model && !model->isUnit()) {
      if (const auto robust =
              std::dynamic_pointer_cast<noiseModel::Robust>(model)) {
        if (!converted.hasRobustNoise) {
          converted.hasRobustNoise = true;
          converted.robustModelsByTrack.resize(converted.data.numberTracks());
          for (size_t trackIndex = 0; trackIndex < converted.data.numberTracks();
               ++trackIndex) {
            converted.robustModelsByTrack[trackIndex].assign(
                converted.data.track(trackIndex).numberMeasurements(),
                CudaSfmProjectionBatch::NoRobustModel());
          }
        }
      robustModel = ExtractProjectionRobustModel(robust->robust());
      model = robust->noise();
      }
    }
    bool factorHasNonUnitNoise = false;
    const CudaSfmSqrtInfo2 sqrtInfo =
        ExtractProjectionSqrtInfo(model, &factorHasNonUnitNoise);
    converted.hasNonUnitNoise =
        converted.hasNonUnitNoise || factorHasNonUnitNoise;

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
    converted.sqrtInfoByTrack[pointSlot->second].push_back(sqrtInfo);
    if (converted.hasRobustNoise) {
      converted.robustModelsByTrack[pointSlot->second].push_back(robustModel);
    }
  }

  return converted;
}

CudaSfmLevenbergMarquardtResult OptimizeCudaSfmImpl(
    const SfmData& data, const std::vector<Key>& cameraKeys,
    const std::vector<Key>& pointKeys,
    const CudaSfmLevenbergMarquardtParams& params,
    const std::vector<std::vector<CudaSfmSqrtInfo2>>* sqrtInfoByTrack,
    const std::vector<std::vector<CudaSfmRobustModel>>* robustModelsByTrack,
    CudaSfmLmExecutionOptions executionOptions);

CudaSfmLevenbergMarquardtOptimizer::CudaSfmLevenbergMarquardtOptimizer(
    const NonlinearFactorGraph& graph, const Values& initialValues,
    const CudaSfmLevenbergMarquardtParams& params)
    : NonlinearOptimizer(
          graph, std::unique_ptr<gtsam::internal::NonlinearOptimizerState>(
                     new gtsam::internal::NonlinearOptimizerState(
                         initialValues,
                         std::numeric_limits<double>::quiet_NaN()))),
      params_(params),
      baseParams_(BaseParamsAdapter(params)) {}

const Values& CudaSfmLevenbergMarquardtOptimizer::optimize() {
  const CudaSfmFactorGraphData converted =
      ConvertGeneralSfmGraphToCudaSfmData(graph(), values());
  result_ = OptimizeCudaSfmImpl(
      converted.data, converted.cameraKeys, converted.pointKeys, params_,
      (converted.hasNonUnitNoise || converted.hasRobustNoise)
          ? &converted.sqrtInfoByTrack
          : nullptr,
      converted.hasRobustNoise ? &converted.robustModelsByTrack : nullptr,
      CudaSfmLmExecutionOptions{true});

  Values merged(values());
  for (Key key : result_.optimizedValues.keys()) {
    merged.update(key, result_.optimizedValues.at(key));
  }
  state_ = std::unique_ptr<gtsam::internal::NonlinearOptimizerState>(
      new gtsam::internal::NonlinearOptimizerState(
          std::move(merged), result_.finalError, result_.iterations));
  return values();
}

GaussianFactorGraph::shared_ptr CudaSfmLevenbergMarquardtOptimizer::iterate() {
  throw std::runtime_error(
      "CudaSfmLevenbergMarquardtOptimizer::iterate is not implemented; use "
      "optimize()");
}

CudaSfmLevenbergMarquardtResult OptimizeCudaSfm(
    const SfmData& data,
    const CudaSfmLevenbergMarquardtParams& params) {
  return OptimizeCudaSfm(data, DefaultCameraKeys(data), DefaultPointKeys(data),
                         params);
}

CudaSfmLevenbergMarquardtResult OptimizeCudaSfmWithoutValueDownload(
    const SfmData& data,
    const CudaSfmLevenbergMarquardtParams& params) {
  return OptimizeCudaSfmWithoutValueDownload(
      data, DefaultCameraKeys(data), DefaultPointKeys(data), params);
}

CudaSfmLevenbergMarquardtResult OptimizeCudaSfm(
    const SfmData& data, const std::vector<Key>& cameraKeys,
    const std::vector<Key>& pointKeys,
    const CudaSfmLevenbergMarquardtParams& params) {
  return OptimizeCudaSfmImpl(data, cameraKeys, pointKeys, params,
                             nullptr, nullptr,
                             CudaSfmLmExecutionOptions{true});
}

CudaSfmLevenbergMarquardtResult OptimizeCudaSfmWithoutValueDownload(
    const SfmData& data, const std::vector<Key>& cameraKeys,
    const std::vector<Key>& pointKeys,
    const CudaSfmLevenbergMarquardtParams& params) {
  return OptimizeCudaSfmImpl(data, cameraKeys, pointKeys, params,
                             nullptr, nullptr,
                             CudaSfmLmExecutionOptions{false});
}

CudaSfmLevenbergMarquardtResult OptimizeCudaSfmImpl(
    const SfmData& data, const std::vector<Key>& cameraKeys,
    const std::vector<Key>& pointKeys,
    const CudaSfmLevenbergMarquardtParams& params,
    const std::vector<std::vector<CudaSfmSqrtInfo2>>* sqrtInfoByTrack,
    const std::vector<std::vector<CudaSfmRobustModel>>* robustModelsByTrack,
    CudaSfmLmExecutionOptions executionOptions) {
#if !GTSAM_ENABLE_CUDSS
  (void)data;
  (void)cameraKeys;
  (void)pointKeys;
  (void)params;
  (void)sqrtInfoByTrack;
  (void)robustModelsByTrack;
  (void)executionOptions;
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
  if (robustModelsByTrack && !sqrtInfoByTrack) {
    throw std::invalid_argument(
        "OptimizeCudaSfm robust noise requires projection sqrt-info");
  }
  const CudaSfmProjectionBatch batch =
      robustModelsByTrack
          ? CudaSfmProjectionBatch::FromSfmData(data, *sqrtInfoByTrack,
                                               *robustModelsByTrack,
                                               context.stream())
      : sqrtInfoByTrack
          ? CudaSfmProjectionBatch::FromSfmData(data, *sqrtInfoByTrack,
                                               context.stream())
          : CudaSfmProjectionBatch::FromSfmData(data, context.stream());
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
    result.finalLambda = params.lambdaInitial;
    if (executionOptions.downloadOptimizedValues) {
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

  double lambda = params.lambdaInitial;
  double currentFactor = params.lambdaFactor;
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
  if (executionOptions.downloadOptimizedValues) {
    stageStart = Clock::now();
    result.optimizedValues = DownloadSfmValues(current, context.stream());
    result.downloadElapsed = ElapsedSince(stageStart);
  }
  result.totalMeasuredElapsed = ElapsedSince(totalStart);
  return result;
#endif
}

}  // namespace gtsam::cuda

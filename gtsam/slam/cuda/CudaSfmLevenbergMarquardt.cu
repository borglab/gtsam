#include <gtsam/base/cuda/CudaContext.h>
#include <gtsam/base/cuda/CudaDeviceArray.h>
#include <gtsam/base/cuda/CudaErrors.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/BatchFactor.h>
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
using BundlerProjectionBatchFactor = BatchFactor<BundlerProjectionFactor, 2>;

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

void AddH2dTransfer(const CudaDeviceTransferSummary& transfer,
                    CudaSfmLevenbergMarquardtResult* result) {
  result->totalH2dBytes += transfer.bytes;
  result->totalH2dCopyElapsed += transfer.copyElapsed;
}

void SetDownloadTransferProfile(
    const CudaSfmValuesDownloadProfile& profile,
    CudaSfmLevenbergMarquardtResult* result) {
  result->downloadHostAllocElapsed = profile.hostAllocElapsed;
  result->downloadD2hCopyElapsed = profile.d2h.copyElapsed;
  result->downloadValuesBuildElapsed = profile.hostBuildElapsed;
  result->downloadD2hBytes = profile.d2h.bytes;
  result->totalD2hCopyElapsed = profile.d2h.copyElapsed;
  result->totalD2hBytes = profile.d2h.bytes;
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

void EnsureRobustModelStorage(CudaSfmFactorGraphData* converted) {
  if (converted->hasRobustNoise) {
    return;
  }
  converted->hasRobustNoise = true;
  converted->robustModelsByTrack.resize(converted->data.numberTracks());
  for (size_t trackIndex = 0; trackIndex < converted->data.numberTracks();
       ++trackIndex) {
    converted->robustModelsByTrack[trackIndex].assign(
        converted->data.track(trackIndex).numberMeasurements(),
        CudaSfmProjectionBatch::NoRobustModel());
  }
}

void AppendProjectionFactorWithSlotsToCudaSfmData(
    const BundlerProjectionFactor& sfmFactor, size_t cameraSlot,
    size_t pointSlot, CudaSfmFactorGraphData* converted) {
  SharedNoiseModel model = sfmFactor.noiseModel();
  CudaSfmRobustModel robustModel =
      CudaSfmProjectionBatch::NoRobustModel();
  if (model && !model->isUnit()) {
    if (const auto robust =
            std::dynamic_pointer_cast<noiseModel::Robust>(model)) {
      EnsureRobustModelStorage(converted);
      robustModel = ExtractProjectionRobustModel(robust->robust());
      model = robust->noise();
    }
  }
  bool factorHasNonUnitNoise = false;
  const CudaSfmSqrtInfo2 sqrtInfo =
      ExtractProjectionSqrtInfo(model, &factorHasNonUnitNoise);
  converted->hasNonUnitNoise =
      converted->hasNonUnitNoise || factorHasNonUnitNoise;

  converted->data.tracks[pointSlot].measurements.emplace_back(
      cameraSlot, sfmFactor.measured());
  converted->sqrtInfoByTrack[pointSlot].push_back(sqrtInfo);
  if (converted->hasRobustNoise) {
    converted->robustModelsByTrack[pointSlot].push_back(robustModel);
  }
}

size_t FindRequiredSlot(const std::unordered_map<Key, size_t>& slots, Key key,
                        const char* description) {
  const auto slot = slots.find(key);
  if (slot == slots.end()) {
    throw std::invalid_argument(description);
  }
  return slot->second;
}

void AppendProjectionFactorToCudaSfmData(
    const BundlerProjectionFactor& sfmFactor,
    const std::unordered_map<Key, size_t>& cameraSlots,
    const std::unordered_map<Key, size_t>& pointSlots,
    CudaSfmFactorGraphData* converted) {
  const size_t cameraSlot = FindRequiredSlot(
      cameraSlots, sfmFactor.key1(),
      "CUDA SFM conversion found a factor camera key missing from Values");
  const size_t pointSlot = FindRequiredSlot(
      pointSlots, sfmFactor.key2(),
      "CUDA SFM conversion found a factor point key missing from Values");
  AppendProjectionFactorWithSlotsToCudaSfmData(sfmFactor, cameraSlot, pointSlot,
                                               converted);
}

void AppendProjectionBatchFactorToCudaSfmData(
    const BundlerProjectionBatchFactor& batchFactor,
    const std::unordered_map<Key, size_t>& cameraSlots,
    const std::unordered_map<Key, size_t>& pointSlots,
    CudaSfmFactorGraphData* converted) {
  const auto& factors = batchFactor.factors();
  if (factors.empty()) {
    return;
  }

  const Key firstCameraKey = factors.front().key1();
  const Key firstPointKey = factors.front().key2();
  bool fixedCamera = true;
  bool fixedPoint = true;
  for (const BundlerProjectionFactor& sfmFactor : factors) {
    fixedCamera = fixedCamera && sfmFactor.key1() == firstCameraKey;
    fixedPoint = fixedPoint && sfmFactor.key2() == firstPointKey;
  }

  if (fixedPoint) {
    const size_t pointSlot = FindRequiredSlot(
        pointSlots, firstPointKey,
        "CUDA SFM conversion found a batch factor point key missing from Values");
    for (const BundlerProjectionFactor& sfmFactor : factors) {
      const size_t cameraSlot = FindRequiredSlot(
          cameraSlots, sfmFactor.key1(),
          "CUDA SFM conversion found a factor camera key missing from Values");
      AppendProjectionFactorWithSlotsToCudaSfmData(
          sfmFactor, cameraSlot, pointSlot, converted);
    }
    return;
  }

  if (fixedCamera) {
    const size_t cameraSlot = FindRequiredSlot(
        cameraSlots, firstCameraKey,
        "CUDA SFM conversion found a batch factor camera key missing from Values");
    for (const BundlerProjectionFactor& sfmFactor : factors) {
      const size_t pointSlot = FindRequiredSlot(
          pointSlots, sfmFactor.key2(),
          "CUDA SFM conversion found a factor point key missing from Values");
      AppendProjectionFactorWithSlotsToCudaSfmData(
          sfmFactor, cameraSlot, pointSlot, converted);
    }
    return;
  }

  for (const BundlerProjectionFactor& sfmFactor : factors) {
    AppendProjectionFactorToCudaSfmData(sfmFactor, cameraSlots, pointSlots,
                                        converted);
  }
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

    if (const auto sfmFactor =
            std::dynamic_pointer_cast<BundlerProjectionFactor>(factor)) {
      AppendProjectionFactorToCudaSfmData(*sfmFactor, cameraSlots, pointSlots,
                                          &converted);
      continue;
    }

    if (const auto batchFactor =
            std::dynamic_pointer_cast<BundlerProjectionBatchFactor>(factor)) {
      AppendProjectionBatchFactorToCudaSfmData(*batchFactor, cameraSlots,
                                               pointSlots, &converted);
      continue;
    }

    throw std::invalid_argument(
        "CUDA SFM conversion only supports GeneralSFMFactor<SfmCamera, "
        "Point3> or BatchFactor<GeneralSFMFactor<SfmCamera, Point3>, 2>");
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
  auto stageStart = Clock::now();
  auto convertedDestructionStart = Clock::now();
  {
    const CudaSfmFactorGraphData converted =
        ConvertGeneralSfmGraphToCudaSfmData(graph(), values());
    const double graphConversionElapsed = ElapsedSince(stageStart);

    stageStart = Clock::now();
    result_ = OptimizeCudaSfmImpl(
        converted.data, converted.cameraKeys, converted.pointKeys, params_,
        (converted.hasNonUnitNoise || converted.hasRobustNoise)
            ? &converted.sqrtInfoByTrack
            : nullptr,
        converted.hasRobustNoise ? &converted.robustModelsByTrack : nullptr,
        CudaSfmLmExecutionOptions{true});
    result_.graphBackendCallElapsed = ElapsedSince(stageStart);
    result_.graphConversionElapsed = graphConversionElapsed;

    stageStart = Clock::now();
    Values merged(values());
    for (Key key : result_.optimizedValues.keys()) {
      merged.update(key, result_.optimizedValues.at(key));
    }
    state_ = std::unique_ptr<gtsam::internal::NonlinearOptimizerState>(
        new gtsam::internal::NonlinearOptimizerState(
            std::move(merged), result_.finalError, result_.iterations));
    result_.graphValueMergeElapsed = ElapsedSince(stageStart);

    convertedDestructionStart = Clock::now();
  }
  result_.graphConvertedDataDestructionElapsed =
      ElapsedSince(convertedDestructionStart);
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
  if (cameraKeys.size() != data.numberCameras()) {
    throw std::invalid_argument(
        "OptimizeCudaSfm camera key count does not match SfmData");
  }
  if (pointKeys.size() != data.numberTracks()) {
    throw std::invalid_argument(
        "OptimizeCudaSfm point key count does not match SfmData");
  }
#if !GTSAM_ENABLE_CUDSS
  if (params.linearSolver == CudaSfmLinearSolverType::CudssFullNormal) {
    throw std::runtime_error(
        "OptimizeCudaSfm CudssFullNormal requires GTSAM_ENABLE_CUDSS=ON");
  }
#endif

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
  CudaSfmValuesPackProfile packValuesProfile;
  DeviceValues current =
      PackSfmValues(data, cameraKeys, pointKeys, context.stream(),
                    &packValuesProfile);
  result.packValuesElapsed =
      ElapsedSinceAfterSync(stageStart, context.stream());
  result.packValuesHostBuildElapsed = packValuesProfile.hostBuildElapsed;
  result.packValuesDeviceAllocElapsed =
      packValuesProfile.deviceAllocElapsed;
  result.packValuesH2dCopyElapsed = packValuesProfile.h2d.copyElapsed;
  result.packValuesH2dBytes = packValuesProfile.h2d.bytes;
  AddH2dTransfer(packValuesProfile.h2d, &result);

  stageStart = Clock::now();
  DeviceValues trial = AllocateSfmValuesLike(current);
  result.allocateTrialElapsed = ElapsedSince(stageStart);

  stageStart = Clock::now();
  if (robustModelsByTrack && !sqrtInfoByTrack) {
    throw std::invalid_argument(
        "OptimizeCudaSfm robust noise requires projection sqrt-info");
  }
  CudaSfmProjectionBatchTransferProfile projectionBatchProfile;
  const CudaSfmProjectionBatch batch =
      robustModelsByTrack
          ? CudaSfmProjectionBatch::FromSfmData(data, *sqrtInfoByTrack,
                                               *robustModelsByTrack,
                                               context.stream(),
                                               &projectionBatchProfile)
      : sqrtInfoByTrack
          ? CudaSfmProjectionBatch::FromSfmData(data, *sqrtInfoByTrack,
                                               context.stream(),
                                               &projectionBatchProfile)
          : CudaSfmProjectionBatch::FromSfmData(
                data, context.stream(), &projectionBatchProfile);
  result.projectionBatchElapsed =
      ElapsedSinceAfterSync(stageStart, context.stream());
  result.projectionBatchHostBuildElapsed =
      projectionBatchProfile.hostBuildElapsed;
  result.projectionBatchDeviceAllocElapsed =
      projectionBatchProfile.deviceAllocElapsed;
  result.projectionBatchH2dCopyElapsed =
      projectionBatchProfile.h2d.copyElapsed;
  result.projectionBatchH2dBytes = projectionBatchProfile.h2d.bytes;
  AddH2dTransfer(projectionBatchProfile.h2d, &result);

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
      CudaSfmValuesDownloadProfile downloadProfile;
      result.optimizedValues =
          DownloadSfmValues(current, context.stream(), &downloadProfile);
      result.downloadElapsed = ElapsedSince(stageStart);
      SetDownloadTransferProfile(downloadProfile, &result);
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
    CudaDeviceTransferSummary uploadPatternProfile;
    system.uploadPattern(structure.dimension(), structure.rowPointers(),
                         structure.colIndices(), context.stream(),
                         &uploadPatternProfile);
    result.uploadPatternElapsed =
        ElapsedSinceAfterSync(stageStart, context.stream());
    result.uploadPatternDeviceAllocElapsed =
        uploadPatternProfile.resizeElapsed;
    result.uploadPatternH2dCopyElapsed = uploadPatternProfile.copyElapsed;
    result.uploadPatternH2dBytes = uploadPatternProfile.bytes;
    AddH2dTransfer(uploadPatternProfile, &result);
  }

  double lambda = params.lambdaInitial;
  double currentFactor = params.lambdaFactor;
  CudaDeviceArray<double> dampingDiagonal;

  result.setupElapsed = ElapsedSince(totalStart);
  const auto solveLoopStart = std::chrono::steady_clock::now();
  bool terminate = false;
  while (result.iterations < params.maxIterations && std::isfinite(currentError) &&
         !terminate) {
    const auto iterationStart = Clock::now();
    CudaSfmLmIterationProfile iterationProfile;
    iterationProfile.iteration =
        static_cast<int>(result.iterationProfiles.size());
    iterationProfile.startError = currentError;
    iterationProfile.startLambda = lambda;

    if (params.diagonalDamping) {
      stageStart = Clock::now();
      ComputeCudaSfmHessianDiagonal(current, batch, numCameras,
                                    params.minDiagonal, params.maxDiagonal,
                                    &dampingDiagonal, context.stream());
      iterationProfile.dampingDiagonalElapsed =
          ElapsedSinceAfterSync(stageStart, context.stream());
      result.dampingDiagonalElapsed +=
          iterationProfile.dampingDiagonalElapsed;
    }

    bool acceptedOrDone = false;
    int attemptIndex = 0;
    while (!acceptedOrDone) {
      const auto attemptStart = Clock::now();
      CudaSfmLmAttemptProfile attemptProfile;
      attemptProfile.iteration = iterationProfile.iteration;
      attemptProfile.attempt = attemptIndex++;
      attemptProfile.lambda = lambda;

      if (params.linearSolver == CudaSfmLinearSolverType::DenseSchur) {
        stageStart = Clock::now();
        if (params.diagonalDamping) {
          denseSchurSolver.solve(current, batch, numCameras, lambda,
                                 dampingDiagonal, &delta, context.stream());
        } else {
          denseSchurSolver.solve(current, batch, numCameras, lambda, &delta,
                                 context.stream());
        }
        attemptProfile.denseSchurSolveElapsed =
            ElapsedSinceAfterSync(stageStart, context.stream());
        result.denseSchurSolveElapsed +=
            attemptProfile.denseSchurSolveElapsed;
      } else {
        stageStart = Clock::now();
        AccumulateCudaSfmNormalEquations(current, batch, numCameras, &system,
                                         context.stream());
        attemptProfile.normalEquationsElapsed =
            ElapsedSinceAfterSync(stageStart, context.stream());
        result.normalEquationsElapsed +=
            attemptProfile.normalEquationsElapsed;

        stageStart = Clock::now();
        if (params.diagonalDamping) {
          system.addDiagonalDamping(lambda, dampingDiagonal, context.stream());
        } else {
          system.addDiagonalDamping(lambda, context.stream());
        }
        attemptProfile.addDampingElapsed =
            ElapsedSinceAfterSync(stageStart, context.stream());
        result.addDampingElapsed += attemptProfile.addDampingElapsed;

        if (!solverAnalyzed) {
          stageStart = Clock::now();
          solver.analyze(system, &delta, context.stream());
          attemptProfile.cudssAnalyzeElapsed =
              ElapsedSinceAfterSync(stageStart, context.stream());
          result.firstCudssAnalyzeElapsed =
              attemptProfile.cudssAnalyzeElapsed;
          result.cudssAnalyzeElapsed += attemptProfile.cudssAnalyzeElapsed;
          solverAnalyzed = true;
        }
        stageStart = Clock::now();
        solver.solve(system, &delta, context.stream());
        attemptProfile.cudssSolveElapsed =
            ElapsedSinceAfterSync(stageStart, context.stream());
        result.cudssSolveElapsed += attemptProfile.cudssSolveElapsed;
      }
      ++result.innerIterations;

      double oldLinearizedError = 0.0;
      double newLinearizedError = 0.0;
      stageStart = Clock::now();
      const double linearizedCostChange =
          ComputeCudaSfmLinearizedErrorChange(
              current, batch, numCameras, delta, &oldLinearizedError,
              &newLinearizedError, context.stream());
      attemptProfile.linearizedErrorElapsed =
          ElapsedSinceAfterSync(stageStart, context.stream());
      result.linearizedErrorElapsed +=
          attemptProfile.linearizedErrorElapsed;
      attemptProfile.oldLinearizedError = oldLinearizedError;
      attemptProfile.newLinearizedError = newLinearizedError;
      attemptProfile.linearizedCostChange = linearizedCostChange;

      double trialError = std::numeric_limits<double>::infinity();
      double costChange = 0.0;
      double modelFidelity = 0.0;
      bool stepSuccessful = false;
      bool stopSearchingLambda = false;

      if (linearizedCostChange >= 0.0) {
        attemptProfile.attemptedTrial = true;
        stageStart = Clock::now();
        ApplyDelta(current, delta, &trial, numCameras, numPoints,
                   context.stream());
        attemptProfile.applyDeltaElapsed =
            ElapsedSinceAfterSync(stageStart, context.stream());
        result.applyDeltaElapsed += attemptProfile.applyDeltaElapsed;

        stageStart = Clock::now();
        trialError =
            ComputeCudaSfmProjectionError(trial, batch, context.stream());
        attemptProfile.trialErrorElapsed =
            ElapsedSinceAfterSync(stageStart, context.stream());
        result.trialErrorElapsed += attemptProfile.trialErrorElapsed;
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
      attemptProfile.trialError = trialError;
      attemptProfile.costChange = costChange;
      attemptProfile.modelFidelity = modelFidelity;
      attemptProfile.stepSuccessful = stepSuccessful;
      attemptProfile.stopSearchingLambda = stopSearchingLambda;

      if (stepSuccessful) {
        const double previousError = currentError;
        stageStart = Clock::now();
        AcceptTrial(trial, &current, context.stream());
        iterationProfile.acceptTrialElapsed =
            ElapsedSinceAfterSync(stageStart, context.stream());
        result.acceptTrialElapsed += iterationProfile.acceptTrialElapsed;
        currentError = trialError;
        ++result.iterations;
        ++result.acceptedSteps;
        stageStart = Clock::now();
        DecreaseLambda(params, modelFidelity, &lambda, &currentFactor);
        acceptedOrDone = true;
        terminate =
            CheckCudaLmConvergence(params, previousError, currentError);
        attemptProfile.lambdaUpdateElapsed = ElapsedSince(stageStart);
        result.lambdaUpdateElapsed += attemptProfile.lambdaUpdateElapsed;
        iterationProfile.acceptedStep = true;
        attemptProfile.accepted = true;
        attemptProfile.terminated = terminate;
      } else if (!stopSearchingLambda) {
        stageStart = Clock::now();
        IncreaseLambda(params, &lambda, &currentFactor);
        if (lambda >= params.lambdaUpperBound) {
          acceptedOrDone = true;
          terminate = true;
          attemptProfile.lambdaUpperBoundReached = true;
        }
        attemptProfile.lambdaUpdateElapsed = ElapsedSince(stageStart);
        result.lambdaUpdateElapsed += attemptProfile.lambdaUpdateElapsed;
        attemptProfile.terminated = terminate;
      } else {
        acceptedOrDone = true;
        terminate = true;
        attemptProfile.terminated = true;
      }
      attemptProfile.totalElapsed = ElapsedSince(attemptStart);
      iterationProfile.attemptProfiles.push_back(attemptProfile);
    }
    iterationProfile.endError = currentError;
    iterationProfile.endLambda = lambda;
    iterationProfile.terminated = terminate;
    iterationProfile.totalElapsed = ElapsedSince(iterationStart);
    result.iterationProfiles.push_back(iterationProfile);
  }
  GTSAM_CUDA_CHECK(cudaStreamSynchronize(context.stream()));
  const auto solveLoopEnd = std::chrono::steady_clock::now();
  result.solveLoopElapsed =
      std::chrono::duration<double>(solveLoopEnd - solveLoopStart).count();

  result.finalError = currentError;
  result.finalLambda = lambda;
  if (executionOptions.downloadOptimizedValues) {
    stageStart = Clock::now();
    CudaSfmValuesDownloadProfile downloadProfile;
    result.optimizedValues =
        DownloadSfmValues(current, context.stream(), &downloadProfile);
    result.downloadElapsed = ElapsedSince(stageStart);
    SetDownloadTransferProfile(downloadProfile, &result);
  }
  result.totalMeasuredElapsed = ElapsedSince(totalStart);
  return result;
}

}  // namespace gtsam::cuda

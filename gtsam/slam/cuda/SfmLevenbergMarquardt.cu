#include <gtsam/base/cuda/Context.h>
#include <gtsam/base/cuda/DeviceArray.h>
#include <gtsam/base/cuda/Errors.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/linear/cuda/BlockOrdering.h>
#include <gtsam/linear/cuda/LinearSolver.h>
#include <gtsam/nonlinear/BatchFactor.h>
#include <gtsam/nonlinear/cuda/DeviceGeometryKernels.h>
#include <gtsam/slam/GeneralSFMFactor.h>
#include <gtsam/slam/cuda/SfmDenseSchurSolver.h>
#include <gtsam/slam/cuda/SfmLevenbergMarquardt.h>
#include <gtsam/slam/cuda/SfmProjectionLinearization.h>
#include <gtsam/slam/cuda/SfmReducedCsrPlan.h>
#include <gtsam/slam/cuda/SfmSchurProblem.h>
#include <gtsam/slam/cuda/SfmValues.h>

#include <algorithm>
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

enum class SfmLinearSolverType {
  DenseSchur,
  CudssSchur,
  PcgSchur,
};

const char* linearSolverName(LinearSolverType backend) {
  switch (backend) {
    case LinearSolverType::DenseCholesky:
      return "DenseCholesky";
    case LinearSolverType::Cudss:
      return "Cudss";
    case LinearSolverType::Pcg:
      return "Pcg";
  }
  return "Unknown";
}

struct SfmLevenbergMarquardtExecutionOptions {
  bool downloadOptimizedValues = true;
};

double elapsedSeconds(Clock::time_point start, Clock::time_point end) {
  return std::chrono::duration<double>(end - start).count();
}

double elapsedSince(Clock::time_point start) {
  return elapsedSeconds(start, Clock::now());
}

double elapsedSinceAfterSync(Clock::time_point start, cudaStream_t stream) {
  GTSAM_CUDA_CHECK(cudaStreamSynchronize(stream));
  return elapsedSince(start);
}

double detailedElapsedSince(bool enabled, Clock::time_point start) {
  return enabled ? elapsedSince(start) : 0.0;
}

double detailedElapsedSinceAfterSync(bool enabled, Clock::time_point start,
                                     cudaStream_t stream) {
  return enabled ? elapsedSinceAfterSync(start, stream) : 0.0;
}

Clock::time_point detailedProfileStart(bool enabled) {
  return enabled ? Clock::now() : Clock::time_point{};
}

void addH2dTransfer(const DeviceTransferSummary& transfer,
                    SfmLevenbergMarquardtResult* result) {
  result->totalH2dBytes += transfer.bytes;
  result->totalH2dCopyElapsed += transfer.copyElapsed;
}

void addReductionD2hTransfer(
    const SfmReductionTransferProfile& profile,
    SfmLevenbergMarquardtResult* result) {
  result->totalD2hBytes += profile.d2hBytes;
  result->totalD2hCopyElapsed += profile.d2hElapsed;
}

void setDownloadTransferProfile(
    const SfmValuesDownloadProfile& profile,
    SfmLevenbergMarquardtResult* result) {
  result->downloadHostAllocElapsed = profile.hostAllocElapsed;
  result->downloadD2hCopyElapsed = profile.d2h.copyElapsed;
  result->downloadValuesBuildElapsed = profile.hostBuildElapsed;
  result->downloadD2hBytes = profile.d2h.bytes;
  result->totalD2hCopyElapsed += profile.d2h.copyElapsed;
  result->totalD2hBytes += profile.d2h.bytes;
}

__global__ void applyDeltaKernel(
    const DevicePinholeCameraCal3Bundler* cameras,
    const DevicePoint3* points, int numCameras, int numPoints,
    const double* delta, DevicePinholeCameraCal3Bundler* trialCameras,
    DevicePoint3* trialPoints) {
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < numCameras) {
    trialCameras[i] = retractCamera(cameras[i], delta + 9 * i);
  }
  if (i < numPoints) {
    const int pointOffset = 9 * numCameras + 3 * i;
    trialPoints[i] = retractPoint(points[i], delta + pointOffset);
  }
}

void applyDelta(const DeviceValues& current, const DeviceArray<double>& delta,
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
  applyDeltaKernel<<<gridSize, kApplyDeltaBlockSize, 0, stream>>>(
      currentCameras.data(), currentPoints.data(), numCameras, numPoints,
      delta.data(), trialCameras.data(), trialPoints.data());
  GTSAM_CUDA_CHECK(cudaGetLastError());
}

void acceptTrial(const DeviceValues& trial, DeviceValues* current,
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

std::vector<Key> defaultCameraKeys(const SfmData& data) {
  std::vector<Key> keys;
  keys.reserve(data.numberCameras());
  for (size_t i = 0; i < data.numberCameras(); ++i) {
    keys.push_back(symbol_shorthand::C(i));
  }
  return keys;
}

std::vector<Key> defaultPointKeys(const SfmData& data) {
  std::vector<Key> keys;
  keys.reserve(data.numberTracks());
  for (size_t i = 0; i < data.numberTracks(); ++i) {
    keys.push_back(symbol_shorthand::P(i));
  }
  return keys;
}

SfmLinearSolverType validateAndSelectSolverMode(
    const SfmLevenbergMarquardtParams& params) {
  if (params.ordering &&
      params.linear.backend != gtsam::cuda::LinearSolverType::Cudss) {
    throw std::invalid_argument(
        "SFM CUDA Ordering is supported only by the cuDSS backend");
  }
  switch (params.linear.backend) {
    case gtsam::cuda::LinearSolverType::DenseCholesky:
      return SfmLinearSolverType::DenseSchur;
    case gtsam::cuda::LinearSolverType::Cudss:
      return SfmLinearSolverType::CudssSchur;
    case gtsam::cuda::LinearSolverType::Pcg:
      return SfmLinearSolverType::PcgSchur;
  }
  throw std::invalid_argument("unknown CUDA SFM linear solver backend");
}

SfmSqrtInfo2 extractProjectionSqrtInfo(
    const SharedNoiseModel& model, bool* isNonUnit) {
  if (isNonUnit) {
    *isNonUnit = false;
  }
  if (!model || model->isUnit()) {
    return SfmProjectionBatch::unitSqrtInfo();
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

  const SfmSqrtInfo2 sqrtInfo{R(0, 0), R(0, 1), R(1, 1)};
  if (!SfmProjectionBatch::isUsableSqrtInfo(sqrtInfo)) {
    throw std::invalid_argument(
        "CUDA SFM conversion found invalid Gaussian sqrt-information");
  }
  if (isNonUnit) {
    *isNonUnit = !SfmProjectionBatch::isUnitSqrtInfo(sqrtInfo);
  }
  return sqrtInfo;
}

SfmRobustReweightScheme extractRobustReweightScheme(
    noiseModel::mEstimator::Base::ReweightScheme scheme) {
  switch (scheme) {
    case noiseModel::mEstimator::Base::Scalar:
      return SfmRobustReweightScheme::Scalar;
    case noiseModel::mEstimator::Base::Block:
      return SfmRobustReweightScheme::Block;
  }
  throw std::invalid_argument(
      "CUDA SFM conversion found unknown robust reweight scheme");
}

SfmRobustModel extractProjectionRobustModel(
    const noiseModel::mEstimator::Base::shared_ptr& model) {
  if (!model) {
    throw std::invalid_argument(
        "CUDA SFM conversion found robust noise without an m-estimator");
  }

  const SfmRobustReweightScheme reweightScheme =
      extractRobustReweightScheme(model->reweightScheme());

  if (const auto huber =
          std::dynamic_pointer_cast<noiseModel::mEstimator::Huber>(model)) {
    return SfmRobustModel{SfmRobustModelKind::Huber, reweightScheme,
                              huber->modelParameter()};
  }
  if (const auto tukey =
          std::dynamic_pointer_cast<noiseModel::mEstimator::Tukey>(model)) {
    return SfmRobustModel{SfmRobustModelKind::Tukey, reweightScheme,
                              tukey->modelParameter()};
  }

  throw std::invalid_argument(
      "CUDA SFM conversion only supports Huber and Tukey robust projection "
      "noise models");
}

void ensureRobustModelStorage(SfmFactorGraphData* converted) {
  if (converted->hasRobustNoise) {
    return;
  }
  converted->hasRobustNoise = true;
  converted->robustModelsByTrack.resize(converted->data.numberTracks());
  for (size_t trackIndex = 0; trackIndex < converted->data.numberTracks();
       ++trackIndex) {
    converted->robustModelsByTrack[trackIndex].assign(
        converted->data.track(trackIndex).numberMeasurements(),
        SfmProjectionBatch::noRobustModel());
  }
}

void appendProjectionFactorWithSlots(
    const BundlerProjectionFactor& sfmFactor, size_t cameraSlot,
    size_t pointSlot, SfmFactorGraphData* converted) {
  SharedNoiseModel model = sfmFactor.noiseModel();
  SfmRobustModel robustModel =
      SfmProjectionBatch::noRobustModel();
  if (model && !model->isUnit()) {
    if (const auto robust =
            std::dynamic_pointer_cast<noiseModel::Robust>(model)) {
      ensureRobustModelStorage(converted);
      robustModel = extractProjectionRobustModel(robust->robust());
      model = robust->noise();
    }
  }
  bool factorHasNonUnitNoise = false;
  const SfmSqrtInfo2 sqrtInfo =
      extractProjectionSqrtInfo(model, &factorHasNonUnitNoise);
  converted->hasNonUnitNoise =
      converted->hasNonUnitNoise || factorHasNonUnitNoise;

  converted->data.tracks[pointSlot].measurements.emplace_back(
      cameraSlot, sfmFactor.measured());
  converted->sqrtInfoByTrack[pointSlot].push_back(sqrtInfo);
  if (converted->hasRobustNoise) {
    converted->robustModelsByTrack[pointSlot].push_back(robustModel);
  }
}

size_t findRequiredSlot(const std::unordered_map<Key, size_t>& slots, Key key,
                        const char* description) {
  const auto slot = slots.find(key);
  if (slot == slots.end()) {
    throw std::invalid_argument(description);
  }
  return slot->second;
}

void appendProjectionFactor(
    const BundlerProjectionFactor& sfmFactor,
    const std::unordered_map<Key, size_t>& cameraSlots,
    const std::unordered_map<Key, size_t>& pointSlots,
    SfmFactorGraphData* converted) {
  const size_t cameraSlot = findRequiredSlot(
      cameraSlots, sfmFactor.key1(),
      "CUDA SFM conversion found a factor camera key missing from Values");
  const size_t pointSlot = findRequiredSlot(
      pointSlots, sfmFactor.key2(),
      "CUDA SFM conversion found a factor point key missing from Values");
  appendProjectionFactorWithSlots(sfmFactor, cameraSlot, pointSlot,
                                               converted);
}

void appendProjectionBatchFactor(
    const BundlerProjectionBatchFactor& batchFactor,
    const std::unordered_map<Key, size_t>& cameraSlots,
    const std::unordered_map<Key, size_t>& pointSlots,
    SfmFactorGraphData* converted) {
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
    const size_t pointSlot = findRequiredSlot(
        pointSlots, firstPointKey,
        "CUDA SFM conversion found a batch factor point key missing from Values");
    for (const BundlerProjectionFactor& sfmFactor : factors) {
      const size_t cameraSlot = findRequiredSlot(
          cameraSlots, sfmFactor.key1(),
          "CUDA SFM conversion found a factor camera key missing from Values");
      appendProjectionFactorWithSlots(
          sfmFactor, cameraSlot, pointSlot, converted);
    }
    return;
  }

  if (fixedCamera) {
    const size_t cameraSlot = findRequiredSlot(
        cameraSlots, firstCameraKey,
        "CUDA SFM conversion found a batch factor camera key missing from Values");
    for (const BundlerProjectionFactor& sfmFactor : factors) {
      const size_t pointSlot = findRequiredSlot(
          pointSlots, sfmFactor.key2(),
          "CUDA SFM conversion found a factor point key missing from Values");
      appendProjectionFactorWithSlots(
          sfmFactor, cameraSlot, pointSlot, converted);
    }
    return;
  }

  for (const BundlerProjectionFactor& sfmFactor : factors) {
    appendProjectionFactor(sfmFactor, cameraSlots, pointSlots,
                                        converted);
  }
}

void increaseLambda(const SfmLevenbergMarquardtParams& params,
                    double* lambda, double* currentFactor) {
  *lambda *= *currentFactor;
  if (!params.useFixedLambdaFactor) {
    *currentFactor *= 2.0;
  }
}

void decreaseLambda(const SfmLevenbergMarquardtParams& params,
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

bool checkLevenbergMarquardtConvergence(
    const SfmLevenbergMarquardtParams& params,
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

SfmLevenbergMarquardtParams::SfmLevenbergMarquardtParams()
    : enableDetailedProfiling(false) {
  LevenbergMarquardtParams::SetLegacyDefaults(this);
}

SfmLevenbergMarquardtParams
SfmLevenbergMarquardtParams::legacyDefaults() {
  SfmLevenbergMarquardtParams p;
  LevenbergMarquardtParams::SetLegacyDefaults(&p);
  return p;
}

SfmLevenbergMarquardtParams
SfmLevenbergMarquardtParams::ceresDefaults() {
  SfmLevenbergMarquardtParams p;
  LevenbergMarquardtParams::SetCeresDefaults(&p);
  return p;
}

void SfmLevenbergMarquardtParams::print(const std::string& str) const {
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
  std::cout << "            diagonalDamping: "
            << dampingParams.diagonalDamping << "\n";
  std::cout << "     enableDetailedProfiling: " << enableDetailedProfiling
            << "\n";
  std::cout << "                minDiagonal: " << dampingParams.minDiagonal
            << "\n";
  std::cout << "                maxDiagonal: " << dampingParams.maxDiagonal
            << "\n";
  std::cout << "               linearBackend: "
            << linearSolverName(linear.backend) << "\n";
}

bool SfmLevenbergMarquardtParams::equals(
    const SfmLevenbergMarquardtParams& other, double tol) const {
  return NonlinearOptimizerParams::equals(other, tol) &&
         std::abs(lambdaInitial - other.lambdaInitial) <= tol &&
         std::abs(lambdaFactor - other.lambdaFactor) <= tol &&
         std::abs(lambdaUpperBound - other.lambdaUpperBound) <= tol &&
         std::abs(lambdaLowerBound - other.lambdaLowerBound) <= tol &&
         std::abs(minModelFidelity - other.minModelFidelity) <= tol &&
         useFixedLambdaFactor == other.useFixedLambdaFactor &&
         verbosityLM == other.verbosityLM && logFile == other.logFile &&
         dampingParams.diagonalDamping ==
             other.dampingParams.diagonalDamping &&
         dampingParams.exactHessianDiagonal ==
             other.dampingParams.exactHessianDiagonal &&
         enableDetailedProfiling == other.enableDetailedProfiling &&
         std::abs(dampingParams.minDiagonal -
                  other.dampingParams.minDiagonal) <= tol &&
         std::abs(dampingParams.maxDiagonal -
                  other.dampingParams.maxDiagonal) <= tol &&
         linear.backend == other.linear.backend &&
         ordering == other.ordering &&
         pcg.maxIterations == other.pcg.maxIterations &&
         std::abs(pcg.relativeTolerance - other.pcg.relativeTolerance) <= tol &&
         pcg.warmStart == other.pcg.warmStart &&
         pcg.convergenceCheckInterval == other.pcg.convergenceCheckInterval;
}

SfmFactorGraphData convertGeneralSfmGraph(
    const NonlinearFactorGraph& graph, const Values& initialValues) {
  SfmFactorGraphData converted;

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
      appendProjectionFactor(*sfmFactor, cameraSlots, pointSlots,
                                          &converted);
      continue;
    }

    if (const auto batchFactor =
            std::dynamic_pointer_cast<BundlerProjectionBatchFactor>(factor)) {
      appendProjectionBatchFactor(*batchFactor, cameraSlots,
                                               pointSlots, &converted);
      continue;
    }

    throw std::invalid_argument(
        "CUDA SFM conversion only supports GeneralSFMFactor<SfmCamera, "
        "Point3> or BatchFactor<GeneralSFMFactor<SfmCamera, Point3>, 2>");
  }

  return converted;
}

SfmLevenbergMarquardtResult optimizeSfmImpl(
    const SfmData& data, const std::vector<Key>& cameraKeys,
    const std::vector<Key>& pointKeys,
    const SfmLevenbergMarquardtParams& params,
    const std::vector<std::vector<SfmSqrtInfo2>>* sqrtInfoByTrack,
    const std::vector<std::vector<SfmRobustModel>>* robustModelsByTrack,
    SfmLevenbergMarquardtExecutionOptions executionOptions);

SfmLevenbergMarquardtOptimizer::SfmLevenbergMarquardtOptimizer(
    const NonlinearFactorGraph& graph, const Values& initialValues,
    const SfmLevenbergMarquardtParams& params)
    : graph_(graph),
      values_(initialValues),
      params_(params) {}

const Values& SfmLevenbergMarquardtOptimizer::optimize() {
  auto stageStart = Clock::now();
  auto convertedDestructionStart = Clock::now();
  {
    const SfmFactorGraphData converted =
        convertGeneralSfmGraph(graph_, values_);
    const double graphConversionElapsed = elapsedSince(stageStart);

    stageStart = Clock::now();
    result_ = optimizeSfmImpl(
        converted.data, converted.cameraKeys, converted.pointKeys, params_,
        (converted.hasNonUnitNoise || converted.hasRobustNoise)
            ? &converted.sqrtInfoByTrack
            : nullptr,
        converted.hasRobustNoise ? &converted.robustModelsByTrack : nullptr,
        SfmLevenbergMarquardtExecutionOptions{true});
    result_.graphBackendCallElapsed = elapsedSince(stageStart);
    result_.graphConversionElapsed = graphConversionElapsed;

    stageStart = Clock::now();
    Values merged(values_);
    for (Key key : result_.optimizedValues.keys()) {
      merged.update(key, result_.optimizedValues.at(key));
    }
    values_ = std::move(merged);
    result_.graphValueMergeElapsed = elapsedSince(stageStart);

    convertedDestructionStart = Clock::now();
  }
  result_.graphConvertedDataDestructionElapsed =
      elapsedSince(convertedDestructionStart);
  return values_;
}

SfmLevenbergMarquardtResult optimizeSfm(
    const SfmData& data,
    const SfmLevenbergMarquardtParams& params) {
  return optimizeSfm(data, defaultCameraKeys(data), defaultPointKeys(data),
                         params);
}

SfmLevenbergMarquardtResult optimizeSfmWithoutValueDownload(
    const SfmData& data,
    const SfmLevenbergMarquardtParams& params) {
  return optimizeSfmWithoutValueDownload(
      data, defaultCameraKeys(data), defaultPointKeys(data), params);
}

SfmLevenbergMarquardtResult optimizeSfm(
    const SfmData& data, const std::vector<Key>& cameraKeys,
    const std::vector<Key>& pointKeys,
    const SfmLevenbergMarquardtParams& params) {
  return optimizeSfmImpl(data, cameraKeys, pointKeys, params,
                             nullptr, nullptr,
                             SfmLevenbergMarquardtExecutionOptions{true});
}

SfmLevenbergMarquardtResult optimizeSfmWithoutValueDownload(
    const SfmData& data, const std::vector<Key>& cameraKeys,
    const std::vector<Key>& pointKeys,
    const SfmLevenbergMarquardtParams& params) {
  return optimizeSfmImpl(data, cameraKeys, pointKeys, params,
                             nullptr, nullptr,
                             SfmLevenbergMarquardtExecutionOptions{false});
}

SfmLevenbergMarquardtResult optimizeSfmImpl(
    const SfmData& data, const std::vector<Key>& cameraKeys,
    const std::vector<Key>& pointKeys,
    const SfmLevenbergMarquardtParams& params,
    const std::vector<std::vector<SfmSqrtInfo2>>* sqrtInfoByTrack,
    const std::vector<std::vector<SfmRobustModel>>* robustModelsByTrack,
    SfmLevenbergMarquardtExecutionOptions executionOptions) {
  if (cameraKeys.size() != data.numberCameras()) {
    throw std::invalid_argument(
        "optimizeSfm camera key count does not match SfmData");
  }
  if (pointKeys.size() != data.numberTracks()) {
    throw std::invalid_argument(
        "optimizeSfm point key count does not match SfmData");
  }
  const SfmLinearSolverType solverMode =
      validateAndSelectSolverMode(params);
#if !GTSAM_ENABLE_CUDSS
  if (solverMode == SfmLinearSolverType::CudssSchur) {
    throw std::runtime_error(
        "optimizeSfm cuDSS solver requires GTSAM_ENABLE_CUDSS=ON");
  }
#endif

  SfmLevenbergMarquardtResult result;
  result.linearBackend = params.linear.backend;
  result.linearSolveStats.backend = params.linear.backend;
  const bool detailedProfiling = params.enableDetailedProfiling;
  const auto totalStart = Clock::now();

  auto stageStart = Clock::now();
  Context context(nullptr);
  result.contextElapsed = elapsedSince(stageStart);

  // TODO(perf): Avoid duplicate host-to-device transfers here. packSfmValues()
  // uploads values, and SfmProjectionBatch::fromSfmData() uploads overlapping
  // SFM data again. Consider constructing the projection batch from existing
  // device buffers or sharing the packed representation.
  stageStart = detailedProfileStart(detailedProfiling);
  SfmValuesPackProfile packValuesProfile;
  DeviceValues current =
      packSfmValues(data, cameraKeys, pointKeys, context.stream(),
                    detailedProfiling ? &packValuesProfile : nullptr);
  result.packValuesElapsed =
      detailedElapsedSinceAfterSync(detailedProfiling, stageStart,
                                    context.stream());
  if (detailedProfiling) {
    result.packValuesHostBuildElapsed = packValuesProfile.hostBuildElapsed;
    result.packValuesDeviceAllocElapsed =
        packValuesProfile.deviceAllocElapsed;
    result.packValuesH2dCopyElapsed = packValuesProfile.h2d.copyElapsed;
    result.packValuesH2dBytes = packValuesProfile.h2d.bytes;
    addH2dTransfer(packValuesProfile.h2d, &result);
  }

  stageStart = Clock::now();
  DeviceValues trial = allocateSfmValuesLike(current);
  result.allocateTrialElapsed = elapsedSince(stageStart);

  stageStart = detailedProfileStart(detailedProfiling);
  if (robustModelsByTrack && !sqrtInfoByTrack) {
    throw std::invalid_argument(
        "optimizeSfm robust noise requires projection sqrt-info");
  }
  SfmProjectionBatchTransferProfile projectionBatchProfile;
  const SfmProjectionBatch batch =
      robustModelsByTrack
          ? SfmProjectionBatch::fromSfmData(data, *sqrtInfoByTrack,
                                               *robustModelsByTrack,
                                               context.stream(),
                                               detailedProfiling
                                                   ? &projectionBatchProfile
                                                   : nullptr)
      : sqrtInfoByTrack
          ? SfmProjectionBatch::fromSfmData(data, *sqrtInfoByTrack,
                                               context.stream(),
                                               detailedProfiling
                                                   ? &projectionBatchProfile
                                                   : nullptr)
          : SfmProjectionBatch::fromSfmData(
                data, context.stream(),
                detailedProfiling ? &projectionBatchProfile : nullptr);
  result.projectionBatchElapsed =
      detailedElapsedSinceAfterSync(detailedProfiling, stageStart,
                                    context.stream());
  if (detailedProfiling) {
    result.projectionBatchHostBuildElapsed =
        projectionBatchProfile.hostBuildElapsed;
    result.projectionBatchDeviceAllocElapsed =
        projectionBatchProfile.deviceAllocElapsed;
    result.projectionBatchH2dCopyElapsed =
        projectionBatchProfile.h2d.copyElapsed;
    result.projectionBatchH2dBytes = projectionBatchProfile.h2d.bytes;
    addH2dTransfer(projectionBatchProfile.h2d, &result);
  }

  const int numCameras = static_cast<int>(data.numberCameras());
  const int numPoints = static_cast<int>(data.numberTracks());
  const int totalDimension = 9 * numCameras + 3 * numPoints;
  result.linearSystemDimension = static_cast<size_t>(9 * numCameras);
  if (solverMode == SfmLinearSolverType::DenseSchur) {
    result.linearSystemKind = LinearSystemKind::Dense;
    result.linearSystemNonzeros =
        result.linearSystemDimension * result.linearSystemDimension;
  } else if (solverMode == SfmLinearSolverType::CudssSchur) {
    result.linearSystemKind = LinearSystemKind::Sparse;
  } else {
    result.linearSystemKind = LinearSystemKind::Operator;
  }

  stageStart = Clock::now();
  SfmReductionTransferProfile initialErrorTransfer;
  double currentError = computeSfmProjectionError(
      current, batch, context.stream(),
      detailedProfiling ? &initialErrorTransfer : nullptr);
  if (detailedProfiling) {
    addReductionD2hTransfer(initialErrorTransfer, &result);
  }
  result.initialErrorElapsed = elapsedSince(stageStart);
  result.initialError = currentError;

  if (params.maxIterations <= 0 || totalDimension == 0 ||
      currentError <= params.errorTol) {
    result.finalError = currentError;
    result.finalLambda = params.lambdaInitial;
    if (executionOptions.downloadOptimizedValues) {
      stageStart = Clock::now();
      SfmValuesDownloadProfile downloadProfile;
      result.optimizedValues =
          downloadSfmValues(current, context.stream(),
                            detailedProfiling ? &downloadProfile : nullptr);
      result.downloadElapsed = elapsedSince(stageStart);
      if (detailedProfiling) {
        setDownloadTransferProfile(downloadProfile, &result);
      }
    }
    result.totalMeasuredElapsed = elapsedSince(totalStart);
    return result;
  }

  DeviceArray<double> delta;
  stageStart = Clock::now();
  SfmDenseSchurSolver denseSchurSolver;
  result.denseSchurSolverConstructionElapsed = elapsedSince(stageStart);
  SfmSchurProblem sparseSchurProblem;
  DeviceArray<double> cameraDelta;
  std::unique_ptr<SfmReducedCsrPlan> reducedPlan;
  std::unique_ptr<LinearSolverSession> reducedSession;
  bool reducedAnalyzed = false;
  if (solverMode == SfmLinearSolverType::CudssSchur ||
      solverMode == SfmLinearSolverType::PcgSchur) {
    LinearSolverOptions options;
    options.backend = solverMode == SfmLinearSolverType::CudssSchur
                          ? gtsam::cuda::LinearSolverType::Cudss
                          : gtsam::cuda::LinearSolverType::Pcg;
    reducedSession = std::make_unique<LinearSolverSession>(options);
    if (options.backend == gtsam::cuda::LinearSolverType::Cudss) {
      reducedPlan =
          std::make_unique<SfmReducedCsrPlan>(data, cameraKeys);
      result.linearSystemNonzeros = reducedPlan->columnIndices().size();
    }
    if (params.ordering) {
      result.appliedScalarPermutation = compileScalarPermutation(
          reducedPlan->cameraBlocks(), *params.ordering);
    }
    sparseSchurProblem.initialize(batch, numCameras);
    if (options.backend == gtsam::cuda::LinearSolverType::Pcg && numCameras > 0) {
      reducedSession->analyze(9 * numCameras, params.pcg, context.stream(),
                              detailedProfiling);
      cameraDelta.resize(static_cast<size_t>(9 * numCameras));
      reducedAnalyzed = true;
    }
  }

  double lambda = params.lambdaInitial;
  double currentFactor = params.lambdaFactor;
  DeviceArray<double> dampingDiagonal;

  result.setupElapsed = elapsedSince(totalStart);
  const auto solveLoopStart = std::chrono::steady_clock::now();
  bool terminate = false;
  while (result.iterations < params.maxIterations && std::isfinite(currentError) &&
         !terminate) {
    const auto iterationStart = detailedProfileStart(detailedProfiling);
    SfmLevenbergMarquardtIterationProfile iterationProfile;
    iterationProfile.iteration =
        static_cast<int>(result.iterationProfiles.size());
    iterationProfile.startError = currentError;
    iterationProfile.startLambda = lambda;

    if (params.dampingParams.diagonalDamping) {
      stageStart = detailedProfileStart(detailedProfiling);
      computeSfmHessianDiagonal(current, batch, numCameras,
                                    params.dampingParams.minDiagonal,
                                    params.dampingParams.maxDiagonal,
                                    &dampingDiagonal, context.stream());
      iterationProfile.dampingDiagonalElapsed =
          detailedElapsedSinceAfterSync(detailedProfiling, stageStart,
                                        context.stream());
      result.dampingDiagonalElapsed +=
          iterationProfile.dampingDiagonalElapsed;
    }

    // Projection Jacobians depend on the accepted outer state, not lambda.
    // Keep them alive across all damping retries in this outer iteration.
    if (solverMode == SfmLinearSolverType::DenseSchur) {
      denseSchurSolver.linearize(current, batch, numCameras,
                                 context.stream());
    } else {
      sparseSchurProblem.linearize(current, context.stream());
    }
    if (solverMode == SfmLinearSolverType::PcgSchur) {
      reducedSession->invalidateWarmStart();
    }

    bool acceptedOrDone = false;
    int attemptIndex = 0;
    while (!acceptedOrDone) {
      const auto attemptStart = detailedProfileStart(detailedProfiling);
      SfmLevenbergMarquardtAttemptProfile attemptProfile;
      attemptProfile.iteration = iterationProfile.iteration;
      attemptProfile.attempt = attemptIndex++;
      attemptProfile.lambda = lambda;

      if (solverMode == SfmLinearSolverType::DenseSchur) {
        stageStart = detailedProfileStart(detailedProfiling);
        if (params.dampingParams.diagonalDamping) {
          denseSchurSolver.solveLinearized(lambda, dampingDiagonal, &delta,
                                           context.stream());
        } else {
          denseSchurSolver.solveLinearized(lambda, &delta, context.stream());
        }
        attemptProfile.denseSchurSolveElapsed =
            detailedElapsedSinceAfterSync(detailedProfiling, stageStart,
                                          context.stream());
        result.denseSchurSolveElapsed +=
            attemptProfile.denseSchurSolveElapsed;
      } else if (solverMode == SfmLinearSolverType::CudssSchur) {
        stageStart = detailedProfileStart(detailedProfiling);
        DeviceSparseSpdSystem& reducedSystem =
            params.dampingParams.diagonalDamping
                ? sparseSchurProblem.prepareSparse(
                      lambda, dampingDiagonal, *reducedPlan, context.stream())
                : sparseSchurProblem.prepareSparse(lambda, *reducedPlan,
                                                   context.stream());
        attemptProfile.normalEquationsElapsed =
            detailedElapsedSinceAfterSync(detailedProfiling, stageStart,
                                          context.stream());
        result.normalEquationsElapsed +=
            attemptProfile.normalEquationsElapsed;

        if (!reducedAnalyzed) {
          stageStart = detailedProfileStart(detailedProfiling);
          if (!params.ordering) {
            reducedSession->analyze(reducedSystem, &cameraDelta,
                                    context.stream());
          } else {
            reducedSession->analyze(reducedSystem, &cameraDelta,
                                    result.appliedScalarPermutation,
                                    context.stream());
          }
          const double analyzeElapsed = detailedElapsedSinceAfterSync(
              detailedProfiling, stageStart, context.stream());
          result.firstCudssAnalyzeElapsed = analyzeElapsed;
          attemptProfile.cudssAnalyzeElapsed = analyzeElapsed;
          result.cudssAnalyzeElapsed += analyzeElapsed;
          reducedAnalyzed = true;
        }
        stageStart = detailedProfileStart(detailedProfiling);
        reducedSession->solve(reducedSystem, &cameraDelta, context.stream());
        if (params.dampingParams.diagonalDamping) {
          sparseSchurProblem.recoverPoints(lambda, dampingDiagonal,
                                           cameraDelta, &delta,
                                           context.stream());
        } else {
          sparseSchurProblem.recoverPoints(lambda, cameraDelta, &delta,
                                           context.stream());
        }
        attemptProfile.cudssSolveElapsed =
            detailedElapsedSinceAfterSync(detailedProfiling, stageStart,
                                          context.stream());
        result.cudssSolveElapsed += attemptProfile.cudssSolveElapsed;
      } else if (solverMode == SfmLinearSolverType::PcgSchur) {
        stageStart = detailedProfileStart(detailedProfiling);
        const SfmImplicitSchurView implicit =
            params.dampingParams.diagonalDamping
                ? sparseSchurProblem.prepareImplicit(
                      lambda, dampingDiagonal, context.stream())
                : sparseSchurProblem.prepareImplicit(lambda, context.stream());
        reducedSession->solve(*implicit.linearOperator,
                              *implicit.preconditioner, implicit.rhs,
                              &cameraDelta, context.stream());
        if (params.dampingParams.diagonalDamping) {
          sparseSchurProblem.recoverPoints(lambda, dampingDiagonal,
                                           cameraDelta, &delta,
                                           context.stream());
        } else {
          sparseSchurProblem.recoverPoints(lambda, cameraDelta, &delta,
                                           context.stream());
        }
        attemptProfile.denseSchurSolveElapsed =
            detailedElapsedSinceAfterSync(detailedProfiling, stageStart,
                                          context.stream());
        result.denseSchurSolveElapsed +=
            attemptProfile.denseSchurSolveElapsed;
      }
      ++result.innerIterations;

      double oldLinearizedError = 0.0;
      double newLinearizedError = 0.0;
      stageStart = detailedProfileStart(detailedProfiling);
      const SfmProjectionLinearization* activeLinearization = nullptr;
      if (solverMode == SfmLinearSolverType::DenseSchur) {
        activeLinearization = &denseSchurSolver.linearization();
      } else {
        activeLinearization = &sparseSchurProblem.linearization();
      }
      SfmReductionTransferProfile linearizedErrorTransfer;
      const double linearizedCostChange =
          computeSfmLinearizedErrorChange(
              *activeLinearization, batch, numCameras, delta,
              &oldLinearizedError, &newLinearizedError, context.stream(),
              detailedProfiling ? &linearizedErrorTransfer : nullptr);
      if (detailedProfiling) {
        addReductionD2hTransfer(linearizedErrorTransfer, &result);
      }
      attemptProfile.linearizedErrorElapsed =
          detailedElapsedSinceAfterSync(detailedProfiling, stageStart,
                                        context.stream());
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
        stageStart = detailedProfileStart(detailedProfiling);
        applyDelta(current, delta, &trial, numCameras, numPoints,
                   context.stream());
        attemptProfile.applyDeltaElapsed =
            detailedElapsedSinceAfterSync(detailedProfiling, stageStart,
                                          context.stream());
        result.applyDeltaElapsed += attemptProfile.applyDeltaElapsed;

        stageStart = detailedProfileStart(detailedProfiling);
        SfmReductionTransferProfile trialErrorTransfer;
        trialError = computeSfmProjectionError(
            trial, batch, context.stream(),
            detailedProfiling ? &trialErrorTransfer : nullptr);
        if (detailedProfiling) {
          addReductionD2hTransfer(trialErrorTransfer, &result);
        }
        attemptProfile.trialErrorElapsed =
            detailedElapsedSinceAfterSync(detailedProfiling, stageStart,
                                          context.stream());
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
        stageStart = detailedProfileStart(detailedProfiling);
        acceptTrial(trial, &current, context.stream());
        iterationProfile.acceptTrialElapsed =
            detailedElapsedSinceAfterSync(detailedProfiling, stageStart,
                                          context.stream());
        result.acceptTrialElapsed += iterationProfile.acceptTrialElapsed;
        currentError = trialError;
        ++result.iterations;
        ++result.acceptedSteps;
        stageStart = detailedProfileStart(detailedProfiling);
        decreaseLambda(params, modelFidelity, &lambda, &currentFactor);
        acceptedOrDone = true;
        terminate =
            checkLevenbergMarquardtConvergence(params, previousError, currentError);
        attemptProfile.lambdaUpdateElapsed =
            detailedElapsedSince(detailedProfiling, stageStart);
        result.lambdaUpdateElapsed += attemptProfile.lambdaUpdateElapsed;
        iterationProfile.acceptedStep = true;
        attemptProfile.accepted = true;
        attemptProfile.terminated = terminate;
      } else if (!stopSearchingLambda) {
        stageStart = detailedProfileStart(detailedProfiling);
        increaseLambda(params, &lambda, &currentFactor);
        if (lambda >= params.lambdaUpperBound) {
          acceptedOrDone = true;
          terminate = true;
          attemptProfile.lambdaUpperBoundReached = true;
        }
        attemptProfile.lambdaUpdateElapsed =
            detailedElapsedSince(detailedProfiling, stageStart);
        result.lambdaUpdateElapsed += attemptProfile.lambdaUpdateElapsed;
        attemptProfile.terminated = terminate;
      } else {
        acceptedOrDone = true;
        terminate = true;
        attemptProfile.terminated = true;
      }
      attemptProfile.totalElapsed =
          detailedElapsedSince(detailedProfiling, attemptStart);
      if (detailedProfiling) {
        iterationProfile.attemptProfiles.push_back(attemptProfile);
      }
    }
    iterationProfile.endError = currentError;
    iterationProfile.endLambda = lambda;
    iterationProfile.terminated = terminate;
    iterationProfile.totalElapsed =
        detailedElapsedSince(detailedProfiling, iterationStart);
    if (detailedProfiling) {
      result.iterationProfiles.push_back(iterationProfile);
    }
  }
  GTSAM_CUDA_CHECK(cudaStreamSynchronize(context.stream()));
  const auto solveLoopEnd = std::chrono::steady_clock::now();
  result.solveLoopElapsed =
      std::chrono::duration<double>(solveLoopEnd - solveLoopStart).count();

  result.finalError = currentError;
  result.finalLambda = lambda;
  if (reducedSession) {
    result.linearSolveStats = reducedSession->stats();
  } else if (solverMode == SfmLinearSolverType::DenseSchur) {
    result.linearSolveStats = denseSchurSolver.linearSolveStats();
  }
  if (detailedProfiling &&
      result.linearSolveStats.backend == gtsam::cuda::LinearSolverType::Pcg) {
    result.totalD2hBytes += result.linearSolveStats.pcgD2hBytes;
    result.totalD2hCopyElapsed += result.linearSolveStats.pcgD2hSeconds;
  }
  if (executionOptions.downloadOptimizedValues) {
    stageStart = Clock::now();
    SfmValuesDownloadProfile downloadProfile;
    result.optimizedValues =
        downloadSfmValues(current, context.stream(),
                          detailedProfiling ? &downloadProfile : nullptr);
    result.downloadElapsed = elapsedSince(stageStart);
    if (detailedProfiling) {
      setDownloadTransferProfile(downloadProfile, &result);
    }
  }
  result.totalMeasuredElapsed = elapsedSince(totalStart);
  return result;
}

}  // namespace gtsam::cuda

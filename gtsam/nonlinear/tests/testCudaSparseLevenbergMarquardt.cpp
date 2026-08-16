#include <CppUnitLite/TestHarness.h>
#include <cuda_runtime_api.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/types.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/CustomFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/cuda/CudaSparseLevenbergMarquardt.h>
#include <gtsam/nonlinear/cuda/DeviceSparseJacobianNormalEquations.h>
#include <gtsam/nonlinear/cuda/HostSparseJacobian.h>
#include <gtsam/nonlinear/cuda/SparseJacobianPlan.h>
#include <gtsam/nonlinear/cuda/StreamingSparseJacobianLinearizer.h>
#include <gtsam/sam/RangeFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/ReferenceFrameFactor.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <typeinfo>
#include <utility>
#include <vector>

using namespace gtsam;
using namespace gtsam::cuda;

namespace {

const Key kPose0 = Symbol('x', 0);
const Key kPose1 = Symbol('x', 1);
const Key kPose2 = Symbol('x', 2);
const Key kScalar0 = Symbol('s', 0);
const Key kScalar1 = Symbol('s', 1);
const Key kHeteroPose0 = Symbol('q', 0);
const Key kHeteroPose1 = Symbol('q', 1);
const Key kGlobalPoint = Symbol('l', 0);
const Key kLocalPoint = Symbol('l', 1);

struct Pose2LmProblem {
  NonlinearFactorGraph graph;
  Values initial;
};

bool IsFiniteNonnegativeTiming(double seconds) {
  return std::isfinite(seconds) && seconds >= 0.0;
}

bool AllSparseLmTimingsAreFiniteNonnegative(
    const CudaSparseLmStageTimings& timings) {
  return IsFiniteNonnegativeTiming(timings.totalWall) &&
         IsFiniteNonnegativeTiming(timings.initialError) &&
         IsFiniteNonnegativeTiming(timings.plan) &&
         IsFiniteNonnegativeTiming(timings.persistentSetupWall) &&
         IsFiniteNonnegativeTiming(timings.deviceInitializeWall) &&
         IsFiniteNonnegativeTiming(timings.patternH2d) &&
         IsFiniteNonnegativeTiming(timings.structureSetup) &&
         IsFiniteNonnegativeTiming(timings.setupD2h) &&
         IsFiniteNonnegativeTiming(timings.hostZero) &&
         IsFiniteNonnegativeTiming(timings.factorLinearizationAndPackingWall) &&
         IsFiniteNonnegativeTiming(timings.factorLinearizationCpuSum) &&
         IsFiniteNonnegativeTiming(timings.csrPackingCpuSum) &&
         IsFiniteNonnegativeTiming(timings.numericH2d) &&
         IsFiniteNonnegativeTiming(timings.transposeUpdate) &&
         IsFiniteNonnegativeTiming(timings.normalJtJ) &&
         IsFiniteNonnegativeTiming(timings.normalJtb) &&
         IsFiniteNonnegativeTiming(timings.diagonalExtraction) &&
         IsFiniteNonnegativeTiming(timings.oldModelError) &&
         IsFiniteNonnegativeTiming(timings.dampingPreparation) &&
         IsFiniteNonnegativeTiming(timings.dampingApplication) &&
         IsFiniteNonnegativeTiming(timings.cudssAnalysis) &&
         IsFiniteNonnegativeTiming(timings.cudssFactorAndSolve) &&
         IsFiniteNonnegativeTiming(timings.cudssDataInfoBoundaryWall) &&
         IsFiniteNonnegativeTiming(timings.newModelError) &&
         IsFiniteNonnegativeTiming(timings.attemptD2h) &&
         IsFiniteNonnegativeTiming(timings.attemptHostBuild) &&
         IsFiniteNonnegativeTiming(timings.retract) &&
         IsFiniteNonnegativeTiming(timings.nonlinearTrialError) &&
         IsFiniteNonnegativeTiming(timings.upload) &&
         IsFiniteNonnegativeTiming(timings.normalEquations) &&
         IsFiniteNonnegativeTiming(timings.damping) &&
         IsFiniteNonnegativeTiming(timings.modelError) &&
         IsFiniteNonnegativeTiming(timings.deltaDownload);
}

bool AllSparseLmTimingsAreZero(const CudaSparseLmStageTimings& timings) {
  return timings.totalWall == 0.0 && timings.initialError == 0.0 &&
         timings.plan == 0.0 && timings.persistentSetupWall == 0.0 &&
         timings.deviceInitializeWall == 0.0 && timings.patternH2d == 0.0 &&
         timings.structureSetup == 0.0 && timings.setupD2h == 0.0 &&
         timings.hostZero == 0.0 &&
         timings.factorLinearizationAndPackingWall == 0.0 &&
         timings.factorLinearizationCpuSum == 0.0 &&
         timings.csrPackingCpuSum == 0.0 && timings.numericH2d == 0.0 &&
         timings.transposeUpdate == 0.0 && timings.normalJtJ == 0.0 &&
         timings.normalJtb == 0.0 && timings.diagonalExtraction == 0.0 &&
         timings.oldModelError == 0.0 && timings.dampingPreparation == 0.0 &&
         timings.dampingApplication == 0.0 && timings.cudssAnalysis == 0.0 &&
         timings.cudssFactorAndSolve == 0.0 &&
         timings.cudssDataInfoBoundaryWall == 0.0 &&
         timings.newModelError == 0.0 && timings.attemptD2h == 0.0 &&
         timings.attemptHostBuild == 0.0 && timings.retract == 0.0 &&
         timings.nonlinearTrialError == 0.0 && timings.upload == 0.0 &&
         timings.normalEquations == 0.0 && timings.damping == 0.0 &&
         timings.modelError == 0.0 && timings.deltaDownload == 0.0;
}

Pose2LmProblem MakePose2LmProblem() {
  Pose2LmProblem problem;
  const auto priorNoise =
      noiseModel::Diagonal::Sigmas((Vector(3) << 0.15, 0.15, 0.1).finished());
  const auto odometryNoise =
      noiseModel::Diagonal::Sigmas((Vector(3) << 0.2, 0.2, 0.15).finished());

  problem.graph.emplace_shared<PriorFactor<Pose2>>(kPose0, Pose2(0.0, 0.0, 0.0),
                                                   priorNoise);
  problem.graph.emplace_shared<BetweenFactor<Pose2>>(
      kPose0, kPose1, Pose2(2.0, 0.2, 0.1), odometryNoise);
  problem.graph.emplace_shared<BetweenFactor<Pose2>>(
      kPose1, kPose2, Pose2(1.5, -0.1, -0.08), odometryNoise);

  problem.initial.insert(kPose0, Pose2(0.35, -0.25, 0.18));
  problem.initial.insert(kPose1, Pose2(2.45, -0.35, -0.05));
  problem.initial.insert(kPose2, Pose2(3.15, 0.45, 0.2));
  return problem;
}

CudaSparseLevenbergMarquardtResult RunTwoOuterProfiledPose2(
    const Pose2LmProblem& problem, bool collectTiming) {
  CudaSparseLevenbergMarquardtParams params;
  LevenbergMarquardtParams::SetCeresDefaults(&params);
  params.maxIterations = 2;
  params.relativeErrorTol = 0.0;
  params.absoluteErrorTol = 0.0;
  params.errorTol = 0.0;
  params.useFixedLambdaFactor = true;
  params.collectTiming = collectTiming;

  CudaSparseLevenbergMarquardtOptimizer optimizer(problem.graph,
                                                  problem.initial, params);
  (void)optimizer.optimize();
  return optimizer.result();
}

bool SparseLmSizesAndTransfersAreExact(
    const CudaSparseLevenbergMarquardtResult& result,
    const NonlinearFactorGraph& graph, const SparseJacobianPlan& plan) {
  const CudaSparseLmSystemSize& size = result.systemSize;
  const CudaSparseLmTransferCounts& transfers = result.transfers;
  if (size.factors != graph.size() ||
      size.jacobianRows != static_cast<size_t>(plan.rows()) ||
      size.jacobianColumns != static_cast<size_t>(plan.columns()) ||
      size.jacobianNonzeros != static_cast<size_t>(plan.nonzeros()) ||
      size.normalNonzeros == 0) {
    return false;
  }

  const size_t expectedPatternH2d =
      sizeof(int) * (static_cast<size_t>(plan.rows() + 1 + plan.nonzeros()) +
                     static_cast<size_t>(plan.columns() + 1 +
                                         size.normalNonzeros + plan.columns()));
  const size_t expectedSetupD2h =
      2 * sizeof(int) *
      static_cast<size_t>(plan.columns() + 1 + size.normalNonzeros);
  const size_t expectedNumericH2d =
      result.outerLinearizations * sizeof(double) *
      static_cast<size_t>(plan.nonzeros() + plan.rows());
  const size_t expectedAttemptD2h = result.lambdaAttempts * sizeof(double) *
                                    static_cast<size_t>(plan.columns() + 2);

  return transfers.patternH2dBytes == expectedPatternH2d &&
         transfers.setupD2hBytes == expectedSetupD2h &&
         transfers.numericH2dBytes == expectedNumericH2d &&
         transfers.attemptD2hBytes == expectedAttemptD2h &&
         transfers.totalH2dBytes() == expectedPatternH2d + expectedNumericH2d &&
         transfers.totalD2hBytes() == expectedSetupD2h + expectedAttemptD2h;
}

bool SparseLmAggregateTimingsAreExact(const CudaSparseLmStageTimings& timings) {
  return timings.upload == timings.numericH2d &&
         timings.normalEquations == timings.normalJtJ + timings.normalJtb +
                                        timings.diagonalExtraction +
                                        timings.oldModelError &&
         timings.damping ==
             timings.dampingPreparation + timings.dampingApplication &&
         timings.modelError == timings.oldModelError + timings.newModelError &&
         timings.deltaDownload == timings.attemptD2h;
}

bool HasCudaDevice() {
  int count = 0;
  const cudaError_t status = cudaGetDeviceCount(&count);
  if (status != cudaSuccess) {
    // Do not leave a discovery error pending for a subsequent fallback test.
    (void)cudaGetLastError();
    return false;
  }
  return count > 0;
}

bool CanRunCudaSparseLm() {
#if GTSAM_ENABLE_CUDSS
  return HasCudaDevice() &&
         DeviceSparseJacobianNormalEquations::preflightCapability().supported;
#else
  return false;
#endif
}

class HessianOnlyScalarFactor : public NonlinearFactor {
 public:
  HessianOnlyScalarFactor(Key key, double target)
      : NonlinearFactor(KeyVector{key}), key_(key), target_(target) {}

  size_t dim() const override { return 1; }

  double error(const Values& values) const override {
    const double residual = values.at<double>(key_) - target_;
    return 0.5 * residual * residual;
  }

  std::shared_ptr<GaussianFactor> linearize(
      const Values& values) const override {
    const double residual = values.at<double>(key_) - target_;
    return std::make_shared<HessianFactor>(
        key_, Matrix::Identity(1, 1), Vector1(-residual), residual * residual);
  }

  NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<HessianOnlyScalarFactor>(*this);
  }

 private:
  Key key_;
  double target_;
};

struct ChangingRowState {
  mutable std::mutex mutex;
  std::vector<double> linearizedAt;
};

class ChangingRowScalarFactor : public NonlinearFactor {
 public:
  ChangingRowScalarFactor(Key key, double target,
                          std::shared_ptr<ChangingRowState> state)
      : NonlinearFactor(KeyVector{key}),
        key_(key),
        target_(target),
        state_(std::move(state)) {}

  size_t dim() const override { return 1; }
  bool sendable() const override { return false; }

  double error(const Values& values) const override {
    const double residual = values.at<double>(key_) - target_;
    return 0.5 * residual * residual;
  }

  std::shared_ptr<GaussianFactor> linearize(
      const Values& values) const override {
    const double value = values.at<double>(key_);
    {
      std::lock_guard<std::mutex> lock(state_->mutex);
      state_->linearizedAt.push_back(value);
    }

    const Eigen::Index rows = value < 0.0 ? 1 : 2;
    const double scale = 1.0 / std::sqrt(static_cast<double>(rows));
    const Matrix A = Matrix::Constant(rows, 1, scale);
    const Vector b = Vector::Constant(rows, (target_ - value) * scale);
    return std::make_shared<JacobianFactor>(key_, A, b);
  }

  NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<ChangingRowScalarFactor>(*this);
  }

 private:
  Key key_;
  double target_;
  std::shared_ptr<ChangingRowState> state_;
};

class ZeroRowScalarFactor : public NonlinearFactor {
 public:
  explicit ZeroRowScalarFactor(Key key) : NonlinearFactor(KeyVector{key}) {}

  size_t dim() const override { return 0; }
  double error(const Values&) const override { return 0.0; }
  std::shared_ptr<GaussianFactor> linearize(const Values&) const override {
    return {};
  }
  NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<ZeroRowScalarFactor>(*this);
  }
};

class NonFiniteJacobianScalarFactor : public NonlinearFactor {
 public:
  explicit NonFiniteJacobianScalarFactor(Key key)
      : NonlinearFactor(KeyVector{key}), key_(key) {}

  size_t dim() const override { return 1; }
  double error(const Values& values) const override {
    const double value = values.at<double>(key_);
    return 0.5 * value * value;
  }
  std::shared_ptr<GaussianFactor> linearize(
      const Values& values) const override {
    Matrix A = Matrix::Identity(1, 1);
    A(0, 0) = std::numeric_limits<double>::quiet_NaN();
    return std::make_shared<JacobianFactor>(key_, A,
                                            Vector1(-values.at<double>(key_)));
  }
  NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<NonFiniteJacobianScalarFactor>(*this);
  }

 private:
  Key key_;
};

struct CustomCallbackState {
  std::atomic<size_t> errorCalls{0};
  std::atomic<size_t> jacobianCalls{0};
  mutable std::mutex mutex;
  std::vector<std::thread::id> jacobianThreads;
};

struct SnapshotState {
  mutable std::mutex mutex;
  std::vector<Values> linearizationPoints;
};

class RecordingPoint2PriorFactor : public NoiseModelFactorN<Point2> {
 public:
  using Base = NoiseModelFactorN<Point2>;

  RecordingPoint2PriorFactor(Key key, const Point2& prior,
                             const SharedNoiseModel& model,
                             std::shared_ptr<SnapshotState> snapshots)
      : Base(model, key), prior_(prior), snapshots_(std::move(snapshots)) {}

  bool sendable() const override { return false; }

  Vector evaluateError(const Point2& point,
                       OptionalMatrixType H = OptionalNone) const override {
    if (H) *H = Matrix::Identity(2, 2);
    return point - prior_;
  }

  std::shared_ptr<GaussianFactor> linearize(
      const Values& values) const override {
    {
      std::lock_guard<std::mutex> lock(snapshots_->mutex);
      snapshots_->linearizationPoints.push_back(values);
    }
    return NoiseModelFactor::linearize(values);
  }

  NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<RecordingPoint2PriorFactor>(*this);
  }

 private:
  Point2 prior_;
  std::shared_ptr<SnapshotState> snapshots_;
};

struct DenseJacobianReference {
  Matrix jacobian;
  Vector rhs;
};

Matrix DenseFromCsr(const SparseJacobianPlan& plan,
                    const HostSparseJacobian& host) {
  Matrix dense = Matrix::Zero(plan.rows(), plan.columns());
  for (int row = 0; row < plan.rows(); ++row) {
    for (int index = plan.rowPointers()[static_cast<size_t>(row)];
         index < plan.rowPointers()[static_cast<size_t>(row + 1)]; ++index) {
      dense(row, plan.columnIndices()[static_cast<size_t>(index)]) =
          host.valuesData()[static_cast<size_t>(index)];
    }
  }
  return dense;
}

Vector RhsFromHost(const HostSparseJacobian& host) {
  Vector rhs(static_cast<Eigen::Index>(host.rhsSize()));
  for (size_t row = 0; row < host.rhsSize(); ++row) {
    rhs(static_cast<Eigen::Index>(row)) = host.rhsData()[row];
  }
  return rhs;
}

DenseJacobianReference AssembleDenseReferenceBySlot(
    const NonlinearFactorGraph& graph, const Values& values,
    const SparseJacobianColumnLayout& columns, const SparseJacobianPlan& plan) {
  DenseJacobianReference reference{Matrix::Zero(plan.rows(), plan.columns()),
                                   Vector::Zero(plan.rows())};
  const GaussianFactorGraph::shared_ptr linear = graph.linearize(values);
  if (linear->size() != graph.size()) {
    throw std::runtime_error("reference linearization lost graph slots");
  }

  for (size_t factorIndex = 0; factorIndex < linear->size(); ++factorIndex) {
    const GaussianFactor::shared_ptr& gaussian = (*linear)[factorIndex];
    if (!gaussian) continue;
    const auto jacobian = std::dynamic_pointer_cast<JacobianFactor>(gaussian);
    if (!jacobian) {
      throw std::runtime_error("reference requires JacobianFactor results");
    }

    const auto [localA, localB] = jacobian->jacobian();
    const SparseJacobianFactorWritePlan& factorPlan = plan.factor(factorIndex);
    Eigen::Index localColumn = 0;
    for (const Key key : jacobian->keys()) {
      const SparseJacobianColumnBlock& column = columns.at(key);
      reference.jacobian.block(factorPlan.rowBegin, column.columnBegin,
                               factorPlan.rowCount, column.dimension) =
          localA.middleCols(localColumn, column.dimension);
      localColumn += column.dimension;
    }
    reference.rhs.segment(factorPlan.rowBegin, factorPlan.rowCount) = localB;
  }
  return reference;
}

SharedNoiseModel MakeHeterogeneousNoise(const Vector& sigmas, bool robust) {
  const SharedNoiseModel diagonal = noiseModel::Diagonal::Sigmas(sigmas);
  if (!robust) return diagonal;
  return noiseModel::Robust::Create(
      noiseModel::mEstimator::Huber::Create(1.345), diagonal);
}

struct HeterogeneousProblem {
  NonlinearFactorGraph graph;
  Values initial;
  std::shared_ptr<SnapshotState> snapshots;
};

HeterogeneousProblem MakeHeterogeneousProblem(bool robust) {
  HeterogeneousProblem problem;
  problem.snapshots = std::make_shared<SnapshotState>();

  const Pose2 truePose0(0.3, -0.2, 0.15);
  const Pose2 odometry(1.2, 0.1, -0.08);
  const Pose2 truePose1 = truePose0.compose(odometry);
  const Point2 trueGlobal(2.4, 1.1);
  const Point2 trueLocal = truePose0.transformFrom(trueGlobal);
  const double trueRange = (trueGlobal - truePose1.translation()).norm();

  problem.graph.emplace_shared<PriorFactor<Pose2>>(
      kHeteroPose0, truePose0,
      MakeHeterogeneousNoise((Vector(3) << 0.12, 0.18, 0.09).finished(),
                             robust));
  problem.graph.push_back(std::make_shared<RecordingPoint2PriorFactor>(
      kGlobalPoint, trueGlobal,
      MakeHeterogeneousNoise((Vector(2) << 0.2, 0.15).finished(), robust),
      problem.snapshots));
  problem.graph.emplace_shared<BetweenFactor<Pose2>>(
      kHeteroPose0, kHeteroPose1, odometry,
      MakeHeterogeneousNoise((Vector(3) << 0.16, 0.14, 0.11).finished(),
                             robust));
  problem.graph.emplace_shared<RangeFactor<Pose2, Point2>>(
      kHeteroPose1, kGlobalPoint, trueRange,
      MakeHeterogeneousNoise(Vector1(0.12), robust));
  problem.graph.emplace_shared<ReferenceFrameFactor<Point2, Pose2>>(
      kGlobalPoint, kHeteroPose0, kLocalPoint,
      MakeHeterogeneousNoise((Vector(2) << 0.18, 0.22).finished(), robust));

  problem.initial.insert(
      kHeteroPose0,
      truePose0.retract((Vector(3) << 0.28, -0.24, 0.16).finished()));
  problem.initial.insert(
      kHeteroPose1,
      truePose1.retract((Vector(3) << -0.35, 0.31, -0.13).finished()));
  problem.initial.insert(kGlobalPoint, trueGlobal + Point2(0.42, -0.36));
  problem.initial.insert(kLocalPoint, trueLocal + Point2(-0.33, 0.29));
  return problem;
}

struct HookRecord {
  size_t iteration = 0;
  double oldError = 0.0;
  double newError = 0.0;
};

struct ReferenceAttempt {
  size_t acceptedIterationsBeforeAttempt = 0;
  // Match the existing CUDA SFM profile convention: zero-based per outer
  // linearization, resetting after each accepted step.
  size_t attempt = 0;
  double lambda = 0.0;
  bool accepted = false;
};

struct ReferenceLmRun {
  Values values;
  CudaSparseLmTerminationReason termination =
      CudaSparseLmTerminationReason::None;
  size_t outerLinearizations = 0;
  size_t acceptedSteps = 0;
  size_t lambdaAttempts = 0;
  double initialError = 0.0;
  double finalError = 0.0;
  double finalLambda = 0.0;
  std::vector<ReferenceAttempt> attempts;
  std::vector<HookRecord> hooks;
};

VectorValues ReferenceSqrtHessianDiagonal(
    const GaussianFactorGraph& linear, const LevenbergMarquardtParams& params) {
  VectorValues result;
  if (!params.dampingParams.diagonalDamping) return result;

  result = linear.hessianDiagonal();
  for (auto& [key, diagonal] : result) {
    (void)key;
    diagonal = diagonal.cwiseMax(params.dampingParams.minDiagonal)
                   .cwiseMin(params.dampingParams.maxDiagonal)
                   .cwiseSqrt();
  }
  return result;
}

// Drive GTSAM's public CPU tryLambda() one attempt at a time. This keeps the
// expected state-machine trace independent of the CUDA optimizer while using
// the same accepted/rejected-step semantics as ordinary GTSAM LM.
ReferenceLmRun RunCpuTryLambdaReference(
    const NonlinearFactorGraph& graph, const Values& initial,
    const LevenbergMarquardtParams& requestedParams) {
  ReferenceLmRun result;
  result.values = initial;
  result.initialError = graph.error(initial);
  result.finalError = result.initialError;
  result.finalLambda = requestedParams.lambdaInitial;

  if (result.initialError <= requestedParams.errorTol) {
    result.termination = CudaSparseLmTerminationReason::ErrorThreshold;
    return result;
  }
  if (requestedParams.maxIterations == 0) {
    result.termination = CudaSparseLmTerminationReason::MaxIterations;
    return result;
  }

  LevenbergMarquardtParams cpuParams = requestedParams;
  cpuParams.iterationHook = {};
  LevenbergMarquardtOptimizer optimizer(graph, initial, cpuParams);

  while (true) {
    const double previousError = optimizer.error();
    const size_t acceptedBefore = optimizer.iterations();
    const GaussianFactorGraph::shared_ptr linear = optimizer.linearize();
    const VectorValues sqrtDiagonal =
        ReferenceSqrtHessianDiagonal(*linear, cpuParams);
    ++result.outerLinearizations;

    bool accepted = false;
    size_t attemptWithinOuter = 0;
    while (true) {
      const double lambdaBeforeAttempt = optimizer.lambda();
      const bool stop = optimizer.tryLambda(*linear, sqrtDiagonal);
      accepted = optimizer.iterations() > acceptedBefore;

      ++result.lambdaAttempts;
      result.attempts.push_back(ReferenceAttempt{
          acceptedBefore, attemptWithinOuter++, lambdaBeforeAttempt, accepted});
      if (stop) break;
    }

    result.hooks.push_back(
        HookRecord{optimizer.iterations(), previousError, optimizer.error()});

    if (!accepted) {
      result.termination = optimizer.lambda() >= cpuParams.lambdaUpperBound
                               ? CudaSparseLmTerminationReason::LambdaUpperBound
                               : CudaSparseLmTerminationReason::SmallCostChange;
      break;
    }
    if (optimizer.iterations() >= cpuParams.maxIterations) {
      result.termination = CudaSparseLmTerminationReason::MaxIterations;
      break;
    }
    if (checkConvergence(cpuParams, previousError, optimizer.error())) {
      result.termination = CudaSparseLmTerminationReason::Converged;
      break;
    }
  }

  result.values = optimizer.values();
  result.acceptedSteps = optimizer.iterations();
  result.finalError = optimizer.error();
  result.finalLambda = optimizer.lambda();
  return result;
}

void CheckCpuFallback(
    TestResult& result_, const std::string& name_,
    const NonlinearFactorGraph& graph, const Values& initial,
    CudaSparseLmFallbackReason expectedReason,
    DirectJacobianFailure expectedFailure = DirectJacobianFailure::None,
    size_t expectedFactorIndex = std::numeric_limits<size_t>::max(),
    const std::string& expectedStatusDetail = {},
    const std::string& exactFallbackDetail = {}) {
  CudaSparseLevenbergMarquardtParams params;
  LevenbergMarquardtParams::SetCeresDefaults(&params);
  params.maxIterations = 20;
  params.collectTiming = true;

  const Values expected =
      LevenbergMarquardtOptimizer(graph, initial, params).optimize();
  CudaSparseLevenbergMarquardtOptimizer optimizer(graph, initial, params);
  const Values actual = optimizer.optimize();
  const CudaSparseLevenbergMarquardtResult& result = optimizer.result();

  DOUBLES_EQUAL(graph.error(expected), graph.error(actual), 1e-8);
  EXPECT(assert_equal(expected, actual, 1e-7));
  CHECK(result.backend == CudaSparseLmBackend::CpuFallback);
  CHECK(result.fallbackReason == expectedReason);
  CHECK(result.fallbackStatus.failure == expectedFailure);
  CHECK(!result.fallbackDetail.empty());
  if (!expectedStatusDetail.empty()) {
    CHECK(result.fallbackStatus.detail == expectedStatusDetail);
  }
  if (!exactFallbackDetail.empty()) {
    CHECK(result.fallbackDetail == exactFallbackDetail);
  }
  DOUBLES_EQUAL(graph.error(initial), result.initialError, 1e-12);
  DOUBLES_EQUAL(graph.error(actual), result.finalError, 1e-12);
  if (expectedReason == CudaSparseLmFallbackReason::PlanIncompatible) {
    CHECK(std::isfinite(result.timings.plan));
    CHECK(result.timings.plan > 0.0);
  }

  if (expectedFailure != DirectJacobianFailure::None) {
    EXPECT_LONGS_EQUAL(expectedFactorIndex, result.fallbackStatus.factorIndex);
    CHECK(!result.fallbackStatus.detail.empty());
    CHECK(result.fallbackDetail.find(std::to_string(expectedFactorIndex)) !=
          std::string::npos);
    const std::string dynamicFactorType =
        demangle(typeid(*graph[expectedFactorIndex]).name());
    CHECK(result.fallbackDetail.find(dynamicFactorType) != std::string::npos);
    CHECK(result.fallbackDetail.find(result.fallbackStatus.detail) !=
          std::string::npos);
  }

  params.fallbackOnUnsupported = false;
  std::string thrownDetail;
  try {
    CudaSparseLevenbergMarquardtOptimizer disabled(graph, initial, params);
    (void)disabled.optimize();
    CHECK(false);
  } catch (const std::runtime_error& error) {
    thrownDetail = error.what();
  }
  CHECK(!thrownDetail.empty());
  CHECK(thrownDetail.find(result.fallbackDetail) != std::string::npos);
  if (!exactFallbackDetail.empty()) {
    CHECK(thrownDetail == exactFallbackDetail);
  }
  if (expectedFailure != DirectJacobianFailure::None) {
    CHECK(thrownDetail.find(std::to_string(expectedFactorIndex)) !=
          std::string::npos);
    CHECK(thrownDetail.find(
              demangle(typeid(*graph[expectedFactorIndex]).name())) !=
          std::string::npos);
  }
}

}  // namespace

static_assert(std::is_base_of_v<LevenbergMarquardtParams,
                                CudaSparseLevenbergMarquardtParams>);

TEST(CudaSparseLevenbergMarquardt, ExposesPrototypeParamsDefaults) {
  const CudaSparseLevenbergMarquardtParams params;
  CHECK(params.fallbackOnUnsupported);
  CHECK(!params.collectTiming);
  CHECK(!params.collectAttemptTrace);
  CHECK(!params.validateStructureEveryIteration);
}

TEST(CudaSparseLevenbergMarquardt,
     ProfilesTwoAcceptedOuterIterationsWithExactTransfers) {
  if (!CanRunCudaSparseLm()) return;
  const Pose2LmProblem problem = MakePose2LmProblem();
  const SparseJacobianColumnLayout columns(problem.initial);
  const SparseJacobianPlan plan(problem.graph, columns);
  const CudaSparseLevenbergMarquardtResult result =
      RunTwoOuterProfiledPose2(problem, true);

  CHECK(result.backend == CudaSparseLmBackend::Cuda);
  EXPECT_LONGS_EQUAL(2, result.outerLinearizations);
  EXPECT_LONGS_EQUAL(2, result.acceptedSteps);
  EXPECT_LONGS_EQUAL(1, result.cudssAnalyses);
  CHECK(result.lambdaAttempts >= result.acceptedSteps);
  CHECK(SparseLmSizesAndTransfersAreExact(result, problem.graph, plan));
  CHECK(AllSparseLmTimingsAreFiniteNonnegative(result.timings));
  CHECK(SparseLmAggregateTimingsAreExact(result.timings));
  CHECK(result.timings.totalWall > 0.0);
  CHECK(result.timings.factorLinearizationCpuSum > 0.0);
  const double mappedDeviceStageTotal =
      result.timings.patternH2d + result.timings.structureSetup +
      result.timings.setupD2h + result.timings.numericH2d +
      result.timings.transposeUpdate + result.timings.normalJtJ +
      result.timings.normalJtb + result.timings.diagonalExtraction +
      result.timings.oldModelError + result.timings.dampingPreparation +
      result.timings.dampingApplication + result.timings.cudssAnalysis +
      result.timings.cudssFactorAndSolve + result.timings.newModelError +
      result.timings.attemptD2h;
  CHECK(mappedDeviceStageTotal > 0.0);
}

TEST(CudaSparseLevenbergMarquardt,
     TimingDisabledKeepsTimingsZeroAndExactTransfers) {
  if (!CanRunCudaSparseLm()) return;
  const Pose2LmProblem problem = MakePose2LmProblem();
  const SparseJacobianColumnLayout columns(problem.initial);
  const SparseJacobianPlan plan(problem.graph, columns);
  const CudaSparseLevenbergMarquardtResult result =
      RunTwoOuterProfiledPose2(problem, false);

  CHECK(result.backend == CudaSparseLmBackend::Cuda);
  EXPECT_LONGS_EQUAL(2, result.outerLinearizations);
  EXPECT_LONGS_EQUAL(2, result.acceptedSteps);
  EXPECT_LONGS_EQUAL(1, result.cudssAnalyses);
  CHECK(SparseLmSizesAndTransfersAreExact(result, problem.graph, plan));
  CHECK(AllSparseLmTimingsAreZero(result.timings));
  CHECK(SparseLmAggregateTimingsAreExact(result.timings));
}

namespace {

void CheckPose2Parity(TestResult& result_, const std::string& name_,
                      bool diagonalDamping) {
  if (!CanRunCudaSparseLm()) return;

  const Pose2LmProblem problem = MakePose2LmProblem();
  CudaSparseLevenbergMarquardtParams params;
  LevenbergMarquardtParams::SetCeresDefaults(&params);
  params.maxIterations = 20;
  params.dampingParams.diagonalDamping = diagonalDamping;

  const Values expected =
      LevenbergMarquardtOptimizer(problem.graph, problem.initial, params)
          .optimize();
  CudaSparseLevenbergMarquardtOptimizer optimizer(problem.graph,
                                                  problem.initial, params);
  const Values actual = optimizer.optimize();
  const CudaSparseLevenbergMarquardtResult& result = optimizer.result();

  DOUBLES_EQUAL(problem.graph.error(expected), problem.graph.error(actual),
                1e-8);
  EXPECT(assert_equal(expected, actual, 1e-7));
  EXPECT(assert_equal(actual, optimizer.values(), 1e-12));
  DOUBLES_EQUAL(problem.graph.error(actual), optimizer.error(), 1e-12);
  CHECK(result.backend == CudaSparseLmBackend::Cuda);
  CHECK(result.fallbackReason == CudaSparseLmFallbackReason::None);
  CHECK(result.termination != CudaSparseLmTerminationReason::None);
  DOUBLES_EQUAL(problem.graph.error(problem.initial), result.initialError,
                1e-12);
  DOUBLES_EQUAL(problem.graph.error(actual), result.finalError, 1e-12);
  EXPECT_LONGS_EQUAL(1, result.cudssAnalyses);
}

}  // namespace

TEST(CudaSparseLevenbergMarquardt, MatchesCpuPose2Result) {
  CheckPose2Parity(result_, name_, true);
}

TEST(CudaSparseLevenbergMarquardt, MatchesCpuPose2ResultWithIdentityDamping) {
  CheckPose2Parity(result_, name_, false);
}

TEST(CudaSparseLevenbergMarquardt, FallsBackForPlanIncompatible) {
  if (!CanRunCudaSparseLm()) return;
  Pose2LmProblem problem = MakePose2LmProblem();
  problem.initial.insert(Symbol('u', 0), Point2(3.0, -2.0));
  CheckCpuFallback(result_, name_, problem.graph, problem.initial,
                   CudaSparseLmFallbackReason::PlanIncompatible);
}

TEST(CudaSparseLevenbergMarquardt, FallsBackForDirectJacobianSemanticFailure) {
  if (!CanRunCudaSparseLm()) return;
  NonlinearFactorGraph graph;
  graph.emplace_shared<PriorFactor<Pose2>>(kPose0, Pose2(),
                                           noiseModel::Constrained::All(3));
  Values initial;
  initial.insert(kPose0, Pose2(0.2, -0.1, 0.05));

  CheckCpuFallback(result_, name_, graph, initial,
                   CudaSparseLmFallbackReason::DirectJacobianUnsupported,
                   DirectJacobianFailure::ConstrainedFactor, 0);
}

TEST(CudaSparseLevenbergMarquardt,
     FallsBackForReturnedHessianWithExactFactorStatus) {
  if (!CanRunCudaSparseLm()) return;

  NonlinearFactorGraph graph;
  graph.emplace_shared<PriorFactor<double>>(
      kScalar0, 1.0, noiseModel::Isotropic::Sigma(1, 0.5));
  graph.push_back(std::make_shared<HessianOnlyScalarFactor>(kScalar0, 2.0));
  Values initial;
  initial.insert(kScalar0, -3.0);

  CheckCpuFallback(result_, name_, graph, initial,
                   CudaSparseLmFallbackReason::DirectJacobianUnsupported,
                   DirectJacobianFailure::UnsupportedGaussianFactor, 1,
                   "linearization result is not a JacobianFactor");
}

TEST(CudaSparseLevenbergMarquardt,
     ChangingRowsRestartCpuFromOriginalAfterAcceptedCudaPrefix) {
  if (!CanRunCudaSparseLm()) return;

  const auto changingState = std::make_shared<ChangingRowState>();
  NonlinearFactorGraph graph;
  graph.emplace_shared<PriorFactor<double>>(kScalar0, 2.0,
                                            noiseModel::Unit::Create(1));
  graph.push_back(
      std::make_shared<ChangingRowScalarFactor>(kScalar0, 2.0, changingState));
  Values initial;
  initial.insert(kScalar0, -2.0);

  CudaSparseLevenbergMarquardtParams params;
  LevenbergMarquardtParams::SetCeresDefaults(&params);
  params.maxIterations = 10;
  params.collectTiming = true;
  const Values expected =
      LevenbergMarquardtOptimizer(graph, initial, params).optimize();
  {
    std::lock_guard<std::mutex> lock(changingState->mutex);
    changingState->linearizedAt.clear();
  }

  std::vector<HookRecord> hooks;
  params.iterationHook = [&](size_t iteration, double oldError,
                             double newError) {
    hooks.push_back(HookRecord{iteration, oldError, newError});
  };
  CudaSparseLevenbergMarquardtOptimizer optimizer(graph, initial, params);
  const Values actual = optimizer.optimize();
  const CudaSparseLevenbergMarquardtResult& result = optimizer.result();

  EXPECT(assert_equal(expected, actual, 1e-9));
  DOUBLES_EQUAL(graph.error(expected), graph.error(actual), 1e-10);
  CHECK(result.backend == CudaSparseLmBackend::CpuFallback);
  CHECK(result.fallbackReason ==
        CudaSparseLmFallbackReason::DirectJacobianUnsupported);
  CHECK(result.fallbackStatus.failure ==
        DirectJacobianFailure::StructuralMismatch);
  EXPECT_LONGS_EQUAL(1, result.fallbackStatus.factorIndex);
  CHECK(result.fallbackStatus.detail ==
        "Jacobian row count does not match the sparse plan");
  CHECK(result.fallbackDetail.find("1") != std::string::npos);
  CHECK(result.fallbackDetail.find(demangle(typeid(*graph[1]).name())) !=
        std::string::npos);

  // Result counters intentionally describe only the abandoned CUDA prefix;
  // the final Values/error below describe a fresh CPU solve from `initial`.
  EXPECT_LONGS_EQUAL(2, result.outerLinearizations);
  EXPECT_LONGS_EQUAL(1, result.iterations);
  EXPECT_LONGS_EQUAL(1, result.acceptedSteps);
  EXPECT_LONGS_EQUAL(1, result.lambdaAttempts);
  EXPECT_LONGS_EQUAL(1, result.cudssAnalyses);
  const SparseJacobianColumnLayout prefixColumns(initial);
  const SparseJacobianPlan prefixPlan(graph, prefixColumns);
  CHECK(result.systemSize.factors == graph.size());
  CHECK(result.systemSize.jacobianRows ==
        static_cast<size_t>(prefixPlan.rows()));
  CHECK(result.systemSize.jacobianColumns ==
        static_cast<size_t>(prefixPlan.columns()));
  CHECK(result.systemSize.jacobianNonzeros ==
        static_cast<size_t>(prefixPlan.nonzeros()));
  CHECK(result.systemSize.normalNonzeros > 0);
  CHECK(result.transfers.numericH2dBytes ==
        sizeof(double) *
            static_cast<size_t>(prefixPlan.nonzeros() + prefixPlan.rows()));
  CHECK(result.transfers.attemptD2hBytes ==
        sizeof(double) * static_cast<size_t>(prefixPlan.columns() + 2));
  CHECK(result.transfers.patternH2dBytes > 0);
  CHECK(result.transfers.setupD2hBytes > 0);
  CHECK(result.timings.totalWall > 0.0);
  DOUBLES_EQUAL(graph.error(initial), result.initialError, 1e-12);
  DOUBLES_EQUAL(graph.error(actual), result.finalError, 1e-12);

  std::vector<double> linearizedAt;
  {
    std::lock_guard<std::mutex> lock(changingState->mutex);
    linearizedAt = changingState->linearizedAt;
  }
  CHECK(linearizedAt.size() >= 3);
  DOUBLES_EQUAL(-2.0, linearizedAt[0], 1e-12);
  CHECK(linearizedAt[1] > 0.0);
  DOUBLES_EQUAL(-2.0, linearizedAt[2], 1e-12);

  // Hooks are also two-phase: one accepted CUDA iteration is reported, then
  // CPU LM restarts its iteration numbering and old error from the original.
  CHECK(hooks.size() >= 2);
  EXPECT_LONGS_EQUAL(1, hooks[0].iteration);
  EXPECT_LONGS_EQUAL(1, hooks[1].iteration);
  DOUBLES_EQUAL(graph.error(initial), hooks[0].oldError, 1e-12);
  DOUBLES_EQUAL(graph.error(initial), hooks[1].oldError, 1e-12);

  params.fallbackOnUnsupported = false;
  params.iterationHook = {};
  std::string disabledDetail;
  try {
    CudaSparseLevenbergMarquardtOptimizer disabled(graph, initial, params);
    (void)disabled.optimize();
    CHECK(false);
  } catch (const std::runtime_error& error) {
    disabledDetail = error.what();
  }
  CHECK(disabledDetail.find("factor 1") != std::string::npos);
  CHECK(disabledDetail.find(demangle(typeid(*graph[1]).name())) !=
        std::string::npos);
  CHECK(disabledDetail.find(
            "Jacobian row count does not match the sparse plan") !=
        std::string::npos);
}

TEST(CudaSparseLevenbergMarquardt,
     ZeroRowOnlyKeyIsPlanIncompatibleButCpuOrderingCanSolve) {
  if (!CanRunCudaSparseLm()) return;

  NonlinearFactorGraph graph;
  graph.emplace_shared<PriorFactor<double>>(kScalar0, 1.5,
                                            noiseModel::Unit::Create(1));
  graph.push_back(std::make_shared<ZeroRowScalarFactor>(kScalar1));
  Values initial;
  initial.insert(kScalar0, -2.0);
  initial.insert(kScalar1, 7.0);

  CudaSparseLevenbergMarquardtParams params;
  LevenbergMarquardtParams::SetCeresDefaults(&params);
  params.maxIterations = 10;
  Ordering cpuSolvableOrdering;
  cpuSolvableOrdering.push_back(kScalar0);
  params.ordering = cpuSolvableOrdering;

  const Values expected =
      LevenbergMarquardtOptimizer(graph, initial, params).optimize();
  CudaSparseLevenbergMarquardtOptimizer optimizer(graph, initial, params);
  const Values actual = optimizer.optimize();
  const CudaSparseLevenbergMarquardtResult& result = optimizer.result();

  EXPECT(assert_equal(expected, actual, 1e-9));
  DOUBLES_EQUAL(7.0, actual.at<double>(kScalar1), 1e-12);
  CHECK(result.backend == CudaSparseLmBackend::CpuFallback);
  CHECK(result.fallbackReason == CudaSparseLmFallbackReason::PlanIncompatible);
  CHECK(result.fallbackStatus.failure == DirectJacobianFailure::None);
  EXPECT_LONGS_EQUAL(std::numeric_limits<size_t>::max(),
                     result.fallbackStatus.factorIndex);
  CHECK(result.fallbackDetail.find("uncovered key") != std::string::npos);
  CHECK(result.fallbackDetail.find(DefaultKeyFormatter(kScalar1)) !=
        std::string::npos);

  params.fallbackOnUnsupported = false;
  std::string disabledDetail;
  try {
    CudaSparseLevenbergMarquardtOptimizer disabled(graph, initial, params);
    (void)disabled.optimize();
    CHECK(false);
  } catch (const std::runtime_error& error) {
    disabledDetail = error.what();
  }
  CHECK(disabledDetail == result.fallbackDetail);
}

TEST(CudaSparseLevenbergMarquardt,
     AppliesCustomGtsamOrderingToCudssWithoutChangingObjective) {
  if (!CanRunCudaSparseLm()) return;
  const Pose2LmProblem problem = MakePose2LmProblem();

  CudaSparseLevenbergMarquardtParams automaticParams;
  LevenbergMarquardtParams::SetCeresDefaults(&automaticParams);
  automaticParams.maxIterations = 5;
  CudaSparseLevenbergMarquardtOptimizer automaticOptimizer(
      problem.graph, problem.initial, automaticParams);
  const Values automaticValues = automaticOptimizer.optimize();

  CudaSparseLevenbergMarquardtParams orderedParams = automaticParams;
  orderedParams.setOrdering(Ordering{kPose2, kPose0, kPose1});
  CudaSparseLevenbergMarquardtOptimizer orderedOptimizer(
      problem.graph, problem.initial, orderedParams);
  const Values orderedValues = orderedOptimizer.optimize();

  EXPECT(assert_equal(automaticValues, orderedValues, 1e-9));
  DOUBLES_EQUAL(automaticOptimizer.error(), orderedOptimizer.error(), 1e-12);
  EXPECT(!automaticOptimizer.result()
              .linearSolveStats.userOrderingApplied);
  EXPECT(orderedOptimizer.result().linearSolveStats.userOrderingApplied);
  EXPECT(std::vector<int>({6, 7, 8, 0, 1, 2, 3, 4, 5}) ==
         orderedOptimizer.result().appliedScalarPermutation);
}

TEST(CudaSparseLevenbergMarquardt,
     GeneralPcgUsesSharedSessionAndMatchesDirectObjective) {
  if (!CanRunCudaSparseLm()) return;
  const Pose2LmProblem problem = MakePose2LmProblem();

  CudaSparseLevenbergMarquardtParams directParams;
  LevenbergMarquardtParams::SetCeresDefaults(&directParams);
  directParams.maxIterations = 5;
  CudaSparseLevenbergMarquardtOptimizer direct(
      problem.graph, problem.initial, directParams);
  (void)direct.optimize();

  CudaSparseLevenbergMarquardtParams pcgParams = directParams;
  pcgParams.linearSolver = CudaSparseLmLinearSolver::Pcg;
  pcgParams.pcg.maxIterations = 100;
  pcgParams.pcg.relativeTolerance = 1e-10;
  pcgParams.collectTiming = true;
  CudaSparseLevenbergMarquardtOptimizer pcg(
      problem.graph, problem.initial, pcgParams);
  const Values pcgValues = pcg.optimize();
  const CudaSparseLevenbergMarquardtResult& result = pcg.result();

  DOUBLES_EQUAL(direct.error(), pcg.error(), 1e-9);
  EXPECT(assert_equal(direct.values(), pcgValues, 1e-7));
  CHECK(result.linearSolveStats.backend == CudaLinearSolverType::Pcg);
  EXPECT_LONGS_EQUAL(1, result.linearSolveStats.analysisCount);
  EXPECT_LONGS_EQUAL(result.lambdaAttempts,
                     result.linearSolveStats.solveCount);
  EXPECT_LONGS_EQUAL(result.linearSolveStats.solveCount, result.pcgSolves);
  EXPECT_LONGS_EQUAL(result.linearSolveStats.pcgIterationsTotal,
                     result.pcgIterationsTotal);
  CHECK(result.linearSolveStats.lastPcgConverged);
  CHECK(result.linearSolveStats.lastPcgIterations > 0);
  CHECK(result.timings.pcgSolve > 0.0);
  EXPECT_LONGS_EQUAL(0, result.systemSize.normalNonzeros);
}

TEST(CudaSparseLevenbergMarquardt,
     NonFiniteJacobianIsFatalWithStageIndexAndType) {
  if (!CanRunCudaSparseLm()) return;

  NonlinearFactorGraph graph;
  graph.emplace_shared<PriorFactor<double>>(kScalar0, 0.0,
                                            noiseModel::Unit::Create(1));
  graph.push_back(std::make_shared<NonFiniteJacobianScalarFactor>(kScalar0));
  Values initial;
  initial.insert(kScalar0, 1.0);

  CudaSparseLevenbergMarquardtParams params;
  LevenbergMarquardtParams::SetCeresDefaults(&params);
  std::string detail;
  try {
    CudaSparseLevenbergMarquardtOptimizer optimizer(graph, initial, params);
    (void)optimizer.optimize();
    CHECK(false);
  } catch (const std::runtime_error& error) {
    detail = error.what();
  }

  CHECK(detail.find("factor linearization") != std::string::npos);
  CHECK(detail.find("factor 1") != std::string::npos);
  CHECK(detail.find(demangle(typeid(*graph[1]).name())) != std::string::npos);
  CHECK(detail.find("Jacobian block contains a non-finite coefficient") !=
        std::string::npos);
}

TEST(CudaSparseLevenbergMarquardt,
     NonSendableCustomFactorRunsEachCallbackExactlyOnCaller) {
  if (!CanRunCudaSparseLm()) return;

  const auto callbackState = std::make_shared<CustomCallbackState>();
  const std::thread::id callerThread = std::this_thread::get_id();
  constexpr double target = 1.25;
  const CustomErrorFunction callback = [callbackState, target](
                                           const CustomFactor&,
                                           const Values& values,
                                           const JacobianVector* jacobians) {
    if (jacobians) {
      callbackState->jacobianCalls.fetch_add(1);
      {
        std::lock_guard<std::mutex> lock(callbackState->mutex);
        callbackState->jacobianThreads.push_back(std::this_thread::get_id());
      }
      auto& mutableJacobians = *const_cast<JacobianVector*>(jacobians);
      mutableJacobians[0] = Matrix::Identity(1, 1);
    } else {
      callbackState->errorCalls.fetch_add(1);
    }
    return Vector1(values.at<double>(kScalar0) - target);
  };

  NonlinearFactorGraph graph;
  graph.emplace_shared<CustomFactor>(noiseModel::Unit::Create(1),
                                     KeyVector{kScalar0}, callback);
  Values initial;
  initial.insert(kScalar0, -3.0);

  CudaSparseLevenbergMarquardtParams params;
  LevenbergMarquardtParams::SetCeresDefaults(&params);
  params.maxIterations = 3;
  params.errorTol = 0.0;
  params.relativeErrorTol = 0.0;
  params.absoluteErrorTol = 0.0;

  CudaSparseLevenbergMarquardtOptimizer optimizer(graph, initial, params);
  (void)optimizer.optimize();
  const CudaSparseLevenbergMarquardtResult& result = optimizer.result();

  CHECK(result.backend == CudaSparseLmBackend::Cuda);
  CHECK(result.outerLinearizations > 0);
  EXPECT_LONGS_EQUAL(result.outerLinearizations,
                     callbackState->jacobianCalls.load());
  EXPECT_LONGS_EQUAL(1 + result.lambdaAttempts,
                     callbackState->errorCalls.load());
  CHECK(std::isfinite(result.finalError));

  std::vector<std::thread::id> jacobianThreads;
  {
    std::lock_guard<std::mutex> lock(callbackState->mutex);
    jacobianThreads = callbackState->jacobianThreads;
  }
  EXPECT_LONGS_EQUAL(result.outerLinearizations, jacobianThreads.size());
  for (const std::thread::id thread : jacobianThreads) {
    CHECK(thread == callerThread);
  }
}

namespace {

void CheckStreamedJacobiansAtSnapshots(TestResult& result_,
                                       const std::string& name_,
                                       const NonlinearFactorGraph& graph,
                                       const std::vector<Values>& snapshots) {
  for (const Values& values : snapshots) {
    const SparseJacobianColumnLayout columns(values);
    const SparseJacobianPlan plan(graph, columns);
    HostSparseJacobian host(plan);
    host.clear();

    const DirectJacobianStatus status =
        StreamingSparseJacobianLinearizer().linearize(graph, values, columns,
                                                      plan, &host);
    CHECK(status.ok());
    const DenseJacobianReference reference =
        AssembleDenseReferenceBySlot(graph, values, columns, plan);
    EXPECT(assert_equal(reference.jacobian, DenseFromCsr(plan, host), 1e-11));
    EXPECT(assert_equal(reference.rhs, RhsFromHost(host), 1e-11));
  }
}

void CheckHeterogeneousParity(TestResult& result_, const std::string& name_,
                              bool robust) {
  if (!CanRunCudaSparseLm()) return;

  HeterogeneousProblem cpuProblem = MakeHeterogeneousProblem(robust);
  HeterogeneousProblem cudaProblem = MakeHeterogeneousProblem(robust);
  CudaSparseLevenbergMarquardtParams params;
  LevenbergMarquardtParams::SetCeresDefaults(&params);
  params.maxIterations = 20;
  params.dampingParams.diagonalDamping = true;

  LevenbergMarquardtOptimizer cpuOptimizer(cpuProblem.graph, cpuProblem.initial,
                                           params);
  const Values expected = cpuOptimizer.optimize();
  CudaSparseLevenbergMarquardtOptimizer cudaOptimizer(
      cudaProblem.graph, cudaProblem.initial, params);
  const Values actual = cudaOptimizer.optimize();
  const CudaSparseLevenbergMarquardtResult& cudaResult = cudaOptimizer.result();

  std::vector<Values> snapshots;
  {
    std::lock_guard<std::mutex> lock(cudaProblem.snapshots->mutex);
    snapshots = cudaProblem.snapshots->linearizationPoints;
  }

  CHECK(cudaResult.backend == CudaSparseLmBackend::Cuda);
  // The CPU and CUDA direct solvers can take different accepted-step paths
  // after a numerically borderline LM trial while still converging to the
  // same solution.  Correctness is established below by objective and Values
  // parity, so require valid progress rather than identical iteration counts.
  CHECK(cpuOptimizer.iterations() > 0);
  CHECK(cudaResult.acceptedSteps > 0);
  CHECK(cudaResult.acceptedSteps <= cudaResult.outerLinearizations);
  CHECK(cudaResult.acceptedSteps <= cudaResult.lambdaAttempts);
  EXPECT_LONGS_EQUAL(cudaResult.outerLinearizations, snapshots.size());
  CHECK(!snapshots.empty());

  const double expectedError = cpuProblem.graph.error(expected);
  const double actualError = cudaProblem.graph.error(actual);
  const double errorTolerance =
      1e-8 * std::max({1.0, std::abs(expectedError), std::abs(actualError)});
  DOUBLES_EQUAL(expectedError, actualError, errorTolerance);
  DOUBLES_EQUAL(actualError, cudaResult.finalError, 1e-12);
  CHECK(expected.localCoordinates(actual).norm() <= 1e-6);

  CheckStreamedJacobiansAtSnapshots(result_, name_, cudaProblem.graph,
                                    snapshots);
}

}  // namespace

TEST(CudaSparseLevenbergMarquardt,
     MatchesHeterogeneousGraphWithDiagonalDamping) {
  CheckHeterogeneousParity(result_, name_, false);
}

TEST(CudaSparseLevenbergMarquardt,
     MatchesHeterogeneousHuberGraphWithDiagonalDamping) {
  CheckHeterogeneousParity(result_, name_, true);
}

TEST(CudaSparseLevenbergMarquardt, FallsBackWhenCudaUnavailable) {
  if (HasCudaDevice()) return;
  const Pose2LmProblem problem = MakePose2LmProblem();
  CheckCpuFallback(result_, name_, problem.graph, problem.initial,
                   CudaSparseLmFallbackReason::CudaUnavailable);
}

TEST(CudaSparseLevenbergMarquardt, FallsBackWhenToolkitUnsupported) {
#if GTSAM_ENABLE_CUDSS
  if (!HasCudaDevice()) return;
  if (DeviceSparseJacobianNormalEquations::preflightCapability().supported)
    return;
  const Pose2LmProblem problem = MakePose2LmProblem();
  CheckCpuFallback(result_, name_, problem.graph, problem.initial,
                   CudaSparseLmFallbackReason::CudaToolkitUnsupported);
#endif
}

#if !GTSAM_ENABLE_CUDSS
TEST(CudaSparseLevenbergMarquardt, FallsBackWhenCudssUnavailable) {
  if (!HasCudaDevice()) return;
  const Pose2LmProblem problem = MakePose2LmProblem();
  CheckCpuFallback(
      result_, name_, problem.graph, problem.initial,
      CudaSparseLmFallbackReason::CudssUnavailable, DirectJacobianFailure::None,
      std::numeric_limits<size_t>::max(), {}, "cuDSS support is not compiled");
}
#endif

namespace {

void CheckCudaAgainstReference(TestResult& result_, const std::string& name_,
                               const NonlinearFactorGraph& graph,
                               const Values& initial,
                               CudaSparseLevenbergMarquardtParams params,
                               const ReferenceLmRun& expected) {
  std::vector<HookRecord> actualHooks;
  params.iterationHook = [&](size_t iteration, double oldError,
                             double newError) {
    actualHooks.push_back(HookRecord{iteration, oldError, newError});
  };

  CudaSparseLevenbergMarquardtOptimizer optimizer(graph, initial, params);
  const Values actualValues = optimizer.optimize();
  const CudaSparseLevenbergMarquardtResult& actual = optimizer.result();

  CHECK(actual.backend == CudaSparseLmBackend::Cuda);
  CHECK(actual.fallbackReason == CudaSparseLmFallbackReason::None);
  CHECK(actual.termination == expected.termination);
  EXPECT_LONGS_EQUAL(expected.outerLinearizations, actual.outerLinearizations);
  EXPECT_LONGS_EQUAL(expected.acceptedSteps, actual.iterations);
  EXPECT_LONGS_EQUAL(expected.acceptedSteps, actual.acceptedSteps);
  EXPECT_LONGS_EQUAL(expected.lambdaAttempts, actual.lambdaAttempts);
  EXPECT_LONGS_EQUAL(expected.attempts.size(), actual.attemptTrace.size());
  DOUBLES_EQUAL(expected.initialError, actual.initialError, 1e-12);
  DOUBLES_EQUAL(expected.finalError, actual.finalError, 1e-8);
  DOUBLES_EQUAL(expected.finalLambda, actual.finalLambda, 1e-14);
  EXPECT(assert_equal(expected.values, actualValues, 1e-7));

  for (size_t i = 0; i < expected.attempts.size(); ++i) {
    const ReferenceAttempt& reference = expected.attempts[i];
    const CudaSparseLmAttemptRecord& attempt = actual.attemptTrace[i];
    EXPECT_LONGS_EQUAL(reference.acceptedIterationsBeforeAttempt,
                       attempt.acceptedIterationsBeforeAttempt);
    EXPECT_LONGS_EQUAL(reference.attempt, attempt.attempt);
    DOUBLES_EQUAL(reference.lambda, attempt.lambda, 1e-14);
    CHECK(reference.accepted == attempt.accepted);
    CHECK(std::isfinite(attempt.linearizedChange));
    CHECK(std::isfinite(attempt.nonlinearChange));
    CHECK(std::isfinite(attempt.modelFidelity));
  }

  EXPECT_LONGS_EQUAL(expected.hooks.size(), actualHooks.size());
  for (size_t i = 0; i < expected.hooks.size(); ++i) {
    EXPECT_LONGS_EQUAL(expected.hooks[i].iteration, actualHooks[i].iteration);
    DOUBLES_EQUAL(expected.hooks[i].oldError, actualHooks[i].oldError, 1e-8);
    DOUBLES_EQUAL(expected.hooks[i].newError, actualHooks[i].newError, 1e-8);
  }

  if (expected.outerLinearizations > 0) {
    EXPECT_LONGS_EQUAL(1, actual.cudssAnalyses);
  } else {
    EXPECT_LONGS_EQUAL(0, actual.cudssAnalyses);
  }
}

}  // namespace

namespace {

void CheckAcceptedTraceMode(TestResult& result_, const std::string& name_,
                            bool useFixedLambdaFactor) {
  if (!CanRunCudaSparseLm()) return;
  const Pose2LmProblem problem = MakePose2LmProblem();

  CudaSparseLevenbergMarquardtParams params;
  LevenbergMarquardtParams::SetCeresDefaults(&params);
  params.maxIterations = 2;
  params.relativeErrorTol = 0.0;
  params.absoluteErrorTol = 0.0;
  params.errorTol = 0.0;
  params.useFixedLambdaFactor = useFixedLambdaFactor;
  params.collectAttemptTrace = true;

  const ReferenceLmRun expected =
      RunCpuTryLambdaReference(problem.graph, problem.initial, params);
  CHECK(expected.termination == CudaSparseLmTerminationReason::MaxIterations);
  EXPECT_LONGS_EQUAL(2, expected.outerLinearizations);
  EXPECT_LONGS_EQUAL(2, expected.acceptedSteps);
  EXPECT_LONGS_EQUAL(2, expected.hooks.size());

  CheckCudaAgainstReference(result_, name_, problem.graph, problem.initial,
                            params, expected);
}

}  // namespace

TEST(CudaSparseLevenbergMarquardt,
     MatchesCpuFixedLambdaTraceAndReusesAnalysis) {
  CheckAcceptedTraceMode(result_, name_, true);
}

TEST(CudaSparseLevenbergMarquardt,
     MatchesCpuAdaptiveLambdaTraceAndReusesAnalysis) {
  CheckAcceptedTraceMode(result_, name_, false);
}

TEST(CudaSparseLevenbergMarquardt, InitialErrorThresholdExitsBeforeCudaSetup) {
  const Pose2LmProblem problem = MakePose2LmProblem();
  const double initialError = problem.graph.error(problem.initial);
  std::vector<HookRecord> hooks;

  CudaSparseLevenbergMarquardtParams params;
  LevenbergMarquardtParams::SetCeresDefaults(&params);
  params.errorTol = initialError + 1.0;
  params.fallbackOnUnsupported = false;
  params.collectAttemptTrace = true;
  params.iterationHook = [&](size_t iteration, double oldError,
                             double newError) {
    hooks.push_back(HookRecord{iteration, oldError, newError});
  };

  CudaSparseLevenbergMarquardtOptimizer optimizer(problem.graph,
                                                  problem.initial, params);
  const Values actual = optimizer.optimize();
  const CudaSparseLevenbergMarquardtResult& result = optimizer.result();

  EXPECT(assert_equal(problem.initial, actual, 1e-12));
  CHECK(result.backend == CudaSparseLmBackend::Cuda);
  CHECK(result.fallbackReason == CudaSparseLmFallbackReason::None);
  CHECK(result.termination == CudaSparseLmTerminationReason::ErrorThreshold);
  EXPECT_LONGS_EQUAL(0, result.outerLinearizations);
  EXPECT_LONGS_EQUAL(0, result.iterations);
  EXPECT_LONGS_EQUAL(0, result.lambdaAttempts);
  EXPECT_LONGS_EQUAL(0, result.acceptedSteps);
  EXPECT_LONGS_EQUAL(0, result.cudssAnalyses);
  EXPECT_LONGS_EQUAL(0, result.attemptTrace.size());
  EXPECT_LONGS_EQUAL(0, hooks.size());
  DOUBLES_EQUAL(initialError, result.initialError, 1e-12);
  DOUBLES_EQUAL(initialError, result.finalError, 1e-12);
  DOUBLES_EQUAL(params.lambdaInitial, result.finalLambda, 1e-15);
}

TEST(CudaSparseLevenbergMarquardt, ZeroMaxIterationsExitsBeforeCudaSetup) {
  const Pose2LmProblem problem = MakePose2LmProblem();
  CudaSparseLevenbergMarquardtParams params;
  LevenbergMarquardtParams::SetCeresDefaults(&params);
  params.maxIterations = 0;
  params.errorTol = 0.0;
  params.fallbackOnUnsupported = false;

  CudaSparseLevenbergMarquardtOptimizer optimizer(problem.graph,
                                                  problem.initial, params);
  const Values actual = optimizer.optimize();
  const CudaSparseLevenbergMarquardtResult& result = optimizer.result();

  EXPECT(assert_equal(problem.initial, actual, 1e-12));
  CHECK(result.termination == CudaSparseLmTerminationReason::MaxIterations);
  EXPECT_LONGS_EQUAL(0, result.outerLinearizations);
  EXPECT_LONGS_EQUAL(0, result.lambdaAttempts);
  EXPECT_LONGS_EQUAL(0, result.cudssAnalyses);
  DOUBLES_EQUAL(problem.graph.error(problem.initial), result.initialError,
                1e-12);
  DOUBLES_EQUAL(result.initialError, result.finalError, 1e-12);
  DOUBLES_EQUAL(params.lambdaInitial, result.finalLambda, 1e-15);
}

TEST(CudaSparseLevenbergMarquardt,
     RejectedSmallCostAttemptTerminatesWithoutIncreasingLambda) {
  if (!CanRunCudaSparseLm()) return;
  const Pose2LmProblem problem = MakePose2LmProblem();

  CudaSparseLevenbergMarquardtParams params;
  LevenbergMarquardtParams::SetCeresDefaults(&params);
  params.maxIterations = 10;
  params.useFixedLambdaFactor = true;
  params.minModelFidelity = 2.0;
  params.relativeErrorTol = 2.0;
  params.absoluteErrorTol = 0.0;
  params.errorTol = 0.0;
  params.collectAttemptTrace = true;

  const ReferenceLmRun expected =
      RunCpuTryLambdaReference(problem.graph, problem.initial, params);
  CHECK(expected.termination == CudaSparseLmTerminationReason::SmallCostChange);
  EXPECT_LONGS_EQUAL(1, expected.outerLinearizations);
  EXPECT_LONGS_EQUAL(1, expected.lambdaAttempts);
  EXPECT_LONGS_EQUAL(0, expected.acceptedSteps);
  CHECK(!expected.attempts.front().accepted);
  DOUBLES_EQUAL(params.lambdaInitial, expected.finalLambda, 1e-15);
  EXPECT_LONGS_EQUAL(1, expected.hooks.size());
  EXPECT_LONGS_EQUAL(0, expected.hooks.front().iteration);
  DOUBLES_EQUAL(expected.hooks.front().oldError,
                expected.hooks.front().newError, 1e-15);

  CheckCudaAgainstReference(result_, name_, problem.graph, problem.initial,
                            params, expected);
}

TEST(CudaSparseLevenbergMarquardt,
     AdaptiveRejectionsStopAtLambdaUpperBoundAfterHook) {
  if (!CanRunCudaSparseLm()) return;
  const Pose2LmProblem problem = MakePose2LmProblem();

  CudaSparseLevenbergMarquardtParams params;
  LevenbergMarquardtParams::SetCeresDefaults(&params);
  params.maxIterations = 10;
  params.lambdaInitial = 1.0;
  params.lambdaFactor = 2.0;
  params.lambdaLowerBound = 0.0;
  params.lambdaUpperBound = 8.0;
  params.useFixedLambdaFactor = false;
  params.minModelFidelity = 2.0;
  params.relativeErrorTol = 0.0;
  params.absoluteErrorTol = 0.0;
  params.errorTol = 0.0;
  params.collectAttemptTrace = true;

  const ReferenceLmRun expected =
      RunCpuTryLambdaReference(problem.graph, problem.initial, params);
  CHECK(expected.termination ==
        CudaSparseLmTerminationReason::LambdaUpperBound);
  EXPECT_LONGS_EQUAL(1, expected.outerLinearizations);
  EXPECT_LONGS_EQUAL(2, expected.lambdaAttempts);
  EXPECT_LONGS_EQUAL(0, expected.acceptedSteps);
  EXPECT_LONGS_EQUAL(2, expected.attempts.size());
  DOUBLES_EQUAL(1.0, expected.attempts[0].lambda, 1e-15);
  DOUBLES_EQUAL(2.0, expected.attempts[1].lambda, 1e-15);
  CHECK(!expected.attempts[0].accepted);
  CHECK(!expected.attempts[1].accepted);
  DOUBLES_EQUAL(params.lambdaUpperBound, expected.finalLambda, 1e-15);
  EXPECT_LONGS_EQUAL(1, expected.hooks.size());
  EXPECT_LONGS_EQUAL(0, expected.hooks.front().iteration);
  DOUBLES_EQUAL(expected.hooks.front().oldError,
                expected.hooks.front().newError, 1e-15);

  CheckCudaAgainstReference(result_, name_, problem.graph, problem.initial,
                            params, expected);
}

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}

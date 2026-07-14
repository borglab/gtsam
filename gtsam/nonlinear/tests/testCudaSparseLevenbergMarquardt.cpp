#include <gtsam/base/Testable.h>
#include <gtsam/base/types.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/cuda/CudaSparseLevenbergMarquardt.h>
#include <gtsam/nonlinear/cuda/DeviceSparseJacobianNormalEquations.h>
#include <gtsam/slam/BetweenFactor.h>

#include <CppUnitLite/TestHarness.h>

#include <cuda_runtime_api.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
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

struct Pose2LmProblem {
  NonlinearFactorGraph graph;
  Values initial;
};

Pose2LmProblem MakePose2LmProblem() {
  Pose2LmProblem problem;
  const auto priorNoise = noiseModel::Diagonal::Sigmas(
      (Vector(3) << 0.15, 0.15, 0.1).finished());
  const auto odometryNoise = noiseModel::Diagonal::Sigmas(
      (Vector(3) << 0.2, 0.2, 0.15).finished());

  problem.graph.emplace_shared<PriorFactor<Pose2>>(
      kPose0, Pose2(0.0, 0.0, 0.0), priorNoise);
  problem.graph.emplace_shared<BetweenFactor<Pose2>>(
      kPose0, kPose1, Pose2(2.0, 0.2, 0.1), odometryNoise);
  problem.graph.emplace_shared<BetweenFactor<Pose2>>(
      kPose1, kPose2, Pose2(1.5, -0.1, -0.08), odometryNoise);

  problem.initial.insert(kPose0, Pose2(0.35, -0.25, 0.18));
  problem.initial.insert(kPose1, Pose2(2.45, -0.35, -0.05));
  problem.initial.insert(kPose2, Pose2(3.15, 0.45, 0.2));
  return problem;
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
    const GaussianFactorGraph& linear,
    const LevenbergMarquardtParams& params) {
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
          acceptedBefore, attemptWithinOuter++, lambdaBeforeAttempt,
          accepted});
      if (stop) break;
    }

    result.hooks.push_back(
        HookRecord{optimizer.iterations(), previousError, optimizer.error()});

    if (!accepted) {
      result.termination =
          optimizer.lambda() >= cpuParams.lambdaUpperBound
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
    size_t expectedFactorIndex = std::numeric_limits<size_t>::max()) {
  CudaSparseLevenbergMarquardtParams params;
  LevenbergMarquardtParams::SetCeresDefaults(&params);
  params.maxIterations = 20;

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
  DOUBLES_EQUAL(graph.error(initial), result.initialError, 1e-12);
  DOUBLES_EQUAL(graph.error(actual), result.finalError, 1e-12);

  if (expectedFailure != DirectJacobianFailure::None) {
    EXPECT_LONGS_EQUAL(expectedFactorIndex,
                       result.fallbackStatus.factorIndex);
    CHECK(!result.fallbackStatus.detail.empty());
    CHECK(result.fallbackDetail.find(std::to_string(expectedFactorIndex)) !=
          std::string::npos);
    const std::string dynamicFactorType =
        demangle(typeid(*graph[expectedFactorIndex]).name());
    CHECK(result.fallbackDetail.find(dynamicFactorType) !=
          std::string::npos);
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
  CHECK(params.collectTiming);
  CHECK(!params.collectAttemptTrace);
  CHECK(!params.validateStructureEveryIteration);
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
  CudaSparseLevenbergMarquardtOptimizer optimizer(
      problem.graph, problem.initial, params);
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

TEST(CudaSparseLevenbergMarquardt,
     MatchesCpuPose2ResultWithIdentityDamping) {
  CheckPose2Parity(result_, name_, false);
}

TEST(CudaSparseLevenbergMarquardt, FallsBackForPlanIncompatible) {
  if (!CanRunCudaSparseLm()) return;
  Pose2LmProblem problem = MakePose2LmProblem();
  problem.initial.insert(Symbol('u', 0), Point2(3.0, -2.0));
  CheckCpuFallback(result_, name_, problem.graph, problem.initial,
                   CudaSparseLmFallbackReason::PlanIncompatible);
}

TEST(CudaSparseLevenbergMarquardt,
     FallsBackForDirectJacobianSemanticFailure) {
  if (!CanRunCudaSparseLm()) return;
  NonlinearFactorGraph graph;
  graph.emplace_shared<PriorFactor<Pose2>>(
      kPose0, Pose2(), noiseModel::Constrained::All(3));
  Values initial;
  initial.insert(kPose0, Pose2(0.2, -0.1, 0.05));

  CheckCpuFallback(result_, name_, graph, initial,
                   CudaSparseLmFallbackReason::DirectJacobianUnsupported,
                   DirectJacobianFailure::ConstrainedFactor, 0);
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
  CheckCpuFallback(result_, name_, problem.graph, problem.initial,
                   CudaSparseLmFallbackReason::CudssUnavailable);
}
#endif

namespace {

void CheckCudaAgainstReference(
    TestResult& result_, const std::string& name_,
    const NonlinearFactorGraph& graph, const Values& initial,
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
  EXPECT_LONGS_EQUAL(expected.outerLinearizations,
                     actual.outerLinearizations);
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
    EXPECT_LONGS_EQUAL(expected.hooks[i].iteration,
                       actualHooks[i].iteration);
    DOUBLES_EQUAL(expected.hooks[i].oldError, actualHooks[i].oldError,
                  1e-8);
    DOUBLES_EQUAL(expected.hooks[i].newError, actualHooks[i].newError,
                  1e-8);
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

TEST(CudaSparseLevenbergMarquardt,
     InitialErrorThresholdExitsBeforeCudaSetup) {
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

  CudaSparseLevenbergMarquardtOptimizer optimizer(
      problem.graph, problem.initial, params);
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

TEST(CudaSparseLevenbergMarquardt,
     ZeroMaxIterationsExitsBeforeCudaSetup) {
  const Pose2LmProblem problem = MakePose2LmProblem();
  CudaSparseLevenbergMarquardtParams params;
  LevenbergMarquardtParams::SetCeresDefaults(&params);
  params.maxIterations = 0;
  params.errorTol = 0.0;
  params.fallbackOnUnsupported = false;

  CudaSparseLevenbergMarquardtOptimizer optimizer(
      problem.graph, problem.initial, params);
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
  CHECK(expected.termination ==
        CudaSparseLmTerminationReason::SmallCostChange);
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

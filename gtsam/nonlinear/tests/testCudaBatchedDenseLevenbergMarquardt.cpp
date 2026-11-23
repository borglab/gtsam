/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testCudaBatchedDenseLevenbergMarquardt.cpp
 * @brief   Unit tests for the batched dense GPU Levenberg-Marquardt optimizer
 * @author  Shaji
 * @date    Nov 20, 2026
 */

#include <CppUnitLite/TestHarness.h>
#include <cuda_runtime_api.h>
#include <gtsam/base/Testable.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/cuda/BatchedDenseLevenbergMarquardt.h>
#include <gtsam/nonlinear/cuda/BatchedFactorGroupHook.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <stdexcept>
#include <thread>
#include <vector>

using namespace gtsam;
using namespace gtsam::cuda;

/* ************************************************************************* */
namespace batched_dense_lm_fixture {

bool HasCudaDevice() {
  int count = 0;
  const cudaError_t status = cudaGetDeviceCount(&count);
  if (status != cudaSuccess) {
    (void)cudaGetLastError();
    return false;
  }
  return count > 0;
}

const Key kPose0 = Symbol('x', 0);
const Key kPose1 = Symbol('x', 1);
const Key kPose2 = Symbol('x', 2);

// Two distinct 3-pose Pose2 LM problems, same topology but different
// odometry/initial values, to exercise per-item independence.
struct BatchPose2Problem {
  std::vector<NonlinearFactorGraph> graphs;
  std::vector<Values> initial;
};

NonlinearFactorGraph MakePose2Graph(double odometryYOffset) {
  NonlinearFactorGraph graph;
  const auto priorNoise =
      noiseModel::Diagonal::Sigmas((Vector(3) << 0.15, 0.15, 0.1).finished());
  const auto odometryNoise =
      noiseModel::Diagonal::Sigmas((Vector(3) << 0.2, 0.2, 0.15).finished());
  graph.emplace_shared<PriorFactor<Pose2>>(kPose0, Pose2(0.0, 0.0, 0.0),
                                           priorNoise);
  graph.emplace_shared<BetweenFactor<Pose2>>(
      kPose0, kPose1, Pose2(2.0, odometryYOffset, 0.1), odometryNoise);
  graph.emplace_shared<BetweenFactor<Pose2>>(
      kPose1, kPose2, Pose2(1.5, -0.1, -0.08), odometryNoise);
  return graph;
}

Values MakePose2Initial(double poseXOffset) {
  Values initial;
  initial.insert(kPose0, Pose2(0.35 + poseXOffset, -0.25, 0.18));
  initial.insert(kPose1, Pose2(2.45 + poseXOffset, -0.35, -0.05));
  initial.insert(kPose2, Pose2(3.15 + poseXOffset, 0.45, 0.2));
  return initial;
}

BatchPose2Problem MakeBatchPose2Problem() {
  BatchPose2Problem problem;
  problem.graphs = {MakePose2Graph(0.2), MakePose2Graph(-0.3)};
  problem.initial = {MakePose2Initial(0.0), MakePose2Initial(0.15)};
  return problem;
}

LevenbergMarquardtParams RealToleranceParams() {
  LevenbergMarquardtParams params;
  params.maxIterations = 20;
  return params;
}

const Key kVec0 = Symbol('v', 0);

// A single 3-vector prior with sigmas spanning 6 orders of magnitude, so
// flat and diagonal damping take genuinely different first steps.
NonlinearFactorGraph MakeIllScaledGraph() {
  NonlinearFactorGraph graph;
  const auto noise = noiseModel::Diagonal::Sigmas(
      (Vector(3) << 0.001, 1.0, 1000.0).finished());
  graph.emplace_shared<PriorFactor<Vector>>(
      kVec0, (Vector(3) << 1.0, 1.0, 1.0).finished(), noise);
  return graph;
}

Values MakeIllScaledInitial() {
  Values initial;
  initial.insert(kVec0, (Vector(3) << 0.0, 0.0, 0.0).finished());
  return initial;
}

// maxIterations=1 and a fixed lambdaInitial isolate the one damped step;
// comparing only final optima could hide a damping-policy mismatch.
LevenbergMarquardtParams OneStepParams(bool diagonalDamping) {
  LevenbergMarquardtParams params;
  params.maxIterations = 1;
  params.lambdaInitial = 10.0;
  params.dampingParams.diagonalDamping = diagonalDamping;
  return params;
}

}  // namespace batched_dense_lm_fixture
/* ************************************************************************* */

using namespace batched_dense_lm_fixture;

// Each item's final Values/error must match the real CPU optimizer.
TEST(BatchedDenseLevenbergMarquardt, MatchesCpuPose2ResultPerGraph) {
  if (!HasCudaDevice()) return;
  const BatchPose2Problem problem = MakeBatchPose2Problem();
  const LevenbergMarquardtParams params = RealToleranceParams();

  BatchedDenseLevenbergMarquardtOptimizer optimizer(problem.graphs,
                                                     problem.initial, params);
  const std::vector<Values>& actual = optimizer.optimize();

  for (size_t i = 0; i < problem.graphs.size(); ++i) {
    const Values expected =
        LevenbergMarquardtOptimizer(problem.graphs[i], problem.initial[i],
                                    params)
            .optimize();
    EXPECT(assert_equal(expected, actual[i], 1e-6));
    DOUBLES_EQUAL(problem.graphs[i].error(expected),
                 problem.graphs[i].error(actual[i]), 1e-9);
  }
}

// Per-item accept/reject should reproduce the CPU optimizer's iteration count.
TEST(BatchedDenseLevenbergMarquardt, AcceptedIterationsMatchCpuOptimizer) {
  if (!HasCudaDevice()) return;
  const BatchPose2Problem problem = MakeBatchPose2Problem();
  const LevenbergMarquardtParams params = RealToleranceParams();

  BatchedDenseLevenbergMarquardtOptimizer optimizer(problem.graphs,
                                                     problem.initial, params);
  optimizer.optimize();
  const auto& results = optimizer.results();

  for (size_t i = 0; i < problem.graphs.size(); ++i) {
    LevenbergMarquardtOptimizer cpu(problem.graphs[i], problem.initial[i],
                                    params);
    cpu.optimize();
    LONGS_EQUAL(static_cast<long>(cpu.iterations()),
               static_cast<long>(results[i].acceptedIterations));
  }
}

// Starting almost at the CPU-converged solution exercises the
// stopSearchingLambda-equivalent path (valid trial, negligible cost change).
TEST(BatchedDenseLevenbergMarquardt, MatchesCpuNearConvergence) {
  if (!HasCudaDevice()) return;
  const BatchPose2Problem problem = MakeBatchPose2Problem();
  const LevenbergMarquardtParams params = RealToleranceParams();

  std::vector<Values> nearConverged;
  for (size_t i = 0; i < problem.graphs.size(); ++i) {
    const Values converged =
        LevenbergMarquardtOptimizer(problem.graphs[i], problem.initial[i],
                                    params)
            .optimize();
    VectorValues tinyPerturbation;
    tinyPerturbation.insert(kPose0, Vector3(1e-6, -1e-6, 1e-7));
    tinyPerturbation.insert(kPose1, Vector3(-1e-6, 1e-6, -1e-7));
    tinyPerturbation.insert(kPose2, Vector3(1e-7, -1e-7, 1e-6));
    nearConverged.push_back(converged.retract(tinyPerturbation));
  }

  BatchedDenseLevenbergMarquardtOptimizer optimizer(
      problem.graphs, nearConverged, params);
  const std::vector<Values>& actual = optimizer.optimize();
  const auto& results = optimizer.results();

  for (size_t i = 0; i < problem.graphs.size(); ++i) {
    LevenbergMarquardtOptimizer cpu(problem.graphs[i], nearConverged[i],
                                    params);
    const Values expected = cpu.optimize();
    EXPECT(assert_equal(expected, actual[i], 1e-9));
    DOUBLES_EQUAL(problem.graphs[i].error(expected),
                 problem.graphs[i].error(actual[i]), 1e-9);
    LONGS_EQUAL(static_cast<long>(cpu.iterations()),
               static_cast<long>(results[i].acceptedIterations));
  }
}

// Verifies BatchedDenseLevenbergMarquardt::ValuesBeforeOptimizeAreInitial.
TEST(BatchedDenseLevenbergMarquardt, ValuesBeforeOptimizeAreInitial) {
  if (!HasCudaDevice()) return;
  const BatchPose2Problem problem = MakeBatchPose2Problem();
  BatchedDenseLevenbergMarquardtOptimizer optimizer(
      problem.graphs, problem.initial, RealToleranceParams());
  for (size_t i = 0; i < problem.graphs.size(); ++i) {
    EXPECT(assert_equal(problem.initial[i], optimizer.values()[i], 1e-12));
  }
}

// Verifies BatchedDenseLevenbergMarquardt::ThrowsForMismatchedBatchSizes.
TEST(BatchedDenseLevenbergMarquardt, ThrowsForMismatchedBatchSizes) {
  if (!HasCudaDevice()) return;
  const BatchPose2Problem problem = MakeBatchPose2Problem();
  std::vector<Values> tooFewInitialValues = {problem.initial.front()};
  CHECK_EXCEPTION(
      BatchedDenseLevenbergMarquardtOptimizer(problem.graphs,
                                              tooFewInitialValues),
      std::invalid_argument);
}

// Verifies BatchedDenseLevenbergMarquardt::ThrowsForMismatchedScalarDimensions
// -- every graph in the batch must linearize to the same total scalar
// dimension, since the batched dense solve requires a fixed per-item size.
TEST(BatchedDenseLevenbergMarquardt, ThrowsForMismatchedScalarDimensions) {
  if (!HasCudaDevice()) return;
  BatchPose2Problem problem = MakeBatchPose2Problem();
  // Give the second graph/initial an extra pose, so its scalar dimension
  // (12) differs from the first graph's (9).
  problem.graphs[1].emplace_shared<BetweenFactor<Pose2>>(
      kPose2, Symbol('x', 3), Pose2(1.0, 0.0, 0.0),
      noiseModel::Diagonal::Sigmas((Vector(3) << 0.2, 0.2, 0.15).finished()));
  problem.initial[1].insert(Symbol('x', 3), Pose2(4.0, 0.5, 0.2));
  CHECK_EXCEPTION(
      BatchedDenseLevenbergMarquardtOptimizer(problem.graphs, problem.initial),
      std::invalid_argument);
}

// One LM step with diagonalDamping=true must match the CPU optimizer's
// actual diagonal-scaled step, not just the eventual converged optimum.
TEST(BatchedDenseLevenbergMarquardt, DiagonalDampingMatchesCpuOnIllScaledGraph) {
  if (!HasCudaDevice()) return;
  const std::vector<NonlinearFactorGraph> graphs = {MakeIllScaledGraph()};
  const std::vector<Values> initial = {MakeIllScaledInitial()};
  const LevenbergMarquardtParams params = OneStepParams(/*diagonalDamping=*/true);

  BatchedDenseLevenbergMarquardtOptimizer optimizer(graphs, initial, params);
  const Values actual = optimizer.optimize().front();

  const Values expected =
      LevenbergMarquardtOptimizer(graphs[0], initial[0], params).optimize();
  EXPECT(assert_equal(expected, actual, 1e-9));
}

// Same problem/step count with diagonalDamping=false (the default): must
// also match the CPU optimizer's flat-damping step.
TEST(BatchedDenseLevenbergMarquardt, FlatDampingMatchesCpuOnIllScaledGraph) {
  if (!HasCudaDevice()) return;
  const std::vector<NonlinearFactorGraph> graphs = {MakeIllScaledGraph()};
  const std::vector<Values> initial = {MakeIllScaledInitial()};
  const LevenbergMarquardtParams params = OneStepParams(/*diagonalDamping=*/false);

  BatchedDenseLevenbergMarquardtOptimizer optimizer(graphs, initial, params);
  const Values actual = optimizer.optimize().front();

  const Values expected =
      LevenbergMarquardtOptimizer(graphs[0], initial[0], params).optimize();
  EXPECT(assert_equal(expected, actual, 1e-9));
}

// The two damping policies must actually disagree on this ill-scaled graph
// -- otherwise the test above wouldn't be exercising diagonal damping at
// all, just flat damping twice by coincidence.
TEST(BatchedDenseLevenbergMarquardt, DiagonalAndFlatDampingDisagreeOnIllScaledGraph) {
  if (!HasCudaDevice()) return;
  const std::vector<NonlinearFactorGraph> graphs = {MakeIllScaledGraph()};
  const std::vector<Values> initial = {MakeIllScaledInitial()};

  BatchedDenseLevenbergMarquardtOptimizer diagonalOptimizer(
      graphs, initial, OneStepParams(/*diagonalDamping=*/true));
  BatchedDenseLevenbergMarquardtOptimizer flatOptimizer(
      graphs, initial, OneStepParams(/*diagonalDamping=*/false));

  const Vector diagonalResult =
      diagonalOptimizer.optimize().front().at<Vector>(kVec0);
  const Vector flatResult = flatOptimizer.optimize().front().at<Vector>(kVec0);
  CHECK((diagonalResult - flatResult).norm() > 1e-3);
}

// Verifies factorGroupHooks wiring actually runs: stage() once per
// active item per outer iteration, computeBatch() once per iteration.
// No test anywhere exercised BatchedFactorGroupHook before this.
TEST(BatchedDenseLevenbergMarquardt, FactorGroupHookIsStagedAndComputed) {
  if (!HasCudaDevice()) return;
  const BatchPose2Problem problem = MakeBatchPose2Problem();
  const LevenbergMarquardtParams params = RealToleranceParams();

  std::vector<int> stagedItems;
  int computeBatchCalls = 0;
  BatchedFactorGroupHook hook(
      [&](int item, const Values&) { stagedItems.push_back(item); },
      [&]() { ++computeBatchCalls; });

  BatchedDenseLevenbergMarquardtOptimizer optimizer(
      problem.graphs, problem.initial, params,
      BatchedDenseSolverOptions(), {hook});
  optimizer.optimize();

  CHECK(computeBatchCalls > 0);
  CHECK(!stagedItems.empty());
  for (int item : stagedItems) {
    CHECK(item >= 0);
    CHECK(item < static_cast<int>(problem.graphs.size()));
  }
}

// A hook that no factor actually reads cached results from must be a
// complete no-op for the optimization result -- adding factorGroupHooks
// support must not perturb existing callers that don't use it.
TEST(BatchedDenseLevenbergMarquardt, FactorGroupHookDoesNotChangeResultWhenUnused) {
  if (!HasCudaDevice()) return;
  const BatchPose2Problem problem = MakeBatchPose2Problem();
  const LevenbergMarquardtParams params = RealToleranceParams();

  BatchedDenseLevenbergMarquardtOptimizer withoutHook(problem.graphs,
                                                     problem.initial, params);
  const std::vector<Values> expected = withoutHook.optimize();

  BatchedFactorGroupHook inertHook([](int, const Values&) {}, []() {});
  BatchedDenseLevenbergMarquardtOptimizer withHook(
      problem.graphs, problem.initial, params, BatchedDenseSolverOptions(),
      {inertHook});
  const std::vector<Values> actual = withHook.optimize();

  for (size_t i = 0; i < expected.size(); ++i) {
    EXPECT(assert_equal(expected[i], actual[i], 1e-9));
  }
}

// Each device's construct+optimize must run on its own host thread:
// cudaSetDevice is thread-global, so interleaving two deviceId-pinned
// instances on one thread silently invalidates the first one's context.
TEST(BatchedDenseLevenbergMarquardt, DispatchAcrossTwoDevicesMatchesCpuPerGraph) {
  int deviceCount = 0;
  if (cudaGetDeviceCount(&deviceCount) != cudaSuccess || deviceCount < 2) return;

  BatchPose2Problem problemA = MakeBatchPose2Problem();
  BatchPose2Problem problemB;
  problemB.graphs = {MakePose2Graph(0.5), MakePose2Graph(-0.6)};
  problemB.initial = {MakePose2Initial(0.3), MakePose2Initial(-0.2)};
  const LevenbergMarquardtParams params = RealToleranceParams();

  std::vector<Values> resultA, resultB;
  auto runOnDevice = [&](int deviceId, const BatchPose2Problem& problem,
                        std::vector<Values>* result) {
    cudaSetDevice(deviceId);
    BatchedDenseSolverOptions options;
    options.deviceId = deviceId;
    BatchedDenseLevenbergMarquardtOptimizer optimizer(
        problem.graphs, problem.initial, params, options);
    *result = optimizer.optimize();
  };
  std::thread threadA(runOnDevice, 0, std::cref(problemA), &resultA);
  std::thread threadB(runOnDevice, 1, std::cref(problemB), &resultB);
  threadA.join();
  threadB.join();

  // A caller's own "multi-GPU batch" is just these two independently
  // computed result vectors concatenated; each item must still
  // independently match the CPU optimizer on its own graph.
  std::vector<NonlinearFactorGraph> combinedGraphs = problemA.graphs;
  combinedGraphs.insert(combinedGraphs.end(), problemB.graphs.begin(),
                       problemB.graphs.end());
  std::vector<Values> combinedInitial = problemA.initial;
  combinedInitial.insert(combinedInitial.end(), problemB.initial.begin(),
                        problemB.initial.end());
  std::vector<Values> combinedActual(resultA.begin(), resultA.end());
  combinedActual.insert(combinedActual.end(), resultB.begin(), resultB.end());

  for (size_t i = 0; i < combinedActual.size(); ++i) {
    const Values expected =
        LevenbergMarquardtOptimizer(combinedGraphs[i], combinedInitial[i],
                                   params)
            .optimize();
    EXPECT(assert_equal(expected, combinedActual[i], 1e-6));
  }
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

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
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <stdexcept>
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

// Two distinct 3-pose Pose2 LM problems with the same topology (same keys,
// same factor types) but different odometry and initial values, so a batch
// of them genuinely exercises per-item independence rather than B identical
// copies of one problem.
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

}  // namespace batched_dense_lm_fixture
/* ************************************************************************* */

using namespace batched_dense_lm_fixture;

// Verifies BatchedDenseLevenbergMarquardtOptimizer::MatchesCpuPose2ResultPerGraph
// -- each item's final Values and error must match running the same graph
// through the real (CPU) LevenbergMarquardtOptimizer independently.
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

// Verifies BatchedDenseLevenbergMarquardt::AcceptedIterationsMatchCpuOptimizer
// -- the per-item trust-region accept/reject logic (gtsam::internal's real
// policy) should reproduce LevenbergMarquardtOptimizer's own iteration count,
// not just a similarly-shaped approximation.
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

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

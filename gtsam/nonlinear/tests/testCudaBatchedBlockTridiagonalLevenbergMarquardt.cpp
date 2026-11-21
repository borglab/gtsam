/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testCudaBatchedBlockTridiagonalLevenbergMarquardt.cpp
 * @brief   Unit tests for the batched block-tridiagonal GPU
 *          Levenberg-Marquardt optimizer
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
#include <gtsam/nonlinear/cuda/BatchedBlockTridiagonalLevenbergMarquardt.h>
#include <gtsam/nonlinear/cuda/BatchedDenseLevenbergMarquardt.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <stdexcept>
#include <vector>

using namespace gtsam;
using namespace gtsam::cuda;

/* ************************************************************************* */
namespace batched_block_tridiagonal_lm_fixture {

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

// A 3-pose Pose2 chain (prior on pose 0, BetweenFactors coupling only
// adjacent poses): numBlocks=3, blockDim=3 (Pose2's tangent dimension).
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

constexpr int kNumBlocks = 3;
constexpr int kBlockDim = 3;

}  // namespace batched_block_tridiagonal_lm_fixture
/* ************************************************************************* */

using namespace batched_block_tridiagonal_lm_fixture;

// Each item's final Values and error must match running the same graph
// through the real (CPU) LevenbergMarquardtOptimizer independently -- same
// bar BatchedDenseLevenbergMarquardt is held to.
TEST(BatchedBlockTridiagonalLevenbergMarquardt, MatchesCpuPose2ResultPerGraph) {
  if (!HasCudaDevice()) return;
  const BatchPose2Problem problem = MakeBatchPose2Problem();
  const LevenbergMarquardtParams params = RealToleranceParams();

  BatchedBlockTridiagonalLevenbergMarquardtOptimizer optimizer(
      problem.graphs, problem.initial, kNumBlocks, kBlockDim, params);
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

// The block-tridiagonal-exact solve must reproduce BatchedDense's own result
// on a graph small enough for both: the block decomposition changes only
// how the linear system is factorized, not what system is solved.
TEST(BatchedBlockTridiagonalLevenbergMarquardt, MatchesBatchedDenseResultPerGraph) {
  if (!HasCudaDevice()) return;
  const BatchPose2Problem problem = MakeBatchPose2Problem();
  const LevenbergMarquardtParams params = RealToleranceParams();

  BatchedBlockTridiagonalLevenbergMarquardtOptimizer blockOptimizer(
      problem.graphs, problem.initial, kNumBlocks, kBlockDim, params);
  const std::vector<Values>& blockResult = blockOptimizer.optimize();

  BatchedDenseLevenbergMarquardtOptimizer denseOptimizer(
      problem.graphs, problem.initial, params);
  const std::vector<Values>& denseResult = denseOptimizer.optimize();

  for (size_t i = 0; i < problem.graphs.size(); ++i) {
    EXPECT(assert_equal(denseResult[i], blockResult[i], 1e-9));
  }
}

// GTSAM's real trust-region policy should reproduce LevenbergMarquardtOptimizer's
// own accepted-iteration count, same bar BatchedDense is held to.
TEST(BatchedBlockTridiagonalLevenbergMarquardt, AcceptedIterationsMatchCpuOptimizer) {
  if (!HasCudaDevice()) return;
  const BatchPose2Problem problem = MakeBatchPose2Problem();
  const LevenbergMarquardtParams params = RealToleranceParams();

  BatchedBlockTridiagonalLevenbergMarquardtOptimizer optimizer(
      problem.graphs, problem.initial, kNumBlocks, kBlockDim, params);
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
TEST(BatchedBlockTridiagonalLevenbergMarquardt, MatchesCpuNearConvergence) {
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

  BatchedBlockTridiagonalLevenbergMarquardtOptimizer optimizer(
      problem.graphs, nearConverged, kNumBlocks, kBlockDim, params);
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

TEST(BatchedBlockTridiagonalLevenbergMarquardt, ValuesBeforeOptimizeAreInitial) {
  if (!HasCudaDevice()) return;
  const BatchPose2Problem problem = MakeBatchPose2Problem();
  BatchedBlockTridiagonalLevenbergMarquardtOptimizer optimizer(
      problem.graphs, problem.initial, kNumBlocks, kBlockDim,
      RealToleranceParams());
  for (size_t i = 0; i < problem.graphs.size(); ++i) {
    EXPECT(assert_equal(problem.initial[i], optimizer.values()[i], 1e-12));
  }
}

TEST(BatchedBlockTridiagonalLevenbergMarquardt, ThrowsForMismatchedBatchSizes) {
  if (!HasCudaDevice()) return;
  const BatchPose2Problem problem = MakeBatchPose2Problem();
  std::vector<Values> tooFewInitialValues = {problem.initial.front()};
  CHECK_EXCEPTION(
      BatchedBlockTridiagonalLevenbergMarquardtOptimizer(
          problem.graphs, tooFewInitialValues, kNumBlocks, kBlockDim),
      std::invalid_argument);
}

// Every graph in the batch must linearize to exactly numBlocks*blockDim
// scalar columns.
TEST(BatchedBlockTridiagonalLevenbergMarquardt, ThrowsForMismatchedScalarDimensions) {
  if (!HasCudaDevice()) return;
  BatchPose2Problem problem = MakeBatchPose2Problem();
  problem.graphs[1].emplace_shared<BetweenFactor<Pose2>>(
      kPose2, Symbol('x', 3), Pose2(1.0, 0.0, 0.0),
      noiseModel::Diagonal::Sigmas((Vector(3) << 0.2, 0.2, 0.15).finished()));
  problem.initial[1].insert(Symbol('x', 3), Pose2(4.0, 0.5, 0.2));
  CHECK_EXCEPTION(
      BatchedBlockTridiagonalLevenbergMarquardtOptimizer(
          problem.graphs, problem.initial, kNumBlocks, kBlockDim),
      std::invalid_argument);
}

// numBlocks <= 1 and blockDim <= 0 must both be rejected before any
// linearization.
TEST(BatchedBlockTridiagonalLevenbergMarquardt, ThrowsForInvalidChainShape) {
  if (!HasCudaDevice()) return;
  const BatchPose2Problem problem = MakeBatchPose2Problem();
  CHECK_EXCEPTION(
      BatchedBlockTridiagonalLevenbergMarquardtOptimizer(
          problem.graphs, problem.initial, /*numBlocks=*/1, kBlockDim),
      std::invalid_argument);
  CHECK_EXCEPTION(
      BatchedBlockTridiagonalLevenbergMarquardtOptimizer(
          problem.graphs, problem.initial, kNumBlocks, /*blockDim=*/0),
      std::invalid_argument);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

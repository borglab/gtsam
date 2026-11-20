/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testCudaBatchedDeviceGradients.cpp
 * @brief   Unit tests for the batched GPU differentiable backward
 * @author  Shaji
 * @date    Nov 20, 2026
 */

#include <CppUnitLite/TestHarness.h>
#include <cuda_runtime_api.h>
#include <gtsam/base/Testable.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Gradients.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/cuda/BatchedDeviceGradients.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <stdexcept>
#include <vector>

using namespace gtsam;
using namespace gtsam::cuda;

/* ************************************************************************* */
namespace batched_device_gradients_fixture {

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

// Same two-problem Pose2 batch shape as
// testCudaBatchedDenseLevenbergMarquardt.cpp, but this file only needs each
// graph's *converged* solution (computed once, directly, via the real CPU
// optimizer) to exercise the backward pass -- the forward optimizer under
// test lives in that other file.
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

struct BatchConvergedProblem {
  std::vector<NonlinearFactorGraph> graphs;
  std::vector<Values> converged;
};

BatchConvergedProblem MakeBatchConvergedProblem() {
  BatchConvergedProblem problem;
  problem.graphs = {MakePose2Graph(0.2), MakePose2Graph(-0.3)};
  const std::vector<Values> initial = {MakePose2Initial(0.0),
                                       MakePose2Initial(0.15)};
  for (size_t i = 0; i < problem.graphs.size(); ++i) {
    problem.converged.push_back(
        LevenbergMarquardtOptimizer(problem.graphs[i], initial[i]).optimize());
  }
  return problem;
}

// A fixed (not randomized) cotangent over the three Pose2 tangent blocks.
VectorValues MakeCotangent(double scale) {
  VectorValues cotangent;
  cotangent.insert(kPose0, scale * (Vector(3) << 1.0, -0.5, 0.2).finished());
  cotangent.insert(kPose1, scale * (Vector(3) << -0.3, 0.8, -0.1).finished());
  cotangent.insert(kPose2, scale * (Vector(3) << 0.5, 0.1, 0.4).finished());
  return cotangent;
}

}  // namespace batched_device_gradients_fixture
/* ************************************************************************* */

using namespace batched_device_gradients_fixture;

// Verifies BatchedDeviceGradients::AdjointMatchesCpuGradientsPerGraph.
TEST(BatchedDeviceGradients, AdjointMatchesCpuGradientsPerGraph) {
  if (!HasCudaDevice()) return;
  const BatchConvergedProblem problem = MakeBatchConvergedProblem();
  const std::vector<VectorValues> cotangents = {MakeCotangent(1.0),
                                                MakeCotangent(-1.0)};

  BatchedDeviceGradients gpuGradients(problem.graphs, problem.converged);
  const std::vector<VectorValues> actual = gpuGradients.adjoint(cotangents);

  for (size_t i = 0; i < problem.graphs.size(); ++i) {
    Gradients cpuGradients(problem.graphs[i], problem.converged[i]);
    const VectorValues expected = cpuGradients.adjoint(cotangents[i]);
    EXPECT(assert_equal(expected, actual[i], 1e-7));
  }
  LONGS_EQUAL(static_cast<long>(problem.graphs.size()),
             gpuGradients.batchSize());
}

// Verifies BatchedDeviceGradients::MultipleAdjointCallsReuseFactorizationAndMatchCpu
// -- the whole point of factorizing once per batch: repeated adjoint() calls
// with different cotangents must all stay correct against CPU, matching the
// "factorize once, solve N RHS" pattern parameterGradient-style queries use.
TEST(BatchedDeviceGradients, MultipleAdjointCallsReuseFactorizationAndMatchCpu) {
  if (!HasCudaDevice()) return;
  const BatchConvergedProblem problem = MakeBatchConvergedProblem();
  BatchedDeviceGradients gpuGradients(problem.graphs, problem.converged);

  std::vector<Gradients> cpuGradients;
  for (size_t i = 0; i < problem.graphs.size(); ++i) {
    cpuGradients.emplace_back(problem.graphs[i], problem.converged[i]);
  }

  for (double scale : {1.0, -2.0, 0.5}) {
    const std::vector<VectorValues> cotangents = {MakeCotangent(scale),
                                                  MakeCotangent(-scale)};
    const std::vector<VectorValues> actual = gpuGradients.adjoint(cotangents);
    for (size_t i = 0; i < problem.graphs.size(); ++i) {
      const VectorValues expected = cpuGradients[i].adjoint(cotangents[i]);
      EXPECT(assert_equal(expected, actual[i], 1e-7));
    }
  }
}

// Verifies BatchedDeviceGradients::ThrowsForMismatchedBatchSizes.
TEST(BatchedDeviceGradients, ThrowsForMismatchedBatchSizes) {
  if (!HasCudaDevice()) return;
  const BatchConvergedProblem problem = MakeBatchConvergedProblem();
  std::vector<Values> tooFewSolutions = {problem.converged.front()};
  CHECK_EXCEPTION(
      BatchedDeviceGradients(problem.graphs, tooFewSolutions),
      std::invalid_argument);
}

// Verifies BatchedDeviceGradients::ThrowsForMismatchedScalarDimensions.
TEST(BatchedDeviceGradients, ThrowsForMismatchedScalarDimensions) {
  if (!HasCudaDevice()) return;
  BatchConvergedProblem problem = MakeBatchConvergedProblem();
  problem.graphs[1].emplace_shared<BetweenFactor<Pose2>>(
      kPose2, Symbol('x', 3), Pose2(1.0, 0.0, 0.0),
      noiseModel::Diagonal::Sigmas((Vector(3) << 0.2, 0.2, 0.15).finished()));
  problem.converged[1].insert(Symbol('x', 3), Pose2(4.0, 0.5, 0.2));
  CHECK_EXCEPTION(
      BatchedDeviceGradients(problem.graphs, problem.converged),
      std::invalid_argument);
}

// Verifies BatchedDeviceGradients::AdjointThrowsForWrongCotangentBatchSize.
TEST(BatchedDeviceGradients, AdjointThrowsForWrongCotangentBatchSize) {
  if (!HasCudaDevice()) return;
  const BatchConvergedProblem problem = MakeBatchConvergedProblem();
  BatchedDeviceGradients gpuGradients(problem.graphs, problem.converged);
  const std::vector<VectorValues> tooFewCotangents = {MakeCotangent(1.0)};
  CHECK_EXCEPTION(gpuGradients.adjoint(tooFewCotangents),
                 std::invalid_argument);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

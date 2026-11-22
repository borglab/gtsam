/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testCudaBatchedBlockTridiagonalDeviceGradients.cpp
 * @brief   Unit tests for the batched block-tridiagonal GPU differentiable
 *          backward
 * @author  Shaji
 * @date    Nov 20, 2026
 */

#include <CppUnitLite/TestHarness.h>
#include <cuda_runtime_api.h>
#include <gtsam/base/Testable.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/LossFunctions.h>
#include <gtsam/nonlinear/Gradients.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>
#include <gtsam/nonlinear/ParameterTag.h>
#include <gtsam/nonlinear/cuda/BatchedBlockTridiagonalDeviceGradients.h>
#include <gtsam/nonlinear/cuda/BatchedDeviceGradients.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <optional>
#include <stdexcept>
#include <vector>

using namespace gtsam;
using namespace gtsam::cuda;

/* ************************************************************************* */
namespace batched_block_tridiagonal_gradients_fixture {

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
constexpr int kPoseNumBlocks = 3;
constexpr int kPoseBlockDim = 3;

// Same chain Pose2 problem as testCudaBatchedBlockTridiagonalLevenbergMarquardt.cpp.
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

VectorValues MakeCotangent(double scale) {
  VectorValues cotangent;
  cotangent.insert(kPose0, scale * (Vector(3) << 1.0, -0.5, 0.2).finished());
  cotangent.insert(kPose1, scale * (Vector(3) << -0.3, 0.8, -0.1).finished());
  cotangent.insert(kPose2, scale * (Vector(3) << 0.5, 0.1, 0.4).finished());
  return cotangent;
}

// A plain Vector-valued chain coupling factor (BetweenFactor without
// Pose2's manifold retraction), coupling only adjacent chain-step keys.
class ChainFactor : public NoiseModelFactorN<Vector, Vector> {
 public:
  using Base = NoiseModelFactorN<Vector, Vector>;
  ChainFactor(Key key1, Key key2, const Vector& measured,
              const SharedNoiseModel& model)
      : Base(model, key1, key2), measured_(measured) {}

  Vector evaluateError(const Vector& x1, const Vector& x2,
                       OptionalMatrixType H1,
                       OptionalMatrixType H2) const override {
    if (H1) *H1 = -Matrix::Identity(x1.size(), x1.size());
    if (H2) *H2 = Matrix::Identity(x2.size(), x2.size());
    return (x2 - x1) - measured_;
  }

 private:
  Vector measured_;
};

// One residual-direction block per factor slot, matching
// MakeBatchConvergedProblem()'s 3-factor chain (Prior + 2 Between, each
// rows=3) -- for pushforward(). See testCudaBatchedDeviceGradients.cpp's
// identical fixture for the full rationale.
std::vector<Vector> MakeFactorResidualDirection(double scale) {
  return {
      scale * (Vector(3) << 0.2, -0.1, 0.3).finished(),
      scale * (Vector(3) << -0.4, 0.15, -0.2).finished(),
      scale * (Vector(3) << 0.1, 0.05, -0.3).finished(),
  };
}

struct GraphWithKnownNoise {
  NonlinearFactorGraph graph;
  SharedNoiseModel odometryNoise;
};

// See testCudaBatchedDeviceGradients.cpp's identical fixture for the full
// rationale (shared instance across both BetweenFactors; a second call
// gets a distinct, value-equal instance for object-identity testing).
GraphWithKnownNoise MakePose2GraphWithKnownNoise(double odometryYOffset) {
  GraphWithKnownNoise result;
  const auto priorNoise =
      noiseModel::Diagonal::Sigmas((Vector(3) << 0.15, 0.15, 0.1).finished());
  result.odometryNoise =
      noiseModel::Diagonal::Sigmas((Vector(3) << 0.2, 0.2, 0.15).finished());
  result.graph.emplace_shared<PriorFactor<Pose2>>(kPose0, Pose2(0.0, 0.0, 0.0),
                                                 priorNoise);
  result.graph.emplace_shared<BetweenFactor<Pose2>>(
      kPose0, kPose1, Pose2(2.0, odometryYOffset, 0.1), result.odometryNoise);
  result.graph.emplace_shared<BetweenFactor<Pose2>>(
      kPose1, kPose2, Pose2(1.5, -0.1, -0.08), result.odometryNoise);
  return result;
}

const Key kBlock0 = Symbol('b', 0);
const Key kBlock1 = Symbol('b', 1);
const Key kBlock2 = Symbol('b', 2);
constexpr int kParamNumBlocks = 3;
constexpr int kParamBlockDim = 2;

// error(x) = x - target, with target playing the role of a learned
// parameter identified by `tag` -- see testCudaBatchedDeviceGradients.cpp's
// identical fixture for the full rationale.
class ParameterFactor : public NoiseModelFactorN<Vector> {
 public:
  using Base = NoiseModelFactorN<Vector>;

  ParameterFactor(Key key, ParameterTag tag, const Vector& target,
                  const SharedNoiseModel& model)
      : Base(model, key), tag_(std::move(tag)), target_(target) {}

  Vector evaluateError(const Vector& x, OptionalMatrixType H) const override {
    if (H) *H = Matrix::Identity(x.size(), x.size());
    return x - target_;
  }

  std::optional<Vector> parameterVjp(
      const Values&, const ParameterTag& parameter,
      const Vector& factorResidualCotangent) const override {
    if (parameter != tag_) return std::nullopt;
    return Vector(-factorResidualCotangent);
  }

  /// Exposed only for ParameterGradientMatchesFiniteDifference -- see
  /// testCudaBatchedDeviceGradients.cpp's identical fixture.
  const Vector& target() const { return target_; }

 private:
  ParameterTag tag_;
  Vector target_;
};

struct BatchParameterProblem {
  std::vector<NonlinearFactorGraph> graphs;
  std::vector<Values> solutions;
};

// Two 3-block chains with a ParameterFactor on the *middle* block, to
// exercise a parameter dependency not on the chain's first/last block.
BatchParameterProblem MakeBatchParameterProblem(const ParameterTag& tag) {
  BatchParameterProblem problem;
  const auto noise = noiseModel::Isotropic::Sigma(2, 0.1);
  for (double offset : {0.0, 1.0}) {
    NonlinearFactorGraph graph;
    const Vector anchor = (Vector(2) << 1.0 + offset, 2.0 - offset).finished();
    const Vector target = (Vector(2) << 0.5 - offset, -0.5 + offset).finished();
    graph.emplace_shared<PriorFactor<Vector>>(kBlock0, anchor, noise);
    graph.emplace_shared<ChainFactor>(
        kBlock0, kBlock1, (Vector(2) << 0.2, -0.1).finished(), noise);
    graph.emplace_shared<ChainFactor>(
        kBlock1, kBlock2, (Vector(2) << -0.3, 0.15).finished(), noise);
    graph.emplace_shared<ParameterFactor>(kBlock1, tag, target, noise);
    Values solution;
    solution.insert(kBlock0, (Vector(2) << 0.1, 0.2).finished());
    solution.insert(kBlock1, (Vector(2) << 0.3, -0.2).finished());
    solution.insert(kBlock2, (Vector(2) << -0.1, 0.05).finished());
    problem.graphs.push_back(graph);
    problem.solutions.push_back(solution);
  }
  return problem;
}

std::vector<VectorValues> MakeChainCotangents(double scale) {
  std::vector<VectorValues> result(2);
  for (size_t i = 0; i < 2; ++i) {
    VectorValues cotangent;
    const double s = (i == 0) ? scale : -scale;
    cotangent.insert(kBlock0, s * (Vector(2) << 1.0, -2.0).finished());
    cotangent.insert(kBlock1, s * (Vector(2) << 0.5, 1.5).finished());
    cotangent.insert(kBlock2, s * (Vector(2) << -0.5, 0.25).finished());
    result[i] = cotangent;
  }
  return result;
}

}  // namespace batched_block_tridiagonal_gradients_fixture
/* ************************************************************************* */

using namespace batched_block_tridiagonal_gradients_fixture;

TEST(BatchedBlockTridiagonalDeviceGradients, AdjointMatchesCpuGradientsPerGraph) {
  if (!HasCudaDevice()) return;
  const BatchConvergedProblem problem = MakeBatchConvergedProblem();
  const std::vector<VectorValues> cotangents = {MakeCotangent(1.0),
                                                MakeCotangent(-1.0)};

  BatchedBlockTridiagonalDeviceGradients gpuGradients(
      problem.graphs, problem.converged, kPoseNumBlocks, kPoseBlockDim);
  const std::vector<VectorValues> actual = gpuGradients.adjoint(cotangents);

  for (size_t i = 0; i < problem.graphs.size(); ++i) {
    Gradients cpuGradients(problem.graphs[i], problem.converged[i]);
    const VectorValues expected = cpuGradients.adjoint(cotangents[i]);
    EXPECT(assert_equal(expected, actual[i], 1e-7));
  }
  LONGS_EQUAL(static_cast<long>(problem.graphs.size()),
             gpuGradients.batchSize());
}

// The block-tridiagonal-exact backward must also agree with
// BatchedDeviceGradients's own dense result on a graph small enough for both.
TEST(BatchedBlockTridiagonalDeviceGradients, AdjointMatchesBatchedDenseGradients) {
  if (!HasCudaDevice()) return;
  const BatchConvergedProblem problem = MakeBatchConvergedProblem();
  const std::vector<VectorValues> cotangents = {MakeCotangent(1.0),
                                                MakeCotangent(-1.0)};

  BatchedBlockTridiagonalDeviceGradients blockGradients(
      problem.graphs, problem.converged, kPoseNumBlocks, kPoseBlockDim);
  const std::vector<VectorValues> blockResult = blockGradients.adjoint(cotangents);

  BatchedDeviceGradients denseGradients(problem.graphs, problem.converged);
  const std::vector<VectorValues> denseResult = denseGradients.adjoint(cotangents);

  for (size_t i = 0; i < problem.graphs.size(); ++i) {
    EXPECT(assert_equal(denseResult[i], blockResult[i], 1e-9));
  }
}

TEST(BatchedBlockTridiagonalDeviceGradients, MultipleAdjointCallsReuseFactorizationAndMatchCpu) {
  if (!HasCudaDevice()) return;
  const BatchConvergedProblem problem = MakeBatchConvergedProblem();
  BatchedBlockTridiagonalDeviceGradients gpuGradients(
      problem.graphs, problem.converged, kPoseNumBlocks, kPoseBlockDim);

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

TEST(BatchedBlockTridiagonalDeviceGradients, ThrowsForMismatchedBatchSizes) {
  if (!HasCudaDevice()) return;
  const BatchConvergedProblem problem = MakeBatchConvergedProblem();
  std::vector<Values> tooFewSolutions = {problem.converged.front()};
  CHECK_EXCEPTION(
      BatchedBlockTridiagonalDeviceGradients(
          problem.graphs, tooFewSolutions, kPoseNumBlocks, kPoseBlockDim),
      std::invalid_argument);
}

TEST(BatchedBlockTridiagonalDeviceGradients, ThrowsForMismatchedScalarDimensions) {
  if (!HasCudaDevice()) return;
  BatchConvergedProblem problem = MakeBatchConvergedProblem();
  problem.graphs[1].emplace_shared<BetweenFactor<Pose2>>(
      kPose2, Symbol('x', 3), Pose2(1.0, 0.0, 0.0),
      noiseModel::Diagonal::Sigmas((Vector(3) << 0.2, 0.2, 0.15).finished()));
  problem.converged[1].insert(Symbol('x', 3), Pose2(4.0, 0.5, 0.2));
  CHECK_EXCEPTION(
      BatchedBlockTridiagonalDeviceGradients(
          problem.graphs, problem.converged, kPoseNumBlocks, kPoseBlockDim),
      std::invalid_argument);
}

TEST(BatchedBlockTridiagonalDeviceGradients, AdjointThrowsForWrongCotangentBatchSize) {
  if (!HasCudaDevice()) return;
  const BatchConvergedProblem problem = MakeBatchConvergedProblem();
  BatchedBlockTridiagonalDeviceGradients gpuGradients(
      problem.graphs, problem.converged, kPoseNumBlocks, kPoseBlockDim);
  const std::vector<VectorValues> tooFewCotangents = {MakeCotangent(1.0)};
  CHECK_EXCEPTION(gpuGradients.adjoint(tooFewCotangents),
                 std::invalid_argument);
}

// A factor connecting block 0 and block 2 directly (skipping block 1) must
// be rejected here too: this class is BatchedBlockHessianAssembler's second
// caller, with the identical dense-Jacobian-to-block-sparse-assembly
// pattern as BatchedBlockTridiagonalLevenbergMarquardt (PR #2937 review,
// high-priority finding 1).
TEST(BatchedBlockTridiagonalDeviceGradients, ThrowsForNonAdjacentBlockCoupling) {
  if (!HasCudaDevice()) return;
  BatchConvergedProblem problem = MakeBatchConvergedProblem();
  problem.graphs[0].emplace_shared<BetweenFactor<Pose2>>(
      kPose0, kPose2, Pose2(3.5, -0.1, 0.0),
      noiseModel::Diagonal::Sigmas((Vector(3) << 0.2, 0.2, 0.15).finished()));
  CHECK_EXCEPTION(
      BatchedBlockTridiagonalDeviceGradients(
          problem.graphs, problem.converged, kPoseNumBlocks, kPoseBlockDim),
      std::invalid_argument);
}

// See testCudaBatchedDeviceGradients.cpp's identical test for the full
// rationale (PR #2937 review, high-priority finding 2).
TEST(BatchedBlockTridiagonalDeviceGradients, ThrowsForRobustNoiseModel) {
  if (!HasCudaDevice()) return;
  BatchConvergedProblem problem = MakeBatchConvergedProblem();
  const auto baseNoise =
      noiseModel::Diagonal::Sigmas((Vector(3) << 0.2, 0.2, 0.15).finished());
  const auto robustNoise =
      noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(1.0), baseNoise);
  problem.graphs[0].emplace_shared<PriorFactor<Pose2>>(
      kPose0, Pose2(0.1, 0.1, 0.0), robustNoise);
  CHECK_EXCEPTION(
      BatchedBlockTridiagonalDeviceGradients(
          problem.graphs, problem.converged, kPoseNumBlocks, kPoseBlockDim),
      std::invalid_argument);
}

TEST(BatchedBlockTridiagonalDeviceGradients, ThrowsForConstrainedNoiseModel) {
  if (!HasCudaDevice()) return;
  BatchConvergedProblem problem = MakeBatchConvergedProblem();
  problem.graphs[0].emplace_shared<PriorFactor<Pose2>>(
      kPose0, Pose2(0.1, 0.1, 0.0), noiseModel::Constrained::All(3));
  CHECK_EXCEPTION(
      BatchedBlockTridiagonalDeviceGradients(
          problem.graphs, problem.converged, kPoseNumBlocks, kPoseBlockDim),
      std::invalid_argument);
}

// See testCudaBatchedDeviceGradients.cpp's identical test for the full
// rationale (PR #2937 review, high-priority finding 3).
TEST(BatchedBlockTridiagonalDeviceGradients,
    AdjointSupportsPartialCotangentMatchingCpu) {
  if (!HasCudaDevice()) return;
  const BatchConvergedProblem problem = MakeBatchConvergedProblem();
  VectorValues partial;
  partial.insert(kPose1, (Vector(3) << -0.3, 0.8, -0.1).finished());
  const std::vector<VectorValues> cotangents = {partial, partial};

  BatchedBlockTridiagonalDeviceGradients gpuGradients(
      problem.graphs, problem.converged, kPoseNumBlocks, kPoseBlockDim);
  const std::vector<VectorValues> actual = gpuGradients.adjoint(cotangents);

  for (size_t i = 0; i < problem.graphs.size(); ++i) {
    Gradients cpuGradients(problem.graphs[i], problem.converged[i]);
    const VectorValues expected = cpuGradients.adjoint(cotangents[i]);
    EXPECT(assert_equal(expected, actual[i], 1e-7));
  }
}

TEST(BatchedBlockTridiagonalDeviceGradients,
    AdjointSupportsEmptyCotangentMatchingCpu) {
  if (!HasCudaDevice()) return;
  const BatchConvergedProblem problem = MakeBatchConvergedProblem();
  const VectorValues empty;
  const std::vector<VectorValues> cotangents = {empty, empty};

  BatchedBlockTridiagonalDeviceGradients gpuGradients(
      problem.graphs, problem.converged, kPoseNumBlocks, kPoseBlockDim);
  const std::vector<VectorValues> actual = gpuGradients.adjoint(cotangents);

  for (size_t i = 0; i < problem.graphs.size(); ++i) {
    Gradients cpuGradients(problem.graphs[i], problem.converged[i]);
    const VectorValues expected = cpuGradients.adjoint(cotangents[i]);
    EXPECT(assert_equal(expected, actual[i], 1e-7));
  }
}

TEST(BatchedBlockTridiagonalDeviceGradients, AdjointThrowsForUnknownCotangentKey) {
  if (!HasCudaDevice()) return;
  const BatchConvergedProblem problem = MakeBatchConvergedProblem();
  VectorValues withUnknownKey;
  withUnknownKey.insert(Symbol('x', 99), (Vector(3) << 1.0, 0.0, 0.0).finished());
  const std::vector<VectorValues> cotangents = {withUnknownKey, withUnknownKey};

  BatchedBlockTridiagonalDeviceGradients gpuGradients(
      problem.graphs, problem.converged, kPoseNumBlocks, kPoseBlockDim);
  CHECK_EXCEPTION(gpuGradients.adjoint(cotangents), std::invalid_argument);
}

// parameterGradient() on a parameter attached to the *middle* chain block
// must match CPU gtsam::Gradients::parameterGradient exactly -- the real
// correctness bar for a drop-in differentiable-backward replacement.
// See testCudaBatchedDeviceGradients.cpp's identical test for the full
// rationale (PR #2937 review's finite-difference-check request). The
// ParameterFactor here is factor index 3 (0=Prior, 1=Chain(0,1),
// 2=Chain(1,2), 3=Parameter on the middle block).
TEST(BatchedBlockTridiagonalDeviceGradients,
    ParameterGradientMatchesFiniteDifference) {
  if (!HasCudaDevice()) return;
  const ParameterTag tag("chain_finite_difference_check");
  const BatchParameterProblem problem = MakeBatchParameterProblem(tag);
  const std::vector<VectorValues> cotangents = MakeChainCotangents(1.0);

  BatchedBlockTridiagonalDeviceGradients gpuGradients(
      problem.graphs, problem.solutions, kParamNumBlocks, kParamBlockDim);
  const std::vector<std::optional<Vector>> analytic =
      gpuGradients.parameterGradient(cotangents, tag);
  const std::vector<std::vector<Vector>> factorResidualCotangents =
      gpuGradients.pullback(cotangents);

  constexpr double kEps = 1e-6;
  constexpr size_t kParameterFactorIndex = 3;
  for (size_t b = 0; b < problem.graphs.size(); ++b) {
    CHECK(analytic[b].has_value());
    const Values& solution = problem.solutions[b];
    const auto& parameterFactor = static_cast<const ParameterFactor&>(
        *problem.graphs[b].at(kParameterFactorIndex));
    const SharedNoiseModel noise = parameterFactor.noiseModel();
    const auto diagonal =
        std::dynamic_pointer_cast<noiseModel::Diagonal>(noise);

    Vector factorResidualCotangent =
        factorResidualCotangents[b][kParameterFactorIndex];
    diagonal->whitenInPlace(factorResidualCotangent);

    Vector numeric(2);
    for (int d = 0; d < 2; ++d) {
      Vector plusTarget = parameterFactor.target();
      Vector minusTarget = parameterFactor.target();
      plusTarget(d) += kEps;
      minusTarget(d) -= kEps;
      const Vector residualPlus =
          ParameterFactor(parameterFactor.keys()[0], tag, plusTarget, noise)
              .unwhitenedError(solution);
      const Vector residualMinus =
          ParameterFactor(parameterFactor.keys()[0], tag, minusTarget, noise)
              .unwhitenedError(solution);
      numeric(d) = factorResidualCotangent.dot(residualPlus - residualMinus) /
                  (2.0 * kEps);
    }
    EXPECT(assert_equal(numeric, *analytic[b], 1e-4));
  }
}

TEST(BatchedBlockTridiagonalDeviceGradients, ParameterGradientMatchesCpuGradients) {
  if (!HasCudaDevice()) return;
  const ParameterTag tag("chain_block1_target");
  const BatchParameterProblem problem = MakeBatchParameterProblem(tag);
  const std::vector<VectorValues> cotangents = MakeChainCotangents(1.0);

  BatchedBlockTridiagonalDeviceGradients gpuGradients(
      problem.graphs, problem.solutions, kParamNumBlocks, kParamBlockDim);
  const std::vector<std::optional<Vector>> actual =
      gpuGradients.parameterGradient(cotangents, tag);

  for (size_t i = 0; i < problem.graphs.size(); ++i) {
    Gradients cpuGradients(problem.graphs[i], problem.solutions[i]);
    const std::optional<Vector> expected =
        cpuGradients.parameterGradient(cotangents[i], tag);
    CHECK(actual[i].has_value());
    CHECK(expected.has_value());
    EXPECT(assert_equal(*expected, *actual[i], 1e-7));
  }
}

// A tag no factor depends on must give std::nullopt for every graph, same
// as CPU gtsam::Gradients::parameterGradient.
TEST(BatchedBlockTridiagonalDeviceGradients, ParameterGradientIsNulloptForUnusedTag) {
  if (!HasCudaDevice()) return;
  const ParameterTag tag("chain_block1_target");
  const ParameterTag unusedTag("not_used_anywhere");
  const BatchParameterProblem problem = MakeBatchParameterProblem(tag);
  const std::vector<VectorValues> cotangents = MakeChainCotangents(1.0);

  BatchedBlockTridiagonalDeviceGradients gpuGradients(
      problem.graphs, problem.solutions, kParamNumBlocks, kParamBlockDim);
  const std::vector<std::optional<Vector>> actual =
      gpuGradients.parameterGradient(cotangents, unusedTag);
  for (const auto& value : actual) CHECK(!value.has_value());
}

// See testCudaBatchedDeviceGradients.cpp's identical test for the full
// rationale (PR #2937 review's pushforward/noiseGradient parity finding).
TEST(BatchedBlockTridiagonalDeviceGradients, PushforwardMatchesCpuGradients) {
  if (!HasCudaDevice()) return;
  const BatchConvergedProblem problem = MakeBatchConvergedProblem();
  const std::vector<std::vector<Vector>> directions = {
      MakeFactorResidualDirection(1.0), MakeFactorResidualDirection(-0.5)};

  BatchedBlockTridiagonalDeviceGradients gpuGradients(
      problem.graphs, problem.converged, kPoseNumBlocks, kPoseBlockDim);
  const std::vector<VectorValues> actual = gpuGradients.pushforward(directions);

  for (size_t i = 0; i < problem.graphs.size(); ++i) {
    Gradients cpuGradients(problem.graphs[i], problem.converged[i]);
    const Errors errors(directions[i].begin(), directions[i].end());
    const VectorValues expected = cpuGradients.pushforward(errors);
    EXPECT(assert_equal(expected, actual[i], 1e-7));
  }
}

TEST(BatchedBlockTridiagonalDeviceGradients,
    NoiseGradientMatchesCpuGradientsByInstanceIdentity) {
  if (!HasCudaDevice()) return;
  const GraphWithKnownNoise g0 = MakePose2GraphWithKnownNoise(0.2);
  const GraphWithKnownNoise g1 = MakePose2GraphWithKnownNoise(-0.3);
  const std::vector<NonlinearFactorGraph> graphs = {g0.graph, g1.graph};
  const std::vector<Values> initial = {MakePose2Initial(0.0),
                                       MakePose2Initial(0.15)};
  std::vector<Values> converged;
  for (size_t i = 0; i < graphs.size(); ++i) {
    converged.push_back(
        LevenbergMarquardtOptimizer(graphs[i], initial[i]).optimize());
  }
  const std::vector<VectorValues> cotangents = {MakeCotangent(1.0),
                                                MakeCotangent(-1.0)};

  BatchedBlockTridiagonalDeviceGradients gpuGradients(
      graphs, converged, kPoseNumBlocks, kPoseBlockDim);
  const std::vector<std::optional<Vector>> actual =
      gpuGradients.noiseGradient(cotangents, g0.odometryNoise);

  Gradients cpuGradients0(graphs[0], converged[0]);
  const std::optional<Vector> expected0 =
      cpuGradients0.noiseGradient(cotangents[0], g0.odometryNoise);
  CHECK(expected0.has_value());
  CHECK(actual[0].has_value());
  EXPECT(assert_equal(*expected0, *actual[0], 1e-7));

  Gradients cpuGradients1(graphs[1], converged[1]);
  const std::optional<Vector> expected1 =
      cpuGradients1.noiseGradient(cotangents[1], g0.odometryNoise);
  CHECK(!expected1.has_value());
  CHECK(!actual[1].has_value());
}

TEST(BatchedBlockTridiagonalDeviceGradients, NoiseGradientThrowsForUnsupportedModel) {
  if (!HasCudaDevice()) return;
  const BatchConvergedProblem problem = MakeBatchConvergedProblem();
  const std::vector<VectorValues> cotangents = {MakeCotangent(1.0),
                                                MakeCotangent(-1.0)};
  BatchedBlockTridiagonalDeviceGradients gpuGradients(
      problem.graphs, problem.converged, kPoseNumBlocks, kPoseBlockDim);
  CHECK_EXCEPTION(
      gpuGradients.noiseGradient(cotangents, noiseModel::Unit::Create(3)),
      std::invalid_argument);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

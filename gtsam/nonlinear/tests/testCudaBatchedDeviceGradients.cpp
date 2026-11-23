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
#include <gtsam/linear/LossFunctions.h>
#include <gtsam/nonlinear/Gradients.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>
#include <gtsam/nonlinear/ParameterTag.h>
#include <gtsam/nonlinear/cuda/BatchedDeviceGradients.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <optional>
#include <stdexcept>
#include <thread>
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

// Same Pose2 batch shape as testCudaBatchedDenseLevenbergMarquardt.cpp,
// but only needs each graph's converged solution for the backward pass.
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

// One residual-direction block per factor slot, matching
// MakeBatchConvergedProblem()'s 3-factor chain (Prior + 2 Between, each
// rows=3) -- for pushforward().
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

// Both BetweenFactors share one odometryNoise instance; a second call gets
// its own distinct instance with equal sigmas, testing noiseGradient's
// object-identity (not value) matching.
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

const Key kScalar0 = Symbol('s', 0);

// error(x) = x - target, target playing a learned parameter `tag`.
// d(error)/d(target) = -I, so the VJP contribution is -factorResidualCotangent.
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

  /// Exposed only for ParameterGradientMatchesFiniteDifference, to rebuild
  /// this factor at a perturbed target without otherwise changing its
  /// (private, intentionally encapsulated) constructor-time state.
  const Vector& target() const { return target_; }

 private:
  ParameterTag tag_;
  Vector target_;
};

struct BatchParameterProblem {
  std::vector<NonlinearFactorGraph> graphs;
  std::vector<Values> solutions;
};

// Two graphs (PriorFactor + ParameterFactor sharing `tag`) at distinct
// fixed solutions, for genuinely different per-graph contributions.
BatchParameterProblem MakeBatchParameterProblem(const ParameterTag& tag) {
  BatchParameterProblem problem;
  const auto noise = noiseModel::Isotropic::Sigma(2, 0.1);
  for (double offset : {0.0, 1.0}) {
    NonlinearFactorGraph graph;
    const Vector anchor = (Vector(2) << 1.0 + offset, 2.0 - offset).finished();
    const Vector target = (Vector(2) << 0.5 - offset, -0.5 + offset).finished();
    graph.emplace_shared<PriorFactor<Vector>>(kScalar0, anchor, noise);
    graph.emplace_shared<ParameterFactor>(kScalar0, tag, target, noise);
    Values solution;
    solution.insert(kScalar0, (Vector(2) << 0.1, 0.2).finished());
    problem.graphs.push_back(graph);
    problem.solutions.push_back(solution);
  }
  return problem;
}

// A fixed cotangent over the single 2-dim kScalar0 block, for
// MakeBatchParameterProblem's graphs.
VectorValues MakeScalarCotangent(double scale) {
  VectorValues cotangent;
  cotangent.insert(kScalar0, scale * (Vector(2) << 1.0, -2.0).finished());
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

// Repeated adjoint() calls, different cotangents, must all match CPU.
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

// Robust noise models are rejected: graph.linearize() would otherwise fold
// IRLS reweighting into A, and the later static_pointer_cast<Gaussian> on
// a noiseModel::Robust (not a Gaussian) would be an invalid cast.
TEST(BatchedDeviceGradients, ThrowsForRobustNoiseModel) {
  if (!HasCudaDevice()) return;
  BatchConvergedProblem problem = MakeBatchConvergedProblem();
  const auto baseNoise =
      noiseModel::Diagonal::Sigmas((Vector(3) << 0.2, 0.2, 0.15).finished());
  const auto robustNoise =
      noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(1.0), baseNoise);
  problem.graphs[0].emplace_shared<PriorFactor<Pose2>>(
      kPose0, Pose2(0.1, 0.1, 0.0), robustNoise);
  CHECK_EXCEPTION(
      BatchedDeviceGradients(problem.graphs, problem.converged),
      std::invalid_argument);
}

// Same contract as ThrowsForRobustNoiseModel, for a hard (Constrained)
// noise model -- Gradients.h: "robust losses, hard constraints ... are not"
// supported.
TEST(BatchedDeviceGradients, ThrowsForConstrainedNoiseModel) {
  if (!HasCudaDevice()) return;
  BatchConvergedProblem problem = MakeBatchConvergedProblem();
  problem.graphs[0].emplace_shared<PriorFactor<Pose2>>(
      kPose0, Pose2(0.1, 0.1, 0.0), noiseModel::Constrained::All(3));
  CHECK_EXCEPTION(
      BatchedDeviceGradients(problem.graphs, problem.converged),
      std::invalid_argument);
}

// A cotangent naming only one of the chain's three keys must match CPU
// gtsam::Gradients::adjoint exactly: omitted known keys mean zero.
TEST(BatchedDeviceGradients, AdjointSupportsPartialCotangentMatchingCpu) {
  if (!HasCudaDevice()) return;
  const BatchConvergedProblem problem = MakeBatchConvergedProblem();
  VectorValues partial;
  partial.insert(kPose1, (Vector(3) << -0.3, 0.8, -0.1).finished());
  const std::vector<VectorValues> cotangents = {partial, partial};

  BatchedDeviceGradients gpuGradients(problem.graphs, problem.converged);
  const std::vector<VectorValues> actual = gpuGradients.adjoint(cotangents);

  for (size_t i = 0; i < problem.graphs.size(); ++i) {
    Gradients cpuGradients(problem.graphs[i], problem.converged[i]);
    const VectorValues expected = cpuGradients.adjoint(cotangents[i]);
    EXPECT(assert_equal(expected, actual[i], 1e-7));
  }
}

// Same contract, for a fully empty cotangent (every known key omitted).
TEST(BatchedDeviceGradients, AdjointSupportsEmptyCotangentMatchingCpu) {
  if (!HasCudaDevice()) return;
  const BatchConvergedProblem problem = MakeBatchConvergedProblem();
  const VectorValues empty;
  const std::vector<VectorValues> cotangents = {empty, empty};

  BatchedDeviceGradients gpuGradients(problem.graphs, problem.converged);
  const std::vector<VectorValues> actual = gpuGradients.adjoint(cotangents);

  for (size_t i = 0; i < problem.graphs.size(); ++i) {
    Gradients cpuGradients(problem.graphs[i], problem.converged[i]);
    const VectorValues expected = cpuGradients.adjoint(cotangents[i]);
    EXPECT(assert_equal(expected, actual[i], 1e-7));
  }
}

// An unknown key (absent from the graph's keys) must be rejected, matching
// Gradients.h: "Unknown keys ... are errors."
TEST(BatchedDeviceGradients, AdjointThrowsForUnknownCotangentKey) {
  if (!HasCudaDevice()) return;
  const BatchConvergedProblem problem = MakeBatchConvergedProblem();
  VectorValues withUnknownKey;
  withUnknownKey.insert(Symbol('x', 99), (Vector(3) << 1.0, 0.0, 0.0).finished());
  const std::vector<VectorValues> cotangents = {withUnknownKey, withUnknownKey};

  BatchedDeviceGradients gpuGradients(problem.graphs, problem.converged);
  CHECK_EXCEPTION(gpuGradients.adjoint(cotangents), std::invalid_argument);
}

// Validates parameterGradient against a central-difference estimate from
// ParameterFactor::unwhitenedError at perturbed targets -- never calls
// parameterVjp, so it is independent of the analytic implementation.
TEST(BatchedDeviceGradients, ParameterGradientMatchesFiniteDifference) {
  if (!HasCudaDevice()) return;
  const ParameterTag tag("finite-difference-check");
  const BatchParameterProblem problem = MakeBatchParameterProblem(tag);
  const std::vector<VectorValues> cotangents = {MakeScalarCotangent(1.0),
                                                MakeScalarCotangent(-0.5)};

  BatchedDeviceGradients gpuGradients(problem.graphs, problem.solutions);
  const std::vector<std::optional<Vector>> analytic =
      gpuGradients.parameterGradient(cotangents, tag);
  const std::vector<std::vector<Vector>> factorResidualCotangents =
      gpuGradients.pullback(cotangents);

  constexpr double kEps = 1e-6;
  for (size_t b = 0; b < problem.graphs.size(); ++b) {
    CHECK(analytic[b].has_value());
    const Values& solution = problem.solutions[b];
    // ParameterFactor is factor index 1 (index 0 is the PriorFactor).
    const auto& parameterFactor =
        static_cast<const ParameterFactor&>(*problem.graphs[b].at(1));
    const SharedNoiseModel noise = parameterFactor.noiseModel();
    const auto diagonal =
        std::dynamic_pointer_cast<noiseModel::Diagonal>(noise);

    // pullback() returns a whitened-space cotangent; apply the same W
    // transpose parameterGradient does (W^T == W for Diagonal/Isotropic).
    Vector factorResidualCotangent = factorResidualCotangents[b][1];
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

// parameterGradient() (pullback() + each factor's parameterVjp) must
// match CPU gtsam::Gradients::parameterGradient.
TEST(BatchedDeviceGradients, ParameterGradientMatchesCpuGradients) {
  if (!HasCudaDevice()) return;
  const ParameterTag tag("test-parameter");
  const BatchParameterProblem problem = MakeBatchParameterProblem(tag);
  const std::vector<VectorValues> cotangents = {MakeScalarCotangent(1.0),
                                                MakeScalarCotangent(-0.5)};

  BatchedDeviceGradients gpuGradients(problem.graphs, problem.solutions);
  const std::vector<std::optional<Vector>> actual =
      gpuGradients.parameterGradient(cotangents, tag);

  LONGS_EQUAL(static_cast<long>(problem.graphs.size()), actual.size());
  for (size_t i = 0; i < problem.graphs.size(); ++i) {
    Gradients cpuGradients(problem.graphs[i], problem.solutions[i]);
    const std::optional<Vector> expected =
        cpuGradients.parameterGradient(cotangents[i], tag);
    CHECK(expected.has_value());
    CHECK(actual[i].has_value());
    EXPECT(assert_equal(*expected, *actual[i], 1e-9));
  }
}

// A tag no factor depends on must return std::nullopt, not an empty vector.
TEST(BatchedDeviceGradients, ParameterGradientReturnsNulloptForUnrelatedTag) {
  if (!HasCudaDevice()) return;
  const ParameterTag usedTag("used");
  const ParameterTag unrelatedTag("unrelated");
  const BatchParameterProblem problem = MakeBatchParameterProblem(usedTag);
  const std::vector<VectorValues> cotangents = {MakeScalarCotangent(1.0),
                                                MakeScalarCotangent(-0.5)};

  BatchedDeviceGradients gpuGradients(problem.graphs, problem.solutions);
  const std::vector<std::optional<Vector>> actual =
      gpuGradients.parameterGradient(cotangents, unrelatedTag);

  for (size_t i = 0; i < problem.graphs.size(); ++i) {
    Gradients cpuGradients(problem.graphs[i], problem.solutions[i]);
    const std::optional<Vector> expected =
        cpuGradients.parameterGradient(cotangents[i], unrelatedTag);
    CHECK(!expected.has_value());
    CHECK(!actual[i].has_value());
  }
}

// pushforward() must match gtsam::Gradients::pushforward exactly.
TEST(BatchedDeviceGradients, PushforwardMatchesCpuGradients) {
  if (!HasCudaDevice()) return;
  const BatchConvergedProblem problem = MakeBatchConvergedProblem();
  const std::vector<std::vector<Vector>> directions = {
      MakeFactorResidualDirection(1.0), MakeFactorResidualDirection(-0.5)};

  BatchedDeviceGradients gpuGradients(problem.graphs, problem.converged);
  const std::vector<VectorValues> actual = gpuGradients.pushforward(directions);

  for (size_t i = 0; i < problem.graphs.size(); ++i) {
    Gradients cpuGradients(problem.graphs[i], problem.converged[i]);
    const Errors errors(directions[i].begin(), directions[i].end());
    const VectorValues expected = cpuGradients.pushforward(errors);
    EXPECT(assert_equal(expected, actual[i], 1e-7));
  }
}

TEST(BatchedDeviceGradients, PushforwardThrowsForWrongFactorResidualDimension) {
  if (!HasCudaDevice()) return;
  const BatchConvergedProblem problem = MakeBatchConvergedProblem();
  std::vector<std::vector<Vector>> directions = {
      MakeFactorResidualDirection(1.0), MakeFactorResidualDirection(-0.5)};
  directions[0][1] = (Vector(2) << 1.0, 2.0).finished();  // wrong dimension
  BatchedDeviceGradients gpuGradients(problem.graphs, problem.converged);
  CHECK_EXCEPTION(gpuGradients.pushforward(directions), std::invalid_argument);
}

// noiseGradient() must match gtsam::Gradients::noiseGradient exactly,
// including its documented object-identity factor matching: a model with
// equal sigmas but a different shared_ptr instance must not match.
TEST(BatchedDeviceGradients, NoiseGradientMatchesCpuGradientsByInstanceIdentity) {
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

  BatchedDeviceGradients gpuGradients(graphs, converged);
  const std::vector<std::optional<Vector>> actual =
      gpuGradients.noiseGradient(cotangents, g0.odometryNoise);

  // Only graph 0's two BetweenFactors use g0.odometryNoise's exact
  // instance (accumulated across both); graph 1's factors use their own,
  // value-equal but distinct, instance and must not match.
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

// Matches gtsam::Gradients::noiseGradient: "Unit, Constrained, Robust and
// full Gaussian models are not supported and throw std::invalid_argument."
TEST(BatchedDeviceGradients, NoiseGradientThrowsForUnsupportedModel) {
  if (!HasCudaDevice()) return;
  const BatchConvergedProblem problem = MakeBatchConvergedProblem();
  const std::vector<VectorValues> cotangents = {MakeCotangent(1.0),
                                                MakeCotangent(-1.0)};
  BatchedDeviceGradients gpuGradients(problem.graphs, problem.converged);
  CHECK_EXCEPTION(
      gpuGradients.noiseGradient(cotangents, noiseModel::Unit::Create(3)),
      std::invalid_argument);
}

// Each device's work must run on its own host thread (not interleaved
// instances on one thread): see the identical test in
// testCudaBatchedDenseLevenbergMarquardt.cpp for why.
TEST(BatchedDeviceGradients, DispatchAcrossTwoDevicesMatchesCpuGradients) {
  int deviceCount = 0;
  if (cudaGetDeviceCount(&deviceCount) != cudaSuccess || deviceCount < 2) return;

  const BatchConvergedProblem problemA = MakeBatchConvergedProblem();
  BatchConvergedProblem problemB;
  problemB.graphs = {MakePose2Graph(0.5), MakePose2Graph(-0.6)};
  const std::vector<Values> initialB = {MakePose2Initial(0.3),
                                        MakePose2Initial(-0.2)};
  for (size_t i = 0; i < problemB.graphs.size(); ++i) {
    problemB.converged.push_back(
        LevenbergMarquardtOptimizer(problemB.graphs[i], initialB[i]).optimize());
  }
  const std::vector<VectorValues> cotangentsA = {MakeCotangent(1.0),
                                                 MakeCotangent(-1.0)};
  const std::vector<VectorValues> cotangentsB = {MakeCotangent(0.5),
                                                 MakeCotangent(-0.5)};

  std::vector<VectorValues> actualA, actualB;
  auto runOnDevice = [&](int deviceId, const BatchConvergedProblem& problem,
                        const std::vector<VectorValues>& cotangents,
                        std::vector<VectorValues>* result) {
    cudaSetDevice(deviceId);
    BatchedDenseSolverOptions options;
    options.deviceId = deviceId;
    BatchedDeviceGradients gradients(problem.graphs, problem.converged,
                                    options);
    *result = gradients.adjoint(cotangents);
  };
  std::thread threadA(runOnDevice, 0, std::cref(problemA),
                      std::cref(cotangentsA), &actualA);
  std::thread threadB(runOnDevice, 1, std::cref(problemB),
                      std::cref(cotangentsB), &actualB);
  threadA.join();
  threadB.join();

  for (size_t i = 0; i < problemA.graphs.size(); ++i) {
    Gradients cpuGradients(problemA.graphs[i], problemA.converged[i]);
    EXPECT(assert_equal(cpuGradients.adjoint(cotangentsA[i]), actualA[i], 1e-7));
  }
  for (size_t i = 0; i < problemB.graphs.size(); ++i) {
    Gradients cpuGradients(problemB.graphs[i], problemB.converged[i]);
    EXPECT(assert_equal(cpuGradients.adjoint(cotangentsB[i]), actualB[i], 1e-7));
  }
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

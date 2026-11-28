// Copyright (c) Georgia Tech Research Corporation. See LICENSE for details.
/**
 * @file testPolicyOptimizationMPC.cpp
 * @author Frank Dellaert
 * @author Akash Ratheesh
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/CustomFactor.h>
#include <gtsam/planning/EliteMPPI.h>
#include <gtsam_unstable/planning/PolicyOptimizationMPC.h>

#include <cmath>

using namespace gtsam;

/* ************************************************************************* */
namespace policy_optimization {

PolicyOptimizationMPCParams small() {
  PolicyOptimizationMPCParams p;
  p.horizon = 2;
  p.samples = 32;
  p.elites = 6;
  p.policySamples = 8;
  p.hiddenSize = 8;
  p.valueBins = 11;
  p.valueLimit = 2.;
  p.batchSize = 4;
  p.updateInterval = 0;
  p.replayCapacity = 8;
  p.reanalyzeInterval = 0;
  p.learningRate = 0.01;
  return p;
}

// Mirrored quadratic control factors steer a seeded planner to different
// elites.
TEST(PolicyOptimizationMPC, GraphAndMixture) {
  auto p = small();
  // Give both two-step goals support in the initial 2-D sequence population.
  p.samples = 256;
  p.elites = 24;
  p.policySamples = 32;
  const Vector state = Vector::Zero(2);
  for (double desired : {-1., 1.}) {
    PolicyOptimizationMPC policy(2, 1, p);
    NonlinearFactorGraph graph;
    for (size_t k = 0; k < p.horizon; ++k)
      graph.addPrior(symbol_shorthand::U(k), Vector(Vector1{desired}),
                     noiseModel::Isotropic::Sigma(1, 0.1));
    const auto result = policy.plan(state, graph);
    EXPECT_LONGS_EQUAL(2, result.mean.cols());
    EXPECT(desired * result.action[0] > 0.65);
    EXPECT(desired * result.mean(0, 1) > 0.65);
    EXPECT((result.stddev.array() >= p.minStddev).all());
    EXPECT((result.stddev.array() <= p.maxStddev).all());
  }
}

// Each candidate contributes exactly one evaluation of the optional task cost.
TEST(PolicyOptimizationMPC, OneGraphEvaluationPerCandidate) {
  const auto params = small();
  PolicyOptimizationMPC learner(1, 1, params);
  size_t evaluations = 0;
  NonlinearFactorGraph graph;
  graph.emplace_shared<CustomFactor>(
      noiseModel::Unit::Create(1), KeyVector{symbol_shorthand::U(0)},
      [&](const CustomFactor&, const Values&, OptionalJacobianVector) {
        ++evaluations;
        return Vector1{1.};
      });
  learner.plan(Vector::Zero(1), graph);
  EXPECT_LONGS_EQUAL(params.samples * params.iterations, evaluations);
}

// Reverse-KL regularization against stored planning marginals pulls the actor
// toward the target even when observations have zero reward.
TEST(PolicyOptimizationMPC, ReplayPriorMovesSamplingPolicy) {
  auto p = small();
  p.learnedPrior = false;
  p.klWeight = 2.;
  p.entropyWeight = 0.;
  PolicyOptimizationMPC policy(1, 1, p);
  const Vector state = Vector::Zero(1);
  PolicyOptimizationPlan behavior;
  behavior.mean =
      Matrix::Constant(1, static_cast<Eigen::Index>(p.horizon), 0.8);
  behavior.stddev =
      Matrix::Constant(1, static_cast<Eigen::Index>(p.horizon), 0.4);
  behavior.action = behavior.mean.col(0);
  const auto initial = policy.policyMean(state);
  for (size_t i = 0; i < p.replayCapacity + 2; ++i)
    policy.observe(state, behavior, 0., state, true);
  EXPECT_LONGS_EQUAL(p.replayCapacity, policy.replaySize());
  policy.train(20);
  EXPECT(policy.policyMean(state)[0] > initial[0] + 0.1);
  EXPECT(policy.plan(state).action.allFinite());
}

// Warmup, both learned-prior KL directions, target updates and lazy reanalysis
// are reproducible over a sequence containing terminal and continuing steps.
TEST(PolicyOptimizationMPC, AdaptivePriorTraining) {
  for (bool forward : {false, true}) {
    auto p = small();
    p.warmupSteps = 2;
    p.updateInterval = 1;
    p.reanalyzeInterval = 2;
    p.forwardPriorKL = forward;
    PolicyOptimizationMPC learner(1, 1, p), replica(1, 1, p);
    Vector state = Vector::Zero(1);
    const Vector initial = learner.policyMean(state);
    for (int t = 0; t < 6; ++t) {
      const auto a = learner.plan(state), b = replica.plan(state);
      EXPECT(assert_equal(a.action, b.action));
      if (t < 2) EXPECT(a.mean.isZero());
      const Vector next = state + 0.1 * a.action;
      const bool terminal = t % 3 == 2;
      learner.observe(state, a, -next.squaredNorm(), next, terminal);
      replica.observe(state, b, -next.squaredNorm(), next, terminal);
      state = terminal ? Vector::Zero(1) : next;
    }
    EXPECT(learner.policyMean(Vector::Zero(1)).allFinite());
    EXPECT((learner.policyMean(Vector::Zero(1)) - initial).norm() > 1e-8);
  }
}

// For a zero reward transition, a distant replay prior lowers the regularized
// TD target. Terminal transitions mask both continuation and KL penalties.
TEST(PolicyOptimizationMPC, CriticTargetsAndEpisodeBoundary) {
  auto p = small();
  p.horizon = 1;
  p.learnedPrior = false;
  p.klWeight = 10.;
  p.batchSize = 1;
  PolicyOptimizationMPC continuing(1, 1, p), terminal(1, 1, p);
  const Vector zero = Vector::Zero(1);
  PolicyOptimizationPlan behavior;
  behavior.mean = Matrix::Constant(1, 1, 3.);
  behavior.stddev = Matrix::Constant(1, 1, 0.2);
  behavior.action = zero;
  // Consecutive observations provide the next-state planning prior in replay.
  for (int i = 0; i < 8; ++i) {
    continuing.observe(zero, behavior, 0., zero, false);
    terminal.observe(zero, behavior, 0., zero, true);
  }
  continuing.train(20);
  terminal.train(20);
  EXPECT(continuing.regularizedValue(zero, zero) <
         terminal.regularizedValue(zero, zero) - 0.01);
  EXPECT(continuing.regularizedValue(zero, zero) <
         continuing.bootstrapValue(zero, zero) - 0.01);
}

// Reset and termination both stop replay. Their critic updates coincide at
// discount zero and differ when the truncated transition has a bootstrap term.
TEST(PolicyOptimizationMPC, ResetStopsReplaySequences) {
  auto p = small();
  PolicyOptimizationPlan behavior;
  behavior.mean = Matrix::Zero(1, static_cast<Eigen::Index>(p.horizon));
  behavior.stddev = Matrix::Ones(1, static_cast<Eigen::Index>(p.horizon));
  behavior.action = Vector1{0.};
  const Vector zero = Vector1{0.};
  for (const double discount : {0.0, 0.99}) {
    p.discount = discount;
    PolicyOptimizationMPC truncated(1, 1, p), terminal(1, 1, p);
    for (int episode = 0; episode < 4; ++episode) {
      const Vector state = Vector1{static_cast<double>(episode)};
      const Vector next = state + Vector1{0.1};
      truncated.observe(state, behavior, 0.3 * episode, next, false);
      truncated.reset();
      terminal.observe(state, behavior, 0.3 * episode, next, true);
    }
    truncated.train(5);
    terminal.train(5);
    const double difference = truncated.bootstrapValue(zero, zero) -
                              terminal.bootstrapValue(zero, zero);
    if (discount == 0.) {
      EXPECT_DOUBLES_EQUAL(0., difference, 1e-12);
      EXPECT(
          assert_equal(terminal.policyMean(zero), truncated.policyMean(zero)));
    } else {
      EXPECT(std::abs(difference) > 1e-8);
    }
  }
}

// A time limit still bootstraps at the final observation. The planner prior
// for that observation is recomputed using the just-finished episode's cost.
TEST(PolicyOptimizationMPC, ResetBootstrapsAtFinalObservation) {
  auto p = small();
  p.learnedPrior = false;
  size_t finalStateEvaluations = 0;
  PolicyOptimizationMPC learner(1, 1, p);
  const Vector zero = Vector1{0.}, finalState = Vector1{2.};
  NonlinearFactorGraph graph;
  const Key key = symbol_shorthand::X(0);
  graph.emplace_shared<CustomFactor>(
      noiseModel::Unit::Create(1), KeyVector{key},
      [&](const CustomFactor&, const Values& values, OptionalJacobianVector) {
        if (values.at<Vector>(key)[0] == finalState[0]) ++finalStateEvaluations;
        return Vector1{0.};
      });
  const auto plan = learner.plan(zero, graph);
  learner.observe(zero, plan, 0., finalState, false);
  learner.reset();
  learner.observe(Vector1{-1.}, plan, 1., zero, true);
  EXPECT_LONGS_EQUAL(0, finalStateEvaluations);
  learner.train(5);
  EXPECT(finalStateEvaluations > 0);
}

// Warmup pretrains the bootstrap critic; the KL-regularized critic starts its
// own updates only after exploration.
TEST(PolicyOptimizationMPC, BootstrapPretraining) {
  auto p = small();
  p.warmupSteps = 4;
  PolicyOptimizationMPC learner(1, 1, p);
  const Vector zero = Vector::Zero(1);
  PolicyOptimizationPlan behavior;
  behavior.mean = Matrix::Zero(1, static_cast<Eigen::Index>(p.horizon));
  behavior.stddev =
      Matrix::Constant(1, static_cast<Eigen::Index>(p.horizon), p.maxStddev);
  behavior.action = zero;
  const double original = learner.regularizedValue(zero, zero);
  for (size_t i = 0; i < p.warmupSteps; ++i)
    learner.observe(zero, behavior, 1.5, zero, true);
  EXPECT(learner.bootstrapValue(zero, zero) > original);
  EXPECT_DOUBLES_EQUAL(original, learner.regularizedValue(zero, zero), 1e-12);
}

// Repeated terminal observations give identical mean gradients, even when a
// large minibatch's summed gradients would activate the clipping threshold.
TEST(PolicyOptimizationMPC, ClipAfterMinibatchAverage) {
  auto p = small();
  p.horizon = 1;
  p.seed = 11;
  p.batchSize = 1;
  PolicyOptimizationMPC single(1, 1, p);
  p.batchSize = 128;
  PolicyOptimizationMPC many(1, 1, p);
  const Vector zero = Vector1{0.};
  PolicyOptimizationPlan behavior;
  behavior.mean = Matrix::Zero(1, 1);
  behavior.stddev = Matrix::Ones(1, 1);
  behavior.action = zero;
  for (auto model : {&single, &many}) {
    model->observe(zero, behavior, 2., zero, true);
    model->train(1);
  }
  EXPECT_DOUBLES_EQUAL(single.bootstrapValue(zero, zero),
                       many.bootstrapValue(zero, zero), 1e-12);
}

// Adam learns a signed control response in both the state and reward outputs.
TEST(PolicyOptimizationMPC, AdamFitsTransitionAndReward) {
  auto p = small();
  p.horizon = 1;
  p.useAdam = true;
  p.learningRate = .02;
  PolicyOptimizationMPC model(1, 1, p);
  const Vector state = Vector1{0.};
  for (double u : {-1., 1.}) {
    PolicyOptimizationPlan behavior;
    behavior.mean = Matrix::Constant(1, 1, u);
    behavior.stddev = Matrix::Ones(1, 1);
    behavior.action = Vector1{u};
    model.observe(state, behavior, -u, Vector1{2. * u}, true);
  }
  model.train(200);
  for (double u : {-1., 1.}) {
    EXPECT(assert_equal(Vector(Vector2{2. * u, -u}),
                        model.predict(state, Vector1{u}), .05));
  }
}

// Independent search sessions copy the trained model, bound its rollout
// candidates, and accept a caller's externally time-aligned warm start.
TEST(PolicyOptimizationMPC, ForkAndBoundedRollout) {
  auto p = small();
  p.actionLimit = 0.25;
  PolicyOptimizationMPC policy(1, 1, p);
  const Vector x = Vector1{0.};
  auto original = policy.plan(x);
  policy.observe(x, original, 2., Vector1{0.5}, true);
  policy.train(2);
  auto a = policy.fork(56), b = policy.fork(56);
  EXPECT_LONGS_EQUAL(0, a->replaySize());
  EXPECT(assert_equal(policy.predict(x, original.action),
                      a->predict(x, original.action)));
  const Matrix nominal =
      Matrix::Constant(1, static_cast<Eigen::Index>(p.horizon), 10.);
  auto left = a->planFrom(x, nominal), right = b->planFrom(x, nominal);
  EXPECT(assert_equal(left.mean, right.mean));
  EXPECT(assert_equal(left.action, right.action));
  EXPECT(left.mean.cwiseAbs().maxCoeff() <= p.actionLimit + 1e-12);
  EXPECT(left.action.cwiseAbs().maxCoeff() <= p.actionLimit);
  CHECK_EXCEPTION(a->planFrom(x, Matrix::Zero(3, 2)), std::invalid_argument);
}

// An external search can change the rollout population and spread on frozen
// weights.
TEST(PolicyOptimizationMPC, ExplicitSearch) {
  const auto p = small();
  PolicyOptimizationMPC model(1, 1, p);
  auto a = model.fork(9), b = model.fork(9);
  const Vector x = Vector1::Zero();
  const Matrix nominal = Matrix::Zero(1, static_cast<Eigen::Index>(p.horizon));
  GaussianNoiseSampler sampler(0.01 * Matrix1::Identity(), 128, 71);
  const EliteMPPI update(32);
  const auto result = a->searchFrom(x, nominal, sampler, 32, update, 1);
  EXPECT_LONGS_EQUAL(128, result.modelRollouts);
  EXPECT(assert_equal(result.mean,
                      b->searchFrom(x, nominal, sampler, 32, update, 1).mean));
  CHECK_EXCEPTION(a->searchFrom(x, nominal, sampler, 129, update, 1),
                  std::invalid_argument);
}

// Invalid planning inputs and training before observation fail explicitly.
TEST(PolicyOptimizationMPC, InputValidation) {
  auto p = small();
  PolicyOptimizationMPC policy(1, 1, p);
  CHECK_EXCEPTION(policy.train(1), std::logic_error);
  CHECK_EXCEPTION(policy.plan(Vector::Zero(2)), std::invalid_argument);
  auto plan = policy.plan(Vector::Zero(1));
  plan.stddev.setZero();
  CHECK_EXCEPTION(
      policy.observe(Vector::Zero(1), plan, 0., Vector::Zero(1), false),
      std::invalid_argument);
  EXPECT_LONGS_EQUAL(0, policy.replaySize());
  p.elites = p.samples + 1;
  CHECK_EXCEPTION(PolicyOptimizationMPC(1, 1, p), std::invalid_argument);
}

}  // namespace policy_optimization
/* ************************************************************************* */

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}

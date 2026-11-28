// Copyright (c) Georgia Tech Research Corporation. See LICENSE for details.

/**
 * @file testGenerativePredictiveControl.cpp
 * @author Frank Dellaert
 * @author Akash Ratheesh
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/planning/SamplingPlanner.h>
#include <gtsam_unstable/planning/GenerativePredictiveControl.h>

#include <cmath>

using namespace gtsam;
using symbol_shorthand::U;
using symbol_shorthand::X;

/* ************************************************************************* */
namespace flow_policy {

// Zero initial velocity makes Equation 28 an exact test of warm starting.
TEST(GPC, WarmStartAndMixture) {
  const Matrix previous{{1., -2., 3.}};
  const Vector observation = Vector::Ones(1);
  GenerativePredictiveControl cold(1, 1, 3, 8, 21), warm = cold;
  const Matrix draw = cold.generate(observation, previous, 0., 3);
  EXPECT(assert_equal(Matrix(0.25 * previous + 0.75 * draw),
                      warm.generate(observation, previous, 0.25, 3)));
  EXPECT(assert_equal(previous, warm.generate(observation, previous, 1., 3)));
  const Matrix batch = warm.candidates(observation, previous, 2, 4, 0., 1., 3);
  EXPECT(assert_equal(Matrix(previous.replicate(1, 6)), batch));
  CHECK_EXCEPTION(warm.generate(observation, previous, 1.1, 2),
                  std::invalid_argument);
}

// Branch copies draw the same Gaussian and flow candidates at each step
// without advancing the standalone policy's sampling stream.
TEST(GPC, CopiedProposalStreams) {
  const Vector observation = Vector1{0.5};
  const Matrix nominal{{1., -2.}};
  auto policy = std::make_shared<GenerativePredictiveControl>(1, 1, 2, 8, 21);
  const auto standalone = *policy;
  GPCProposal proposal(policy, observation, Matrix1::Identity(), 2, 2, 0.5, 3);
  const auto search = proposal.initialize(nominal);
  const Matrix first = proposal.sample(search, {}, 1., false).controls;
  const GaussianProposal& base = proposal;
  const auto branch = base.clone();
  EXPECT(dynamic_cast<GPCProposal*>(branch.get()) != nullptr);
  const Matrix second = proposal.sample(search, {}, 1., false).controls;
  EXPECT(assert_equal(second, branch->sample(search, {}, 1., false).controls));
  const Matrix third = proposal.sample(search, {}, 1., false).controls;
  EXPECT(assert_equal(third, branch->sample(search, {}, 1., false).controls));
  EXPECT(!first.rightCols(4).isApprox(second.rightCols(4)));
  EXPECT(!second.rightCols(4).isApprox(third.rightCols(4)));
  EXPECT(assert_equal(standalone.generate(observation, nominal, 0.5, 3),
                      policy->generate(observation, nominal, 0.5, 3)));
}

// A conditional flow fitted to two opposing demonstrations follows the
// observation.
TEST(GPC, SupervisedFlowMatching) {
  GenerativePredictiveControl model(1, 1, 1, 16, 7);
  const Matrix observations{{-1., 1.}}, targets{{-1., 1.}};
  const Matrix guesses = Matrix::Zero(1, 2);
  const double loss =
      model.fit(observations, guesses, targets, 400, 16, .018, 0.);
  EXPECT(std::isfinite(loss));
  for (double y : {-1., 1.}) {
    auto plan = model.generate(Vector1{y}, Matrix::Zero(1, 1), 1., 12);
    EXPECT(std::abs(plan(0, 0) - y) < .35);
  }
}

// Two consecutive AdamW fits keep both optimizer moments and sampling state.
TEST(GPC, AdamWContinuesAcrossCycles) {
  const Matrix observations{{-1., 1.}}, targets{{-1., 1.}, {0.4, -0.4}};
  const Matrix previous = Matrix::Zero(2, 2);
  GenerativePredictiveControl whole(1, 1, 2, 24, 109), split = whole;
  const double expected =
      whole.fitAdamW(observations, previous, targets, 500, 32, 0.003, 0.);
  split.fitAdamW(observations, previous, targets, 250, 32, 0.003, 0.);
  const double actual =
      split.fitAdamW(observations, previous, targets, 250, 32, 0.003, 0.);
  DOUBLES_EQUAL(expected, actual, 1e-12);
  for (Eigen::Index j = 0; j < observations.cols(); ++j) {
    const Matrix start = Matrix::Zero(1, 2);
    const Matrix plan = whole.generate(observations.col(j), start, 1., 10);
    EXPECT(
        assert_equal(plan, split.generate(observations.col(j), start, 1., 10)));
    EXPECT((plan.transpose() - targets.col(j)).norm() < 0.35);
  }
}

// C++ Algorithm 1 alternates closed-loop predictive search with flow fitting.
TEST(GPC, TrainAndPlanWithGraph) {
  const SamplingPlanner<Vector1, Vector1> planner({X(0), X(1), X(2)},
                                                  {U(0), U(1)}, 1.);
  NonlinearFactorGraph cost;
  cost.addPrior(X(2), Vector1{0.}, noiseModel::Unit::Create(1));
  const auto dynamics = [](const Vector1&, const Vector1& u) { return u; };
  GenerativePredictiveControl model(1, 1, 2, 12, 42);
  const auto score = [&](const Vector& x, const Matrix& samples) {
    Values initial;
    initial.insert(X(0), Vector1{x[0]});
    Vector costs(samples.cols() / 2);
    for (Eigen::Index j = 0; j < costs.size(); ++j)
      costs[j] = cost.error(
          planner.rollout(initial, samples.middleCols(2 * j, 2), dynamics));
    return costs;
  };
  GPCTrainingParams params;
  params.episodeLength = 5;
  params.gaussianSamples = 10;
  params.policySamples = 4;
  params.flowSteps = 3;
  params.gradientSteps = 15;
  params.batchSize = 4;
  size_t transitions = 0;
  for (int cycle = 0; cycle < 2; ++cycle) {
    const double loss = model.trainCycle(
        Matrix{{-1., 1.}}, params, [](const Vector& x) { return x; },
        [&](const Vector& x, const Vector& u) -> Vector {
          ++transitions;
          return x + u;
        },
        score,
        [](const Vector& costs) {
          return PredictiveSampling().weights(costs);
        });
    EXPECT(std::isfinite(loss));
  }
  EXPECT_LONGS_EQUAL(20, transitions);
  const Vector1 x{1.};
  Values initial;
  initial.insert(X(0), x);
  initial.insert(U(0), Vector1{0.});
  initial.insert(U(1), Vector1{0.});
  GPCProposal proposal(std::make_shared<GenerativePredictiveControl>(model), x,
                       Matrix1::Identity(), 10, 6, 0.5, 3);
  const auto plan = planner.optimize(cost, initial, dynamics, proposal,
                                     PredictiveSampling(), 2);
  EXPECT(cost.error(plan) < 0.5);
}

// A fractional tail includes partial mass on the quantile boundary.
TEST(GPC, RiskAcrossDomains) {
  const Matrix costs{{20., 0., 10.}, {3., 9., 0.}};
  EXPECT(assert_equal(Vector2(10., 4.),
                      GenerativePredictiveControl::riskCosts(costs, 1.)));
  EXPECT(assert_equal(Vector2(20., 9.),
                      GenerativePredictiveControl::riskCosts(costs, 0.)));
  EXPECT(assert_equal(Vector2(25. / 1.5, 10.5 / 1.5),
                      GenerativePredictiveControl::riskCosts(costs, 0.5)));
}
}  // namespace flow_policy
/* ************************************************************************* */

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}

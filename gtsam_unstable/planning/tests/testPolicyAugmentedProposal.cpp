// Copyright (c) Georgia Tech Research Corporation. See LICENSE for details.
/**
 * @file testPolicyAugmentedProposal.cpp
 * @author Frank Dellaert
 * @author Akash Ratheesh
 */
#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/planning/MPPITracking.h>
#include <gtsam_unstable/planning/PolicyAugmentedProposal.h>

using namespace gtsam;

/* ************************************************************************* */
namespace policy_sampling {
using symbol_shorthand::X;
KeyVector stateKeys() { return {0, 1, 2}; }
KeyVector controlKeys() { return {3, 4}; }
using Planner = SamplingPlanner<Vector1, Vector1>;
const auto dynamics = [](const Vector1&, const Vector1& u) { return u; };
const auto feedback = [](size_t, const Vector1& x, const Vector1& ref,
                         const Vector1&) { return Vector1(ref - x); };

Values initial(double start) {
  Values values;
  for (Key key : stateKeys()) values.insert(key, Vector1{start});
  for (Key key : controlKeys()) values.insert(key, Vector1{0.});
  return values;
}
NonlinearFactorGraph cost() {
  NonlinearFactorGraph graph;
  graph.addPrior(stateKeys().front(), Vector1{0.}, noiseModel::Unit::Create(1));
  return graph;
}

// The extra policy draws see the measured observation and the current plan.
// A winning injected plan centers both candidate sources on the next draw.
TEST(ControlProposals, PolicyAugmentation) {
  const Matrix nominal{{2., -1.}};
  Values context;
  context.insert(X(0), 4.0);
  size_t calls = 0;
  const auto generator = [&](const Values& initial,
                             const Matrix& mean) -> Matrix {
    ++calls;
    return mean.array() + initial.at<double>(X(0));
  };
  PolicyAugmentedProposal proposal(Matrix1::Identity(), 2, 1, generator);
  auto search = proposal.initialize(nominal);
  const auto batch = proposal.sample(search, context, 0.1, true);
  EXPECT_LONGS_EQUAL(1, calls);
  EXPECT_LONGS_EQUAL(6, batch.controls.cols());
  EXPECT(assert_equal(nominal, Matrix(batch.controls.leftCols(2))));
  const Matrix guided = nominal.array() + 4.;
  EXPECT(assert_equal(guided, Matrix(batch.controls.rightCols(2))));
  PredictiveSampling().apply(proposal, search, batch, Vector3{9., 4., 0.});
  const auto next = proposal.sample(search, context, 0.1, true);
  EXPECT(assert_equal(guided, Matrix(next.controls.leftCols(2))));
  EXPECT(assert_equal(Matrix(guided.array() + 4.),
                      Matrix(next.controls.rightCols(2))));
  PolicyAugmentedProposal bad(
      Matrix1::Identity(), 1, 1,
      [](const Values&, const Matrix&) { return Matrix(); });
  CHECK_EXCEPTION(bad.sample(search, context, 0.1, false),
                  std::invalid_argument);
}

// Initial states condition policy trajectories while the seed of each policy
// candidate is replayed across Tube branches and robust line-search points.
TEST(PolicyAugmentedProposal, CoupledSeededPolicy) {
  std::vector<std::pair<double, std::uint64_t>> draws;
  PolicyAugmentedProposal proposal(
      Matrix1::Identity(), 2, 2,
      [&](const Values& v, const Matrix& mean, std::uint64_t seed) {
        const double x = v.at<Vector1>(stateKeys()[0])[0];
        draws.emplace_back(x, seed);
        return Matrix::Constant(1, mean.cols(), x + double(seed % 100) / 100.);
      },
      23);
  Planner planner(stateKeys(), controlKeys(), 1.);
  MPPITracking tracking(Matrix1::Identity(), 16, 1.);
  Values nominal = initial(0.);
  nominal.update(stateKeys()[1], Vector1{1.});
  const GaussianProposal& base = proposal;
  tracking.tube(planner, cost(), nominal, initial(2.), dynamics, feedback, base,
                0., 1);
  EXPECT_LONGS_EQUAL(4, draws.size());
  EXPECT_DOUBLES_EQUAL(2., draws[0].first, 0.);
  EXPECT_DOUBLES_EQUAL(0., draws[2].first, 0.);
  EXPECT(draws[0].second == draws[2].second);
  EXPECT(draws[1].second == draws[3].second);
  draws.clear();
  auto result =
      tracking.robust(planner, cost(), nominal, initial(2.), dynamics, feedback,
                      base, 1e3, 3, 1, 1, Matrix1::Identity());
  EXPECT_LONGS_EQUAL(2, result.candidate);
  EXPECT_LONGS_EQUAL(8, draws.size());
  for (size_t i = 0; i < 3; ++i) {
    EXPECT_DOUBLES_EQUAL(double(i), draws[2 * i].first, 0.);
    EXPECT(draws[0].second == draws[2 * i].second);
    EXPECT(draws[1].second == draws[2 * i + 1].second);
  }
  EXPECT(draws[6].second != draws[0].second);
}

}  // namespace policy_sampling

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}

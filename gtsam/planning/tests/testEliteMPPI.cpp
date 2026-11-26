// Copyright (c) Georgia Tech Research Corporation. See LICENSE for details.
/**
 * @file testEliteMPPI.cpp
 * @author Frank Dellaert
 * @author Akash Ratheesh
 */
#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/planning/EliteMPPI.h>

#include <cmath>
#include <limits>
using namespace gtsam;

// Rejected and infeasible candidates receive zero weight; elite odds equal
// exp(-beta * delta).
TEST(EliteMPPI, NormalizedEliteWeights) {
  const EliteMPPI rule(2, 0.5);
  const Vector4 costs{4., 0., 2., std::numeric_limits<double>::infinity()};
  const double first = 1. / (1. + std::exp(-1.));
  EXPECT(assert_equal(Vector4(0., first, 1. - first, 0.), rule.weights(costs)));
}

// The Gaussian fit recovers weighted elite mean and marginal population
// deviation.
TEST(EliteMPPI, WeightedFit) {
  GaussianProposal proposal(Matrix1::Identity(), 3);
  auto state = proposal.initialize(Matrix::Zero(1, 1));
  const ControlBatch batch{Matrix{{-8., 1., 3.}}, {}};
  const EliteMPPI rule(2, std::log(3.));
  const auto weights = rule.apply(proposal, state, batch, Vector3{10., 0., 1.});
  EXPECT(assert_equal(Vector3(0., 0.75, 0.25), weights));
  EXPECT_DOUBLES_EQUAL(1.5, state.mean(0), 1e-12);
  EXPECT_DOUBLES_EQUAL(std::sqrt(0.75), state.stddev(0), 1e-12);
}

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}

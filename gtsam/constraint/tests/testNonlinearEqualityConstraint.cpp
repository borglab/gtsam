/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testNonlinearEqualityConstraint.cpp
 * @brief   unit tests for nonlinear equality constraints
 * @author  Yetong Zhang
 * @date    Aug 3, 2024
 */

#include <CppUnitLite/Test.h>
#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/constraint/NonlinearEqualityConstraint.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/slam/BetweenFactor.h>

#include "constrainedExample.h"

using namespace gtsam;
using constrained_example::pow;
using constrained_example::x1, constrained_example::x2;
using constrained_example::x1_key, constrained_example::x2_key;

// Test methods of DoubleExpressionEquality.
TEST(ExpressionEqualityConstraint, double) {
  // create constraint from double expression
  // g(x1, x2) = x1 + x1^3 + x2 + x2^2, from Vanderbergh slides
  Vector sigmas = Vector1(0.1);
  auto g = x1 + pow(x1, 3) + x2 + pow(x2, 2);
  auto constraint = ExpressionEqualityConstraint<double>(g, 0.0, sigmas);

  EXPECT(constraint.noiseModel()->isConstrained());
  EXPECT(assert_equal(sigmas, constraint.noiseModel()->sigmas()));

  // Create 2 sets of values for testing.
  Values values1, values2;
  values1.insert(x1_key, 0.0);
  values1.insert(x2_key, 0.0);
  values2.insert(x1_key, 1.0);
  values2.insert(x2_key, 1.0);

  // Check that values1 are feasible.
  EXPECT(constraint.feasible(values1));

  // Check that violation evaluates as 0 at values1.
  EXPECT(assert_equal(Vector::Zero(1), constraint.unwhitenedError(values1)));
  EXPECT(assert_equal(Vector::Zero(1), constraint.whitenedError(values1)));
  EXPECT(assert_equal(0.0, constraint.error(values1)));

  // Check that values2 are indeed deemed infeasible.
  EXPECT(!constraint.feasible(values2));

  // Check constraint violation is indeed g(x) at values2.
  EXPECT(assert_equal(Vector::Constant(1, 4.0), constraint.unwhitenedError(values2)));
  EXPECT(assert_equal(Vector::Constant(1, 40), constraint.whitenedError(values2)));
  EXPECT(assert_equal(800, constraint.error(values2)));

  // Check dimension is 1 for scalar g.
  EXPECT(constraint.dim() == 1);

  // Check keys include x1, x2.
  EXPECT(constraint.keys().size() == 2);
  EXPECT(x1_key == *constraint.keys().begin());
  EXPECT(x2_key == *constraint.keys().rbegin());

  // Generate factor representing the term in merit function.
  double mu = 4;
  auto merit_factor = constraint.penaltyFactor(mu);

  // Check that noise model sigma == sigmas/sqrt(mu).
  auto expected_noise = noiseModel::Diagonal::Sigmas(sigmas / sqrt(mu));
  EXPECT(expected_noise->equals(*merit_factor->noiseModel()));

  // Check that error is equal to 0.5*mu * (g(x)+bias)^2/sigmas^2.
  double expected_error1 = 0;  // 0.5 * 4 * ||0||_(0.1^2)^2
  EXPECT(assert_equal(expected_error1, merit_factor->error(values1)));
  double expected_error2 = 3200;  // 0.5 * 4 * ||4||_(0.1^2)^2
  EXPECT(assert_equal(expected_error2, merit_factor->error(values2)));

  // Check jacobian computation is correct.
  EXPECT_CORRECT_FACTOR_JACOBIANS(*merit_factor, values1, 1e-7, 1e-5);
  EXPECT_CORRECT_FACTOR_JACOBIANS(*merit_factor, values2, 1e-7, 1e-5);
}

// Test methods of VectorExpressionEquality.
TEST(ExpressionEqualityConstraint, Vector2) {
  // g(v1, v2) = v1 + v2, our own example.
  Vector2_ x1_vec_expr(x1_key);
  Vector2_ x2_vec_expr(x2_key);
  auto g = x1_vec_expr + x2_vec_expr;
  auto sigmas = Vector2(0.1, 0.5);
  auto constraint = ExpressionEqualityConstraint<Vector2>(g, Vector2::Zero(), sigmas);

  EXPECT(constraint.noiseModel()->isConstrained());
  EXPECT(assert_equal(sigmas, constraint.noiseModel()->sigmas()));

  // Create 2 sets of values for testing.
  Values values1, values2;
  values1.insert(x1_key, Vector2(1, 1));
  values1.insert(x2_key, Vector2(-1, -1));
  values2.insert(x1_key, Vector2(1, 1));
  values2.insert(x2_key, Vector2(1, 1));

  // Check that values1 are feasible.
  EXPECT(constraint.feasible(values1));

  // Check that violation evaluates as 0 at values1.
  auto expected_violation1 = (Vector(2) << 0, 0).finished();
  EXPECT(assert_equal(expected_violation1, constraint.unwhitenedError(values1)));
  auto expected_scaled_violation1 = (Vector(2) << 0, 0).finished();
  EXPECT(assert_equal(expected_scaled_violation1, constraint.whitenedError(values1)));

  // Check that values2 are indeed deemed infeasible.
  EXPECT(!constraint.feasible(values2));

  // Check constraint violation is indeed g(x) at values2.
  auto expected_violation2 = (Vector(2) << 2, 2).finished();
  EXPECT(assert_equal(expected_violation2, constraint.unwhitenedError(values2)));

  // Check scaled violation is indeed g(x)/sigmas at values2.
  auto expected_scaled_violation2 = (Vector(2) << 20, 4).finished();
  EXPECT(assert_equal(expected_scaled_violation2, constraint.whitenedError(values2)));

  // Check dim is the dimension of the vector.
  EXPECT(constraint.dim() == 2);

  // Generate factor representing the term in merit function.
  double mu = 4;
  auto merit_factor = constraint.penaltyFactor(mu);

  // Check that noise model sigma == sigmas/sqrt(mu).
  auto expected_noise = noiseModel::Diagonal::Sigmas(sigmas / sqrt(mu));
  EXPECT(expected_noise->equals(*merit_factor->noiseModel()));

  // Check that error is equal to 0.5*mu*||g(x)+bias)||^2_Diag(sigmas^2).
  double expected_error1 = 0;  // 0.5 * 4 * ||[1, 0.5]||_([0.1,0.5]^2)^2
  EXPECT(assert_equal(expected_error1, merit_factor->error(values1)));
  double expected_error2 = 832;  // 0.5 * 4 * ||[2, 2]||_([0.1,0.5]^2)^2
  EXPECT(assert_equal(expected_error2, merit_factor->error(values2)));

  // Check jacobian computation is correct.
  EXPECT_CORRECT_FACTOR_JACOBIANS(*merit_factor, values1, 1e-7, 1e-5);
  EXPECT_CORRECT_FACTOR_JACOBIANS(*merit_factor, values2, 1e-7, 1e-5);
}

// Test methods of FactorZeroErrorConstraint.
TEST(ZeroCostConstraint, BetweenFactor) {
  Key x1_key = 1;
  Key x2_key = 2;
  Vector sigmas = Vector2(0.5, 0.1);
  auto noise = noiseModel::Diagonal::Sigmas(sigmas);

  auto factor = std::make_shared<BetweenFactor<Vector2>>(x1_key, x2_key, Vector2(1, 1), noise);
  auto constraint = ZeroCostConstraint(factor);

  EXPECT(constraint.noiseModel()->isConstrained());
  EXPECT(assert_equal(sigmas, constraint.noiseModel()->sigmas()));

  // Create 2 sets of values for testing.
  Values values1, values2;
  values1.insert(x1_key, Vector2(1, 1));
  values1.insert(x2_key, Vector2(2, 2));
  values2.insert(x1_key, Vector2(0, 0));
  values2.insert(x2_key, Vector2(2, 3));

  // Check that values1 are feasible.
  EXPECT(constraint.feasible(values1));

  // Check that violation evaluates as 0 at values1.
  auto expected_violation1 = (Vector(2) << 0, 0).finished();
  EXPECT(assert_equal(expected_violation1, constraint.unwhitenedError(values1)));
  auto expected_scaled_violation1 = (Vector(2) << 0, 0).finished();
  EXPECT(assert_equal(expected_scaled_violation1, constraint.whitenedError(values1)));

  // Check that values2 are indeed deemed infeasible.
  EXPECT(!constraint.feasible(values2));

  // Check constraint violation is indeed g(x) at values2.
  auto expected_violation2 = (Vector(2) << 1, 2).finished();
  EXPECT(assert_equal(expected_violation2, constraint.unwhitenedError(values2)));

  // Check scaled violation is indeed g(x)/sigmas at values2.
  auto expected_scaled_violation2 = (Vector(2) << 2, 20).finished();
  EXPECT(assert_equal(expected_scaled_violation2, constraint.whitenedError(values2)));

  // Check dim is the dimension of the vector.
  EXPECT(constraint.dim() == 2);

  // Generate factor representing the term in merit function.
  double mu = 4;
  auto merit_factor = constraint.penaltyFactor(mu);

  // Check that noise model sigma == sigmas/sqrt(mu).
  auto expected_noise = noiseModel::Diagonal::Sigmas(sigmas / sqrt(mu));
  EXPECT(expected_noise->equals(*merit_factor->noiseModel()));

  // Check that error is equal to 0.5*mu*||g(x)+bias)||^2_Diag(sigmas^2).
  double expected_error1 = 0;  // 0.5 * 4 * ||[0, 0]||_([0.5,0.1]^2)^2
  EXPECT(assert_equal(expected_error1, merit_factor->error(values1)));
  double expected_error2 = 808;  // 0.5 * 4 * ||[1, 2]||_([0.5,0.1]^2)^2
  EXPECT(assert_equal(expected_error2, merit_factor->error(values2)));

  // Check jacobian computation is correct.
  EXPECT_CORRECT_FACTOR_JACOBIANS(*merit_factor, values1, 1e-7, 1e-5);
  EXPECT_CORRECT_FACTOR_JACOBIANS(*merit_factor, values2, 1e-7, 1e-5);
}

TEST(NonlinearEqualityConstraints, Container) {
  NonlinearEqualityConstraints constraints;

  Vector sigmas1 = Vector1(10);
  auto g1 = x1 + pow(x1, 3) + x2 + pow(x2, 2);
  Vector2_ x1_vec_expr(x1_key);
  Vector2_ x2_vec_expr(x2_key);
  auto g2 = x1_vec_expr + x2_vec_expr;
  Vector sigmas2 = Vector2(0.1, 0.5);

  constraints.emplace_shared<ExpressionEqualityConstraint<double>>(g1, 0.0, sigmas1);
  constraints.emplace_shared<ExpressionEqualityConstraint<Vector2>>(g2, Vector2::Zero(), sigmas2);

  // Check size.
  EXPECT_LONGS_EQUAL(2, constraints.size());

  // Check dimension.
  EXPECT_LONGS_EQUAL(3, constraints.dim());

  // Check keys.
  KeySet expected_keys;
  expected_keys.insert(x1_key);
  expected_keys.insert(x2_key);
  EXPECT(assert_container_equality(expected_keys, constraints.keys()));

  // Check VariableIndex.
  VariableIndex vi(constraints);
  FactorIndices expected_vi_x1{0, 1};
  FactorIndices expected_vi_x2{0, 1};
  EXPECT(assert_container_equality(expected_vi_x1, vi[x1_key]));
  EXPECT(assert_container_equality(expected_vi_x2, vi[x2_key]));

  // Check constraint violation.
}

TEST(NonlinearEqualityConstraints, FromCostGraph) {}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}

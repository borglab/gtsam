/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    NonlinearInequalityConstraint.h
 * @brief   Nonlinear inequality constraints in constrained optimization.
 * @author  Yetong Zhang
 * @date    Aug 3, 2024
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/constraint/NonlinearInequalityConstraint.h>
#include <gtsam/nonlinear/factorTesting.h>

#include "constrainedExample.h"

using namespace gtsam;
using constrained_example::pow;
using constrained_example::x1, constrained_example::x2;
using constrained_example::x1_key, constrained_example::x2_key;

// Test methods of DoubleExpressionEquality.
TEST(NonlinearInequalityConstraint, ScalarExpressionInequalityConstraint) {
  // create constraint from double expression
  // g(x1, x2) = x1 + x1^3 + x2 + x2^2, from Vanderbergh slides
  double sigma = 0.1;
  auto g = x1 + pow(x1, 3) + x2 + pow(x2, 2);
  auto constraint_geq = ScalarExpressionInequalityConstraint::GeqZero(g, sigma);
  auto constraint_leq = ScalarExpressionInequalityConstraint::LeqZero(g, sigma);

  // Check dimension is 1 for scalar g.
  EXPECT_LONGS_EQUAL(1, constraint_geq->dim());
  EXPECT_LONGS_EQUAL(1, constraint_leq->dim());

  // Check keys include x1, x2.
  KeyVector expected_keys{x1_key, x2_key};
  EXPECT(assert_container_equality(expected_keys, constraint_leq->keys()));

  // Create 3 sets of values for testing.
  Values values1, values2, values3;
  values1.insert(x1_key, -1.0);
  values1.insert(x2_key, 1.0);
  values2.insert(x1_key, 1.0);
  values2.insert(x2_key, 1.0);
  values3.insert(x1_key, -2.0);
  values3.insert(x2_key, 1.0);

  // Check that expression evaluation at different values.
  EXPECT(assert_equal(Vector1(0.0), constraint_leq->unwhitenedExpr(values1)));
  EXPECT(assert_equal(Vector1(4.0), constraint_leq->unwhitenedExpr(values2)));
  EXPECT(assert_equal(Vector1(-8.0), constraint_leq->unwhitenedExpr(values3)));
  EXPECT(assert_equal(Vector1(0.0), constraint_geq->unwhitenedExpr(values1)));
  EXPECT(assert_equal(Vector1(-4.0), constraint_geq->unwhitenedExpr(values2)));
  EXPECT(assert_equal(Vector1(8.0), constraint_geq->unwhitenedExpr(values3)));

  // Check that constraint violation at different values.
  EXPECT(assert_equal(Vector1(0.0), constraint_leq->unwhitenedError(values1)));
  EXPECT(assert_equal(Vector1(4.0), constraint_leq->unwhitenedError(values2)));
  EXPECT(assert_equal(Vector1(0.0), constraint_leq->unwhitenedError(values3)));
  EXPECT(assert_equal(Vector1(0.0), constraint_geq->unwhitenedError(values1)));
  EXPECT(assert_equal(Vector1(0.0), constraint_geq->unwhitenedError(values2)));
  EXPECT(assert_equal(Vector1(8.0), constraint_geq->unwhitenedError(values3)));

  // Check feasible.
  EXPECT(constraint_leq->feasible(values1));
  EXPECT(!constraint_leq->feasible(values2));
  EXPECT(constraint_leq->feasible(values3));
  EXPECT(constraint_geq->feasible(values1));
  EXPECT(constraint_geq->feasible(values2));
  EXPECT(!constraint_geq->feasible(values3));

  // Check active.
  EXPECT(constraint_leq->active(values1));
  EXPECT(constraint_leq->active(values2));
  EXPECT(!constraint_leq->active(values3));
  EXPECT(constraint_geq->active(values1));
  EXPECT(!constraint_geq->active(values2));
  EXPECT(constraint_geq->active(values3));

  // Check whitenedError of penalty factor.
  // Expected to be sqrt(mu) / sigma * ramp(g(x))
  double mu = 9.0;
  auto penalty_leq = constraint_leq->penaltyFactor(mu);
  auto penalty_geq = constraint_geq->penaltyFactor(mu);
  EXPECT(assert_equal(Vector1(0.0), penalty_leq->whitenedError(values1)));
  EXPECT(assert_equal(Vector1(120.0), penalty_leq->whitenedError(values2)));
  EXPECT(assert_equal(Vector1(0.0), penalty_leq->whitenedError(values3)));
  EXPECT(assert_equal(Vector1(0.0), penalty_geq->whitenedError(values1)));
  EXPECT(assert_equal(Vector1(0.0), penalty_geq->whitenedError(values2)));
  EXPECT(assert_equal(Vector1(240.0), penalty_geq->whitenedError(values3)));

  // Check create equality constraint
  auto constraint_eq1 = constraint_leq->createEqualityConstraint();
  auto constraint_eq2 = constraint_geq->createEqualityConstraint();
  EXPECT(assert_equal(0.0, constraint_eq1->error(values1)));
  EXPECT(assert_equal(800.0, constraint_eq1->error(values2)));
  EXPECT(assert_equal(3200.0, constraint_eq1->error(values3)));
  EXPECT(assert_equal(0.0, constraint_eq2->error(values1)));
  EXPECT(assert_equal(800.0, constraint_eq2->error(values2)));
  EXPECT(assert_equal(3200.0, constraint_eq2->error(values3)));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}

/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testLpProblem.cpp
 * @brief   Unit tests for LP constrained optimization problems.
 * @author  Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/constrained/LpProblem.h>
#include <gtsam/inference/Symbol.h>

using namespace gtsam;

/* ************************************************************************* */
namespace lp_problem_fixture {

const Key x = Symbol('x', 0);

Values VectorValue(double first, double second) {
  Values values;
  values.insert(x, (Vector(2) << first, second).finished());
  return values;
}

// Verifies LpCost evaluates the stored linear objective over direct values.
TEST(LpCost, Value) {
  const LpCost cost(
      JacobianFactor(x, (Matrix(1, 2) << 2.0, -1.0).finished(), Vector1(3.0)));

  EXPECT_DOUBLES_EQUAL(3.0, cost.value(VectorValue(4.0, 2.0)), 1e-12);
}

// Verifies LpProblem stores linear costs and LinearConstraint constraints.
TEST(LpProblem, ObjectiveAndConstraints) {
  LpProblem problem;
  problem.addCost(
      JacobianFactor(x, (Matrix(1, 2) << -1.0, -1.0).finished(), Vector1(0.0)));
  problem.addConstraint(LinearConstraint::LessEqual(
      JacobianFactor(x, (Matrix(1, 2) << 1.0, 1.0).finished(), Vector1(2.0))));

  const Values values = VectorValue(0.5, 1.0);
  const auto [cost, eqViolation, ineqViolation] = problem.evaluate(values);

  EXPECT_DOUBLES_EQUAL(-1.5, problem.objective(values), 1e-12);
  EXPECT_DOUBLES_EQUAL(0.0, cost, 1e-12);
  EXPECT_DOUBLES_EQUAL(0.0, eqViolation, 1e-12);
  EXPECT_DOUBLES_EQUAL(0.0, ineqViolation, 1e-12);
  CHECK_EQUAL(1, problem.linearCosts().size());
}

}  // namespace lp_problem_fixture
/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}

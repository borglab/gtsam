/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testLpSolver.cpp
 * @brief   Unit tests for linearly constrained LP active-set solving.
 * @author  Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/constrained/ActiveSetSolver.h>
#include <gtsam/inference/Symbol.h>

#include <stdexcept>

using namespace gtsam;

/* ************************************************************************* */
namespace scalar_lp_examples {

const Key x = Symbol('x', 0);

Matrix Matrix1(double value) { return (Matrix(1, 1) << value).finished(); }

Vector Vector1D(double value) { return (Vector(1) << value).finished(); }

Values Values1(double value) {
  Values values;
  values.insert(x, Vector1D(value));
  return values;
}

LpProblem CreateBounded1D() {
  LpProblem problem;
  problem.addCost(JacobianFactor(x, Matrix1(-1.0), Vector1D(0.0)));
  problem.addConstraint(LinearConstraint::GreaterEqual(
      JacobianFactor(x, Matrix1(1.0), Vector1D(0.0))));
  problem.addConstraint(LinearConstraint::LessEqual(
      JacobianFactor(x, Matrix1(1.0), Vector1D(2.0))));
  return problem;
}

// Verifies a bounded scalar LP optimizes to the active upper bound.
TEST(ActiveSetSolver, Bounded1DLP) {
  const Values result =
      ActiveSetSolver(CreateBounded1D()).optimize(Values1(0.0));

  EXPECT_DOUBLES_EQUAL(2.0, result.at<Vector>(x)(0), 1e-7);
}

// Verifies vector-valued no-initial LP solve uses phase-I initialization.
TEST(ActiveSetSolver, VectorOnlyNoInitialLP) {
  const Values result = CreateBounded1D().optimize();

  EXPECT_DOUBLES_EQUAL(2.0, result.at<Vector>(x)(0), 1e-7);
}

// Verifies infeasible initial values are rejected before iteration.
TEST(ActiveSetSolver, InfeasibleInitialLP) {
  LpProblem problem;
  problem.addCost(JacobianFactor(x, Matrix1(1.0), Vector1D(0.0)));
  problem.addConstraint(LinearConstraint::GreaterEqual(
      JacobianFactor(x, Matrix1(1.0), Vector1D(1.0))));

  CHECK_EXCEPTION(ActiveSetSolver(problem).optimize(Values1(0.0)),
                  std::invalid_argument);
}

// Verifies unbounded LP descent is detected when no inequality blocks it.
TEST(ActiveSetSolver, UnboundedLP) {
  LpProblem problem;
  problem.addCost(JacobianFactor(x, Matrix1(-1.0), Vector1D(0.0)));
  problem.addConstraint(LinearConstraint::GreaterEqual(
      JacobianFactor(x, Matrix1(1.0), Vector1D(0.0))));

  CHECK_EXCEPTION(ActiveSetSolver(problem).optimize(Values1(0.0)),
                  std::runtime_error);
}

}  // namespace scalar_lp_examples
/* ************************************************************************* */
namespace vector_lp_examples {

const Key x = Symbol('x', 1);

Values Values2(double first, double second) {
  Values values;
  values.insert(x, (Vector(2) << first, second).finished());
  return values;
}

LpProblem CreateOldSimpleLP() {
  LpProblem problem;
  problem.addCost(
      JacobianFactor(x, (Matrix(1, 2) << -1.0, -1.0).finished(), Vector1(0.0)));
  problem.addConstraint(LinearConstraint::GreaterEqual(
      JacobianFactor(x, (Matrix(1, 2) << 1.0, 0.0).finished(), Vector1(0.0))));
  problem.addConstraint(LinearConstraint::GreaterEqual(
      JacobianFactor(x, (Matrix(1, 2) << 0.0, 1.0).finished(), Vector1(0.0))));
  problem.addConstraint(LinearConstraint::LessEqual(
      JacobianFactor(x, (Matrix(1, 2) << 1.0, 2.0).finished(), Vector1(4.0))));
  problem.addConstraint(LinearConstraint::LessEqual(
      JacobianFactor(x, (Matrix(1, 2) << 4.0, 2.0).finished(), Vector1(12.0))));
  problem.addConstraint(LinearConstraint::LessEqual(
      JacobianFactor(x, (Matrix(1, 2) << -1.0, 1.0).finished(), Vector1(1.0))));
  return problem;
}

// Verifies a bounded 2D LP matches the legacy textbook solution.
TEST(ActiveSetSolver, Bounded2DLP) {
  const Values result =
      ActiveSetSolver(CreateOldSimpleLP()).optimize(Values2(0.0, 0.0));

  EXPECT(assert_equal((Vector(2) << 8.0 / 3.0, 2.0 / 3.0).finished(),
                      result.at<Vector>(x), 1e-7));
}

// Verifies equality-constrained LPs project onto the equality surface.
TEST(ActiveSetSolver, EqualityConstrainedLP) {
  LpProblem problem;
  problem.addCost(
      JacobianFactor(x, (Matrix(1, 2) << 1.0, 0.0).finished(), Vector1(0.0)));
  problem.addConstraint(LinearConstraint::Equal(
      JacobianFactor(x, (Matrix(1, 2) << 1.0, 1.0).finished(), Vector1(1.0))));
  problem.addConstraint(LinearConstraint::GreaterEqual(
      JacobianFactor(x, (Matrix(1, 2) << 1.0, 0.0).finished(), Vector1(0.0))));
  problem.addConstraint(LinearConstraint::GreaterEqual(
      JacobianFactor(x, (Matrix(1, 2) << 0.0, 1.0).finished(), Vector1(0.0))));

  const Values result = ActiveSetSolver(problem).optimize(Values2(0.5, 0.5));

  EXPECT(assert_equal((Vector(2) << 0.0, 1.0).finished(), result.at<Vector>(x),
                      1e-7));
}

// Verifies equality rows are handled by sparse phase-I initialization.
TEST(ActiveSetSolver, EqualityConstrainedNoInitialLP) {
  LpProblem problem;
  problem.addCost(
      JacobianFactor(x, (Matrix(1, 2) << 1.0, 0.0).finished(), Vector1(0.0)));
  problem.addConstraint(LinearConstraint::Equal(
      JacobianFactor(x, (Matrix(1, 2) << 1.0, 1.0).finished(), Vector1(1.0))));
  problem.addConstraint(LinearConstraint::GreaterEqual(
      JacobianFactor(x, (Matrix(1, 2) << 1.0, 0.0).finished(), Vector1(0.0))));
  problem.addConstraint(LinearConstraint::GreaterEqual(
      JacobianFactor(x, (Matrix(1, 2) << 0.0, 1.0).finished(), Vector1(0.0))));

  const Values result = problem.optimize();

  EXPECT(assert_equal((Vector(2) << 0.0, 1.0).finished(), result.at<Vector>(x),
                      1e-7));
}

// Verifies inequalities can enter and leave the active set during LP solve.
TEST(ActiveSetSolver, ActiveInequalityEnteringLeavingLP) {
  const auto [result, state] =
      ActiveSetSolver(CreateOldSimpleLP()).optimizeWithState(Values2(0.0, 0.0));

  EXPECT(assert_equal((Vector(2) << 8.0 / 3.0, 2.0 / 3.0).finished(),
                      result.at<Vector>(x), 1e-7));
  CHECK(state.iterations > 1);
  CHECK_EQUAL(5, state.inequalityMultipliers.size());
}

}  // namespace vector_lp_examples
/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}

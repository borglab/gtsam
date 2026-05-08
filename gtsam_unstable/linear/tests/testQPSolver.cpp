/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testQPSolver.cpp
 * @brief Tests for QPSolver (active-set QP with sparse Cholesky + Schur complement).
 * @date  May 2026
 * @author Frank Dellaert
 */

#include <gtsam/base/Testable.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam_unstable/linear/InfeasibleInitialValues.h>
#include <gtsam_unstable/linear/QPSolver.h>

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;
using namespace gtsam::symbol_shorthand;

static const Vector kOne = Vector::Ones(1), kZero = Vector::Zero(1);

/* ************************************************************************* */
// Problem factories
/* ************************************************************************* */

static QP createTestCase() {
  QP qp;
  qp.cost.push_back(HessianFactor(X(1), X(2), 2.0 * I_1x1, -I_1x1,
                                   3.0 * I_1x1, 2.0 * I_1x1, Z_1x1, 10.0));
  qp.inequalities.add(X(1), I_1x1, X(2), I_1x1, 2, 0);
  qp.inequalities.add(X(1), -I_1x1, 0, 1);
  qp.inequalities.add(X(2), -I_1x1, 0, 2);
  qp.inequalities.add(X(1), I_1x1, 1.5, 3);
  return qp;
}

static QP createEqualityConstrainedTest() {
  QP qp;
  qp.cost.push_back(HessianFactor(X(1), X(2), 2.0 * I_1x1, Z_1x1, Z_1x1,
                                   2.0 * I_1x1, Z_1x1, 0.0));
  Matrix A1 = I_1x1, A2 = I_1x1;
  Vector b = -kOne;
  qp.equalities.add(X(1), A1, X(2), A2, b, 0);
  return qp;
}

// From Matlab QP example: min 0.5*(x1^2 - 2*x1*x2 + 2*x2^2) - 2*x1 - 6*x2
// subject to x1+x2<=2, -x1+2*x2<=2, 2*x1+x2<=3, x1>=0, x2>=0
// Optimal solution: x1=2/3, x2=4/3
static QP createTestMatlabQPEx() {
  QP qp;
  qp.cost.push_back(HessianFactor(X(1), X(2), 1.0 * I_1x1, -I_1x1,
                                   2.0 * I_1x1, 2.0 * I_1x1, 6 * I_1x1,
                                   1000.0));
  qp.inequalities.add(X(1), I_1x1, X(2), I_1x1, 2, 0);
  qp.inequalities.add(X(1), -I_1x1, X(2), 2 * I_1x1, 2, 1);
  qp.inequalities.add(X(1), 2 * I_1x1, X(2), I_1x1, 3, 2);
  qp.inequalities.add(X(1), -I_1x1, 0, 3);
  qp.inequalities.add(X(2), -I_1x1, 0, 4);
  return qp;
}

// From Nocedal & Wright, Example 16.4: min 0.5*(x1-1)^2 + 0.5*(x2-2.5)^2
// subject to -x1+2*x2<=2, x1+2*x2<=6, x1-2*x2<=2, x1>=0, x2>=0
// Optimal solution: x1=1.4, x2=1.7
static QP createTestNocedal06bookEx16_4() {
  QP qp;
  qp.cost.add(X(1), I_1x1, I_1x1);
  qp.cost.add(X(2), I_1x1, 2.5 * I_1x1);
  qp.inequalities.add(X(1), -I_1x1, X(2), 2 * I_1x1, 2, 0);
  qp.inequalities.add(X(1), I_1x1, X(2), 2 * I_1x1, 6, 1);
  qp.inequalities.add(X(1), I_1x1, X(2), -2 * I_1x1, 2, 2);
  qp.inequalities.add(X(1), -I_1x1, 0.0, 3);
  qp.inequalities.add(X(2), -I_1x1, 0.0, 4);
  return qp;
}

/* ************************************************************************* */
TEST(QPSolver, Forst10book_pg171Ex5) {
  QP qp = createTestCase();
  VectorValues initial;
  initial.insert(X(1), Z_1x1);
  initial.insert(X(2), Z_1x1);

  QPSolver solver(qp);
  VectorValues solution = solver.optimize(initial).first;
  VectorValues expected;
  expected.insert(X(1), (Vector(1) << 1.5).finished());
  expected.insert(X(2), (Vector(1) << 0.5).finished());
  CHECK(assert_equal(expected, solution, 1e-7));
}

/* ************************************************************************* */
TEST(QPSolver, EqualityConstrained) {
  QP qp = createEqualityConstrainedTest();
  VectorValues initial;
  initial.insert(X(1), I_1x1);
  initial.insert(X(2), I_1x1);
  QPSolver solver(qp);
  VectorValues solution = solver.optimize(initial).first;
  // Cost x1^2 + x2^2 subject to x1 + x2 = -1 => x1 = x2 = -0.5
  VectorValues expected;
  expected.insert(X(1), (Vector(1) << -0.5).finished());
  expected.insert(X(2), (Vector(1) << -0.5).finished());
  CHECK(assert_equal(expected, solution, 1e-7));
}

/* ************************************************************************* */
TEST(QPSolver, MatlabEx) {
  QP qp = createTestMatlabQPEx();
  VectorValues initial;
  initial.insert(X(1), Z_1x1);
  initial.insert(X(2), Z_1x1);
  QPSolver solver(qp);
  VectorValues solution = solver.optimize(initial).first;
  VectorValues expected;
  expected.insert(X(1), (Vector(1) << 2.0 / 3.0).finished());
  expected.insert(X(2), (Vector(1) << 4.0 / 3.0).finished());
  CHECK(assert_equal(expected, solution, 1e-7));
}

/* ************************************************************************* */
TEST(QPSolver, MatlabExNoInitials) {
  QP qp = createTestMatlabQPEx();
  QPSolver solver(qp);
  VectorValues solution = solver.optimize().first;
  VectorValues expected;
  expected.insert(X(1), (Vector(1) << 2.0 / 3.0).finished());
  expected.insert(X(2), (Vector(1) << 4.0 / 3.0).finished());
  CHECK(assert_equal(expected, solution, 1e-7));
}

/* ************************************************************************* */
TEST(QPSolver, Nocedal06bookEx16_4) {
  QP qp = createTestNocedal06bookEx16_4();
  VectorValues initial;
  initial.insert(X(1), (Vector(1) << 2.0).finished());
  initial.insert(X(2), Z_1x1);
  QPSolver solver(qp);
  VectorValues solution = solver.optimize(initial).first;
  VectorValues expected;
  expected.insert(X(1), (Vector(1) << 1.4).finished());
  expected.insert(X(2), (Vector(1) << 1.7).finished());
  CHECK(assert_equal(expected, solution, 1e-7));
}

/* ************************************************************************* */
TEST(QPSolver, FailedSubproblem) {
  QP qp;
  qp.cost.add(X(1), I_2x2, Z_2x1);
  qp.cost.push_back(HessianFactor(X(1), Z_2x2, Z_2x1, 100.0));
  qp.inequalities.add(X(1), (Matrix(1, 2) << -1.0, 0.0).finished(), -1.0, 0);

  VectorValues initial;
  initial.insert(X(1), (Vector(2) << 10.0, 100.0).finished());

  VectorValues expected;
  expected.insert(X(1), (Vector(2) << 1.0, 0.0).finished());

  QPSolver solver(qp);
  VectorValues solution = solver.optimize(initial).first;
  CHECK(assert_equal(expected, solution, 1e-7));
}

/* ************************************************************************* */
TEST(QPSolver, InfeasibleInitial) {
  QP qp;
  qp.cost.add(X(1), I_2x2, Vector::Zero(2));
  qp.cost.push_back(HessianFactor(X(1), Z_2x2, Vector::Zero(2), 100.0));
  qp.inequalities.add(X(1), (Matrix(1, 2) << -1.0, 0.0).finished(), -1.0, 0);

  VectorValues initial;
  initial.insert(X(1), (Vector(2) << -10.0, 100.0).finished());

  QPSolver solver(qp);
  CHECK_EXCEPTION(solver.optimize(initial), InfeasibleInitialValues);
}

/* ************************************************************************* */
TEST(QPSolver, WarmStart) {
  QP qp = createTestMatlabQPEx();

  VectorValues initial;
  initial.insert(X(1), Z_1x1);
  initial.insert(X(2), Z_1x1);

  QPSolver solver(qp);
  auto [x0, d0, s0] = solver.optimizeWithState(initial);
  auto [x1, d1, s1] = solver.optimizeWithState(s0.values, s0.duals, true);

  VectorValues expected;
  expected.insert(X(1), (Vector(1) << 2.0 / 3.0).finished());
  expected.insert(X(2), (Vector(1) << 4.0 / 3.0).finished());
  CHECK(assert_equal(expected, x0, 1e-7));
  CHECK(assert_equal(expected, x1, 1e-7));
  CHECK(s1.iterations <= s0.iterations);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

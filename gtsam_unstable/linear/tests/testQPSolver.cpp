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

#include <gtsam/config.h>
#if GTSAM_USE_BOOST_FEATURES
#include <gtsam_unstable/linear/QPSParser.h>
#endif

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
#if GTSAM_USE_BOOST_FEATURES
static pair<QP, QP> testParser(QPSParser parser) {
  QP exampleqp = parser.Parse();
  QP expected;
  Key X1(Symbol('X', 1)), X2(Symbol('X', 2));
  // min f(x,y) = 4 + 1.5x -y + 0.58x^2 + 2xy + 2yx + 10y^2
  expected.cost.push_back(HessianFactor(X1, X2, 8.0 * I_1x1, 2.0 * I_1x1,
                                        -1.5 * kOne, 10.0 * I_1x1, 2.0 * kOne,
                                        8.0));
  expected.inequalities.add(X1, -2.0 * I_1x1, X2, -I_1x1, -2, 0);  // 2x + y >= 2
  expected.inequalities.add(X1, -I_1x1, X2, 2.0 * I_1x1, 6, 1);    // -x + 2y <= 6
  expected.inequalities.add(X1, I_1x1, 20, 4);                       // x <= 20
  expected.inequalities.add(X1, -I_1x1, 0, 2);                       // x >= 0
  expected.inequalities.add(X2, -I_1x1, 0, 3);                       // y >= 0
  return {expected, exampleqp};
}

TEST(QPSolver, ParserSyntaticTest) {
  auto result = testParser(QPSParser("QPExample.QPS"));
  CHECK(assert_equal(result.first.cost, result.second.cost, 1e-7));
  CHECK(assert_equal(result.first.inequalities, result.second.inequalities, 1e-7));
  CHECK(assert_equal(result.first.equalities, result.second.equalities, 1e-7));
}

TEST(QPSolver, ParserSemanticTest) {
  auto result = testParser(QPSParser("QPExample.QPS"));
  VectorValues expected = QPSolver(result.first).optimize().first;
  VectorValues actual = QPSolver(result.second).optimize().first;
  CHECK(assert_equal(actual, expected, 1e-7));
}

TEST(QPSolver, QPExampleTest) {
  QP problem = QPSParser("QPExample.QPS").Parse();
  auto solver = QPSolver(problem);
  VectorValues actual = solver.optimize().first;
  VectorValues expected;
  expected.insert(Symbol('X', 1), 0.7625 * I_1x1);
  expected.insert(Symbol('X', 2), 0.4750 * I_1x1);
  double error_expected = problem.cost.error(expected);
  double error_actual = problem.cost.error(actual);
  CHECK(assert_equal(expected, actual, 1e-7))
  CHECK(assert_equal(error_expected, error_actual))
}

TEST(QPSolver, HS21) {
  QP problem = QPSParser("HS21.QPS").Parse();
  VectorValues expected;
  expected.insert(Symbol('X', 1), 2.0 * I_1x1);
  expected.insert(Symbol('X', 2), 0.0 * I_1x1);
  VectorValues actual = QPSolver(problem).optimize().first;
  double error_actual = problem.cost.error(actual);
  CHECK(assert_equal(-99.9599999, error_actual, 1e-7))
  CHECK(assert_equal(expected, actual))
}

TEST(QPSolver, HS35) {
  QP problem = QPSParser("HS35.QPS").Parse();
  VectorValues actual = QPSolver(problem).optimize().first;
  double error_actual = problem.cost.error(actual);
  CHECK(assert_equal(1.11111111e-01, error_actual, 1e-7))
}

TEST(QPSolver, HS35MOD) {
  QP problem = QPSParser("HS35MOD.QPS").Parse();
  VectorValues actual = QPSolver(problem).optimize().first;
  double error_actual = problem.cost.error(actual);
  CHECK(assert_equal(2.50000001e-01, error_actual, 1e-7))
}

TEST(QPSolver, HS51) {
  QP problem = QPSParser("HS51.QPS").Parse();
  VectorValues actual = QPSolver(problem).optimize().first;
  double error_actual = problem.cost.error(actual);
  CHECK(assert_equal(8.88178420e-16, error_actual, 1e-7))
}

TEST(QPSolver, HS52) {
  QP problem = QPSParser("HS52.QPS").Parse();
  VectorValues actual = QPSolver(problem).optimize().first;
  double error_actual = problem.cost.error(actual);
  CHECK(assert_equal(5.32664756, error_actual, 1e-7))
}

TEST(QPSolver, HS268) {  // This test needs an extra order of magnitude of
                         // tolerance than the rest
  QP problem = QPSParser("HS268.QPS").Parse();
  VectorValues actual = QPSolver(problem).optimize().first;
  double error_actual = problem.cost.error(actual);
  CHECK(assert_equal(5.73107049e-07, error_actual, 1e-6))
}

TEST(QPSolver, QPTEST) {  // REQUIRES Jacobian Fix
  QP problem = QPSParser("QPTEST.QPS").Parse();
  VectorValues actual = QPSolver(problem).optimize().first;
  double error_actual = problem.cost.error(actual);
  CHECK(assert_equal(0.437187500e01, error_actual, 1e-7))
}
#endif

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

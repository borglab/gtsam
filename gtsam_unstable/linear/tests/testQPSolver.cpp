/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testQPSolver.cpp
 * @brief Test simple QP solver for a linear inequality constraint
 * @date Apr 10, 2014
 * @author Duy-Nguyen Ta
 * @author Ivan Dario Jimenez
 */

#include <gtsam_unstable/linear/QPSParser.h>
#include <gtsam_unstable/linear/QPSolver.h>

#include <gtsam/base/Testable.h>
#include <gtsam/inference/Symbol.h>

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;
using namespace gtsam::symbol_shorthand;

static const Vector kOne = Vector::Ones(1), kZero = Vector::Zero(1);

/* ************************************************************************* */
// Create test graph according to Forst10book_pg171Ex5
QP createTestCase() {
  QP qp;

  // Objective functions x1^2 - x1*x2 + x2^2 - 3*x1 + 5
  // Note the Hessian encodes:
  //        0.5*x1'*G11*x1 + x1'*G12*x2 + 0.5*x2'*G22*x2 - x1'*g1 - x2'*g2 + 0.5*f
  // Hence, we have G11=2, G12 = -1, g1 = +3, G22 = 2, g2 = 0, f = 10
  //TODO:  THIS TEST MIGHT BE WRONG : the last parameter  might be 5 instead of 10 because the form of the equation
  // Should be 0.5x'Gx + gx + f : Nocedal 449
  qp.cost.push_back(HessianFactor(X(1), X(2), 2.0 * I_1x1, -I_1x1, 3.0 * I_1x1,
                                  2.0 * I_1x1, Z_1x1, 10.0));

  // Inequality constraints
  qp.inequalities.add(X(1), I_1x1, X(2), I_1x1, 2,
                      0); // x1 + x2 <= 2 --> x1 + x2 -2 <= 0, --> b=2
  qp.inequalities.add(X(1), -I_1x1, 0, 1);  // -x1     <= 0
  qp.inequalities.add(X(2), -I_1x1, 0, 2);  //    -x2  <= 0
  qp.inequalities.add(X(1), I_1x1, 1.5, 3); // x1      <= 3/2

  return qp;
}

TEST(QPSolver, TestCase) {
  VectorValues values;
  double x1 = 5, x2 = 7;
  values.insert(X(1), x1 * I_1x1);
  values.insert(X(2), x2 * I_1x1);
  QP qp = createTestCase();
  DOUBLES_EQUAL(29, x1 * x1 - x1 * x2 + x2 * x2 - 3 * x1 + 5, 1e-9);
  DOUBLES_EQUAL(29, qp.cost[0]->error(values), 1e-9);
}

TEST(QPSolver, constraintsAux) {
  QP qp = createTestCase();

  QPSolver solver(qp);

  VectorValues lambdas;
  lambdas.insert(0, (Vector(1) << -0.5).finished());
  lambdas.insert(1, kZero);
  lambdas.insert(2, (Vector(1) << 0.3).finished());
  lambdas.insert(3, (Vector(1) << 0.1).finished());
  int factorIx = solver.identifyLeavingConstraint(qp.inequalities, lambdas);
  LONGS_EQUAL(2, factorIx);

  VectorValues lambdas2;
  lambdas2.insert(0, (Vector(1) << -0.5).finished());
  lambdas2.insert(1, kZero);
  lambdas2.insert(2, (Vector(1) << -0.3).finished());
  lambdas2.insert(3, (Vector(1) << -0.1).finished());
  int factorIx2 = solver.identifyLeavingConstraint(qp.inequalities, lambdas2);
  LONGS_EQUAL(-1, factorIx2);
}

/* ************************************************************************* */
// Create a simple test graph with one equality constraint
QP createEqualityConstrainedTest() {
  QP qp;

  // Objective functions x1^2 + x2^2
  // Note the Hessian encodes:
  //        0.5*x1'*G11*x1 + x1'*G12*x2 + 0.5*x2'*G22*x2 - x1'*g1 - x2'*g2 + 0.5*f
  // Hence, we have G11=2, G12 = 0, g1 = 0, G22 = 2, g2 = 0, f = 0
  qp.cost.push_back(HessianFactor(X(1), X(2), 2.0 * I_1x1, Z_1x1, Z_1x1,
                                  2.0 * I_1x1, Z_1x1, 0.0));

  // Equality constraints
  // x1 + x2 = 1 --> x1 + x2 -1 = 0, hence we negate the b vector
  Matrix A1 = I_1x1;
  Matrix A2 = I_1x1;
  Vector b = -kOne;
  qp.equalities.add(X(1), A1, X(2), A2, b, 0);

  return qp;
}

TEST(QPSolver, dual) {
  QP qp = createEqualityConstrainedTest();

  // Initials values
  VectorValues initialValues;
  initialValues.insert(X(1), I_1x1);
  initialValues.insert(X(2), I_1x1);

  QPSolver solver(qp);

  auto dualGraph = solver.buildDualGraph(qp.inequalities, initialValues);
  VectorValues dual = dualGraph.optimize();
  VectorValues expectedDual;
  expectedDual.insert(0, (Vector(1) << 2.0).finished());
  CHECK(assert_equal(expectedDual, dual, 1e-10));
}

/* ************************************************************************* */
TEST(QPSolver, indentifyActiveConstraints) {
  QP qp = createTestCase();
  QPSolver solver(qp);

  VectorValues currentSolution;
  currentSolution.insert(X(1), Z_1x1);
  currentSolution.insert(X(2), Z_1x1);

  auto workingSet =
      solver.identifyActiveConstraints(qp.inequalities, currentSolution);

  CHECK(!workingSet.at(0)->active()); // inactive
  CHECK(workingSet.at(1)->active());  // active
  CHECK(workingSet.at(2)->active());  // active
  CHECK(!workingSet.at(3)->active()); // inactive

  VectorValues solution = solver.buildWorkingGraph(workingSet).optimize();
  VectorValues expected;
  expected.insert(X(1), kZero);
  expected.insert(X(2), kZero);
  CHECK(assert_equal(expected, solution, 1e-100));
}

/* ************************************************************************* */
TEST(QPSolver, iterate) {
  QP qp = createTestCase();
  QPSolver solver(qp);

  VectorValues currentSolution;
  currentSolution.insert(X(1), Z_1x1);
  currentSolution.insert(X(2), Z_1x1);

  std::vector<VectorValues> expected(4), expectedDuals(4);
  expected[0].insert(X(1), kZero);
  expected[0].insert(X(2), kZero);
  expectedDuals[0].insert(1, (Vector(1) << 3).finished());
  expectedDuals[0].insert(2, kZero);

  expected[1].insert(X(1), (Vector(1) << 1.5).finished());
  expected[1].insert(X(2), kZero);
  expectedDuals[1].insert(3, (Vector(1) << 1.5).finished());

  expected[2].insert(X(1), (Vector(1) << 1.5).finished());
  expected[2].insert(X(2), (Vector(1) << 0.75).finished());

  expected[3].insert(X(1), (Vector(1) << 1.5).finished());
  expected[3].insert(X(2), (Vector(1) << 0.5).finished());

  auto workingSet =
      solver.identifyActiveConstraints(qp.inequalities, currentSolution);

  QPSolver::State state(currentSolution, VectorValues(), workingSet, false,
                        100);

  int it = 0;
  while (!state.converged) {
    state = solver.iterate(state);
    // These checks will fail because the expected solutions obtained from
    // Forst10book do not follow exactly what we implemented from Nocedal06book.
    // Specifically, we do not re-identify active constraints and
    // do not recompute dual variables after every step!!!
    //    CHECK(assert_equal(expected[it], state.values, 1e-10));
    //    CHECK(assert_equal(expectedDuals[it], state.duals, 1e-10));
    it++;
  }

  CHECK(assert_equal(expected[3], state.values, 1e-10));
}

/* ************************************************************************* */

TEST(QPSolver, optimizeForst10book_pg171Ex5) {
  QP qp = createTestCase();
  QPSolver solver(qp);
  VectorValues initialValues;
  initialValues.insert(X(1), Z_1x1);
  initialValues.insert(X(2), Z_1x1);
  VectorValues solution = solver.optimize(initialValues).first;
  VectorValues expected;
  expected.insert(X(1), (Vector(1) << 1.5).finished());
  expected.insert(X(2), (Vector(1) << 0.5).finished());
  CHECK(assert_equal(expected, solution, 1e-100));
}

pair<QP, QP> testParser(QPSParser parser) {
  QP exampleqp = parser.Parse();
  QP expected;
  Key X1(Symbol('X', 1)), X2(Symbol('X', 2));
  // min f(x,y) = 4 + 1.5x -y + 0.58x^2 + 2xy + 2yx + 10y^2
  expected.cost.push_back(HessianFactor(X1, X2, 8.0 * I_1x1, 2.0 * I_1x1,
                                        -1.5 * kOne, 10.0 * I_1x1, 2.0 * kOne,
                                        8.0));

  expected.inequalities.add(X1, -2.0 * I_1x1, X2, -I_1x1, -2, 0); // 2x + y >= 2
  expected.inequalities.add(X1, -I_1x1, X2, 2.0 * I_1x1, 6, 1); // -x + 2y <= 6
  expected.inequalities.add(X1, I_1x1, 20, 4);                  // x<= 20
  expected.inequalities.add(X1, -I_1x1, 0, 2);                  // x >= 0
  expected.inequalities.add(X2, -I_1x1, 0, 3);                  // y > = 0
  return {expected, exampleqp};
}

TEST(QPSolver, ParserSyntaticTest) {
  auto result = testParser(QPSParser("QPExample.QPS"));
  CHECK(assert_equal(result.first.cost, result.second.cost, 1e-7));
  CHECK(assert_equal(result.first.inequalities, result.second.inequalities,
                     1e-7));
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

TEST(QPSolver, HS268) { // This test needs an extra order of magnitude of
                        // tolerance than the rest
  QP problem = QPSParser("HS268.QPS").Parse();
  VectorValues actual = QPSolver(problem).optimize().first;
  double error_actual = problem.cost.error(actual);
  CHECK(assert_equal(5.73107049e-07, error_actual, 1e-6))
}

TEST(QPSolver, QPTEST) { // REQUIRES Jacobian Fix
  QP problem = QPSParser("QPTEST.QPS").Parse();
  VectorValues actual = QPSolver(problem).optimize().first;
  double error_actual = problem.cost.error(actual);
  CHECK(assert_equal(0.437187500e01, error_actual, 1e-7))
}

/* ************************************************************************* */
// Create Matlab's test graph as in
// http://www.mathworks.com/help/optim/ug/quadprog.html
QP createTestMatlabQPEx() {
  QP qp;

  // Objective functions 0.5*x1^2 + x2^2 - x1*x2 - 2*x1 -6*x2
  // Note the Hessian encodes:
  //        0.5*x1'*G11*x1 + x1'*G12*x2 + 0.5*x2'*G22*x2 - x1'*g1 - x2'*g2 +
  //        0.5*f
  // Hence, we have G11=1, G12 = -1, g1 = +2, G22 = 2, g2 = +6, f = 0
  qp.cost.push_back(HessianFactor(X(1), X(2), 1.0 * I_1x1, -I_1x1, 2.0 * I_1x1,
                                  2.0 * I_1x1, 6 * I_1x1, 1000.0));

  // Inequality constraints
  qp.inequalities.add(X(1), I_1x1, X(2), I_1x1, 2, 0);      // x1 + x2 <= 2
  qp.inequalities.add(X(1), -I_1x1, X(2), 2 * I_1x1, 2, 1); //-x1 + 2*x2 <=2
  qp.inequalities.add(X(1), 2 * I_1x1, X(2), I_1x1, 3, 2);  // 2*x1 + x2 <=3
  qp.inequalities.add(X(1), -I_1x1, 0, 3);                  // -x1      <= 0
  qp.inequalities.add(X(2), -I_1x1, 0, 4);                  //      -x2 <= 0

  return qp;
}

///* *************************************************************************
///*/
TEST(QPSolver, optimizeMatlabEx) {
  QP qp = createTestMatlabQPEx();
  QPSolver solver(qp);
  VectorValues initialValues;
  initialValues.insert(X(1), Z_1x1);
  initialValues.insert(X(2), Z_1x1);
  VectorValues solution = solver.optimize(initialValues).first;
  VectorValues expected;
  expected.insert(X(1), (Vector(1) << 2.0 / 3.0).finished());
  expected.insert(X(2), (Vector(1) << 4.0 / 3.0).finished());
  CHECK(assert_equal(expected, solution, 1e-7));
}

///* *************************************************************************
///*/
TEST(QPSolver, optimizeMatlabExNoinitials) {
  QP qp = createTestMatlabQPEx();
  QPSolver solver(qp);
  VectorValues solution = solver.optimize().first;
  VectorValues expected;
  expected.insert(X(1), (Vector(1) << 2.0 / 3.0).finished());
  expected.insert(X(2), (Vector(1) << 4.0 / 3.0).finished());
  CHECK(assert_equal(expected, solution, 1e-7));
}

/* ************************************************************************* */
// Create test graph as in Nocedal06book, Ex 16.4, pg. 475
QP createTestNocedal06bookEx16_4() {
  QP qp;

  qp.cost.add(X(1), I_1x1, I_1x1);
  qp.cost.add(X(2), I_1x1, 2.5 * I_1x1);

  // Inequality constraints
  qp.inequalities.add(X(1), -I_1x1, X(2), 2 * I_1x1, 2, 0);
  qp.inequalities.add(X(1), I_1x1, X(2), 2 * I_1x1, 6, 1);
  qp.inequalities.add(X(1), I_1x1, X(2), -2 * I_1x1, 2, 2);
  qp.inequalities.add(X(1), -I_1x1, 0.0, 3);
  qp.inequalities.add(X(2), -I_1x1, 0.0, 4);

  return qp;
}

TEST(QPSolver, optimizeNocedal06bookEx16_4) {
  QP qp = createTestNocedal06bookEx16_4();
  QPSolver solver(qp);
  VectorValues initialValues;
  initialValues.insert(X(1), (Vector(1) << 2.0).finished());
  initialValues.insert(X(2), Z_1x1);

  VectorValues solution = solver.optimize(initialValues).first;
  VectorValues expected;
  expected.insert(X(1), (Vector(1) << 1.4).finished());
  expected.insert(X(2), (Vector(1) << 1.7).finished());
  CHECK(assert_equal(expected, solution, 1e-7));
}

/* ************************************************************************* */
TEST(QPSolver, failedSubproblem) {
  QP qp;
  qp.cost.add(X(1), I_2x2, Z_2x1);
  qp.cost.push_back(HessianFactor(X(1), Z_2x2, Z_2x1, 100.0));
  qp.inequalities.add(X(1), (Matrix(1, 2) << -1.0, 0.0).finished(), -1.0, 0);

  VectorValues expected;
  expected.insert(X(1), (Vector(2) << 1.0, 0.0).finished());

  VectorValues initialValues;
  initialValues.insert(X(1), (Vector(2) << 10.0, 100.0).finished());

  QPSolver solver(qp);
  VectorValues solution = solver.optimize(initialValues).first;

  CHECK(assert_equal(expected, solution, 1e-7));
}

/* ************************************************************************* */
TEST(QPSolver, infeasibleInitial) {
  QP qp;
  qp.cost.add(X(1), I_2x2, Vector::Zero(2));
  qp.cost.push_back(HessianFactor(X(1), Z_2x2, Vector::Zero(2), 100.0));
  qp.inequalities.add(X(1), (Matrix(1, 2) << -1.0, 0.0).finished(), -1.0, 0);

  VectorValues expected;
  expected.insert(X(1), (Vector(2) << 1.0, 0.0).finished());

  VectorValues initialValues;
  initialValues.insert(X(1), (Vector(2) << -10.0, 100.0).finished());

  QPSolver solver(qp);
  VectorValues solution;
  CHECK_EXCEPTION(solver.optimize(initialValues), InfeasibleInitialValues);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

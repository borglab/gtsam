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
 */

#include <gtsam/base/Testable.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam_unstable/linear/QPSolver.h>
#include <gtsam_unstable/linear/QPSParser.h>
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
  qp.cost.push_back(
      HessianFactor(X(1), X(2), 2.0 * I_1x1, -I_1x1, 3.0 * I_1x1, 2.0 * I_1x1,
          Z_1x1, 10.0));

  // Inequality constraints
  qp.inequalities.push_back(LinearInequality(X(1), I_1x1, X(2), I_1x1, 2, 0)); // x1 + x2 <= 2 --> x1 + x2 -2 <= 0, --> b=2
  qp.inequalities.push_back(LinearInequality(X(1), -I_1x1, 0, 1)); // -x1     <= 0
  qp.inequalities.push_back(LinearInequality(X(2), -I_1x1, 0, 2)); //    -x2  <= 0
  qp.inequalities.push_back(LinearInequality(X(1), I_1x1, 1.5, 3)); // x1      <= 3/2

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
  qp.cost.push_back(
      HessianFactor(X(1), X(2), 2.0 * I_1x1, Z_1x1, Z_1x1, 2.0 * I_1x1, Z_1x1,
          0.0));

  // Equality constraints
  // x1 + x2 = 1 --> x1 + x2 -1 = 0, hence we negate the b vector
  Matrix A1 = I_1x1;
  Matrix A2 = I_1x1;
  Vector b = -kOne;
  qp.equalities.push_back(LinearEquality(X(1), A1, X(2), A2, b, 0));

  return qp;
}

TEST(QPSolver, dual) {
  QP qp = createEqualityConstrainedTest();

  // Initials values
  VectorValues initialValues;
  initialValues.insert(X(1), I_1x1);
  initialValues.insert(X(2), I_1x1);

  QPSolver solver(qp);

  GaussianFactorGraph::shared_ptr dualGraph = solver.buildDualGraph(
      qp.inequalities, initialValues);
  VectorValues dual = dualGraph->optimize();
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

  InequalityFactorGraph workingSet = solver.identifyActiveConstraints(
      qp.inequalities, currentSolution);

  CHECK(!workingSet.at(0)->active()); // inactive
  CHECK(workingSet.at(1)->active());// active
  CHECK(workingSet.at(2)->active());// active
  CHECK(!workingSet.at(3)->active());// inactive

  VectorValues solution = solver.buildWorkingGraph(workingSet).optimize();
  VectorValues expectedSolution;
  expectedSolution.insert(X(1), kZero);
  expectedSolution.insert(X(2), kZero);
  CHECK(assert_equal(expectedSolution, solution, 1e-100));
}

/* ************************************************************************* */
TEST(QPSolver, iterate) {
  QP qp = createTestCase();
  QPSolver solver(qp);

  VectorValues currentSolution;
  currentSolution.insert(X(1), Z_1x1);
  currentSolution.insert(X(2), Z_1x1);

  std::vector<VectorValues> expectedSolutions(4), expectedDuals(4);
  expectedSolutions[0].insert(X(1), kZero);
  expectedSolutions[0].insert(X(2), kZero);
  expectedDuals[0].insert(1, (Vector(1) << 3).finished());
  expectedDuals[0].insert(2, kZero);

  expectedSolutions[1].insert(X(1), (Vector(1) << 1.5).finished());
  expectedSolutions[1].insert(X(2), kZero);
  expectedDuals[1].insert(3, (Vector(1) << 1.5).finished());

  expectedSolutions[2].insert(X(1), (Vector(1) << 1.5).finished());
  expectedSolutions[2].insert(X(2), (Vector(1) << 0.75).finished());

  expectedSolutions[3].insert(X(1), (Vector(1) << 1.5).finished());
  expectedSolutions[3].insert(X(2), (Vector(1) << 0.5).finished());

  InequalityFactorGraph workingSet = solver.identifyActiveConstraints(
      qp.inequalities, currentSolution);

  QPSolver::State state(currentSolution, VectorValues(), workingSet, false,
                        100);

  int it = 0;
  while (!state.converged) {
    state = solver.iterate(state);
    // These checks will fail because the expected solutions obtained from
    // Forst10book do not follow exactly what we implemented from Nocedal06book.
    // Specifically, we do not re-identify active constraints and
    // do not recompute dual variables after every step!!!
//    CHECK(assert_equal(expectedSolutions[it], state.values, 1e-10));
//    CHECK(assert_equal(expectedDuals[it], state.duals, 1e-10));
    it++;
  }

  CHECK(assert_equal(expectedSolutions[3], state.values, 1e-10));
}

/* ************************************************************************* */

TEST(QPSolver, optimizeForst10book_pg171Ex5) {
  QP qp = createTestCase();
  QPSolver solver(qp);
  VectorValues initialValues;
  initialValues.insert(X(1), Z_1x1);
  initialValues.insert(X(2), Z_1x1);
  VectorValues solution;
  boost::tie(solution, boost::tuples::ignore) = solver.optimize(initialValues);
  VectorValues expectedSolution;
  expectedSolution.insert(X(1), (Vector(1) << 1.5).finished());
  expectedSolution.insert(X(2), (Vector(1) << 0.5).finished());
  CHECK(assert_equal(expectedSolution, solution, 1e-100));
}

pair<QP, QP> testParser(QPSParser parser) {
  QP exampleqp = parser.Parse();
  QP expectedqp;
  Key X1(Symbol('X', 1)), X2(Symbol('X', 2));
  // min f(x,y) = 4 + 1.5x -y + 0.58x^2 + 2xy + 2yx + 10y^2
  expectedqp.cost.push_back(
      HessianFactor(X1, X2, 8.0 * I_1x1, 2.0 * I_1x1, -1.5 * kOne, 10.0 * I_1x1,
          2.0 * kOne, 4.0));
  // 2x + y >= 2
  // -x + 2y <= 6
  expectedqp.inequalities.push_back(
      LinearInequality(X1, -2.0 * I_1x1, X2, -I_1x1, -2, 0));
  expectedqp.inequalities.push_back(
      LinearInequality(X1, -I_1x1, X2, 2.0 * I_1x1, 6, 1));
  // x<= 20
  expectedqp.inequalities.push_back(LinearInequality(X1, I_1x1, 20, 4));
  //x >= 0
  expectedqp.inequalities.push_back(LinearInequality(X1, -I_1x1, 0, 2));
  // y > = 0
  expectedqp.inequalities.push_back(LinearInequality(X2, -I_1x1, 0, 3));
  return std::make_pair(expectedqp, exampleqp);
}
;

TEST(QPSolver, ParserSyntaticTest) {
  auto expectedActual = testParser(QPSParser("QPExample.QPS"));
  CHECK(assert_equal(expectedActual.first.cost, expectedActual.second.cost,
                     1e-7));
  CHECK(assert_equal(expectedActual.first.inequalities,
                     expectedActual.second.inequalities, 1e-7));
  CHECK(assert_equal(expectedActual.first.equalities,
                     expectedActual.second.equalities, 1e-7));
}

TEST(QPSolver, ParserSemanticTest) {
  auto expected_actual = testParser(QPSParser("QPExample.QPS"));
  VectorValues actualSolution, expectedSolution;
  boost::tie(expectedSolution, boost::tuples::ignore) =
      QPSolver(expected_actual.first).optimize();
  boost::tie(actualSolution, boost::tuples::ignore) =
      QPSolver(expected_actual.second).optimize();
  CHECK(assert_equal(actualSolution, expectedSolution, 1e-7));
}

TEST(QPSolver, QPExampleTest){
  QP problem = QPSParser("QPExample.QPS").Parse();
  VectorValues actualSolution;
  auto solver = QPSolver(problem);
  boost::tie(actualSolution, boost::tuples::ignore) = solver.optimize();
  VectorValues expectedSolution;
  expectedSolution.insert(Symbol('X',1),0.7625*I_1x1);
  expectedSolution.insert(Symbol('X',2),0.4750*I_1x1);
  double error_expected = problem.cost.error(expectedSolution);
  double error_actual = problem.cost.error(actualSolution);
  CHECK(assert_equal(expectedSolution, actualSolution, 1e-7))
  CHECK(assert_equal(error_expected, error_actual))
}

/* ************************************************************************* */
// Create Matlab's test graph as in http://www.mathworks.com/help/optim/ug/quadprog.html
QP createTestMatlabQPEx() {
  QP qp;

  // Objective functions 0.5*x1^2 + x2^2 - x1*x2 - 2*x1 -6*x2
  // Note the Hessian encodes:
  //        0.5*x1'*G11*x1 + x1'*G12*x2 + 0.5*x2'*G22*x2 - x1'*g1 - x2'*g2 + 0.5*f
  // Hence, we have G11=1, G12 = -1, g1 = +2, G22 = 2, g2 = +6, f = 0
  qp.cost.push_back(
      HessianFactor(X(1), X(2), 1.0 * I_1x1, -I_1x1, 2.0 * I_1x1, 2.0 * I_1x1,
          6 * I_1x1, 1000.0));

  // Inequality constraints
  qp.inequalities.push_back(LinearInequality(X(1), I_1x1, X(2), I_1x1, 2, 0)); // x1 + x2 <= 2
  qp.inequalities.push_back(
      LinearInequality(X(1), -I_1x1, X(2), 2 * I_1x1, 2, 1)); //-x1 + 2*x2 <=2
  qp.inequalities.push_back(
      LinearInequality(X(1), 2 * I_1x1, X(2), I_1x1, 3, 2)); // 2*x1 + x2 <=3
  qp.inequalities.push_back(LinearInequality(X(1), -I_1x1, 0, 3)); // -x1      <= 0
  qp.inequalities.push_back(LinearInequality(X(2), -I_1x1, 0, 4)); //      -x2 <= 0

  return qp;
}

///* ************************************************************************* */
TEST(QPSolver, optimizeMatlabEx) {
  QP qp = createTestMatlabQPEx();
  QPSolver solver(qp);
  VectorValues initialValues;
  initialValues.insert(X(1), Z_1x1);
  initialValues.insert(X(2), Z_1x1);
  VectorValues solution;
  boost::tie(solution, boost::tuples::ignore) = solver.optimize(initialValues);
  VectorValues expectedSolution;
  expectedSolution.insert(X(1), (Vector(1) << 2.0 / 3.0).finished());
  expectedSolution.insert(X(2), (Vector(1) << 4.0 / 3.0).finished());
  CHECK(assert_equal(expectedSolution, solution, 1e-7));
}

///* ************************************************************************* */
TEST(QPSolver, optimizeMatlabExNoinitials) {
  QP qp = createTestMatlabQPEx();
  QPSolver solver(qp);
  VectorValues solution;
  boost::tie(solution, boost::tuples::ignore) = solver.optimize();
  VectorValues expectedSolution;
  expectedSolution.insert(X(1), (Vector(1) << 2.0 / 3.0).finished());
  expectedSolution.insert(X(2), (Vector(1) << 4.0 / 3.0).finished());
  CHECK(assert_equal(expectedSolution, solution, 1e-7));
}

/* ************************************************************************* */
// Create test graph as in Nocedal06book, Ex 16.4, pg. 475
QP createTestNocedal06bookEx16_4() {
  QP qp;

  qp.cost.push_back(JacobianFactor(X(1), I_1x1, I_1x1));
  qp.cost.push_back(JacobianFactor(X(2), I_1x1, 2.5 * I_1x1));

  // Inequality constraints
  qp.inequalities.push_back(
      LinearInequality(X(1), -I_1x1, X(2), 2 * I_1x1, 2, 0));
  qp.inequalities.push_back(
      LinearInequality(X(1), I_1x1, X(2), 2 * I_1x1, 6, 1));
  qp.inequalities.push_back(
      LinearInequality(X(1), I_1x1, X(2), -2 * I_1x1, 2, 2));
  qp.inequalities.push_back(LinearInequality(X(1), -I_1x1, 0.0, 3));
  qp.inequalities.push_back(LinearInequality(X(2), -I_1x1, 0.0, 4));

  return qp;
}

TEST(QPSolver, optimizeNocedal06bookEx16_4) {
  QP qp = createTestNocedal06bookEx16_4();
  QPSolver solver(qp);
  VectorValues initialValues;
  initialValues.insert(X(1), (Vector(1) << 2.0).finished());
  initialValues.insert(X(2), Z_1x1);

  VectorValues solution;
  boost::tie(solution, boost::tuples::ignore) = solver.optimize(initialValues);
  VectorValues expectedSolution;
  expectedSolution.insert(X(1), (Vector(1) << 1.4).finished());
  expectedSolution.insert(X(2), (Vector(1) << 1.7).finished());
  CHECK(assert_equal(expectedSolution, solution, 1e-7));
}

/* ************************************************************************* */
TEST(QPSolver, failedSubproblem) {
  QP qp;
  qp.cost.push_back(JacobianFactor(X(1), I_2x2, Z_2x1));
  qp.cost.push_back(HessianFactor(X(1), Z_2x2, Z_2x1, 100.0));
  qp.inequalities.push_back(
      LinearInequality(X(1), (Matrix(1, 2) << -1.0, 0.0).finished(), -1.0, 0));

  VectorValues expected;
  expected.insert(X(1), (Vector(2) << 1.0, 0.0).finished());

  VectorValues initialValues;
  initialValues.insert(X(1), (Vector(2) << 10.0, 100.0).finished());

  QPSolver solver(qp);
  VectorValues solution;
  boost::tie(solution, boost::tuples::ignore) = solver.optimize(initialValues);

  CHECK(assert_equal(expected, solution, 1e-7));
}

/* ************************************************************************* */
TEST(QPSolver, infeasibleInitial) {
  QP qp;
  qp.cost.push_back(JacobianFactor(X(1), I_2x2, Vector::Zero(2)));
  qp.cost.push_back(HessianFactor(X(1), Z_2x2, Vector::Zero(2), 100.0));
  qp.inequalities.push_back(
      LinearInequality(X(1), (Matrix(1, 2) << -1.0, 0.0).finished(), -1.0, 0));

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


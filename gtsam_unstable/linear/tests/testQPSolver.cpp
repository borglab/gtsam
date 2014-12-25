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

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;
using namespace gtsam::symbol_shorthand;

const Matrix One = ones(1,1);

/* ************************************************************************* */
// Create test graph according to Forst10book_pg171Ex5
QP createTestCase() {
  QP qp;

  // Objective functions x1^2 - x1*x2 + x2^2 - 3*x1 + 5
  // Note the Hessian encodes:
  //        0.5*x1'*G11*x1 + x1'*G12*x2 + 0.5*x2'*G22*x2 - x1'*g1 - x2'*g2 + 0.5*f
  // Hence, we have G11=2, G12 = -1, g1 = +3, G22 = 2, g2 = 0, f = 10
  qp.cost.push_back(
      HessianFactor(X(1), X(2), 2.0 * ones(1, 1), -ones(1, 1), 3.0 * ones(1),
          2.0 * ones(1, 1), zero(1), 10.0));

  // Inequality constraints
  qp.inequalities.push_back(LinearInequality(X(1), ones(1,1), X(2), ones(1,1), 2, 0)); // x1 + x2 <= 2 --> x1 + x2 -2 <= 0, --> b=2
  qp.inequalities.push_back(LinearInequality(X(1), -ones(1,1), 0, 1));                 // -x1     <= 0
  qp.inequalities.push_back(LinearInequality(X(2), -ones(1,1), 0, 2));                 //    -x2  <= 0
  qp.inequalities.push_back(LinearInequality(X(1), ones(1,1), 1.5, 3));                // x1      <= 3/2

  return qp;
}

TEST(QPSolver, TestCase) {
  VectorValues values;
  double x1 = 5, x2 = 7;
  values.insert(X(1), x1 * ones(1, 1));
  values.insert(X(2), x2 * ones(1, 1));
  QP qp = createTestCase();
  DOUBLES_EQUAL(29, x1 * x1 - x1 * x2 + x2 * x2 - 3 * x1 + 5, 1e-9);
  DOUBLES_EQUAL(29, qp.cost[0]->error(values), 1e-9);
}

TEST(QPSolver, constraintsAux) {
  QP qp = createTestCase();

  QPSolver solver(qp);

  VectorValues lambdas;
  lambdas.insert(0, (Vector(1) << -0.5).finished());
  lambdas.insert(1, (Vector(1) <<  0.0).finished());
  lambdas.insert(2, (Vector(1) <<  0.3).finished());
  lambdas.insert(3, (Vector(1) <<  0.1).finished());
  int factorIx = solver.identifyLeavingConstraint(qp.inequalities, lambdas);
  LONGS_EQUAL(2, factorIx);

  VectorValues lambdas2;
  lambdas2.insert(0, (Vector(1) << -0.5).finished());
  lambdas2.insert(1, (Vector(1) <<  0.0).finished());
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
      HessianFactor(X(1), X(2), 2.0 * ones(1, 1), zeros(1, 1), zero(1),
          2.0 * ones(1, 1), zero(1), 0.0));

  // Equality constraints
  // x1 + x2 = 1 --> x1 + x2 -1 = 0, hence we negate the b vector
  Matrix A1 = (Matrix(1, 1) << 1).finished();
  Matrix A2 = (Matrix(1, 1) << 1).finished();
  Vector b = -(Vector(1) << 1).finished();
  qp.equalities.push_back(LinearEquality(X(1), A1, X(2), A2, b, 0));

  return qp;
}

TEST(QPSolver, dual) {
  QP qp = createEqualityConstrainedTest();

  // Initials values
  VectorValues initialValues;
  initialValues.insert(X(1), ones(1));
  initialValues.insert(X(2), ones(1));

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
  currentSolution.insert(X(1), zero(1));
  currentSolution.insert(X(2), zero(1));

  LinearInequalityFactorGraph workingSet =
      solver.identifyActiveConstraints(qp.inequalities, currentSolution);

  CHECK(!workingSet.at(0)->active());   // inactive
  CHECK(workingSet.at(1)->active());    // active
  CHECK(workingSet.at(2)->active());    // active
  CHECK(!workingSet.at(3)->active());   // inactive

  VectorValues solution  = solver.solveWithCurrentWorkingSet(workingSet);
  VectorValues expectedSolution;
  expectedSolution.insert(X(1), (Vector(1) << 0.0).finished());
  expectedSolution.insert(X(2), (Vector(1) << 0.0).finished());
  CHECK(assert_equal(expectedSolution, solution, 1e-100));

}

/* ************************************************************************* */
TEST(QPSolver, iterate) {
  QP qp = createTestCase();
  QPSolver solver(qp);

  VectorValues currentSolution;
  currentSolution.insert(X(1), zero(1));
  currentSolution.insert(X(2), zero(1));

  std::vector<VectorValues> expectedSolutions(4), expectedDuals(4);
  expectedSolutions[0].insert(X(1), (Vector(1) << 0.0).finished());
  expectedSolutions[0].insert(X(2), (Vector(1) << 0.0).finished());
  expectedDuals[0].insert(1, (Vector(1) << 3).finished());
  expectedDuals[0].insert(2, (Vector(1) << 0).finished());

  expectedSolutions[1].insert(X(1), (Vector(1) << 1.5).finished());
  expectedSolutions[1].insert(X(2), (Vector(1) << 0.0).finished());
  expectedDuals[1].insert(3, (Vector(1) << 1.5).finished());

  expectedSolutions[2].insert(X(1), (Vector(1) << 1.5).finished());
  expectedSolutions[2].insert(X(2), (Vector(1) << 0.75).finished());

  expectedSolutions[3].insert(X(1), (Vector(1) << 1.5).finished());
  expectedSolutions[3].insert(X(2), (Vector(1) << 0.5).finished());

  LinearInequalityFactorGraph workingSet =
      solver.identifyActiveConstraints(qp.inequalities, currentSolution);

  QPState state(currentSolution, VectorValues(), workingSet, false, 100);

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
  initialValues.insert(X(1), zero(1));
  initialValues.insert(X(2), zero(1));
  VectorValues solution;
  boost::tie(solution, boost::tuples::ignore) = solver.optimize(initialValues);
  VectorValues expectedSolution;
  expectedSolution.insert(X(1), (Vector(1) << 1.5).finished());
  expectedSolution.insert(X(2), (Vector(1) << 0.5).finished());
  CHECK(assert_equal(expectedSolution, solution, 1e-100));
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
      HessianFactor(X(1), X(2), 1.0 * ones(1, 1), -ones(1, 1), 2.0 * ones(1),
          2.0 * ones(1, 1), 6 * ones(1), 1000.0));

  // Inequality constraints
  qp.inequalities.push_back(LinearInequality(X(1), One, X(2), One, 2, 0));      // x1 + x2 <= 2
  qp.inequalities.push_back(LinearInequality(X(1), -One, X(2), 2*One, 2, 1));   //-x1 + 2*x2 <=2
  qp.inequalities.push_back(LinearInequality(X(1), 2*One, X(2), One, 3, 2));    // 2*x1 + x2 <=3
  qp.inequalities.push_back(LinearInequality(X(1), -One, 0, 3));                // -x1      <= 0
  qp.inequalities.push_back(LinearInequality(X(2), -One, 0, 4));                //      -x2 <= 0

  return qp;
}

TEST(QPSolver, optimizeMatlabEx) {
  QP qp = createTestMatlabQPEx();
  QPSolver solver(qp);
  VectorValues initialValues;
  initialValues.insert(X(1), zero(1));
  initialValues.insert(X(2), zero(1));
  VectorValues solution;
  boost::tie(solution, boost::tuples::ignore) = solver.optimize(initialValues);
  VectorValues expectedSolution;
  expectedSolution.insert(X(1), (Vector(1) << 2.0 / 3.0).finished());
  expectedSolution.insert(X(2), (Vector(1) << 4.0 / 3.0).finished());
  CHECK(assert_equal(expectedSolution, solution, 1e-7));
}

/* ************************************************************************* */
// Create test graph as in Nocedal06book, Ex 16.4, pg. 475
QP createTestNocedal06bookEx16_4() {
  QP qp;

  qp.cost.push_back(JacobianFactor(X(1), ones(1, 1), ones(1)));
  qp.cost.push_back(JacobianFactor(X(2), ones(1, 1), 2.5 * ones(1)));

  // Inequality constraints
  qp.inequalities.push_back(LinearInequality(X(1), -One, X(2), 2 * One, 2, 0));
  qp.inequalities.push_back(LinearInequality(X(1), One, X(2), 2 * One, 6, 1));
  qp.inequalities.push_back(LinearInequality(X(1), One, X(2), -2 * One, 2, 2));
  qp.inequalities.push_back(LinearInequality(X(1), -One, 0.0, 3));
  qp.inequalities.push_back(LinearInequality(X(2), -One, 0.0, 4));

  return qp;
}

TEST(QPSolver, optimizeNocedal06bookEx16_4) {
  QP qp = createTestNocedal06bookEx16_4();
  QPSolver solver(qp);
  VectorValues initialValues;
  initialValues.insert(X(1), (Vector(1) << 2.0).finished());
  initialValues.insert(X(2), zero(1));

  VectorValues solution;
  boost::tie(solution, boost::tuples::ignore) = solver.optimize(initialValues);
  VectorValues expectedSolution;
  expectedSolution.insert(X(1), (Vector(1) << 1.4).finished());
  expectedSolution.insert(X(2), (Vector(1) << 1.7).finished());
  CHECK(assert_equal(expectedSolution, solution, 1e-7));
}

/* ************************************************************************* */
/* Create test graph as in Nocedal06book, Ex 16.4, pg. 475
 with the first constraint (16.49b) is replaced by
 x1 - 2 x2 - 1 >=0
 so that the trivial initial point (0,0) is infeasible
 ====
 H = [2 0; 0 2];
 f = [-2; -5];
 A =[-1 2;
 1 2
 1 -2];
 b = [-1; 6; 2];
 lb = zeros(2,1);

 opts = optimoptions('quadprog','Algorithm','active-set','Display','off');

 [x,fval,exitflag,output,lambda] = ...
 quadprog(H,f,A,b,[],[],lb,[],[],opts);
 ====
 x =
 2.0000
 0.5000
 */
QP modifyNocedal06bookEx16_4() {
  QP qp;

  qp.cost.push_back(JacobianFactor(X(1), ones(1, 1), ones(1)));
  qp.cost.push_back(JacobianFactor(X(2), ones(1, 1), 2.5 * ones(1)));

  // Inequality constraints
  noiseModel::Constrained::shared_ptr noise =
      noiseModel::Constrained::MixedSigmas((Vector(1) << -1).finished());
  qp.inequalities.push_back(LinearInequality(X(1), -One, X(2), 2 * One, -1, 0));
  qp.inequalities.push_back(LinearInequality(X(1), One, X(2), 2 * One, 6, 1));
  qp.inequalities.push_back(LinearInequality(X(1), One, X(2), -2 * One, 2, 2));
  qp.inequalities.push_back(LinearInequality(X(1), -One, 0.0, 3));
  qp.inequalities.push_back(LinearInequality(X(2), -One, 0.0, 4));

  return qp;
}

TEST(QPSolver, optimizeNocedal06bookEx16_4_findInitialPoint) {
  QP qp = modifyNocedal06bookEx16_4();
  QPSolver solver(qp);
  VectorValues initialsLP;
  Key firstSlackKey, lastSlackKey;
  boost::tie(initialsLP, firstSlackKey, lastSlackKey) = solver.initialValuesLP();
  EXPECT(assert_equal(zero(1), initialsLP.at(X(1))));
  EXPECT(assert_equal(zero(1), initialsLP.at(X(2))));
  LONGS_EQUAL(X(2) + 1, firstSlackKey);
  EXPECT(assert_equal(zero(1), initialsLP.at(firstSlackKey)));
  EXPECT(assert_equal(ones(1) * 6.0, initialsLP.at(firstSlackKey + 1)));
  EXPECT(assert_equal(ones(1) * 2.0, initialsLP.at(firstSlackKey + 2)));
  EXPECT(assert_equal(zero(1), initialsLP.at(firstSlackKey + 3)));
  EXPECT(assert_equal(zero(1), initialsLP.at(firstSlackKey + 4)));

  VectorValues objCoeffs = solver.objectiveCoeffsLP(firstSlackKey);
  for (size_t i = 0; i < 5; ++i)
    EXPECT(assert_equal(ones(1), objCoeffs.at(firstSlackKey + i)));

  LinearEqualityFactorGraph::shared_ptr equalities;
  LinearInequalityFactorGraph::shared_ptr inequalities;
  VectorValues lowerBounds;
  boost::tie(equalities, inequalities, lowerBounds) = solver.constraintsLP(
      firstSlackKey);
  for (size_t i = 0; i < 5; ++i)
    EXPECT(assert_equal(zero(1), lowerBounds.at(firstSlackKey + i)));

  LinearInequalityFactorGraph expectedInequalities;
  expectedInequalities.push_back(
      LinearInequality(X(1), -One, X(2), 2 * One, X(3), -One, -1, 0));
  expectedInequalities.push_back(
      LinearInequality(X(1), One, X(2), 2 * One, X(4), -One, 6, 1));
  expectedInequalities.push_back(
      LinearInequality(X(1), One, X(2), -2 * One, X(5), -One, 2, 2));
  expectedInequalities.push_back(
      LinearInequality(X(1), -One, X(6), -One, 0, 3));
  expectedInequalities.push_back(
      LinearInequality(X(2), -One, X(7), -One, 0, 4));
  EXPECT(assert_equal(expectedInequalities, *inequalities));

  bool isFeasible;
  VectorValues initialValues;
  boost::tie(isFeasible, initialValues) = solver.findFeasibleInitialValues();
  EXPECT(assert_equal(1.0 * ones(1), initialValues.at(X(1))));
  EXPECT(assert_equal(0.0 * ones(1), initialValues.at(X(2))));

  VectorValues solution;
  boost::tie(solution, boost::tuples::ignore) = solver.optimize();
  EXPECT(assert_equal(2.0 * ones(1), solution.at(X(1))));
  EXPECT(assert_equal(0.5 * ones(1), solution.at(X(2))));
}

TEST(QPSolver, optimizeNocedal06bookEx16_4_2) {
  QP qp = createTestNocedal06bookEx16_4();
  QPSolver solver(qp);
  VectorValues initialValues;
  initialValues.insert(X(1), (Vector(1) << 0.0).finished());
  initialValues.insert(X(2), (Vector(1) << 100.0).finished());

  VectorValues expectedSolution;
  expectedSolution.insert(X(1), (Vector(1) << 1.4).finished());
  expectedSolution.insert(X(2), (Vector(1) << 1.7).finished());

  VectorValues solution;
  boost::tie(solution, boost::tuples::ignore) = solver.optimize(initialValues);
  // THIS should fail because of the bad infeasible initial point!!
//  CHECK(assert_equal(expectedSolution, solution, 1e-7));

  VectorValues solution2;
  boost::tie(solution2, boost::tuples::ignore) = solver.optimize();
  CHECK(assert_equal(expectedSolution, solution2, 1e-7));
}

/* ************************************************************************* */

TEST(QPSolver, failedSubproblem) {
  QP qp;
  qp.cost.push_back(JacobianFactor(X(1), eye(2), zero(2)));
  qp.cost.push_back(HessianFactor(X(1), zeros(2, 2), zero(2), 100.0));
  qp.inequalities.push_back(
      LinearInequality(X(1), (Matrix(1,2) << -1.0, 0.0).finished(), -1.0, 0));

  VectorValues expected;
  expected.insert(X(1), (Vector(2) << 1.0, 0.0).finished());

  QPSolver solver(qp);
  VectorValues solution;
  boost::tie(solution, boost::tuples::ignore) = solver.optimize();
//  graph.print("Graph: ");
//  solution.print("Solution: ");
  CHECK(assert_equal(expected, solution, 1e-7));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */


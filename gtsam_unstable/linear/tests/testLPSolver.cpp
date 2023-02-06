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

#include <gtsam_unstable/linear/LPInitSolver.h>
#include <gtsam_unstable/linear/LPSolver.h>

#include <gtsam/base/Testable.h>
#include <gtsam/inference/FactorGraph-inst.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam_unstable/linear/EqualityFactorGraph.h>
#include <gtsam_unstable/linear/InequalityFactorGraph.h>
#include <gtsam_unstable/linear/InfeasibleInitialValues.h>

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;
using namespace gtsam::symbol_shorthand;

static const Vector kOne = Vector::Ones(1), kZero = Vector::Zero(1);

/* ************************************************************************* */
/**
 * min -x1-x2
 * s.t.   x1 + 2x2 <= 4
 *       4x1 + 2x2 <= 12
 *       -x1 +  x2 <= 1
 *       x1, x2 >= 0
 */
LP simpleLP1() {
  LP lp;
  lp.cost = LinearCost(1, Vector2(-1., -1.));   // min -x1-x2 (max x1+x2)
  lp.inequalities.add(1, Vector2(-1, 0), 0, 1); // x1 >= 0
  lp.inequalities.add(1, Vector2(0, -1), 0, 2); //  x2 >= 0
  lp.inequalities.add(1, Vector2(1, 2), 4, 3);  //  x1 + 2*x2 <= 4
  lp.inequalities.add(1, Vector2(4, 2), 12, 4); //  4x1 + 2x2 <= 12
  lp.inequalities.add(1, Vector2(-1, 1), 1, 5); //  -x1 + x2 <= 1
  return lp;
}

/* ************************************************************************* */
namespace gtsam {

TEST(LPInitSolver, InfiniteLoopSingleVar) {
  LP lp;
  lp.cost = LinearCost(1, Vector3(0, 0, 1));          // min alpha
  lp.inequalities.add(1, Vector3(-2, -1, -1), -2, 1); //-2x-y-a <= -2
  lp.inequalities.add(1, Vector3(-1, 2, -1), 6, 2);   // -x+2y-a <= 6
  lp.inequalities.add(1, Vector3(-1, 0, -1), 0, 3);   // -x - a <= 0
  lp.inequalities.add(1, Vector3(1, 0, -1), 20, 4);   // x - a <= 20
  lp.inequalities.add(1, Vector3(0, -1, -1), 0, 5);   // -y - a <= 0
  LPSolver solver(lp);
  VectorValues starter;
  starter.insert(1, Vector3(0, 0, 2));
  const auto [results, duals] = solver.optimize(starter);
  VectorValues expected;
  expected.insert(1, Vector3(13.5, 6.5, -6.5));
  CHECK(assert_equal(results, expected, 1e-7));
}

TEST(LPInitSolver, InfiniteLoopMultiVar) {
  LP lp;
  Key X = symbol('X', 1);
  Key Y = symbol('Y', 1);
  Key Z = symbol('Z', 1);
  lp.cost = LinearCost(Z, kOne); // min alpha
  lp.inequalities.add(X, -2.0 * kOne, Y, -1.0 * kOne, Z, -1.0 * kOne, -2,
                      1); //-2x-y-alpha <= -2
  lp.inequalities.add(X, -1.0 * kOne, Y, 2.0 * kOne, Z, -1.0 * kOne, 6,
                      2); // -x+2y-alpha <= 6
  lp.inequalities.add(X, -1.0 * kOne, Z, -1.0 * kOne, 0,
                      3); // -x - alpha <= 0
  lp.inequalities.add(X, 1.0 * kOne, Z, -1.0 * kOne, 20,
                      4); // x - alpha <= 20
  lp.inequalities.add(Y, -1.0 * kOne, Z, -1.0 * kOne, 0,
                      5); // -y - alpha <= 0
  LPSolver solver(lp);
  VectorValues starter;
  starter.insert(X, kZero);
  starter.insert(Y, kZero);
  starter.insert(Z, Vector::Constant(1, 2.0));
  const auto [results, duals] = solver.optimize(starter);
  VectorValues expected;
  expected.insert(X, Vector::Constant(1, 13.5));
  expected.insert(Y, Vector::Constant(1, 6.5));
  expected.insert(Z, Vector::Constant(1, -6.5));
  CHECK(assert_equal(results, expected, 1e-7));
}

TEST(LPInitSolver, Initialization) {
  LP lp = simpleLP1();
  LPInitSolver initSolver(lp);

  GaussianFactorGraph::shared_ptr initOfInitGraph =
      initSolver.buildInitOfInitGraph();
  VectorValues x0 = initOfInitGraph->optimize();
  VectorValues expected_x0;
  expected_x0.insert(1, Vector::Zero(2));
  CHECK(assert_equal(expected_x0, x0, 1e-10));

  double y0 = initSolver.compute_y0(x0);
  double expected_y0 = 0.0;
  DOUBLES_EQUAL(expected_y0, y0, 1e-7);

  Key yKey = 2;
  LP::shared_ptr initLP = initSolver.buildInitialLP(yKey);
  LP expectedInitLP;
  expectedInitLP.cost = LinearCost(yKey, kOne);
  expectedInitLP.inequalities.add(1, Vector2(-1, 0), 2, Vector::Constant(1, -1),
                                  0, 1); // -x1 - y <= 0
  expectedInitLP.inequalities.add(1, Vector2(0, -1), 2, Vector::Constant(1, -1),
                                  0, 2); // -x2 - y <= 0
  expectedInitLP.inequalities.add(1, Vector2(1, 2), 2, Vector::Constant(1, -1),
                                  4,
                                  3); //  x1 + 2*x2 - y <= 4
  expectedInitLP.inequalities.add(1, Vector2(4, 2), 2, Vector::Constant(1, -1),
                                  12,
                                  4); //  4x1 + 2x2 - y <= 12
  expectedInitLP.inequalities.add(1, Vector2(-1, 1), 2, Vector::Constant(1, -1),
                                  1,
                                  5); //  -x1 + x2 - y <= 1
  CHECK(assert_equal(expectedInitLP, *initLP, 1e-10));
  LPSolver lpSolveInit(*initLP);
  VectorValues xy0(x0);
  xy0.insert(yKey, Vector::Constant(1, y0));
  VectorValues xyInit = lpSolveInit.optimize(xy0).first;
  VectorValues expected_init;
  expected_init.insert(1, Vector::Ones(2));
  expected_init.insert(2, Vector::Constant(1, -1));
  CHECK(assert_equal(expected_init, xyInit, 1e-10));

  VectorValues x = initSolver.solve();
  CHECK(lp.isFeasible(x));
}
} // namespace gtsam

/* ************************************************************************* */
/**
 * TEST gtsam solver with an over-constrained system
 *  x + y = 1
 *  x - y = 5
 *  x + 2y = 6
 */
TEST(LPSolver, OverConstrainedLinearSystem) {
  GaussianFactorGraph graph;
  Matrix A1 = Vector3(1, 1, 1);
  Matrix A2 = Vector3(1, -1, 2);
  Vector b = Vector3(1, 5, 6);
  graph.add(1, A1, 2, A2, b, noiseModel::Constrained::All(3));

  VectorValues x = graph.optimize();
  // This check confirms that gtsam linear constraint solver can't handle
  // over-constrained system
  CHECK(graph[0]->error(x) != 0.0);
}

TEST(LPSolver, overConstrainedLinearSystem2) {
  GaussianFactorGraph graph;
  graph.add(1, I_1x1, 2, I_1x1, kOne, noiseModel::Constrained::All(1));
  graph.add(1, I_1x1, 2, -I_1x1, 5 * kOne, noiseModel::Constrained::All(1));
  graph.add(1, I_1x1, 2, 2 * I_1x1, 6 * kOne, noiseModel::Constrained::All(1));
  VectorValues x = graph.optimize();
  // This check confirms that gtsam linear constraint solver can't handle
  // over-constrained system
  CHECK(graph.error(x) != 0.0);
}

/* ************************************************************************* */
TEST(LPSolver, SimpleTest1) {
  LP lp = simpleLP1();
  LPSolver lpSolver(lp);
  VectorValues init;
  init.insert(1, Vector::Zero(2));

  VectorValues x1 =
      lpSolver.buildWorkingGraph(InequalityFactorGraph(), init).optimize();
  VectorValues expected_x1;
  expected_x1.insert(1, Vector::Ones(2));
  CHECK(assert_equal(expected_x1, x1, 1e-10));

  const auto [result, duals] = lpSolver.optimize(init);
  VectorValues expectedResult;
  expectedResult.insert(1, Vector2(8. / 3., 2. / 3.));
  CHECK(assert_equal(expectedResult, result, 1e-10));
}

/* ************************************************************************* */
TEST(LPSolver, TestWithoutInitialValues) {
  LP lp = simpleLP1();
  LPSolver lpSolver(lp);
  VectorValues expectedResult;
  expectedResult.insert(1, Vector2(8. / 3., 2. / 3.));
  const auto [result, duals] = lpSolver.optimize();
  CHECK(assert_equal(expectedResult, result));
}

/**
 * TODO: More TEST cases:
 * - Infeasible
 * - Unbounded
 * - Underdetermined
 */
/* ************************************************************************* */
TEST(LPSolver, LinearCost) {
  LinearCost cost(1, Vector3(2., 4., 6.));
  VectorValues x;
  x.insert(1, Vector3(1., 3., 5.));
  double error = cost.error(x);
  double expectedError = 44.0;
  DOUBLES_EQUAL(expectedError, error, 1e-100);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testLinearSolver.cpp
 * @brief   Tests for Common Interface for Linear Solvers
 * @author  Fan Jiang
 */

#include <gtsam/linear/LinearSolver.h>
#include <gtsam/linear/JacobianGlobalConstraintFactor.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/base/TestableAssertions.h>
#include <CppUnitLite/TestHarness.h>

using namespace gtsam;

/* ************************************************************************* */
/// Factor graph with 2 2D factors on 3 2D variables
static GaussianFactorGraph createSimpleGaussianFactorGraph() {
  GaussianFactorGraph fg;
  Key x1 = 2, x2 = 0, l1 = 1;
  SharedDiagonal unit2 = noiseModel::Unit::Create(2);
  // linearized prior on x1: c[_x1_]+x1=0 i.e. x1=-c[_x1_]
  fg += JacobianFactor(x1, 10 * I_2x2, -1.0 * Vector::Ones(2), unit2);
  // odometry between x1 and x2: x2-x1=[0.2;-0.1]
  fg += JacobianFactor(x2, 10 * I_2x2, x1, -10 * I_2x2, Vector2(2.0, -1.0), unit2);
  // measurement between x1 and l1: l1-x1=[0.0;0.2]
  fg += JacobianFactor(l1, 5 * I_2x2, x1, -5 * I_2x2, Vector2(0.0, 1.0), unit2);
  // measurement between x2 and l1: l1-x2=[-0.2;0.3]
  fg += JacobianFactor(x2, -5 * I_2x2, l1, 5 * I_2x2, Vector2(-1.0, 1.5), unit2);
  return fg;
}

/* ************************************************************************* */
TEST(EigenOptimizer, optimizeEigenQR) {
  GaussianFactorGraph A = createSimpleGaussianFactorGraph();

  VectorValues expected;
  expected.insert(2, Vector2(-0.1, -0.1));
  expected.insert(0, Vector2(0.1, -0.2));
  expected.insert(1, Vector2(-0.1, 0.1));

  LinearSolverParams params;
  params.linearSolverType = LinearSolverParams::EIGEN_QR;
  params.ordering = Ordering::Colamd(A);

  auto solver = LinearSolver::fromLinearSolverParams(params);
  VectorValues actual = solver->solve(A);
  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST(EigenOptimizer, optimizeEigenCholesky) {
  GaussianFactorGraph A = createSimpleGaussianFactorGraph();

  VectorValues expected;
  expected.insert(2, Vector2(-0.1, -0.1));
  expected.insert(0, Vector2(0.1, -0.2));
  expected.insert(1, Vector2(-0.1, 0.1));

  LinearSolverParams params;
  params.linearSolverType = LinearSolverParams::EIGEN_CHOLESKY;
  params.ordering = Ordering::Colamd(A);

  auto solver = LinearSolver::fromLinearSolverParams(params);
  VectorValues actual = solver->solve(A);
  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST(EigenOptimizer, optimizeGlobalConstraintSequential1D) {
  VectorValues expected;
  Key x1 = 0, x2 = 1, x3 = 2;
  expected.insert(x1, Vector1(2.57142857));
  expected.insert(x2, Vector1(1.64285714));
  expected.insert(x3, Vector1(5.78571429));


  SharedDiagonal unit1 = noiseModel::Unit::Create(1);
  GaussianFactorGraph gfg;
  gfg.emplace_shared<JacobianGlobalConstraintFactor>(
      x1, I_1x1, x2, I_1x1, x3, I_1x1, Vector1(10.0), unit1);
  gfg.emplace_shared<JacobianFactor>(x1, I_1x1, x2, -I_1x1, Vector1(1.0), unit1);
  gfg.emplace_shared<JacobianFactor>(x2, I_1x1, x3, -I_1x1, Vector1(-4.0), unit1);
  gfg.emplace_shared<JacobianFactor>(x3, I_1x1, Vector1(6.0), unit1);

  // solve
  LinearSolverParams params;
  params.ordering = Ordering::Colamd(gfg);
  params.linearSolverType = LinearSolverParams::GLOBALCONSTRAINT_SEQUENTIAL_QR;
  auto solver = LinearSolver::fromLinearSolverParams(params);
  VectorValues actual = solver->solve(gfg);
  EXPECT(assert_equal(expected, actual, 1e-7));
}

/* ************************************************************************* */
TEST(EigenOptimizer, optimizeGlobalConstraintSequential2D) {
  Key x1 = 0, x2 = 1, x3 = 2;

  // no constraints - don't segfault!
  SharedDiagonal unit2 = noiseModel::Unit::Create(2);
  GaussianFactorGraph gfg;
  gfg.emplace_shared<JacobianFactor>(x1, I_2x2, x2, -1*I_2x2, Vector2(0.0, -2.0), unit2);
  gfg.emplace_shared<JacobianFactor>(x2, I_2x2, x3, -1*I_2x2, Vector2(-3.0, 1.0), unit2);
  gfg.emplace_shared<JacobianFactor>(x3, I_2x2, x1, -1*I_2x2, Vector2(3.0, 1.0), unit2);

  // check that no segfault occurs
  LinearSolverParams params;
  params.ordering = Ordering::Colamd(gfg);
  params.linearSolverType = LinearSolverParams::GLOBALCONSTRAINT_SEQUENTIAL_QR;
  auto solver = LinearSolver::fromLinearSolverParams(params);
  solver->solve(gfg); // don't segfault!

  // underconstrained H
  gfg.emplace_shared<JacobianGlobalConstraintFactor>(
      x1, I_2x2, x2, I_2x2, x3, I_2x2, Vector2(0.0, 0.0),
      noiseModel::Constrained::All(2));

  // Solve with Normal Elimination QR
  params.linearSolverType = LinearSolverParams::SEQUENTIAL_QR;
  VectorValues expected =
      gfg.eliminateSequential(*params.ordering, params.getEliminationFunction(),
                              boost::none, params.orderingType)->optimize();

  // Solve with global constraint
  VectorValues actual = solver->solve(gfg);
  EXPECT(assert_equal(expected, actual));

  // ----- overcontrained H -----
  gfg.emplace_shared<JacobianFactor>(x1, I_2x2, x2, I_2x2, x3, -1*I_2x2, Vector2(-3.0, 0.0), unit2);
  // solve with Elimination QR
  expected =
      gfg.eliminateSequential(*params.ordering, params.getEliminationFunction(),
                              boost::none, params.orderingType)->optimize();

  // Solve with global constraint
  actual = solver->solve(gfg);
  EXPECT(assert_equal(expected, actual));

}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

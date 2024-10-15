/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testPCGSolver.cpp
 * @brief   Unit tests for PCGSolver class
 * @author  Yong-Dian Jian
 * @date    Aug 06, 2014
 */

#include <tests/smallExample.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/PCGSolver.h>
#include <gtsam/linear/SubgraphPreconditioner.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/Matrix.h>

#include <CppUnitLite/TestHarness.h>

#include <iostream>
#include <fstream>

using namespace std;
using namespace gtsam;

const double tol = 1e-3;

using symbol_shorthand::X;
using symbol_shorthand::L;

/* ************************************************************************* */
// Test cholesky decomposition
TEST( PCGSolver, llt ) {
  Matrix R = (Matrix(3,3) <<
                1., -1., -1.,
                0.,  2., -1.,
                0.,  0.,  1.).finished();
  Matrix AtA = R.transpose() * R;

  Vector Rvector = (Vector(9) << 1., -1., -1.,
                                 0.,  2., -1.,
                                 0.,  0.,  1.).finished();
//  Vector Rvector = (Vector(6) << 1., -1., -1.,
//                                      2., -1.,
//                                           1.).finished();

  Vector b = Vector3(1., 2., 3.);

  Vector x = Vector3(6.5, 2.5, 3.) ;

  /* test cholesky */
  Matrix Rhat = AtA.llt().matrixL().transpose();
  EXPECT(assert_equal(R, Rhat, 1e-5));

  /* test backward substitution */
  Vector xhat = Rhat.triangularView<Eigen::Upper>().solve(b);
  EXPECT(assert_equal(x, xhat, 1e-5));

  /* test in-place back substitution */
  xhat = b;
  Rhat.triangularView<Eigen::Upper>().solveInPlace(xhat);
  EXPECT(assert_equal(x, xhat, 1e-5));

  /* test triangular matrix map */
  Eigen::Map<Eigen::MatrixXd> Radapter(Rvector.data(), 3, 3);
  xhat = Radapter.transpose().triangularView<Eigen::Upper>().solve(b);
  EXPECT(assert_equal(x, xhat, 1e-5));

}

/* ************************************************************************* */
// Test GaussianFactorGraphSystem::multiply and getb
TEST( GaussianFactorGraphSystem, multiply_getb)
{
  // Create a Gaussian Factor Graph
  GaussianFactorGraph simpleGFG;
  SharedDiagonal unit2 = noiseModel::Diagonal::Sigmas(Vector2(0.5, 0.3));
  simpleGFG.emplace_shared<JacobianFactor>(2, (Matrix(2,2)<< 10, 0, 0, 10).finished(), (Vector(2) << -1, -1).finished(), unit2);
  simpleGFG.emplace_shared<JacobianFactor>(2, (Matrix(2,2)<< -10, 0, 0, -10).finished(), 0, (Matrix(2,2)<< 10, 0, 0, 10).finished(), (Vector(2) << 2, -1).finished(), unit2);
  simpleGFG.emplace_shared<JacobianFactor>(2, (Matrix(2,2)<< -5, 0, 0, -5).finished(), 1, (Matrix(2,2)<< 5, 0, 0, 5).finished(), (Vector(2) << 0, 1).finished(), unit2);
  simpleGFG.emplace_shared<JacobianFactor>(0, (Matrix(2,2)<< -5, 0, 0, -5).finished(), 1, (Matrix(2,2)<< 5, 0, 0, 5).finished(), (Vector(2) << -1, 1.5).finished(), unit2);
  simpleGFG.emplace_shared<JacobianFactor>(0, (Matrix(2,2)<< 1, 0, 0, 1).finished(), (Vector(2) << 0, 0).finished(), unit2);
  simpleGFG.emplace_shared<JacobianFactor>(1, (Matrix(2,2)<< 1, 0, 0, 1).finished(), (Vector(2) << 0, 0).finished(), unit2);
  simpleGFG.emplace_shared<JacobianFactor>(2, (Matrix(2,2)<< 1, 0, 0, 1).finished(), (Vector(2) << 0, 0).finished(), unit2);

  // Create a dummy-preconditioner and a GaussianFactorGraphSystem
  DummyPreconditioner dummyPreconditioner;
  KeyInfo keyInfo(simpleGFG);
  std::map<Key,Vector> lambda;
  dummyPreconditioner.build(simpleGFG, keyInfo, lambda);
  GaussianFactorGraphSystem gfgs(simpleGFG, dummyPreconditioner, keyInfo, lambda);

  // Prepare container for each variable
  Vector initial, residual, preconditionedResidual, p, actualAp;
  initial = (Vector(6) << 0., 0., 0., 0., 0., 0.).finished();

  // Calculate values using GaussianFactorGraphSystem same as inside of PCGSolver
  residual = gfgs.residual(initial); /* r = b-Ax */
  preconditionedResidual = gfgs.leftPrecondition(
      residual, preconditionedResidual); /* pr = L^{-1} (b-Ax) */
  p = gfgs.rightPrecondition(preconditionedResidual, p); /* p = L^{-T} pr */
  actualAp = gfgs.multiply(p);                           /* A p */

  // Expected value of Ap for the first iteration of this example problem
  Vector expectedAp = (Vector(6) << 100400, -249074.074, -2080, 148148.148, -146480, 37962.963).finished();
  EXPECT(assert_equal(expectedAp, actualAp, 1e-3));

  // Expected value of getb
  Vector expectedb = (Vector(6) << 100.0, -194.444, -20.0, 138.889, -120.0, -55.556).finished();
  Vector actualb = gfgs.getb();
  EXPECT(assert_equal(expectedb, actualb, 1e-3));
}

/* ************************************************************************* */
// Test Dummy Preconditioner
TEST(PCGSolver, dummy) {
  LevenbergMarquardtParams params;
  params.linearSolverType = LevenbergMarquardtParams::Iterative;
  auto pcg = std::make_shared<PCGSolverParameters>(
      std::make_shared<DummyPreconditionerParameters>());
  params.iterativeParams = pcg;

  NonlinearFactorGraph fg = example::createReallyNonlinearFactorGraph();

  Point2 x0(10, 10);
  Values c0;
  c0.insert(X(1), x0);

  Values actualPCG = LevenbergMarquardtOptimizer(fg, c0, params).optimize();

  DOUBLES_EQUAL(0, fg.error(actualPCG), tol);
}

/* ************************************************************************* */
// Test Block-Jacobi Precondioner
TEST(PCGSolver, blockjacobi) {
  LevenbergMarquardtParams params;
  params.linearSolverType = LevenbergMarquardtParams::Iterative;
  auto pcg = std::make_shared<PCGSolverParameters>(
      std::make_shared<BlockJacobiPreconditionerParameters>());
  params.iterativeParams = pcg;

  NonlinearFactorGraph fg = example::createReallyNonlinearFactorGraph();

  Point2 x0(10, 10);
  Values c0;
  c0.insert(X(1), x0);

  Values actualPCG = LevenbergMarquardtOptimizer(fg, c0, params).optimize();

  DOUBLES_EQUAL(0, fg.error(actualPCG), tol);
}

/* ************************************************************************* */
// Test Incremental Subgraph PCG Solver
TEST(PCGSolver, subgraph) {
  LevenbergMarquardtParams params;
  params.linearSolverType = LevenbergMarquardtParams::Iterative;
  auto pcg = std::make_shared<PCGSolverParameters>(
      std::make_shared<SubgraphPreconditionerParameters>());
  params.iterativeParams = pcg;

  NonlinearFactorGraph fg = example::createReallyNonlinearFactorGraph();

  Point2 x0(10, 10);
  Values c0;
  c0.insert(X(1), x0);

  Values actualPCG = LevenbergMarquardtOptimizer(fg, c0, params).optimize();

  DOUBLES_EQUAL(0, fg.error(actualPCG), tol);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}


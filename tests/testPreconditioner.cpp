/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   testPreconditioner.cpp
 *  @brief  Unit tests for Preconditioners
 *  @author Sungtae An
 *  @date   Nov 6, 2014
 **/

#include <CppUnitLite/TestHarness.h>

#include <gtsam/nonlinear/Values.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/linear/Preconditioner.h>
#include <gtsam/linear/PCGSolver.h>
#include <gtsam/geometry/Point2.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST( PCGsolver, verySimpleLinearSystem) {

  // Ax = [4 1][u] = [1]  x0 = [2]
  //      [1 3][v]   [2]       [1]
  //
  // exact solution x = [1/11, 7/11]';
  //

  // Create a Gaussian Factor Graph
  GaussianFactorGraph simpleGFG;
  simpleGFG.emplace_shared<JacobianFactor>(0, (Matrix(2,2)<< 4, 1, 1, 3).finished(), (Vector(2) << 1,2 ).finished(), noiseModel::Unit::Create(2));

  // Exact solution already known
  VectorValues exactSolution;
  exactSolution.insert(0, (Vector(2) << 1./11., 7./11.).finished());
  //exactSolution.print("Exact");

  // Solve the system using direct method
  VectorValues deltaDirect = simpleGFG.optimize();
  EXPECT(assert_equal(exactSolution, deltaDirect, 1e-7));
  //deltaDirect.print("Direct");

  // Solve the system using Preconditioned Conjugate Gradient solver
  // Common PCG parameters
  gtsam::PCGSolverParameters::shared_ptr pcg = std::make_shared<gtsam::PCGSolverParameters>();
  pcg->maxIterations = 500;
  pcg->epsilon_abs = 0.0;
  pcg->epsilon_rel = 0.0;
  //pcg->setVerbosity("ERROR");

  // With Dummy preconditioner
  pcg->preconditioner =
      std::make_shared<gtsam::DummyPreconditionerParameters>();
  VectorValues deltaPCGDummy = PCGSolver(*pcg).optimize(simpleGFG);
  EXPECT(assert_equal(exactSolution, deltaPCGDummy, 1e-7));
  //deltaPCGDummy.print("PCG Dummy");

  // With Block-Jacobi preconditioner
  pcg->preconditioner =
      std::make_shared<gtsam::BlockJacobiPreconditionerParameters>();
  // It takes more than 1000 iterations for this test
  pcg->maxIterations = 1500;
  VectorValues deltaPCGJacobi = PCGSolver(*pcg).optimize(simpleGFG);

  EXPECT(assert_equal(exactSolution, deltaPCGJacobi, 1e-5));
  //deltaPCGJacobi.print("PCG Jacobi");
}

/* ************************************************************************* */
TEST(PCGSolver, simpleLinearSystem) {
  // Create a Gaussian Factor Graph
  GaussianFactorGraph simpleGFG;
  //SharedDiagonal unit2 = noiseModel::Unit::Create(2);
  SharedDiagonal unit2 = noiseModel::Diagonal::Sigmas(Vector2(0.5, 0.3));
  simpleGFG.emplace_shared<JacobianFactor>(2, (Matrix(2,2)<< 10, 0, 0, 10).finished(), (Vector(2) << -1, -1).finished(), unit2);
  simpleGFG.emplace_shared<JacobianFactor>(2, (Matrix(2,2)<< -10, 0, 0, -10).finished(), 0, (Matrix(2,2)<< 10, 0, 0, 10).finished(), (Vector(2) << 2, -1).finished(), unit2);
  simpleGFG.emplace_shared<JacobianFactor>(2, (Matrix(2,2)<< -5, 0, 0, -5).finished(), 1, (Matrix(2,2)<< 5, 0, 0, 5).finished(), (Vector(2) << 0, 1).finished(), unit2);
  simpleGFG.emplace_shared<JacobianFactor>(0, (Matrix(2,2)<< -5, 0, 0, -5).finished(), 1, (Matrix(2,2)<< 5, 0, 0, 5).finished(), (Vector(2) << -1, 1.5).finished(), unit2);
  simpleGFG.emplace_shared<JacobianFactor>(0, (Matrix(2,2)<< 1, 0, 0, 1).finished(), (Vector(2) << 0, 0).finished(), unit2);
  simpleGFG.emplace_shared<JacobianFactor>(1, (Matrix(2,2)<< 1, 0, 0, 1).finished(), (Vector(2) << 0, 0).finished(), unit2);
  simpleGFG.emplace_shared<JacobianFactor>(2, (Matrix(2,2)<< 1, 0, 0, 1).finished(), (Vector(2) << 0, 0).finished(), unit2);
  //simpleGFG.print("system");

  // Expected solution
  VectorValues expectedSolution;
  expectedSolution.insert(0, (Vector(2) << 0.100498, -0.196756).finished());
  expectedSolution.insert(2, (Vector(2) << -0.0990413, -0.0980577).finished());
  expectedSolution.insert(1, (Vector(2) << -0.0973252, 0.100582).finished());
  //expectedSolution.print("Expected");

  // Solve the system using direct method
  VectorValues deltaDirect = simpleGFG.optimize();
  EXPECT(assert_equal(expectedSolution, deltaDirect, 1e-5));
  //deltaDirect.print("Direct");

  // Solve the system using Preconditioned Conjugate Gradient solver
  // Common PCG parameters
  gtsam::PCGSolverParameters::shared_ptr pcg = std::make_shared<gtsam::PCGSolverParameters>();
  pcg->maxIterations = 500;
  pcg->epsilon_abs = 0.0;
  pcg->epsilon_rel = 0.0;
  //pcg->setVerbosity("ERROR");

  // With Dummy preconditioner
  pcg->preconditioner =
      std::make_shared<gtsam::DummyPreconditionerParameters>();
  VectorValues deltaPCGDummy = PCGSolver(*pcg).optimize(simpleGFG);
  EXPECT(assert_equal(expectedSolution, deltaPCGDummy, 1e-5));
  //deltaPCGDummy.print("PCG Dummy");

  // With Block-Jacobi preconditioner
  pcg->preconditioner =
      std::make_shared<gtsam::BlockJacobiPreconditionerParameters>();
  VectorValues deltaPCGJacobi = PCGSolver(*pcg).optimize(simpleGFG);
  EXPECT(assert_equal(expectedSolution, deltaPCGJacobi, 1e-5));
  //deltaPCGJacobi.print("PCG Jacobi");

}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */

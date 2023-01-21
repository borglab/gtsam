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
 * @author  Fan Jiang and Gerry Chen
 */

#include <gtsam/linear/LinearSolver.h>
#include <gtsam/linear/PCGSolver.h>
#include <gtsam/linear/SubgraphSolver.h>
#include <gtsam/linear/Preconditioner.h>
#include <gtsam/linear/GaussianFactorGraph.h>
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
TEST(LinearOptimizer, solverCheckIndividually) {
  GaussianFactorGraph gfg = createSimpleGaussianFactorGraph();

  VectorValues expected;
  expected.insert(2, Vector2(-0.1, -0.1));
  expected.insert(0, Vector2(0.1, -0.2));
  expected.insert(1, Vector2(-0.1, 0.1));

  LinearSolverParams params;
  params.orderingType = Ordering::COLAMD;

  // Below we solve with different backend linear solver choices
  // Note: these tests are not in a for loop to enable easier debugging

  // Multifrontal Cholesky (more sensitive to conditioning, but faster)
  params.linearSolverType = LinearSolverParams::MULTIFRONTAL_CHOLESKY;
  auto solver = LinearSolver::CreateFromParameters(params);
  VectorValues actual = (*solver)(gfg);
  EXPECT(assert_equal(expected, actual));
  actual = solver->solve(gfg);
  EXPECT(assert_equal(expected, actual));

  // Multifrontal QR, will be parallel if TBB installed
  params.linearSolverType = LinearSolverParams::MULTIFRONTAL_QR;
  actual = LinearSolver::CreateFromParameters(params)->solve(gfg);
  EXPECT(assert_equal(expected, actual));

  // Sequential Cholesky (more sensitive to conditioning, but faster)
  params.linearSolverType = LinearSolverParams::SEQUENTIAL_CHOLESKY;
  actual = LinearSolver::CreateFromParameters(params)->solve(gfg);
  EXPECT(assert_equal(expected, actual));

  // Sequential QR, not parallelized
  params.linearSolverType = LinearSolverParams::SEQUENTIAL_QR;
  actual = LinearSolver::CreateFromParameters(params)->solve(gfg);
  EXPECT(assert_equal(expected, actual));

  // Iterative - either PCGSolver or SubgraphSolver
  // first PCGSolver
  params = LinearSolverParams(
      LinearSolverParams::Iterative, Ordering::COLAMD,
      boost::make_shared<PCGSolverParameters>(
          boost::make_shared<DummyPreconditionerParameters>()));
  actual = LinearSolver::CreateFromParameters(params)->solve(gfg);
  EXPECT(assert_equal(expected, actual));
  // then SubgraphSolver
  params.ordering = Ordering::Colamd(gfg);
  params.iterativeParams = boost::make_shared<SubgraphSolverParameters>();
  actual = LinearSolver::CreateFromParameters(params)->solve(gfg);
  EXPECT(assert_equal(expected, actual));

  // cholmod - this flag exists for backwards compatibility but doesn't really
  // correspond to any meaningful action
  // TODO: merge CHOLMOD and SuiteSparse ?
  params.linearSolverType = LinearSolverParams::CHOLMOD;
  THROWS_EXCEPTION(actual =
                       LinearSolver::CreateFromParameters(params)->solve(gfg);)

  // PCG - Preconditioned Conjugate Gradient, an iterative method
  params = LinearSolverParams(
      LinearSolverParams::PCG, Ordering::COLAMD,
      boost::make_shared<PCGSolverParameters>(
          boost::make_shared<DummyPreconditionerParameters>()));
  actual = LinearSolver::CreateFromParameters(params)->solve(gfg);
  EXPECT(assert_equal(expected, actual));

  // Subgraph - SPCG, see SubgraphSolver.h
  params.linearSolverType = LinearSolverParams::SUBGRAPH;
  params.ordering = Ordering::Colamd(gfg);
  params.iterativeParams = boost::make_shared<SubgraphSolverParameters>();
  actual = LinearSolver::CreateFromParameters(params)->solve(gfg);
  EXPECT(assert_equal(expected, actual));

  // Sparse Eigen QR
  params.linearSolverType = LinearSolverParams::EIGEN_QR;
  actual = LinearSolver::CreateFromParameters(params)->solve(gfg);
  EXPECT(assert_equal(expected, actual));

  // Sparse Eigen Cholesky
  params.linearSolverType = LinearSolverParams::EIGEN_CHOLESKY;
  actual = LinearSolver::CreateFromParameters(params)->solve(gfg);
  EXPECT(assert_equal(expected, actual));

  // SuiteSparse Cholesky
  #ifdef GTSAM_USE_SUITESPARSE
  params.linearSolverType = LinearSolverParams::SUITESPARSE;
  actual = LinearSolver::CreateFromParameters(params)->solve(gfg);
  EXPECT(assert_equal(expected, actual));
  #endif

  // CuSparse Cholesky
  #ifdef GTSAM_USE_CUSPARSE
  params.linearSolverType = LinearSolverParams::CUSPARSE;
  actual = LinearSolver::CreateFromParameters(params)->solve(gfg);
  EXPECT(assert_equal(expected, actual));
  #endif
}

// creates a dummy iterativeParams object for the appropriate solver type
IterativeOptimizationParameters::shared_ptr createIterativeParams(int solver) {
  typedef LinearSolverParams LSP;
  return (solver == LSP::Iterative) || (solver == LSP::PCG)
             ? boost::make_shared<PCGSolverParameters>(
                   boost::make_shared<DummyPreconditionerParameters>())
             : (solver == LSP::SUBGRAPH)
                   ? boost::make_shared<SubgraphSolverParameters>()
                   : boost::make_shared<IterativeOptimizationParameters>();
}

/* ************************************************************************* */
TEST(LinearOptimizer, solverCheckWithLoop) {
  // the same as the previous test except in a loop for consistency
  GaussianFactorGraph gfg = createSimpleGaussianFactorGraph();

  VectorValues expected;
  expected.insert(2, Vector2(-0.1, -0.1));
  expected.insert(0, Vector2(0.1, -0.2));
  expected.insert(1, Vector2(-0.1, 0.1));

  LinearSolverParams params;
  params.ordering = Ordering::Colamd(gfg);

  // Test all linear solvers
  typedef LinearSolverParams LSP;
  for (int solverType = LSP::MULTIFRONTAL_CHOLESKY; solverType != LSP::LAST;
       solverType++) {
    if (solverType == LSP::CHOLMOD) continue;  // CHOLMOD is an undefined option
#ifndef GTSAM_USE_SUITESPARSE
    if (solverType == LSP::SUITESPARSE) continue;
#endif
#ifndef GTSAM_USE_CUSPARSE
    if (solverType == LSP::CUSPARSE) continue;
#endif
    auto params = LinearSolverParams(
        static_cast<LSP::LinearSolverType>(solverType), Ordering::Colamd(gfg),
        createIterativeParams(solverType));
    auto linearSolver = LinearSolver::CreateFromParameters(params);
    auto actual = (*linearSolver)(gfg);
    EXPECT(assert_equal(expected, actual));
  }
}

/* ************************************************************************* */
// assert Iterative, PCG, and Subgraph will throw errors if iterativeParams not
// set
TEST(LinearOptimizer, IterativeThrowError) {
  LinearSolverParams params;
  params.orderingType = Ordering::COLAMD;

  params.linearSolverType = LinearSolverParams::Iterative;
  CHECK_EXCEPTION(LinearSolver::CreateFromParameters(params),
                  std::runtime_error)
  params.linearSolverType = LinearSolverParams::PCG;
  CHECK_EXCEPTION(LinearSolver::CreateFromParameters(params),
                  std::runtime_error)
  params.linearSolverType = LinearSolverParams::SUBGRAPH;
  CHECK_EXCEPTION(LinearSolver::CreateFromParameters(params),
                  std::runtime_error)
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

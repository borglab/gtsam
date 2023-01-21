/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testNonlinearOptimizer.cpp
 * @brief   Unit tests for NonlinearOptimizer class
 * @author  Frank Dellaert
 */

#include <tests/smallExample.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearConjugateGradientOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/linear/LinearSolverParams.h>
#include <gtsam/linear/PCGSolver.h>
#include <gtsam/linear/SubgraphSolver.h>
#include <gtsam/linear/Preconditioner.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/base/Matrix.h>

#include <CppUnitLite/TestHarness.h>

#include <boost/range/adaptor/map.hpp>
#include <boost/shared_ptr.hpp>
using boost::adaptors::map_values;

#include <iostream>
#include <fstream>

using namespace std;
using namespace gtsam;

const double tol = 1e-5;

using symbol_shorthand::X;
using symbol_shorthand::L;

/* ************************************************************************* */
TEST( NonlinearOptimizer, paramsEquals )
{
  // default constructors lead to two identical params
  GaussNewtonParams gnParams1;
  GaussNewtonParams gnParams2;
  CHECK(gnParams1.equals(gnParams2));

  // but the params become different if we change something in gnParams2
  gnParams2.setVerbosity("DELTA");
  CHECK(!gnParams1.equals(gnParams2));
}

/* ************************************************************************* */
TEST( NonlinearOptimizer, iterateLM )
{
  // really non-linear factor graph
  auto fg = example::createReallyNonlinearFactorGraph();

  // config far from minimum
  Point2 x0(3,0);
  Values config;
  config.insert(X(1), x0);

  // normal iterate
  GaussNewtonOptimizer gnOptimizer(fg, config);
  gnOptimizer.iterate();

  // LM iterate with lambda 0 should be the same
  LevenbergMarquardtParams lmParams;
  lmParams.lambdaInitial = 0.0;
  LevenbergMarquardtOptimizer lmOptimizer(fg, config, lmParams);
  lmOptimizer.iterate();

  CHECK(assert_equal(gnOptimizer.values(), lmOptimizer.values(), 1e-9));
}

/* ************************************************************************* */
TEST( NonlinearOptimizer, optimize )
{
  auto fg = example::createReallyNonlinearFactorGraph();

  // test error at minimum
  Point2 xstar(0,0);
  Values cstar;
  cstar.insert(X(1), xstar);
  DOUBLES_EQUAL(0.0,fg.error(cstar),0.0);

  // test error at initial = [(1-cos(3))^2 + (sin(3))^2]*50 =
  Point2 x0(3,3);
  Values c0;
  c0.insert(X(1), x0);
  DOUBLES_EQUAL(199.0,fg.error(c0),1e-3);

  // optimize parameters
  Ordering ord;
  ord.push_back(X(1));

  // Gauss-Newton
  GaussNewtonParams gnParams;
  gnParams.ordering = ord;
  Values actual1 = GaussNewtonOptimizer(fg, c0, gnParams).optimize();
  DOUBLES_EQUAL(0,fg.error(actual1),tol);

  // Levenberg-Marquardt
  LevenbergMarquardtParams lmParams;
  lmParams.ordering = ord;
  Values actual2 = LevenbergMarquardtOptimizer(fg, c0, lmParams).optimize();
  DOUBLES_EQUAL(0,fg.error(actual2),tol);

  // Dogleg
  DoglegParams dlParams;
  dlParams.ordering = ord;
  Values actual3 = DoglegOptimizer(fg, c0, dlParams).optimize();
  DOUBLES_EQUAL(0,fg.error(actual3),tol);
}

/* ************************************************************************* */
TEST( NonlinearOptimizer, SimpleLMOptimizer )
{
  auto fg = example::createReallyNonlinearFactorGraph();

  Point2 x0(3,3);
  Values c0;
  c0.insert(X(1), x0);

  Values actual = LevenbergMarquardtOptimizer(fg, c0).optimize();
  DOUBLES_EQUAL(0,fg.error(actual),tol);
}

/* ************************************************************************* */
TEST( NonlinearOptimizer, SimpleGNOptimizer )
{
  auto fg = example::createReallyNonlinearFactorGraph();

  Point2 x0(3,3);
  Values c0;
  c0.insert(X(1), x0);

  Values actual = GaussNewtonOptimizer(fg, c0).optimize();
  DOUBLES_EQUAL(0,fg.error(actual),tol);
}

/* ************************************************************************* */
TEST( NonlinearOptimizer, SimpleDLOptimizer )
{
  auto fg = example::createReallyNonlinearFactorGraph();

  Point2 x0(3,3);
  Values c0;
  c0.insert(X(1), x0);

  Values actual = DoglegOptimizer(fg, c0).optimize();
  DOUBLES_EQUAL(0,fg.error(actual),tol);
}

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
TEST(NonlinearOptimizer, optimization_method) {
  // Create nonlinear example
  auto fg = example::createReallyNonlinearFactorGraph();

  // Create some test Values (just one 2D point, in this case)
  Point2 x0(3, 3);
  Values c0;
  c0.insert(X(1), x0);

  LevenbergMarquardtParams params;

  // Test all linear solvers
  typedef LinearSolverParams LSP;
  for (int solver = LSP::MULTIFRONTAL_CHOLESKY; solver != LSP::LAST; solver++) {
    if (solver == LSP::CHOLMOD) continue;  // CHOLMOD is an undefined option
#ifndef GTSAM_USE_SUITESPARSE
    if (solver == LSP::SUITESPARSE) continue;
#endif
#ifndef GTSAM_USE_CUSPARSE
    if (solver == LSP::CUSPARSE) continue;
#endif
    params.linearSolverType = static_cast<LSP::LinearSolverType>(solver);
    params.iterativeParams = createIterativeParams(solver);
    Values actual = LevenbergMarquardtOptimizer(fg, c0, params).optimize();
    DOUBLES_EQUAL(0, fg.error(actual), tol);
  }
}

/* ************************************************************************* */
TEST( NonlinearOptimizer, Factorization )
{
  Values config;
  config.insert(X(1), Pose2(0.,0.,0.));
  config.insert(X(2), Pose2(1.5,0.,0.));

  NonlinearFactorGraph graph;
  graph.addPrior(X(1), Pose2(0.,0.,0.), noiseModel::Isotropic::Sigma(3, 1e-10));
  graph += BetweenFactor<Pose2>(X(1),X(2), Pose2(1.,0.,0.), noiseModel::Isotropic::Sigma(3, 1));

  Ordering ordering;
  ordering.push_back(X(1));
  ordering.push_back(X(2));

  LevenbergMarquardtParams params;
  LevenbergMarquardtParams::SetLegacyDefaults(&params);
  LevenbergMarquardtOptimizer optimizer(graph, config, ordering, params);
  optimizer.iterate();

  Values expected;
  expected.insert(X(1), Pose2(0.,0.,0.));
  expected.insert(X(2), Pose2(1.,0.,0.));
  CHECK(assert_equal(expected, optimizer.values(), 1e-5));
}

/* ************************************************************************* */
TEST(NonlinearOptimizer, NullFactor) {

  auto fg = example::createReallyNonlinearFactorGraph();

  // Add null factor
  fg.push_back(NonlinearFactorGraph::sharedFactor());

  // test error at minimum
  Point2 xstar(0,0);
  Values cstar;
  cstar.insert(X(1), xstar);
  DOUBLES_EQUAL(0.0,fg.error(cstar),0.0);

  // test error at initial = [(1-cos(3))^2 + (sin(3))^2]*50 =
  Point2 x0(3,3);
  Values c0;
  c0.insert(X(1), x0);
  DOUBLES_EQUAL(199.0,fg.error(c0),1e-3);

  // optimize parameters
  Ordering ord;
  ord.push_back(X(1));

  // Gauss-Newton
  Values actual1 = GaussNewtonOptimizer(fg, c0, ord).optimize();
  DOUBLES_EQUAL(0,fg.error(actual1),tol);

  // Levenberg-Marquardt
  Values actual2 = LevenbergMarquardtOptimizer(fg, c0, ord).optimize();
  DOUBLES_EQUAL(0,fg.error(actual2),tol);

  // Dogleg
  Values actual3 = DoglegOptimizer(fg, c0, ord).optimize();
  DOUBLES_EQUAL(0,fg.error(actual3),tol);
}

TEST(NonlinearOptimizerParams, RuleOfFive) {
  // test copy and move constructors.
  NonlinearOptimizerParams params;
  typedef LinearSolverParams LSP;
  params.maxIterations = 2;
  params.linearSolverType = LSP::MULTIFRONTAL_QR;

  // test copy's
  auto params2 = params;                                 // copy-assignment
  auto params3{params};                                  // copy-constructor
  EXPECT(params2.maxIterations == params.maxIterations);
  EXPECT(params2.linearSolverType == params.linearSolverType);
  EXPECT(params.linearSolverType ==
         params.linearSolverParams.linearSolverType);
  EXPECT(params2.linearSolverType ==
         params2.linearSolverParams.linearSolverType);
  EXPECT(params3.maxIterations == params.maxIterations);
  EXPECT(params3.linearSolverType == params.linearSolverType);
  EXPECT(params3.linearSolverType ==
         params3.linearSolverParams.linearSolverType);
  params2.linearSolverType = LSP::MULTIFRONTAL_CHOLESKY;
  params3.linearSolverType = LSP::SEQUENTIAL_QR;
  EXPECT(params.linearSolverType == LSP::MULTIFRONTAL_QR);
  EXPECT(params.linearSolverParams.linearSolverType == LSP::MULTIFRONTAL_QR);
  EXPECT(params2.linearSolverType == LSP::MULTIFRONTAL_CHOLESKY);
  EXPECT(params2.linearSolverParams.linearSolverType == LSP::MULTIFRONTAL_CHOLESKY);
  EXPECT(params3.linearSolverType == LSP::SEQUENTIAL_QR);
  EXPECT(params3.linearSolverParams.linearSolverType == LSP::SEQUENTIAL_QR);

  // test move's
  NonlinearOptimizerParams params4 = std::move(params2);  // move-constructor
  NonlinearOptimizerParams params5;
  params5 = std::move(params3);                           // move-assignment
  EXPECT(params4.linearSolverType == LSP::MULTIFRONTAL_CHOLESKY);
  EXPECT(params4.linearSolverParams.linearSolverType == LSP::MULTIFRONTAL_CHOLESKY);
  EXPECT(params5.linearSolverType == LSP::SEQUENTIAL_QR);
  EXPECT(params5.linearSolverParams.linearSolverType == LSP::SEQUENTIAL_QR);
  params4.linearSolverType = LSP::SEQUENTIAL_CHOLESKY;
  params5.linearSolverType = LSP::EIGEN_QR;
  EXPECT(params4.linearSolverType == LSP::SEQUENTIAL_CHOLESKY);
  EXPECT(params4.linearSolverParams.linearSolverType == LSP::SEQUENTIAL_CHOLESKY);
  EXPECT(params5.linearSolverType == LSP::EIGEN_QR);
  EXPECT(params5.linearSolverParams.linearSolverType == LSP::EIGEN_QR);

  // test destructor
  {
    NonlinearOptimizerParams params6;
  }
}

/* ************************************************************************* */
TEST_UNSAFE(NonlinearOptimizer, MoreOptimization) {

  NonlinearFactorGraph fg;
  fg.addPrior(0, Pose2(0, 0, 0), noiseModel::Isotropic::Sigma(3, 1));
  fg += BetweenFactor<Pose2>(0, 1, Pose2(1, 0, M_PI / 2),
      noiseModel::Isotropic::Sigma(3, 1));
  fg += BetweenFactor<Pose2>(1, 2, Pose2(1, 0, M_PI / 2),
      noiseModel::Isotropic::Sigma(3, 1));

  Values init;
  init.insert(0, Pose2(3, 4, -M_PI));
  init.insert(1, Pose2(10, 2, -M_PI));
  init.insert(2, Pose2(11, 7, -M_PI));

  Values expected;
  expected.insert(0, Pose2(0, 0, 0));
  expected.insert(1, Pose2(1, 0, M_PI / 2));
  expected.insert(2, Pose2(1, 1, M_PI));

  VectorValues expectedGradient;
  expectedGradient.insert(0,Z_3x1);
  expectedGradient.insert(1,Z_3x1);
  expectedGradient.insert(2,Z_3x1);

  // Try LM and Dogleg
  auto params = LevenbergMarquardtParams::LegacyDefaults();
  {
    LevenbergMarquardtOptimizer optimizer(fg, init, params);

    // test convergence
    Values actual = optimizer.optimize();
    EXPECT(assert_equal(expected, actual));

    // Check that the gradient is zero
    GaussianFactorGraph::shared_ptr linear = optimizer.linearize();
    EXPECT(assert_equal(expectedGradient,linear->gradientAtZero()));
  }
  EXPECT(assert_equal(expected, DoglegOptimizer(fg, init).optimize()));

  // Try LM with diagonal damping
  Values initBetter;
    initBetter.insert(0, Pose2(3,4,0));
    initBetter.insert(1, Pose2(10,2,M_PI/3));
    initBetter.insert(2, Pose2(11,7,M_PI/2));

  {
    params.diagonalDamping = true;
    LevenbergMarquardtOptimizer optimizer(fg, initBetter, params);

    // test the diagonal
    GaussianFactorGraph::shared_ptr linear = optimizer.linearize();
    VectorValues d = linear->hessianDiagonal();
    VectorValues sqrtHessianDiagonal = d;
    for (Vector& v : sqrtHessianDiagonal | map_values) v = v.cwiseSqrt();
    GaussianFactorGraph damped = optimizer.buildDampedSystem(*linear, sqrtHessianDiagonal);
    VectorValues  expectedDiagonal = d + params.lambdaInitial * d;
    EXPECT(assert_equal(expectedDiagonal, damped.hessianDiagonal()));

    // test convergence (does not!)
    Values actual = optimizer.optimize();
    EXPECT(assert_equal(expected, actual));

    // Check that the gradient is zero (it is not!)
    linear = optimizer.linearize();
    EXPECT(assert_equal(expectedGradient,linear->gradientAtZero()));

    // Check that the gradient is zero for damped system (it is not!)
    damped = optimizer.buildDampedSystem(*linear, sqrtHessianDiagonal);
    VectorValues actualGradient = damped.gradientAtZero();
    EXPECT(assert_equal(expectedGradient,actualGradient));

    /* This block was made to test the original initial guess "init"
    // Check errors at convergence and errors in direction of gradient (decreases!)
    EXPECT_DOUBLES_EQUAL(46.02558,fg.error(actual),1e-5);
    EXPECT_DOUBLES_EQUAL(44.742237,fg.error(actual.retract(-0.01*actualGradient)),1e-5);

    // Check that solve yields gradient (it's not equal to gradient, as predicted)
    VectorValues delta = damped.optimize();
    double factor = actualGradient[0][0]/delta[0][0];
    EXPECT(assert_equal(actualGradient,factor*delta));

    // Still pointing downhill wrt actual gradient !
    EXPECT_DOUBLES_EQUAL( 0.1056157,dot(-1*actualGradient,delta),1e-3);

    // delta.print("This is the delta value computed by LM with diagonal damping");

    // Still pointing downhill wrt expected gradient (IT DOES NOT! actually they are perpendicular)
    EXPECT_DOUBLES_EQUAL( 0.0,dot(-1*expectedGradient,delta),1e-5);

    // Check errors at convergence and errors in direction of solution (does not decrease!)
    EXPECT_DOUBLES_EQUAL(46.0254859,fg.error(actual.retract(delta)),1e-5);

    // Check errors at convergence and errors at a small step in direction of solution (does not decrease!)
    EXPECT_DOUBLES_EQUAL(46.0255,fg.error(actual.retract(0.01*delta)),1e-3);
    */
  }
}

/* ************************************************************************* */
TEST(NonlinearOptimizer, Pose2OptimizationWithHuberNoOutlier) {

  NonlinearFactorGraph fg;
  fg.addPrior(0, Pose2(0,0,0), noiseModel::Isotropic::Sigma(3,1));
  fg += BetweenFactor<Pose2>(0, 1, Pose2(1,1.1,M_PI/4),
                              noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(2.0),
                                                         noiseModel::Isotropic::Sigma(3,1)));
  fg += BetweenFactor<Pose2>(0, 1, Pose2(1,0.9,M_PI/2),
                              noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(3.0),
                                                         noiseModel::Isotropic::Sigma(3,1)));

  Values init;
  init.insert(0, Pose2(0,0,0));
  init.insert(1, Pose2(0.961187, 0.99965, 1.1781));

  Values expected;
  expected.insert(0, Pose2(0,0,0));
  expected.insert(1, Pose2(0.961187, 0.99965, 1.1781));

  LevenbergMarquardtParams lm_params;

  auto gn_result = GaussNewtonOptimizer(fg, init).optimize();
  auto lm_result = LevenbergMarquardtOptimizer(fg, init, lm_params).optimize();
  auto dl_result = DoglegOptimizer(fg, init).optimize();

  EXPECT(assert_equal(expected, gn_result, 3e-2));
  EXPECT(assert_equal(expected, lm_result, 3e-2));
  EXPECT(assert_equal(expected, dl_result, 3e-2));
}

/* ************************************************************************* */
TEST(NonlinearOptimizer, Point2LinearOptimizationWithHuber) {

  NonlinearFactorGraph fg;
  fg.addPrior(0, Point2(0,0), noiseModel::Isotropic::Sigma(2,0.01));
  fg += BetweenFactor<Point2>(0, 1, Point2(1,1.8),
                              noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(1.0),
                                                         noiseModel::Isotropic::Sigma(2,1)));
  fg += BetweenFactor<Point2>(0, 1, Point2(1,0.9),
                              noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(1.0),
                                                         noiseModel::Isotropic::Sigma(2,1)));
  fg += BetweenFactor<Point2>(0, 1, Point2(1,90),
                              noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(1.0),
                                                         noiseModel::Isotropic::Sigma(2,1)));

  Values init;
  init.insert(0, Point2(1,1));
  init.insert(1, Point2(1,0));

  Values expected;
  expected.insert(0, Point2(0,0));
  expected.insert(1, Point2(1,1.85));

  LevenbergMarquardtParams params;

  auto gn_result = GaussNewtonOptimizer(fg, init).optimize();
  auto lm_result = LevenbergMarquardtOptimizer(fg, init, params).optimize();
  auto dl_result = DoglegOptimizer(fg, init).optimize();

  EXPECT(assert_equal(expected, gn_result, 1e-4));
  EXPECT(assert_equal(expected, lm_result, 1e-4));
  EXPECT(assert_equal(expected, dl_result, 1e-4));
}

/* ************************************************************************* */
TEST(NonlinearOptimizer, Pose2OptimizationWithHuber) {

  NonlinearFactorGraph fg;
  fg.addPrior(0, Pose2(0,0, 0), noiseModel::Isotropic::Sigma(3,0.1));
  fg += BetweenFactor<Pose2>(0, 1, Pose2(0,9, M_PI/2),
                              noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(0.2),
                                                         noiseModel::Isotropic::Sigma(3,1)));
  fg += BetweenFactor<Pose2>(0, 1, Pose2(0, 11, M_PI/2),
                              noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(0.2),
                                                         noiseModel::Isotropic::Sigma(3,1)));
  fg += BetweenFactor<Pose2>(0, 1, Pose2(0, 10, M_PI/2),
                             noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(0.2),
                                                        noiseModel::Isotropic::Sigma(3,1)));
  fg += BetweenFactor<Pose2>(0, 1, Pose2(0,9, 0),
                              noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(0.2),
                                                         noiseModel::Isotropic::Sigma(3,1)));

  Values init;
  init.insert(0, Pose2(0, 0, 0));
  init.insert(1, Pose2(0, 10, M_PI/4));

  Values expected;
  expected.insert(0, Pose2(0, 0, 0));
  expected.insert(1, Pose2(0, 10, 1.45212));

  LevenbergMarquardtParams params;

  auto gn_result = GaussNewtonOptimizer(fg, init).optimize();
  auto lm_result = LevenbergMarquardtOptimizer(fg, init, params).optimize();
  auto dl_result = DoglegOptimizer(fg, init).optimize();

  EXPECT(assert_equal(expected, gn_result, 1e-1));
  EXPECT(assert_equal(expected, lm_result, 1e-1));
  EXPECT(assert_equal(expected, dl_result, 1e-1));
}

/* ************************************************************************* */
TEST(NonlinearOptimizer, RobustMeanCalculation) {

  NonlinearFactorGraph fg;

  Values init;

  Values expected;

  auto huber = noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(20),
                                          noiseModel::Isotropic::Sigma(1, 1));

  vector<double> pts{-10,-3,-1,1,3,10,1000};
  for(auto pt : pts) {
    fg.addPrior(0, pt, huber);
  }

  init.insert(0, 100.0);
  expected.insert(0, 3.33333333);

  DoglegParams params_dl;
  params_dl.setRelativeErrorTol(1e-10);

  auto gn_result = GaussNewtonOptimizer(fg, init).optimize();
  auto lm_result = LevenbergMarquardtOptimizer(fg, init).optimize();
  auto dl_result = DoglegOptimizer(fg, init, params_dl).optimize();

  EXPECT(assert_equal(expected, gn_result, tol));
  EXPECT(assert_equal(expected, lm_result, tol));
  EXPECT(assert_equal(expected, dl_result, tol));
}

/* ************************************************************************* */
TEST(NonlinearOptimizer, disconnected_graph) {
  Values expected;
  expected.insert(X(1), Pose2(0.,0.,0.));
  expected.insert(X(2), Pose2(1.5,0.,0.));
  expected.insert(X(3), Pose2(3.0,0.,0.));

  Values init;
  init.insert(X(1), Pose2(0.,0.,0.));
  init.insert(X(2), Pose2(0.,0.,0.));
  init.insert(X(3), Pose2(0.,0.,0.));

  NonlinearFactorGraph graph;
  graph.addPrior(X(1), Pose2(0.,0.,0.), noiseModel::Isotropic::Sigma(3,1));
  graph += BetweenFactor<Pose2>(X(1),X(2), Pose2(1.5,0.,0.), noiseModel::Isotropic::Sigma(3,1));
  graph.addPrior(X(3), Pose2(3.,0.,0.), noiseModel::Isotropic::Sigma(3,1));

  EXPECT(assert_equal(expected, LevenbergMarquardtOptimizer(graph, init).optimize()));
}

/* ************************************************************************* */
#include <gtsam/linear/iterative.h>

class IterativeLM : public LevenbergMarquardtOptimizer {
  /// Solver specific parameters
  ConjugateGradientParameters cgParams_;
  Values initial_;

 public:
  /// Constructor
  IterativeLM(const NonlinearFactorGraph& graph, const Values& initialValues,
              const ConjugateGradientParameters& p,
              const LevenbergMarquardtParams& params =
                  LevenbergMarquardtParams::LegacyDefaults())
      : LevenbergMarquardtOptimizer(graph, initialValues, params),
        cgParams_(p),
        initial_(initialValues) {}

  /// Solve that uses conjugate gradient
  VectorValues solve(const GaussianFactorGraph& gfg,
                             const NonlinearOptimizerParams& params) const override {
    VectorValues zeros = initial_.zeroVectors();
    return conjugateGradientDescent(gfg, zeros, cgParams_);
  }
};

/* ************************************************************************* */
TEST(NonlinearOptimizer, subclass_solver) {
  Values expected;
  expected.insert(X(1), Pose2(0., 0., 0.));
  expected.insert(X(2), Pose2(1.5, 0., 0.));
  expected.insert(X(3), Pose2(3.0, 0., 0.));

  Values init;
  init.insert(X(1), Pose2(0., 0., 0.));
  init.insert(X(2), Pose2(0., 0., 0.));
  init.insert(X(3), Pose2(0., 0., 0.));

  NonlinearFactorGraph graph;
  graph.addPrior(X(1), Pose2(0., 0., 0.), noiseModel::Isotropic::Sigma(3, 1));
  graph += BetweenFactor<Pose2>(X(1), X(2), Pose2(1.5, 0., 0.),
                                noiseModel::Isotropic::Sigma(3, 1));
  graph.addPrior(X(3), Pose2(3., 0., 0.), noiseModel::Isotropic::Sigma(3, 1));

  ConjugateGradientParameters p;
  Values actual = IterativeLM(graph, init, p).optimize();
  EXPECT(assert_equal(expected, actual, 1e-4));
}

/* ************************************************************************* */
TEST( NonlinearOptimizer, logfile )
{
  auto fg = example::createReallyNonlinearFactorGraph();

  Point2 x0(3,3);
  Values c0;
  c0.insert(X(1), x0);

  // Levenberg-Marquardt
  LevenbergMarquardtParams lmParams;
  static const string filename("testNonlinearOptimizer.log");
  lmParams.logFile = filename;
  LevenbergMarquardtOptimizer(fg, c0, lmParams).optimize();

//  stringstream expected,actual;
//  ifstream ifs(("../../gtsam/tests/" + filename).c_str());
//  if(!ifs) throw std::runtime_error(filename);
//  expected << ifs.rdbuf();
//  ifs.close();
//  ifstream ifs2(filename.c_str());
//  if(!ifs2) throw std::runtime_error(filename);
//  actual << ifs2.rdbuf();
//  EXPECT(actual.str()==expected.str());
}

/* ************************************************************************* */
TEST( NonlinearOptimizer, iterationHook_LM )
{
  NonlinearFactorGraph fg(example::createReallyNonlinearFactorGraph());

  Point2 x0(3,3);
  Values c0;
  c0.insert(X(1), x0);

  // Levenberg-Marquardt
  LevenbergMarquardtParams lmParams;
  size_t lastIterCalled = 0;
  // lmParams.iterationHook = [&](size_t iteration, double oldError, double newError)
  // {
  //   // Tests:
  //   lastIterCalled = iteration;
  //   EXPECT(newError<oldError);
    
  //   // Example of evolution printout:
  //   //std::cout << "iter: " << iteration << " error: " << oldError << " => " << newError <<"\n";
  // };
  LevenbergMarquardtOptimizer(fg, c0, lmParams).optimize();
  
  // EXPECT(lastIterCalled>5);
}
/* ************************************************************************* */
TEST( NonlinearOptimizer, iterationHook_CG )
{
  NonlinearFactorGraph fg(example::createReallyNonlinearFactorGraph());

  Point2 x0(3,3);
  Values c0;
  c0.insert(X(1), x0);

  // Levenberg-Marquardt
  NonlinearConjugateGradientOptimizer::Parameters cgParams;
  size_t lastIterCalled = 0;
  // cgParams.iterationHook = [&](size_t iteration, double oldError, double newError)
  // {
  //   // Tests:
  //   lastIterCalled = iteration;
  //   EXPECT(newError<oldError);
    
  //   // Example of evolution printout:
  //   //std::cout << "iter: " << iteration << " error: " << oldError << " => " << newError <<"\n";
  // };
  NonlinearConjugateGradientOptimizer(fg, c0, cgParams).optimize();
  
  // EXPECT(lastIterCalled>5);
}


/* ************************************************************************* */
//// Minimal traits example
struct MyType : public Vector3 {
  using Vector3::Vector3;
};

namespace gtsam {
template <>
struct traits<MyType> {
  static bool Equals(const MyType& a, const MyType& b, double tol) {
    return (a - b).array().abs().maxCoeff() < tol;
  }
  static void Print(const MyType&, const string&) {}
  static int GetDimension(const MyType&) { return 3; }
  static MyType Retract(const MyType& a, const Vector3& b) { return a + b; }
  static Vector3 Local(const MyType& a, const MyType& b) { return b - a; }
};
}

TEST(NonlinearOptimizer, Traits) {
  NonlinearFactorGraph fg;
  fg.addPrior(0, MyType(0, 0, 0), noiseModel::Isotropic::Sigma(3, 1));

  Values init;
  init.insert(0, MyType(0,0,0));

  LevenbergMarquardtOptimizer optimizer(fg, init);
  Values actual = optimizer.optimize();
  EXPECT(assert_equal(init, actual));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

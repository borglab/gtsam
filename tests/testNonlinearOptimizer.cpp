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
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/base/Matrix.h>

#include <CppUnitLite/TestHarness.h>

#include <boost/shared_ptr.hpp>
#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include <iostream>

using namespace std;
using namespace gtsam;

const double tol = 1e-5;

using symbol_shorthand::X;
using symbol_shorthand::L;

/* ************************************************************************* */
TEST( NonlinearOptimizer, iterateLM )
{
	// really non-linear factor graph
  example::Graph fg(example::createReallyNonlinearFactorGraph());

	// config far from minimum
	Point2 x0(3,0);
	Values config;
	config.insert(X(1), x0);

	// normal iterate
	GaussNewtonParams gnParams;
	GaussNewtonOptimizer gnOptimizer(fg, config, gnParams);
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
  example::Graph fg(example::createReallyNonlinearFactorGraph());

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
	example::Graph fg(example::createReallyNonlinearFactorGraph());

	Point2 x0(3,3);
	Values c0;
	c0.insert(X(1), x0);

	Values actual = LevenbergMarquardtOptimizer(fg, c0).optimize();
	DOUBLES_EQUAL(0,fg.error(actual),tol);
}

/* ************************************************************************* */
TEST( NonlinearOptimizer, SimpleGNOptimizer )
{
  example::Graph fg(example::createReallyNonlinearFactorGraph());

  Point2 x0(3,3);
  Values c0;
  c0.insert(X(1), x0);

  Values actual = GaussNewtonOptimizer(fg, c0).optimize();
	DOUBLES_EQUAL(0,fg.error(actual),tol);
}

/* ************************************************************************* */
TEST( NonlinearOptimizer, SimpleDLOptimizer )
{
  example::Graph fg(example::createReallyNonlinearFactorGraph());

  Point2 x0(3,3);
  Values c0;
  c0.insert(X(1), x0);

  Values actual = DoglegOptimizer(fg, c0).optimize();
  DOUBLES_EQUAL(0,fg.error(actual),tol);
}

/* ************************************************************************* */
TEST( NonlinearOptimizer, optimization_method )
{
  LevenbergMarquardtParams paramsQR;
  paramsQR.linearSolverType = LevenbergMarquardtParams::MULTIFRONTAL_QR;
  LevenbergMarquardtParams paramsChol;
  paramsChol.linearSolverType = LevenbergMarquardtParams::MULTIFRONTAL_CHOLESKY;

	example::Graph fg = example::createReallyNonlinearFactorGraph();

	Point2 x0(3,3);
	Values c0;
	c0.insert(X(1), x0);

	Values actualMFQR = LevenbergMarquardtOptimizer(fg, c0, paramsQR).optimize();
	DOUBLES_EQUAL(0,fg.error(actualMFQR),tol);

  Values actualMFChol = LevenbergMarquardtOptimizer(fg, c0, paramsChol).optimize();
  DOUBLES_EQUAL(0,fg.error(actualMFChol),tol);
}

/* ************************************************************************* */
TEST( NonlinearOptimizer, Factorization )
{
	Values config;
	config.insert(X(1), Pose2(0.,0.,0.));
	config.insert(X(2), Pose2(1.5,0.,0.));

	NonlinearFactorGraph graph;
	graph.add(PriorFactor<Pose2>(X(1), Pose2(0.,0.,0.), noiseModel::Isotropic::Sigma(3, 1e-10)));
	graph.add(BetweenFactor<Pose2>(X(1),X(2), Pose2(1.,0.,0.), noiseModel::Isotropic::Sigma(3, 1)));

	Ordering ordering;
	ordering.push_back(X(1));
	ordering.push_back(X(2));

	LevenbergMarquardtOptimizer optimizer(graph, config, ordering);
	optimizer.iterate();

	Values expected;
	expected.insert(X(1), Pose2(0.,0.,0.));
	expected.insert(X(2), Pose2(1.,0.,0.));
	CHECK(assert_equal(expected, optimizer.values(), 1e-5));
}

/* ************************************************************************* */
TEST(NonlinearOptimizer, NullFactor) {

  example::Graph fg = example::createReallyNonlinearFactorGraph();

  // Add null factor
  fg.push_back(example::Graph::sharedFactor());

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

/* ************************************************************************* */
TEST(NonlinearOptimizer, MoreOptimization) {

  NonlinearFactorGraph fg;
  fg.add(PriorFactor<Pose2>(0, Pose2(0,0,0), noiseModel::Isotropic::Sigma(3,1)));
  fg.add(BetweenFactor<Pose2>(0, 1, Pose2(1,0,M_PI/2), noiseModel::Isotropic::Sigma(3,1)));
  fg.add(BetweenFactor<Pose2>(1, 2, Pose2(1,0,M_PI/2), noiseModel::Isotropic::Sigma(3,1)));

  Values init;
  init.insert(0, Pose2(3,4,-M_PI));
  init.insert(1, Pose2(10,2,-M_PI));
  init.insert(2, Pose2(11,7,-M_PI));

  Values expected;
  expected.insert(0, Pose2(0,0,0));
  expected.insert(1, Pose2(1,0,M_PI/2));
  expected.insert(2, Pose2(1,1,M_PI));

  // Try LM and Dogleg
  EXPECT(assert_equal(expected, LevenbergMarquardtOptimizer(fg, init).optimize()));
  EXPECT(assert_equal(expected, DoglegOptimizer(fg, init).optimize()));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

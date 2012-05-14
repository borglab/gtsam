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

#include <iostream>
using namespace std;

#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>

#include <boost/shared_ptr.hpp>
using namespace boost;

#include <gtsam/base/Matrix.h>
#include <gtsam/slam/smallExample.h>
#include <gtsam/slam/pose2SLAM.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>

using namespace gtsam;

const double tol = 1e-5;

Key kx(size_t i) { return Symbol('x',i); }
Key kl(size_t i) { return Symbol('l',i); }

/* ************************************************************************* */
TEST( NonlinearOptimizer, iterateLM )
{
	// really non-linear factor graph
  example::Graph fg(example::createReallyNonlinearFactorGraph());

	// config far from minimum
	Point2 x0(3,0);
	Values config;
	config.insert(simulated2D::PoseKey(1), x0);

	// normal iterate
	GaussNewtonOptimizer gnOptimizer(fg, config);
	gnOptimizer.iterate();

	// LM iterate with lambda 0 should be the same
	LevenbergMarquardtOptimizer lmOptimizer(fg, config);
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
	cstar.insert(simulated2D::PoseKey(1), xstar);
	DOUBLES_EQUAL(0.0,fg.error(cstar),0.0);

	// test error at initial = [(1-cos(3))^2 + (sin(3))^2]*50 =
	Point2 x0(3,3);
	Values c0;
	c0.insert(simulated2D::PoseKey(1), x0);
	DOUBLES_EQUAL(199.0,fg.error(c0),1e-3);

	// Gauss-Newton
	Values actual1 = GaussNewtonOptimizer(fg, c0).optimize();
	DOUBLES_EQUAL(0,fg.error(actual1),tol);

	// Levenberg-Marquardt
  Values actual2 = LevenbergMarquardtOptimizer(fg, c0).optimize();
  DOUBLES_EQUAL(0,fg.error(actual2),tol);

  // Dogleg
  Values actual3 = DoglegOptimizer(fg, c0).optimize();
  DOUBLES_EQUAL(0,fg.error(actual3),tol);
}

/* ************************************************************************* */
TEST( NonlinearOptimizer, SimpleLMOptimizer )
{
	example::Graph fg(example::createReallyNonlinearFactorGraph());

	Point2 x0(3,3);
	Values c0;
	c0.insert(simulated2D::PoseKey(1), x0);

	Values actual = LevenbergMarquardtOptimizer(fg, c0).optimize();
	DOUBLES_EQUAL(0,fg.error(actual),tol);
}

/* ************************************************************************* */
TEST( NonlinearOptimizer, SimpleGNOptimizer )
{
  example::Graph fg(example::createReallyNonlinearFactorGraph());

  Point2 x0(3,3);
  Values c0;
  c0.insert(simulated2D::PoseKey(1), x0);

  Values actual = GaussNewtonOptimizer(fg, c0).optimize();
	DOUBLES_EQUAL(0,fg.error(actual),tol);
}

/* ************************************************************************* */
TEST( NonlinearOptimizer, SimpleDLOptimizer )
{
  example::Graph fg(example::createReallyNonlinearFactorGraph());

  Point2 x0(3,3);
  Values c0;
  c0.insert(simulated2D::PoseKey(1), x0);

  Values actual = DoglegOptimizer(fg, c0).optimize();
  DOUBLES_EQUAL(0,fg.error(actual),tol);
}

/* ************************************************************************* */
TEST( NonlinearOptimizer, optimization_method )
{
  LevenbergMarquardtParams paramsQR;
  paramsQR.factorization = LevenbergMarquardtParams::QR;
  LevenbergMarquardtParams paramsLDL;
  paramsLDL.factorization = LevenbergMarquardtParams::LDL;

	example::Graph fg = example::createReallyNonlinearFactorGraph();

	Point2 x0(3,3);
	Values c0;
	c0.insert(simulated2D::PoseKey(1), x0);

	Values actualMFQR = LevenbergMarquardtOptimizer(fg, c0, paramsQR).optimize();
	DOUBLES_EQUAL(0,fg.error(actualMFQR),tol);

	Values actualMFLDL = LevenbergMarquardtOptimizer(fg, c0, paramsLDL).optimize();
	DOUBLES_EQUAL(0,fg.error(actualMFLDL),tol);
}

/* ************************************************************************* */
TEST( NonlinearOptimizer, Factorization )
{
	Values config;
	config.insert(pose2SLAM::PoseKey(1), Pose2(0.,0.,0.));
	config.insert(pose2SLAM::PoseKey(2), Pose2(1.5,0.,0.));

	pose2SLAM::Graph graph;
	graph.addPrior(1, Pose2(0.,0.,0.), noiseModel::Isotropic::Sigma(3, 1e-10));
	graph.addOdometry(1,2, Pose2(1.,0.,0.), noiseModel::Isotropic::Sigma(3, 1));

	Ordering ordering;
	ordering.push_back(pose2SLAM::PoseKey(1));
	ordering.push_back(pose2SLAM::PoseKey(2));

	LevenbergMarquardtOptimizer optimizer(graph, config, ordering);
	optimizer.iterate();

	Values expected;
	expected.insert(pose2SLAM::PoseKey(1), Pose2(0.,0.,0.));
	expected.insert(pose2SLAM::PoseKey(2), Pose2(1.,0.,0.));
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
  cstar.insert(simulated2D::PoseKey(1), xstar);
  DOUBLES_EQUAL(0.0,fg.error(cstar),0.0);

  // test error at initial = [(1-cos(3))^2 + (sin(3))^2]*50 =
  Point2 x0(3,3);
  Values c0;
  c0.insert(simulated2D::PoseKey(1), x0);
  DOUBLES_EQUAL(199.0,fg.error(c0),1e-3);

  // optimize parameters
  Ordering ord;
  ord.push_back(kx(1));

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
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

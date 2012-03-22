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

// template definitions
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>

using namespace gtsam;

const double tol = 1e-5;

Key kx(size_t i) { return Symbol('x',i); }
Key kl(size_t i) { return Symbol('l',i); }

/* ************************************************************************* */
TEST( NonlinearOptimizer, iterateLM )
{
	// really non-linear factor graph
  shared_ptr<example::Graph> fg(new example::Graph(
  		example::createReallyNonlinearFactorGraph()));

	// config far from minimum
	Point2 x0(3,0);
	boost::shared_ptr<Values> config(new Values);
	config->insert(simulated2D::PoseKey(1), x0);

	// ordering
	shared_ptr<Ordering> ord(new Ordering());
	ord->push_back(kx(1));

	// create initial optimization state, with lambda=0
	NonlinearOptimizer::auto_ptr optimizer = LevenbergMarquardtOptimizer(fg, config, LevenbergMarquardtParams(), ord).update(0.0);

	// normal iterate
	NonlinearOptimizer::auto_ptr iterated1 = GaussNewtonOptimizer(fg, config, GaussNewtonParams(), ord).iterate();

	// LM iterate with lambda 0 should be the same
	NonlinearOptimizer::auto_ptr iterated2 = LevenbergMarquardtOptimizer(fg, config, LevenbergMarquardtParams(), ord).update(0.0)->iterate();

	CHECK(assert_equal(*iterated1->values(), *iterated2->values(), 1e-9));
}

/* ************************************************************************* */
TEST( NonlinearOptimizer, optimize )
{
  shared_ptr<example::Graph> fg(new example::Graph(
  		example::createReallyNonlinearFactorGraph()));

	// test error at minimum
	Point2 xstar(0,0);
	Values cstar;
	cstar.insert(simulated2D::PoseKey(1), xstar);
	DOUBLES_EQUAL(0.0,fg->error(cstar),0.0);

	// test error at initial = [(1-cos(3))^2 + (sin(3))^2]*50 =
	Point2 x0(3,3);
	boost::shared_ptr<Values> c0(new Values);
	c0->insert(simulated2D::PoseKey(1), x0);
	DOUBLES_EQUAL(199.0,fg->error(*c0),1e-3);

	// optimize parameters
	shared_ptr<Ordering> ord(new Ordering());
	ord->push_back(kx(1));

	// Gauss-Newton
	NonlinearOptimizer::auto_ptr actual1 = GaussNewtonOptimizer(fg, c0, GaussNewtonParams(), ord).optimize();
	DOUBLES_EQUAL(0,fg->error(*(actual1->values())),tol);

	// Levenberg-Marquardt
	NonlinearOptimizer::auto_ptr actual2 = LevenbergMarquardtOptimizer(fg, c0, LevenbergMarquardtParams(), ord).optimize();
	DOUBLES_EQUAL(0,fg->error(*(actual2->values())),tol);
}

/* ************************************************************************* */
TEST( NonlinearOptimizer, SimpleLMOptimizer )
{
	shared_ptr<example::Graph> fg(new example::Graph(
			example::createReallyNonlinearFactorGraph()));

	Point2 x0(3,3);
	boost::shared_ptr<Values> c0(new Values);
	c0->insert(simulated2D::PoseKey(1), x0);

	Values::const_shared_ptr actual = LevenbergMarquardtOptimizer(fg, c0).optimize()->values();
	DOUBLES_EQUAL(0,fg->error(*actual),tol);
}

/* ************************************************************************* */
TEST( NonlinearOptimizer, SimpleLMOptimizer_noshared )
{
	example::Graph fg = example::createReallyNonlinearFactorGraph();

	Point2 x0(3,3);
	Values c0;
	c0.insert(simulated2D::PoseKey(1), x0);

  Values::const_shared_ptr actual = LevenbergMarquardtOptimizer(fg, c0).optimize()->values();
	DOUBLES_EQUAL(0,fg.error(*actual),tol);
}

/* ************************************************************************* */
TEST( NonlinearOptimizer, SimpleGNOptimizer )
{
	shared_ptr<example::Graph> fg(new example::Graph(
			example::createReallyNonlinearFactorGraph()));

	Point2 x0(3,3);
	boost::shared_ptr<Values> c0(new Values);
	c0->insert(simulated2D::PoseKey(1), x0);

  Values::const_shared_ptr actual = GaussNewtonOptimizer(fg, c0).optimize()->values();
	DOUBLES_EQUAL(0,fg->error(*actual),tol);
}

/* ************************************************************************* */
TEST( NonlinearOptimizer, SimpleGNOptimizer_noshared )
{
	example::Graph fg = example::createReallyNonlinearFactorGraph();

	Point2 x0(3,3);
	Values c0;
	c0.insert(simulated2D::PoseKey(1), x0);

  Values::const_shared_ptr actual = GaussNewtonOptimizer(fg, c0).optimize()->values();
	DOUBLES_EQUAL(0,fg.error(*actual),tol);
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

	Values actualMFQR = *LevenbergMarquardtOptimizer(fg, c0, paramsQR).optimize()->values();
	DOUBLES_EQUAL(0,fg.error(actualMFQR),tol);

	Values actualMFLDL = *LevenbergMarquardtOptimizer(fg, c0, paramsLDL).optimize()->values();
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

	NonlinearOptimizer::auto_ptr optimized = LevenbergMarquardtOptimizer(graph, config, ordering).iterate();

	Values expected;
	expected.insert(pose2SLAM::PoseKey(1), Pose2(0.,0.,0.));
	expected.insert(pose2SLAM::PoseKey(2), Pose2(1.,0.,0.));
	CHECK(assert_equal(expected, *optimized->values(), 1e-5));
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
  NonlinearOptimizer::auto_ptr actual1 = GaussNewtonOptimizer(fg, c0, ord).optimize();
  DOUBLES_EQUAL(0,fg.error(*actual1->values()),tol);

  // Levenberg-Marquardt
  NonlinearOptimizer::auto_ptr actual2 = LevenbergMarquardtOptimizer(fg, c0, ord).optimize();
  DOUBLES_EQUAL(0,fg.error(*actual2->values()),tol);
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

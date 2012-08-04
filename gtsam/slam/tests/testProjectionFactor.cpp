/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  testProjectionFactor.cpp
 *  @brief Unit tests for ProjectionFactor Class
 *  @author Frank Dellaert
 *  @date Nov 2009
 */

#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

// make cube
static Point3
	x000(-1, -1, -1), x001(-1, -1, +1), x010(-1, +1, -1), x011(-1, +1, +1),
	x100(-1, -1, -1), x101(-1, -1, +1), x110(-1, +1, -1), x111(-1, +1, +1);

// make a realistic calibration matrix
static double fov = 60; // degrees
static size_t w=640,h=480;
static Cal3_S2 K(fov,w,h);

static SharedNoiseModel sigma(noiseModel::Unit::Create(1));
static shared_ptrK sK(new Cal3_S2(K));

// Convenience for named keys
using symbol_shorthand::X;
using symbol_shorthand::L;

typedef GenericProjectionFactor<Pose3, Point3> MyProjectionFactor;

/* ************************************************************************* */
TEST( ProjectionFactor, nonStandard )
{
  GenericProjectionFactor<Pose3, Point3, Cal3DS2> f;
}

/* ************************************************************************* */
TEST( ProjectionFactor, error )
  {
	// Create the factor with a measurement that is 3 pixels off in x
	Point2 z(323.,240.);
	int i=1, j=1;
	boost::shared_ptr<MyProjectionFactor>
	factor(new MyProjectionFactor(z, sigma, X(i), L(j), sK));

	// For the following values structure, the factor predicts 320,240
	Values config;
	Rot3 R;Point3 t1(0,0,-6); Pose3 x1(R,t1); config.insert(X(1), x1);
	Point3 l1;  config.insert(L(1), l1);
	// Point should project to Point2(320.,240.)
	CHECK(assert_equal(Vector_(2, -3.0, 0.0), factor->unwhitenedError(config)));

	// Which yields an error of 3^2/2 = 4.5
	DOUBLES_EQUAL(4.5,factor->error(config),1e-9);

	// Check linearize
	Ordering ordering; ordering += X(1),L(1);
  Matrix Ax1 = Matrix_(2, 6, 0., -554.256, 0., -92.376, 0., 0., 554.256, 0., 0., 0., -92.376, 0.);
	Matrix Al1 = Matrix_(2, 3, 92.376, 0., 0., 0., 92.376, 0.);
	Vector b = Vector_(2,3.,0.);
	SharedDiagonal probModel1 = noiseModel::Unit::Create(2);
	JacobianFactor expected(ordering[X(1)], Ax1, ordering[L(1)], Al1, b, probModel1);
	JacobianFactor::shared_ptr actual =
	    boost::dynamic_pointer_cast<JacobianFactor>(factor->linearize(config, ordering));
	CHECK(assert_equal(expected,*actual,1e-3));

	// linearize graph
	NonlinearFactorGraph graph;
	graph.push_back(factor);
	FactorGraph<GaussianFactor> expected_lfg;
	expected_lfg.push_back(actual);
	boost::shared_ptr<FactorGraph<GaussianFactor> > actual_lfg = graph.linearize(config, ordering);
	CHECK(assert_equal(expected_lfg,*actual_lfg));

	// expmap on a config
	Values expected_config;
  Point3 t2(1,1,-5); Pose3 x2(R,t2); expected_config.insert(X(1), x2);
  Point3 l2(1,2,3); expected_config.insert(L(1), l2);
	VectorValues delta(expected_config.dims(ordering));
	delta[ordering[X(1)]] = Vector_(6, 0.,0.,0., 1.,1.,1.);
	delta[ordering[L(1)]] = Vector_(3, 1.,2.,3.);
	Values actual_config = config.retract(delta, ordering);
	CHECK(assert_equal(expected_config,actual_config,1e-9));
}

/* ************************************************************************* */
TEST( ProjectionFactor, equals )
{
	// Create two identical factors and make sure they're equal
	Vector z = Vector_(2,323.,240.);
	int i=1, j=1;
	boost::shared_ptr<MyProjectionFactor>
	  factor1(new MyProjectionFactor(z, sigma, X(i), L(j), sK));

	boost::shared_ptr<MyProjectionFactor>
		factor2(new MyProjectionFactor(z, sigma, X(i), L(j), sK));

	CHECK(assert_equal(*factor1, *factor2));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */


/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**********************************************************
 Written by Frank Dellaert, Nov 2009
 **********************************************************/

#include <CppUnitLite/TestHarness.h>

#define GTSAM_MAGIC_KEY

#include <gtsam/slam/visualSLAM.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>

using namespace std;
using namespace gtsam;
using namespace gtsam::visualSLAM;

// make cube
static Point3
	x000(-1, -1, -1), x001(-1, -1, +1), x010(-1, +1, -1), x011(-1, +1, +1),
	x100(-1, -1, -1), x101(-1, -1, +1), x110(-1, +1, -1), x111(-1, +1, +1);

// make a realistic calibration matrix
static double fov = 60; // degrees
static size_t w=640,h=480;
static Cal3_S2 K(fov,w,h);

static SharedGaussian sigma(noiseModel::Unit::Create(1));
static shared_ptrK sK(new Cal3_S2(K));

// make cameras

/* ************************************************************************* */
TEST( ProjectionFactor, error )
{
	// Create the factor with a measurement that is 3 pixels off in x
	Point2 z(323.,240.);
	int cameraFrameNumber=1, landmarkNumber=1;
	boost::shared_ptr<ProjectionFactor>
	factor(new ProjectionFactor(z, sigma, cameraFrameNumber, landmarkNumber, sK));

	// For the following values structure, the factor predicts 320,240
	Values config;
	Rot3 R;Point3 t1(0,0,-6); Pose3 x1(R,t1); config.insert(1, x1);
	Point3 l1;  config.insert(1, l1);
	// Point should project to Point2(320.,240.)
	CHECK(assert_equal(Vector_(2, -3.0, 0.0), factor->unwhitenedError(config)));

	// Which yields an error of 3^2/2 = 4.5
	DOUBLES_EQUAL(4.5,factor->error(config),1e-9);

	// Check linearize
	Ordering ordering; ordering += "x1","l1";
  Matrix Ax1 = Matrix_(2, 6, 0., -369.504, 0., -61.584, 0., 0., 369.504, 0., 0., 0., -61.584, 0.);
	Matrix Al1 = Matrix_(2, 3, 61.584, 0., 0., 0., 61.584, 0.);
	Vector b = Vector_(2,3.,0.);
	SharedDiagonal probModel1 = noiseModel::Unit::Create(2);
	GaussianFactor expected(ordering["x1"], Ax1, ordering["l1"], Al1, b, probModel1);
	GaussianFactor::shared_ptr actual = factor->linearize(config, ordering);
	CHECK(assert_equal(expected,*actual,1e-3));

	// linearize graph
	Graph graph;
	graph.push_back(factor);
	GaussianFactorGraph expected_lfg;
	expected_lfg.push_back(actual);
	boost::shared_ptr<GaussianFactorGraph> actual_lfg = graph.linearize(config, ordering);
	CHECK(assert_equal(expected_lfg,*actual_lfg));

	// expmap on a config
  Values expected_config;
  Point3 t2(1,1,-5); Pose3 x2(R,t2); expected_config.insert(1, x2);
  Point3 l2(1,2,3); expected_config.insert(1, l2);
	VectorValues delta(expected_config.dims(ordering));
	delta[ordering["x1"]] = Vector_(6, 0.,0.,0., 1.,1.,1.);
	delta[ordering["l1"]] = Vector_(3, 1.,2.,3.);
	Values actual_config = config.expmap(delta, ordering);
	CHECK(assert_equal(expected_config,actual_config,1e-9));
}

/* ************************************************************************* */
TEST( ProjectionFactor, equals )
{
	// Create two identical factors and make sure they're equal
	Vector z = Vector_(2,323.,240.);
	int cameraFrameNumber=1, landmarkNumber=1;
	boost::shared_ptr<ProjectionFactor>
	  factor1(new ProjectionFactor(z, sigma, cameraFrameNumber, landmarkNumber, sK));

	boost::shared_ptr<ProjectionFactor>
		factor2(new ProjectionFactor(z, sigma, cameraFrameNumber, landmarkNumber, sK));

	CHECK(assert_equal(*factor1, *factor2));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */


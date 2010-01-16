/**********************************************************
 Written by Frank Dellaert, Nov 2009
 **********************************************************/

#include <CppUnitLite/TestHarness.h>

#include "visualSLAM.h"
#include "Point3.h"
#include "Pose3.h"

using namespace std;
using namespace gtsam;
using namespace gtsam::visualSLAM;

// make cube
Point3 x000(-1, -1, -1), x001(-1, -1, +1), x010(-1, +1, -1), x011(-1, +1, +1),
		x100(-1, -1, -1), x101(-1, -1, +1), x110(-1, +1, -1), x111(-1, +1, +1);

// make a realistic calibration matrix
double fov = 60; // degrees
size_t w=640,h=480;
Cal3_S2 K(fov,w,h);

// make cameras

/* ************************************************************************* */
TEST( ProjectionFactor, error )
{
	// Create the factor with a measurement that is 3 pixels off in x
	Point2 z(323.,240.);
	double sigma=1.0;
	int cameraFrameNumber=1, landmarkNumber=1;
	boost::shared_ptr<ProjectionFactor>
		factor(new ProjectionFactor(z, sigma, cameraFrameNumber, landmarkNumber, shared_ptrK(new Cal3_S2(K))));

  // For the following configuration, the factor predicts 320,240
  Config config;
  Rot3 R;Point3 t1(0,0,-6); Pose3 x1(R,t1); config.insert(1, x1);
  Point3 l1;  config.insert(1, l1);
  // Point should project to Point2(320.,240.)
  CHECK(assert_equal(Vector_(2, -3.0, 0.0), factor->error_vector(config)));

  // Which yields an error of 3^2/2 = 4.5
  DOUBLES_EQUAL(4.5,factor->error(config),1e-9);

  // Check linearize
  Matrix Al1 = Matrix_(2, 3, 61.584, 0., 0., 0., 61.584, 0.);
  Matrix Ax1 = Matrix_(2, 6, 0., -369.504, 0., -61.584, 0., 0., 369.504, 0., 0., 0., -61.584, 0.);
  Vector b = Vector_(2,3.,0.);
  GaussianFactor expected("l1", Al1, "x1", Ax1, b, 1);
  GaussianFactor::shared_ptr actual = factor->linearize(config);
  CHECK(assert_equal(expected,*actual,1e-3));

  // linearize graph
  Graph graph;
	graph.push_back(factor);
	GaussianFactorGraph expected_lfg;
	expected_lfg.push_back(actual);
	GaussianFactorGraph actual_lfg = graph.linearize(config);
	CHECK(assert_equal(expected_lfg,actual_lfg));

	// expmap on a config
	VectorConfig delta;
	delta.insert("x1",Vector_(6, 0.,0.,0., 1.,1.,1.));
	delta.insert("l1",Vector_(3, 1.,2.,3.));
	Config actual_config = expmap(config, delta);
  Config expected_config;
  Point3 t2(1,1,-5); Pose3 x2(R,t2); expected_config.insert(1, x2);
  Point3 l2(1,2,3); expected_config.insert(1, l2);
  CHECK(assert_equal(expected_config,actual_config,1e-9));
}

TEST( ProjectionFactor, equals )
{
	// Create two identical factors and make sure they're equal
	Vector z = Vector_(2,323.,240.);
	double sigma=1.0;
	int cameraFrameNumber=1, landmarkNumber=1;
	boost::shared_ptr<ProjectionFactor>
	  factor1(new ProjectionFactor(z, sigma, cameraFrameNumber, landmarkNumber, shared_ptrK(new Cal3_S2(K))));

	boost::shared_ptr<ProjectionFactor>
		factor2(new ProjectionFactor(z, sigma, cameraFrameNumber, landmarkNumber, shared_ptrK(new Cal3_S2(K))));

	CHECK(assert_equal(*factor1, *factor2));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	TestRegistry::runAllTests(tr);
	return 0;
}
/* ************************************************************************* */


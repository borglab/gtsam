/**********************************************************
 Written by Frank Dellaert, Nov 2009
 **********************************************************/

#include <CppUnitLite/TestHarness.h>

#include "VSLAMConfig.h"
#include "VSLAMFactor.h"
#include "VSLAMGraph.h"
#include "Point3.h"
#include "Pose3.h"

using namespace std;
using namespace gtsam;

// make cube
Point3 x000(-1, -1, -1), x001(-1, -1, +1), x010(-1, +1, -1), x011(-1, +1, +1),
		x100(-1, -1, -1), x101(-1, -1, +1), x110(-1, +1, -1), x111(-1, +1, +1);

// make a realistic calibration matrix
double fov = 60; // degrees
size_t w=640,h=480;
Cal3_S2 K(fov,w,h);

// make cameras

/* ************************************************************************* */
TEST( VSLAMFactor, error )
{
	// Create the factor with a measurement that is 3 pixels off in x
	Point2 z(323.,240.);
	double sigma=1.0;
	int cameraFrameNumber=1, landmarkNumber=1;
	boost::shared_ptr<VSLAMFactor>
		factor(new VSLAMFactor(z, sigma, cameraFrameNumber, landmarkNumber, VSLAMFactor::shared_ptrK(new Cal3_S2(K))));

  // For the following configuration, the factor predicts 320,240
  VSLAMConfig config;
  Rot3 R;Point3 t1(0,0,-6); Pose3 x1(R,t1); config.addCameraPose(1, x1);
  Point3 l1;  config.addLandmarkPoint(1, l1);
  CHECK(assert_equal(Vector_(2,320.,240.),factor->predict(config)));

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
  VSLAMGraph graph;
	graph.push_back(factor);
	GaussianFactorGraph expected_lfg;
	expected_lfg.push_back(actual);
	GaussianFactorGraph actual_lfg = graph.linearize(config);
	CHECK(assert_equal(expected_lfg,actual_lfg));

	// exmap on a config
	VectorConfig delta;
	delta.insert("x1",Vector_(6, 0.,0.,0., 1.,1.,1.));
	delta.insert("l1",Vector_(3, 1.,2.,3.));
	VSLAMConfig actual_config = config.exmap(delta);
  VSLAMConfig expected_config;
  Point3 t2(1,1,-5); Pose3 x2(R,t2); expected_config.addCameraPose(1, x2);
  Point3 l2(1,2,3); expected_config.addLandmarkPoint(1, l2);
  CHECK(assert_equal(expected_config,actual_config,1e-9));
}

TEST( VSLAMFactor, equals )
{
	// Create two identical factors and make sure they're equal
	Vector z = Vector_(2,323.,240.);
	double sigma=1.0;
	int cameraFrameNumber=1, landmarkNumber=1;
	boost::shared_ptr<VSLAMFactor>
	  factor1(new VSLAMFactor(z, sigma, cameraFrameNumber, landmarkNumber, VSLAMFactor::shared_ptrK(new Cal3_S2(K))));

	boost::shared_ptr<VSLAMFactor>
		factor2(new VSLAMFactor(z, sigma, cameraFrameNumber, landmarkNumber, VSLAMFactor::shared_ptrK(new Cal3_S2(K))));

	CHECK(assert_equal(*factor1, *factor2));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	TestRegistry::runAllTests(tr);
	return 0;
}
/* ************************************************************************* */


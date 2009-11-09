/**********************************************************
 Written by Frank Dellaert, Nov 2009
 **********************************************************/

#include <CppUnitLite/TestHarness.h>

#include "VSLAMFactor.h"
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
	Vector z = Vector_(2,323.,240.);
	double sigma=1.0;
	int cameraFrameNumber=1, landmarkNumber=1;
  VSLAMFactor factor(z, sigma, cameraFrameNumber, landmarkNumber, K);

  // For the following configuration, the factor predicts 320,240
  VectorConfig config;
  Rot3 R;Point3 t1(0,0,-6); Pose3 x1(R,t1); config.insert("x1",x1.vector());
  Point3 l1; config.insert("l1",l1.vector());
  CHECK(assert_equal(Vector_(2,320.,240.),factor.predict(config)));

  // Which yields an error of 3^2/2 = 4.5
  DOUBLES_EQUAL(4.5,factor.error(config),1e-9);

  // Check linearize
  Matrix Al1 = Matrix_(2, 3, 61.584, 0., 0., 0., 61.584, 0.);
  Matrix Ax1 = Matrix_(2, 6, 0., -369.504, 0., -61.584, 0., 0., 369.504, 0., 0., 0., -61.584, 0.);
  Vector b = Vector_(2,3.,0.);
  LinearFactor expected("l1", Al1, "x1", Ax1, b, 1);
  LinearFactor::shared_ptr actual = factor.linearize(config);
  CHECK(assert_equal(expected,*actual,1e-3));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	TestRegistry::runAllTests(tr);
	return 0;
}
/* ************************************************************************* */


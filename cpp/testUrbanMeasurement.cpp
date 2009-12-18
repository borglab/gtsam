/**********************************************************
 Written by Frank Dellaert, Dec 2009
 **********************************************************/

#include <CppUnitLite/TestHarness.h>

#include "UrbanConfig.h"
#include "UrbanMeasurement.h"
#include "Vector.h"

using namespace std;
using namespace gtsam;

Point2 landmark(2,5);

/* Robot is at (0,0,0) looking in global "y" direction.
 * For the local frame we used Navlab convention,
 * x-looks forward, y-looks right, z- down*/
Pose3 robotPose(Matrix_(3,3,
		       0., 1., 0.,
		       1., 0., 0.,
		       0., 0.,-1.
		       ),
	      Point3(0,0,0));

/* ************************************************************************* */
TEST( UrbanMeasurement, transform_to )
{
	Point2 expected(5,2);
	Point2 actual = transform_to(robotPose, landmark);
	CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST( UrbanMeasurement, error_vector )
{
	// Create a measurement, no-noise measurement would yield 5,2
	Point2 z(4,2);
	double sigma = 0.2;
	UrbanMeasurement factor(z,sigma,44,66);

	// Create a config
	UrbanConfig config;
	config.addRobotPose(44,robotPose);
	config.addLandmark(66,landmark);

	// test error_vector
	Vector expected = Vector_(2, -1.0, 0.0);
	Vector actual = factor.error_vector(config);
	CHECK(assert_equal(expected,actual));

	// test error
	double expected2 = 12.5; // 0.5 * 5^2
	double actual2 = factor.error(config);
	DOUBLES_EQUAL(expected2,actual2,1e-9);
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	TestRegistry::runAllTests(tr);
	return 0;
}
/* ************************************************************************* */


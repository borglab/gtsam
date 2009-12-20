/**********************************************************
 Written by Frank Dellaert, Dec 2009
 **********************************************************/

#include <CppUnitLite/TestHarness.h>

#include "UrbanConfig.h"
#include "UrbanMeasurement.h"
#include "Vector.h"

using namespace std;
using namespace gtsam;


/*
 * This unit test cross checks UrbanMeasurement with the
 * results from Justin's thesis codebase. 
 */

static boost::shared_ptr<Matrix> sensorMatrix(new Matrix(4, 4));

static bool _data_inited = false;

static void _init() {
	// Not MT-safe.  Don't think that's an issue for CppUnitLite
	if (_data_inited) {
		return;
	}
	(*sensorMatrix)(0, 0) = 0.0251;
	(*sensorMatrix)(0, 1) = 0.9996;
	(*sensorMatrix)(0, 2) = 0.0091;
	(*sensorMatrix)(0, 3) = 1.5574;
	(*sensorMatrix)(1, 0) = -0.9996;
	(*sensorMatrix)(1, 1) = 0.0253;
	(*sensorMatrix)(1, 2) = -0.016;
	(*sensorMatrix)(1, 3) = -0.86;
	(*sensorMatrix)(2, 0) = -0.0163;
	(*sensorMatrix)(2, 1) = -0.0087;
	(*sensorMatrix)(2, 2) = 0.9998;
	(*sensorMatrix)(2, 3) = -0.6478;
	(*sensorMatrix)(3, 0) = 0;
	(*sensorMatrix)(3, 1) = 0;
	(*sensorMatrix)(3, 2) = 0;
	(*sensorMatrix)(3, 3) = 1;
	_data_inited = true;

}
/* Robot is at (0,0,0) looking in global "y" direction.
 * For the local frame we used Navlab convention,
 * x-looks forward, y-looks right, z- down*/
static const Pose3 robotPose(Matrix_(4, 4, 
			       -0.0848808908061426,    0.993833753552749900, 0.0713421661796702400, 377.7509309858,
			       -0.9963883079644135,   -0.08449313834814816, -0.008440931458908668,  1922.5523404841,
			       -0.002360959078213238, -0.07180097402774338,  0.9974161849503438,   -6369417.591562273,
			        0.0, 0.0, 0.0, 1));

static const Point2 landmark(362.707949418, 1936.2843137418);




/* ************************************************************************* */
TEST( UrbanMeasurement, transform_to )
{
	_init();
	UrbanMeasurement::Transform tr = transform_to(sensorMatrix, robotPose, landmark, true);
	
	// Check the transformation from global to sensor coordinates
	CHECK(assert_equal(Point2(-12.40679724375481, -16.14944758846734),*(tr.get<0>())));

	Point2 &foo = *(tr.get<0>());
	
	// Check the jacobian w.r.t. the pose
	CHECK(assert_equal(Matrix_(2, 6,
 				    0.08471201853652961, 0.9964082882836194, 0.0, 0.03822695627705205, -0.5427133154140961, -16.15221468780303,
				   -0.998969460290058,   0.0851007754705408, 0.0, 0.5147498807397776,   0.0,                 12.43765251715327),
			   *(tr.get<1>())));
	
	// Check the jacobian w.r.t the landmark
	CHECK(assert_equal(Matrix_(2, 2,
				   -0.08471201853652961, -0.9964082882836194,
				   0.998969460290058,    -0.0851007754705408),
			   *(tr.get<2>())));
}

/* ************************************************************************* */
TEST( UrbanMeasurement, error_vector )
{
	_init();

	Point2 z(-12.42876033350471, -16.10191021850896);
	double sigma = 0.2;
	UrbanMeasurement factor(sensorMatrix, z,sigma,44,66);

	// Create a config
	UrbanConfig config;
	config.addRobotPose(44,robotPose);
	config.addLandmark(66,landmark);

	// test error_vector
	Vector expected = Vector_(2,
				  -.02196308974990, .04753736995838);
	Vector actual = factor.error_vector(config);
	CHECK(assert_equal(expected,actual));

	// test error.  Expected was generated via manual calculation from
	// error vector.
	double expected2 = .03427723560;
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


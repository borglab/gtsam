/**
 * @file   testPose2.cpp
 * @brief  Unit tests for Pose2 class
 */

#include <math.h>
#include <CppUnitLite/TestHarness.h>
#include "numericalDerivative.h"
#include "Pose2.h"
#include "Point2.h"

using namespace gtsam;

/* ************************************************************************* */
TEST(Pose2, constructors) {
	Point2 p;
	Pose2 pose(p,0);
	Pose2 origin;
	assert_equal(pose,origin);
}

/* ************************************************************************* */
TEST(Pose2, rotate) {
	double theta = 0.1, c=cos(theta),s=sin(theta);
	Pose2 p1(1,0,0.2), p2(0,1,0.4);
	CHECK(assert_equal(Pose2( c,s,0.3),p1.rotate(theta)));
	CHECK(assert_equal(Pose2(-s,c,0.5),p2.rotate(theta)));
}

/* ************************************************************************* */
TEST(Pose2, operators) {
	CHECK(assert_equal(Pose2(2,2,2),Pose2(1,1,1)+Pose2(1,1,1)));
	CHECK(assert_equal(Pose2(0,0,0),Pose2(1,1,1)-Pose2(1,1,1)));
}

/* ************************************************************************* */
TEST( Pose2, transform_to )
{
	Pose2 pose(1,2,M_PI_2); // robot at (1,2) looking towards y
	Point2 point(-1,4);    // landmark at (-1,4)

	// expected
	Point2 expected(2,2);
	Matrix expectedH1 = Matrix_(2,3, 0.0, -1.0, 2.0,  1.0, 0.0, -2.0);
	Matrix expectedH2 = Matrix_(2,2, 0.0, 1.0,  -1.0, 0.0);

	// actual
	Point2 actual = transform_to(pose,point);
	Matrix actualH1 = Dtransform_to1(pose,point);
	Matrix actualH2 = Dtransform_to2(pose,point);

	CHECK(assert_equal(expected,actual));
	CHECK(assert_equal(expectedH1,actualH1));
	CHECK(assert_equal(expectedH2,actualH2));

  Matrix numericalH1 = numericalDerivative21(transform_to, pose, point, 1e-5);
	CHECK(assert_equal(numericalH1,actualH1));

  Matrix numericalH2 = numericalDerivative22(transform_to, pose, point, 1e-5);
	CHECK(assert_equal(numericalH2,actualH2));
}

/* ************************************************************************* */
TEST( Pose2, between )
{
	Pose2 p1(1,2,M_PI_2); // robot at (1,2) looking towards y
	Pose2 p2(-1,4,M_PI);  // robot at (-1,4) loooking at negative x

	// expected
	Pose2 expected(2,2,M_PI_2);
	Matrix expectedH1 = Matrix_(3,3,
			0.0,-1.0,2.0,
			1.0,0.0,-2.0,
			0.0,0.0,-1.0
			);
	Matrix expectedH2 = Matrix_(3,3,
			 0.0,1.0,0.0,
			-1.0,0.0,0.0,
			 0.0,0.0,1.0
			 );

	// actual
	Pose2 actual = between(p1,p2);
	Matrix actualH1 = Dbetween1(p1,p2);
	Matrix actualH2 = Dbetween2(p1,p2);

	CHECK(assert_equal(expected,actual));
	CHECK(assert_equal(expectedH1,actualH1));
	CHECK(assert_equal(expectedH2,actualH2));

  Matrix numericalH1 = numericalDerivative21(between, p1, p2, 1e-5);
	CHECK(assert_equal(numericalH1,actualH1));

  Matrix numericalH2 = numericalDerivative22(between, p1, p2, 1e-5);
	CHECK(assert_equal(numericalH2,actualH2));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */


/**
 * Frank Dellaert
 * brief: test SimpleCamera class
 * based on testVSLAMFactor.cpp
 */

#include <math.h>
#include <iostream>

#include <CppUnitLite/TestHarness.h>
#include "numericalDerivative.h"
#include "SimpleCamera.h"

using namespace std;
using namespace gtsam;

const Cal3_S2 K(625, 625, 0, 0, 0);

const Pose3 pose1(Matrix_(3,3,
				      1., 0., 0.,
				      0.,-1., 0.,
				      0., 0.,-1.
				      ),
			      Point3(0,0,0.5));
 
const SimpleCamera camera(K, pose1);

const Point3 point1(-0.08,-0.08, 0.0);
const Point3 point2(-0.08, 0.08, 0.0);
const Point3 point3( 0.08, 0.08, 0.0);
const Point3 point4( 0.08,-0.08, 0.0);

/* ************************************************************************* */
TEST( SimpleCamera, constructor)
{
  CHECK(assert_equal( camera.calibration(), K));
  CHECK(assert_equal( camera.pose(), pose1));
}

/* ************************************************************************* */
TEST( SimpleCamera, level2)
{
	// Create a level camera, looking in Y-direction
	Pose2 pose2(0.4,0.3,M_PI_2);
	SimpleCamera camera = SimpleCamera::level(K, pose2, 0.1);

	// expected
	Point3 x(1,0,0),y(0,0,-1),z(0,1,0);
	Rot3 wRc(x,y,z);
	Pose3 expected(wRc,Point3(0.4,0.3,0.1));
  CHECK(assert_equal( camera.pose(), expected));
}

/* ************************************************************************* */
TEST( SimpleCamera, project)
{
  CHECK(assert_equal( camera.project(point1), Point2(-100,  100) ));
  CHECK(assert_equal( camera.project(point2), Point2(-100, -100) ));
  CHECK(assert_equal( camera.project(point3), Point2( 100, -100) ));
  CHECK(assert_equal( camera.project(point4), Point2( 100,  100) ));
}

/* ************************************************************************* */
Point2 project2(const Pose3& pose, const Point3& point) {
	return project(SimpleCamera(K,pose), point);
}

TEST( SimpleCamera, Dproject_pose)
{
	Matrix computed = Dproject_pose(camera, point1);
	Matrix numerical = numericalDerivative21(project2, pose1, point1);
	CHECK(assert_equal(computed, numerical,1e-7));
}

TEST( SimpleCamera, Dproject_point)
{
	Matrix computed = Dproject_point(camera, point1);
	Matrix numerical = numericalDerivative22(project2, pose1, point1);
	CHECK(assert_equal(computed, numerical,1e-7));
}

TEST( SimpleCamera, Dproject_point_pose)
{
	Matrix Dpose, Dpoint;
	Point2 result = Dproject_pose_point(camera, point1, Dpose, Dpoint);
	Matrix numerical_pose  = numericalDerivative21(project2, pose1, point1);
	Matrix numerical_point = numericalDerivative22(project2, pose1, point1);
  CHECK(assert_equal(result, Point2(-100,  100) ));
	CHECK(assert_equal(Dpose,  numerical_pose, 1e-7));
	CHECK(assert_equal(Dpoint, numerical_point,1e-7));
}

/* ************************************************************************* */
int main() { TestResult tr; TestRegistry::runAllTests(tr); return 0; }
/* ************************************************************************* */



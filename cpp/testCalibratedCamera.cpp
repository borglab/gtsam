/**
 * Frank Dellaert
 * brief: test CalibratedCamera class
 * based on testVSLAMFactor.cpp
 */

#include <iostream>

#include <CppUnitLite/TestHarness.h>
#include "numericalDerivative.h"
#include "CalibratedCamera.h"

using namespace std;
using namespace gtsam;

const Pose3 pose1(Matrix_(3,3,
				      1., 0., 0.,
				      0.,-1., 0.,
				      0., 0.,-1.
				      ),
			      Point3(0,0,500));
 
const CalibratedCamera camera(pose1);

const Point3 point1(-80.0,-80.0, 0.0);
const Point3 point2(-80.0, 80.0, 0.0);
const Point3 point3( 80.0, 80.0, 0.0);
const Point3 point4( 80.0,-80.0, 0.0);

/* ************************************************************************* */
TEST( CalibratedCamera, constructor)
{
  CHECK(assert_equal( camera.pose(), pose1));
}

/* ************************************************************************* */
TEST( CalibratedCamera, project)
{
  CHECK(assert_equal( camera.project(point1), Point2(-.16,  .16) ));
  CHECK(assert_equal( camera.project(point2), Point2(-.16, -.16) ));
  CHECK(assert_equal( camera.project(point3), Point2( .16, -.16) ));
  CHECK(assert_equal( camera.project(point4), Point2( .16,  .16) ));
}

/* ************************************************************************* */

TEST( CalibratedCamera, Dproject_to_camera1) {
	Point3 pp(155,233,131);
	Matrix test1 = Dproject_to_camera1(pp);
	Matrix test2 = numericalDerivative11(project_to_camera, pp);
	CHECK(assert_equal(test1, test2));
}

/* ************************************************************************* */
Point2 project2(const Pose3& pose, const Point3& point) {
	return project(CalibratedCamera(pose), point);
}

TEST( CalibratedCamera, Dproject_pose)
{
	Matrix computed = Dproject_pose(camera, point1);
	Matrix numerical = numericalDerivative21(project2, pose1, point1);
	CHECK(assert_equal(computed, numerical,1e-7));
}

TEST( CalibratedCamera, Dproject_point)
{
	Matrix computed = Dproject_point(camera, point1);
	Matrix numerical = numericalDerivative22(project2, pose1, point1);
	CHECK(assert_equal(computed, numerical,1e-7));
}

TEST( CalibratedCamera, Dproject_point_pose)
{
  Point2 result;
	Matrix Dpose, Dpoint;
	Dproject_pose_point(camera, point1, result, Dpose, Dpoint);
	Matrix numerical_pose  = numericalDerivative21(project2, pose1, point1);
	Matrix numerical_point = numericalDerivative22(project2, pose1, point1);
  CHECK(assert_equal(result, Point2(-.16,  .16) ));
	CHECK(assert_equal(Dpose,  numerical_pose, 1e-7));
	CHECK(assert_equal(Dpoint, numerical_point,1e-7));
}

/* ************************************************************************* */
int main() { TestResult tr; TestRegistry::runAllTests(tr); return 0; }
/* ************************************************************************* */



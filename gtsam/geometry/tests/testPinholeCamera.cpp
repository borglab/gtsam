/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testPinholeCamera.cpp
 * @author Frank Dellaert
 * @brief test PinholeCamera class
 */

#include <cmath>
#include <iostream>

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Cal3_S2.h>

using namespace std;
using namespace gtsam;

static const Cal3_S2 K(625, 625, 0, 0, 0);

static const Pose3 pose1((Matrix)(Matrix(3,3) <<
              1., 0., 0.,
              0.,-1., 0.,
              0., 0.,-1.
              ),
            Point3(0,0,0.5));

typedef PinholeCamera<Cal3_S2> Camera;
static const Camera camera(pose1, K);

static const Point3 point1(-0.08,-0.08, 0.0);
static const Point3 point2(-0.08, 0.08, 0.0);
static const Point3 point3( 0.08, 0.08, 0.0);
static const Point3 point4( 0.08,-0.08, 0.0);

static const Point3 point1_inf(-0.16,-0.16, -1.0);
static const Point3 point2_inf(-0.16, 0.16, -1.0);
static const Point3 point3_inf( 0.16, 0.16, -1.0);
static const Point3 point4_inf( 0.16,-0.16, -1.0);

/* ************************************************************************* */
TEST( PinholeCamera, constructor)
{
  CHECK(assert_equal( camera.calibration(), K));
  CHECK(assert_equal( camera.pose(), pose1));
}

/* ************************************************************************* */
TEST( PinholeCamera, level2)
{
  // Create a level camera, looking in Y-direction
  Pose2 pose2(0.4,0.3,M_PI/2.0);
  Camera camera = Camera::Level(K, pose2, 0.1);

  // expected
  Point3 x(1,0,0),y(0,0,-1),z(0,1,0);
  Rot3 wRc(x,y,z);
  Pose3 expected(wRc,Point3(0.4,0.3,0.1));
  CHECK(assert_equal( camera.pose(), expected));
}

/* ************************************************************************* */
TEST( PinholeCamera, lookat)
{
  // Create a level camera, looking in Y-direction
  Point3 C(10.0,0.0,0.0);
  Camera camera = Camera::Lookat(C, Point3(), Point3(0.0,0.0,1.0));

  // expected
  Point3 xc(0,1,0),yc(0,0,-1),zc(-1,0,0);
  Pose3 expected(Rot3(xc,yc,zc),C);
  CHECK(assert_equal( camera.pose(), expected));

  Point3 C2(30.0,0.0,10.0);
  Camera camera2 = Camera::Lookat(C2, Point3(), Point3(0.0,0.0,1.0));

  Matrix R = camera2.pose().rotation().matrix();
  Matrix I = trans(R)*R;
  CHECK(assert_equal(I, eye(3)));
}

/* ************************************************************************* */
TEST( PinholeCamera, project)
{
  CHECK(assert_equal( camera.project(point1), Point2(-100,  100) ));
  CHECK(assert_equal( camera.project(point2), Point2(-100, -100) ));
  CHECK(assert_equal( camera.project(point3), Point2( 100, -100) ));
  CHECK(assert_equal( camera.project(point4), Point2( 100,  100) ));
}

/* ************************************************************************* */
TEST( PinholeCamera, backproject)
{
  CHECK(assert_equal( camera.backproject(Point2(-100,  100), 0.5),  point1));
  CHECK(assert_equal( camera.backproject(Point2(-100, -100), 0.5),  point2));
  CHECK(assert_equal( camera.backproject(Point2( 100, -100), 0.5),  point3));
  CHECK(assert_equal( camera.backproject(Point2( 100,  100), 0.5),  point4));
}

/* ************************************************************************* */
TEST( PinholeCamera, backprojectInfinity)
{
  CHECK(assert_equal( camera.backprojectPointAtInfinity(Point2(-100,  100)),  point1_inf));
  CHECK(assert_equal( camera.backprojectPointAtInfinity(Point2(-100, -100)),  point2_inf));
  CHECK(assert_equal( camera.backprojectPointAtInfinity(Point2( 100, -100)),  point3_inf));
  CHECK(assert_equal( camera.backprojectPointAtInfinity(Point2( 100,  100)),  point4_inf));
}

/* ************************************************************************* */
TEST( PinholeCamera, backproject2)
{
  Point3 origin;
  Rot3 rot(1., 0., 0., 0., 0., 1., 0., -1., 0.); // a camera looking down
  Camera camera(Pose3(rot, origin), K);

  Point3 actual = camera.backproject(Point2(), 1.);
  Point3 expected(0., 1., 0.);
  pair<Point2, bool> x = camera.projectSafe(expected);

  CHECK(assert_equal(expected, actual));
  CHECK(assert_equal(Point2(), x.first));
  CHECK(x.second);
}

/* ************************************************************************* */
TEST( PinholeCamera, backprojectInfinity2)
{
  Point3 origin;
  Rot3 rot(1., 0., 0., 0., 0., 1., 0., -1., 0.); // a camera looking down
  Camera camera(Pose3(rot, origin), K);

  Point3 actual = camera.backprojectPointAtInfinity(Point2());
  Point3 expected(0., 1., 0.);
  Point2 x = camera.projectPointAtInfinity(expected);

  CHECK(assert_equal(expected, actual));
  CHECK(assert_equal(Point2(), x));
}

/* ************************************************************************* */
TEST( PinholeCamera, backprojectInfinity3)
{
  Point3 origin;
  Rot3 rot(1., 0., 0., 0., 1., 0., 0., 0., 1.); // identity
  Camera camera(Pose3(rot, origin), K);

  Point3 actual = camera.backprojectPointAtInfinity(Point2());
  Point3 expected(0., 0., 1.);
  Point2 x = camera.projectPointAtInfinity(expected);

  CHECK(assert_equal(expected, actual));
  CHECK(assert_equal(Point2(), x));
}

/* ************************************************************************* */
static Point2 project3(const Pose3& pose, const Point3& point, const Cal3_S2& cal) {
  return Camera(pose,cal).project(point);
}

/* ************************************************************************* */
TEST( PinholeCamera, Dproject)
{
  Matrix Dpose, Dpoint, Dcal;
  Point2 result = camera.project(point1, Dpose, Dpoint, Dcal);
  Matrix numerical_pose  = numericalDerivative31(project3, pose1, point1, K);
  Matrix numerical_point = numericalDerivative32(project3, pose1, point1, K);
  Matrix numerical_cal   = numericalDerivative33(project3, pose1, point1, K);
  CHECK(assert_equal(Point2(-100,  100), result));
  CHECK(assert_equal(numerical_pose,  Dpose,  1e-7));
  CHECK(assert_equal(numerical_point, Dpoint, 1e-7));
  CHECK(assert_equal(numerical_cal,   Dcal,   1e-7));
}

/* ************************************************************************* */
static Point2 projectInfinity3(const Pose3& pose, const Point3& point3D, const Cal3_S2& cal) {
  return Camera(pose,cal).projectPointAtInfinity(point3D);
}

TEST( PinholeCamera, Dproject_Infinity)
{
  Matrix Dpose, Dpoint, Dcal;
  Point3 point3D(point1.x(), point1.y(), -10.0); // a point in front of the camera

  // test Projection
  Point2 actual = camera.projectPointAtInfinity(point3D, Dpose, Dpoint, Dcal);
  Point2 expected(-5.0, 5.0);
  CHECK(assert_equal(actual, expected,  1e-7));

  // test Jacobians
  Matrix numerical_pose     = numericalDerivative31(projectInfinity3, pose1, point3D, K);
  Matrix numerical_point    = numericalDerivative32(projectInfinity3, pose1, point3D, K);
  Matrix numerical_point2x2 = numerical_point.block(0,0,2,2); // only the direction to the point matters
  Matrix numerical_cal      = numericalDerivative33(projectInfinity3, pose1, point3D, K);
  CHECK(assert_equal(numerical_pose,     Dpose,  1e-7));
  CHECK(assert_equal(numerical_point2x2, Dpoint, 1e-7));
  CHECK(assert_equal(numerical_cal,      Dcal,   1e-7));
}

/* ************************************************************************* */
static Point2 project4(const Camera& camera, const Point3& point) {
  return camera.project2(point);
}

/* ************************************************************************* */
TEST( PinholeCamera, Dproject2)
{
  Matrix Dcamera, Dpoint;
  Point2 result = camera.project2(point1, Dcamera, Dpoint);
  Matrix numerical_camera = numericalDerivative21(project4, camera, point1);
  Matrix numerical_point  = numericalDerivative22(project4, camera, point1);
  CHECK(assert_equal(result, Point2(-100,  100) ));
  CHECK(assert_equal(numerical_camera, Dcamera, 1e-7));
  CHECK(assert_equal(numerical_point,  Dpoint,  1e-7));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */



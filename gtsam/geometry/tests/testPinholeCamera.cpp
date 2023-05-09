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

#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Cal3Bundler.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>

#include <CppUnitLite/TestHarness.h>

#include <cmath>
#include <iostream>

using namespace std::placeholders;
using namespace std;
using namespace gtsam;

typedef PinholeCamera<Cal3_S2> Camera;

static const Cal3_S2 K(625, 625, 0, 0, 0);

static const Pose3 pose(Rot3(Vector3(1, -1, -1).asDiagonal()), Point3(0, 0, 0.5));
static const Camera camera(pose, K);

static const Pose3 pose1(Rot3(), Point3(0, 1, 0.5));
static const Camera camera1(pose1, K);

static const Point3 point1(-0.08,-0.08, 0.0);
static const Point3 point2(-0.08, 0.08, 0.0);
static const Point3 point3( 0.08, 0.08, 0.0);
static const Point3 point4( 0.08,-0.08, 0.0);

static const Unit3 point1_inf(-0.16,-0.16, -1.0);
static const Unit3 point2_inf(-0.16, 0.16, -1.0);
static const Unit3 point3_inf( 0.16, 0.16, -1.0);
static const Unit3 point4_inf( 0.16,-0.16, -1.0);

/* ************************************************************************* */
TEST( PinholeCamera, constructor)
{
  EXPECT(assert_equal( K, camera.calibration()));
  EXPECT(assert_equal( pose, camera.pose()));
}

//******************************************************************************
TEST(PinholeCamera, Create) {

  Matrix actualH1, actualH2;
  EXPECT(assert_equal(camera, Camera::Create(pose,K, actualH1, actualH2)));

  // Check derivative
  std::function<Camera(Pose3, Cal3_S2)> f =  //
      std::bind(Camera::Create, std::placeholders::_1, std::placeholders::_2,
                boost::none, boost::none);
  Matrix numericalH1 = numericalDerivative21<Camera,Pose3,Cal3_S2>(f,pose,K);
  EXPECT(assert_equal(numericalH1, actualH1, 1e-9));
  Matrix numericalH2 = numericalDerivative22<Camera,Pose3,Cal3_S2>(f,pose,K);
  EXPECT(assert_equal(numericalH2, actualH2, 1e-8));
}

//******************************************************************************
TEST(PinholeCamera, Pose) {

  Matrix actualH;
  EXPECT(assert_equal(pose, camera.getPose(actualH)));

  // Check derivative
  std::function<Pose3(Camera)> f =  //
      std::bind(&Camera::getPose, std::placeholders::_1, boost::none);
  Matrix numericalH = numericalDerivative11<Pose3,Camera>(f,camera);
  EXPECT(assert_equal(numericalH, actualH, 1e-9));
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
  EXPECT(assert_equal( camera.pose(), expected));
}

/* ************************************************************************* */
TEST( PinholeCamera, lookat)
{
  // Create a level camera, looking in Y-direction
  Point3 C(10,0,0);
  Camera camera = Camera::Lookat(C, Point3(0,0,0), Point3(0,0,1));

  // expected
  Point3 xc(0,1,0),yc(0,0,-1),zc(-1,0,0);
  Pose3 expected(Rot3(xc,yc,zc),C);
  EXPECT(assert_equal(camera.pose(), expected));

  Point3 C2(30,0,10);
  Camera camera2 = Camera::Lookat(C2, Point3(0,0,0), Point3(0,0,1));

  Matrix R = camera2.pose().rotation().matrix();
  Matrix I = trans(R)*R;
  EXPECT(assert_equal(I, I_3x3));
}

/* ************************************************************************* */
TEST( PinholeCamera, project)
{
  EXPECT(assert_equal( camera.project(point1), Point2(-100,  100) ));
  EXPECT(assert_equal( camera.project(point2), Point2(-100, -100) ));
  EXPECT(assert_equal( camera.project(point3), Point2( 100, -100) ));
  EXPECT(assert_equal( camera.project(point4), Point2( 100,  100) ));
}

/* ************************************************************************* */
TEST( PinholeCamera, backproject)
{
  EXPECT(assert_equal( camera.backproject(Point2(-100,  100), 0.5),  point1));
  EXPECT(assert_equal( camera.backproject(Point2(-100, -100), 0.5),  point2));
  EXPECT(assert_equal( camera.backproject(Point2( 100, -100), 0.5),  point3));
  EXPECT(assert_equal( camera.backproject(Point2( 100,  100), 0.5),  point4));
}

/* ************************************************************************* */
TEST( PinholeCamera, backprojectInfinity)
{
  EXPECT(assert_equal( camera.backprojectPointAtInfinity(Point2(-100,  100)),  point1_inf));
  EXPECT(assert_equal( camera.backprojectPointAtInfinity(Point2(-100, -100)),  point2_inf));
  EXPECT(assert_equal( camera.backprojectPointAtInfinity(Point2( 100, -100)),  point3_inf));
  EXPECT(assert_equal( camera.backprojectPointAtInfinity(Point2( 100,  100)),  point4_inf));
}

/* ************************************************************************* */
TEST( PinholeCamera, backproject2)
{
  Point3 origin(0,0,0);
  Rot3 rot(1., 0., 0., 0., 0., 1., 0., -1., 0.); // a camera1 looking down
  Camera camera(Pose3(rot, origin), K);

  Point3 actual = camera.backproject(Point2(0,0), 1.);
  Point3 expected(0., 1., 0.);
  pair<Point2, bool> x = camera.projectSafe(expected);

  EXPECT(assert_equal(expected, actual));
  EXPECT(assert_equal(Point2(0,0), x.first));
  EXPECT(x.second);
}

/* ************************************************************************* */
TEST( PinholeCamera, backprojectInfinity2)
{
  Point3 origin(0,0,0);
  Rot3 rot(1., 0., 0., 0., 0., 1., 0., -1., 0.); // a camera1 looking down
  Camera camera(Pose3(rot, origin), K);

  Unit3 actual = camera.backprojectPointAtInfinity(Point2(0,0));
  Unit3 expected(0., 1., 0.);
  Point2 x = camera.project(expected);

  EXPECT(assert_equal(expected, actual));
  EXPECT(assert_equal(Point2(0,0), x));
}

/* ************************************************************************* */
TEST( PinholeCamera, backprojectInfinity3)
{
  Point3 origin(0,0,0);
  Rot3 rot(1., 0., 0., 0., 1., 0., 0., 0., 1.); // identity
  Camera camera(Pose3(rot, origin), K);

  Unit3 actual = camera.backprojectPointAtInfinity(Point2(0,0));
  Unit3 expected(0., 0., 1.);
  Point2 x = camera.project(expected);

  EXPECT(assert_equal(expected, actual));
  EXPECT(assert_equal(Point2(0,0), x));
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
  Matrix numerical_pose  = numericalDerivative31(project3, pose, point1, K);
  Matrix Hexpected2 = numericalDerivative32(project3, pose, point1, K);
  Matrix numerical_cal   = numericalDerivative33(project3, pose, point1, K);
  EXPECT(assert_equal(Point2(-100,  100), result));
  EXPECT(assert_equal(numerical_pose,  Dpose,  1e-7));
  EXPECT(assert_equal(Hexpected2, Dpoint, 1e-7));
  EXPECT(assert_equal(numerical_cal,   Dcal,   1e-7));
}

/* ************************************************************************* */
static Point2 projectInfinity3(const Pose3& pose, const Unit3& point3D, const Cal3_S2& cal) {
  return Camera(pose,cal).project(point3D);
}

TEST( PinholeCamera, Dproject_Infinity)
{
  Matrix Dpose, Dpoint, Dcal;
  Unit3 point3D(point1.x(), point1.y(), -10.0); // a point in front of the camera1

  // test Projection
  Point2 actual = camera.project(point3D, Dpose, Dpoint, Dcal);
  Point2 expected(-5.0, 5.0);
  EXPECT(assert_equal(actual, expected,  1e-7));

  // test Jacobians
  Matrix numerical_pose     = numericalDerivative31(projectInfinity3, pose, point3D, K);
  Matrix Hexpected2    = numericalDerivative32(projectInfinity3, pose, point3D, K);
  Matrix numerical_point2x2 = Hexpected2.block(0,0,2,2); // only the direction to the point matters
  Matrix numerical_cal      = numericalDerivative33(projectInfinity3, pose, point3D, K);
  EXPECT(assert_equal(numerical_pose,     Dpose,  1e-7));
  EXPECT(assert_equal(numerical_point2x2, Dpoint, 1e-7));
  EXPECT(assert_equal(numerical_cal,      Dcal,   1e-7));
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
  Matrix Hexpected1 = numericalDerivative21(project4, camera, point1);
  Matrix Hexpected2  = numericalDerivative22(project4, camera, point1);
  EXPECT(assert_equal(result, Point2(-100,  100) ));
  EXPECT(assert_equal(Hexpected1, Dcamera, 1e-7));
  EXPECT(assert_equal(Hexpected2,  Dpoint,  1e-7));
}

/* ************************************************************************* */
// Add a test with more arbitrary rotation
TEST( PinholeCamera, Dproject3)
{
  static const Pose3 pose1(Rot3::Ypr(0.1, -0.1, 0.4), Point3(0, 0, -10));
  static const Camera camera(pose1);
  Matrix Dpose, Dpoint;
  camera.project2(point1, Dpose, Dpoint);
  Matrix numerical_pose  = numericalDerivative21(project4, camera, point1);
  Matrix numerical_point = numericalDerivative22(project4, camera, point1);
  CHECK(assert_equal(numerical_pose,  Dpose, 1e-7));
  CHECK(assert_equal(numerical_point, Dpoint, 1e-7));
}

/* ************************************************************************* */
static double range0(const Camera& camera, const Point3& point) {
  return camera.range(point);
}

/* ************************************************************************* */
TEST( PinholeCamera, range0) {
  Matrix D1; Matrix D2;
  double result = camera.range(point1, D1, D2);
  Matrix Hexpected1 = numericalDerivative21(range0, camera, point1);
  Matrix Hexpected2 = numericalDerivative22(range0, camera, point1);
  EXPECT_DOUBLES_EQUAL(distance3(point1, camera.pose().translation()), result,
      1e-9);
  EXPECT(assert_equal(Hexpected1, D1, 1e-7));
  EXPECT(assert_equal(Hexpected2, D2, 1e-7));
}

/* ************************************************************************* */
static double range1(const Camera& camera, const Pose3& pose) {
  return camera.range(pose);
}

/* ************************************************************************* */
TEST( PinholeCamera, range1) {
  Matrix D1; Matrix D2;
  double result = camera.range(pose1, D1, D2);
  Matrix Hexpected1 = numericalDerivative21(range1, camera, pose1);
  Matrix Hexpected2 = numericalDerivative22(range1, camera, pose1);
  EXPECT_DOUBLES_EQUAL(1, result, 1e-9);
  EXPECT(assert_equal(Hexpected1, D1, 1e-7));
  EXPECT(assert_equal(Hexpected2, D2, 1e-7));
}

/* ************************************************************************* */
typedef PinholeCamera<Cal3Bundler> Camera2;
static const Cal3Bundler K2(625, 1e-3, 1e-3);
static const Camera2 camera2(pose1, K2);
static double range2(const Camera& camera, const Camera2& camera2) {
  return camera.range<Cal3Bundler>(camera2);
}

/* ************************************************************************* */
TEST( PinholeCamera, range2) {
  Matrix D1; Matrix D2;
  double result = camera.range<Cal3Bundler>(camera2, D1, D2);
  Matrix Hexpected1 = numericalDerivative21(range2, camera, camera2);
  Matrix Hexpected2 = numericalDerivative22(range2, camera, camera2);
  EXPECT_DOUBLES_EQUAL(1, result, 1e-9);
  EXPECT(assert_equal(Hexpected1, D1, 1e-7));
  EXPECT(assert_equal(Hexpected2, D2, 1e-7));
}

/* ************************************************************************* */
static const CalibratedCamera camera3(pose1);
static double range3(const Camera& camera, const CalibratedCamera& camera3) {
  return camera.range(camera3);
}

/* ************************************************************************* */
TEST( PinholeCamera, range3) {
  Matrix D1; Matrix D2;
  double result = camera.range(camera3, D1, D2);
  Matrix Hexpected1 = numericalDerivative21(range3, camera, camera3);
  Matrix Hexpected2 = numericalDerivative22(range3, camera, camera3);
  EXPECT_DOUBLES_EQUAL(1, result, 1e-9);
  EXPECT(assert_equal(Hexpected1, D1, 1e-7));
  EXPECT(assert_equal(Hexpected2, D2, 1e-7));
}

/* ************************************************************************* */
TEST( PinholeCamera, Cal3Bundler) {
  Cal3Bundler calibration;
  Pose3 wTc;
  PinholeCamera<Cal3Bundler> camera(wTc, calibration);
  Point2 p(50, 100);
  camera.backproject(p, 10);
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */



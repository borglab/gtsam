/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testCalibratedCamera.cpp
 * @author Frank Dellaert
 * @brief test CalibratedCamera class
 */

#include <gtsam/geometry/CalibratedCamera.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>

#include <CppUnitLite/TestHarness.h>

#include <iostream>

using namespace std;
using namespace gtsam;

GTSAM_CONCEPT_MANIFOLD_INST(CalibratedCamera)

// Camera situated at 0.5 meters high, looking down
static const Pose3 kDefaultPose(Rot3(Vector3(1, -1, -1).asDiagonal()),
                         Point3(0, 0, 0.5));

static const CalibratedCamera camera(kDefaultPose);

static const Point3 point1(-0.08,-0.08, 0.0);
static const Point3 point2(-0.08, 0.08, 0.0);
static const Point3 point3( 0.08, 0.08, 0.0);
static const Point3 point4( 0.08,-0.08, 0.0);

/* ************************************************************************* */
TEST( CalibratedCamera, constructor)
{
  CHECK(assert_equal( camera.pose(), kDefaultPose));
}

//******************************************************************************
TEST(CalibratedCamera, Create) {
  Matrix actualH;
  EXPECT(assert_equal(camera, CalibratedCamera::Create(kDefaultPose, actualH)));

  // Check derivative
  boost::function<CalibratedCamera(Pose3)> f =  //
      boost::bind(CalibratedCamera::Create, _1, boost::none);
  Matrix numericalH = numericalDerivative11<CalibratedCamera, Pose3>(f, kDefaultPose);
  EXPECT(assert_equal(numericalH, actualH, 1e-9));
}

/* ************************************************************************* */
TEST( CalibratedCamera, level1)
{
  // Create a level camera, looking in X-direction
  Pose2 pose2(0.1,0.2,0);
  CalibratedCamera camera = CalibratedCamera::Level(pose2, 0.3);

  // expected
  Point3 x(0,-1,0),y(0,0,-1),z(1,0,0);
  Rot3 wRc(x,y,z);
  Pose3 expected(wRc,Point3(0.1,0.2,0.3));
  CHECK(assert_equal( camera.pose(), expected));
}

/* ************************************************************************* */
TEST( CalibratedCamera, level2)
{
  // Create a level camera, looking in Y-direction
  Pose2 pose2(0.4,0.3,M_PI/2.0);
  CalibratedCamera camera = CalibratedCamera::Level(pose2, 0.1);

  // expected
  Point3 x(1,0,0),y(0,0,-1),z(0,1,0);
  Rot3 wRc(x,y,z);
  Pose3 expected(wRc,Point3(0.4,0.3,0.1));
  CHECK(assert_equal( camera.pose(), expected));
}

/* ************************************************************************* */
TEST( CalibratedCamera, project)
{
  CHECK(assert_equal( Point2(-.16,  .16), camera.project(point1) ));
  CHECK(assert_equal( Point2(-.16, -.16), camera.project(point2) ));
  CHECK(assert_equal( Point2( .16, -.16), camera.project(point3) ));
  CHECK(assert_equal( Point2( .16,  .16), camera.project(point4) ));
}

/* ************************************************************************* */
static Point2 Project1(const Point3& point) {
  return PinholeBase::Project(point);
}

TEST( CalibratedCamera, DProject1) {
  Point3 pp(155, 233, 131);
  Matrix test1;
  CalibratedCamera::Project(pp, test1);
  Matrix test2 = numericalDerivative11<Point2, Point3>(Project1, pp);
  CHECK(assert_equal(test1, test2));
}

/* ************************************************************************* */
static Point2 Project2(const Unit3& point) {
  return PinholeBase::Project(point);
}

Unit3 pointAtInfinity(0, 0, 1000);
TEST( CalibratedCamera, DProjectInfinity) {
  Matrix test1;
  CalibratedCamera::Project(pointAtInfinity, test1);
  Matrix test2 = numericalDerivative11<Point2, Unit3>(Project2,
      pointAtInfinity);
  CHECK(assert_equal(test1, test2));
}

/* ************************************************************************* */
static Point2 project2(const CalibratedCamera& camera, const Point3& point) {
  return camera.project(point);
}

TEST( CalibratedCamera, Dproject_point_pose)
{
  Matrix Dpose, Dpoint;
  Point2 result = camera.project(point1, Dpose, Dpoint);
  Matrix numerical_pose  = numericalDerivative21(project2, camera, point1);
  Matrix numerical_point = numericalDerivative22(project2, camera, point1);
  CHECK(assert_equal(Point3(-0.08, 0.08, 0.5), camera.pose().transform_to(point1)));
  CHECK(assert_equal(Point2(-.16,  .16), result));
  CHECK(assert_equal(numerical_pose,  Dpose, 1e-7));
  CHECK(assert_equal(numerical_point, Dpoint, 1e-7));
}

/* ************************************************************************* */
// Add a test with more arbitrary rotation
TEST( CalibratedCamera, Dproject_point_pose2)
{
  static const Pose3 kDefaultPose(Rot3::Ypr(0.1, -0.1, 0.4), Point3(0, 0, -10));
  static const CalibratedCamera camera(kDefaultPose);
  Matrix Dpose, Dpoint;
  camera.project(point1, Dpose, Dpoint);
  Matrix numerical_pose  = numericalDerivative21(project2, camera, point1);
  Matrix numerical_point = numericalDerivative22(project2, camera, point1);
  CHECK(assert_equal(numerical_pose,  Dpose, 1e-7));
  CHECK(assert_equal(numerical_point, Dpoint, 1e-7));
}

/* ************************************************************************* */
static Point2 projectAtInfinity(const CalibratedCamera& camera, const Unit3& point) {
  return camera.project2(point);
}

TEST( CalibratedCamera, Dproject_point_pose_infinity)
{
  Matrix Dpose, Dpoint;
  Point2 result = camera.project2(pointAtInfinity, Dpose, Dpoint);
  Matrix numerical_pose  = numericalDerivative21(projectAtInfinity, camera, pointAtInfinity);
  Matrix numerical_point = numericalDerivative22(projectAtInfinity, camera, pointAtInfinity);
  CHECK(assert_equal(Point2(0,0), result));
  CHECK(assert_equal(numerical_pose,  Dpose, 1e-7));
  CHECK(assert_equal(numerical_point, Dpoint, 1e-7));
}

/* ************************************************************************* */
// Add a test with more arbitrary rotation
TEST( CalibratedCamera, Dproject_point_pose2_infinity)
{
  static const Pose3 pose(Rot3::Ypr(0.1, -0.1, 0.4), Point3(0, 0, -10));
  static const CalibratedCamera camera(pose);
  Matrix Dpose, Dpoint;
  camera.project2(pointAtInfinity, Dpose, Dpoint);
  Matrix numerical_pose  = numericalDerivative21(projectAtInfinity, camera, pointAtInfinity);
  Matrix numerical_point = numericalDerivative22(projectAtInfinity, camera, pointAtInfinity);
  CHECK(assert_equal(numerical_pose,  Dpose, 1e-7));
  CHECK(assert_equal(numerical_point, Dpoint, 1e-7));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */


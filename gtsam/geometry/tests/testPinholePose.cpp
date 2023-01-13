/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   testPinholePose.cpp
 * @author Frank Dellaert
 * @brief  test PinholePose class
 * @date   Feb 20, 2015
 */

#include <gtsam/geometry/PinholePose.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Cal3Bundler.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>

#include <CppUnitLite/TestHarness.h>

#include <cmath>
#include <iostream>

using namespace std;
using namespace gtsam;

typedef PinholePose<Cal3_S2> Camera;

static const Cal3_S2::shared_ptr K = boost::make_shared<Cal3_S2>(625, 625, 0, 0, 0);

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
TEST( PinholePose, constructor)
{
  EXPECT(assert_equal( pose, camera.pose()));
}

//******************************************************************************
/* Already in testPinholeCamera??? 
TEST(PinholeCamera, Pose) {

  Matrix actualH;
  EXPECT(assert_equal(pose, camera.getPose(actualH)));

  // Check derivative
  std::function<Pose3(Camera)> f = //
      std::bind(&Camera::getPose,_1,{});
  Matrix numericalH = numericalDerivative11<Pose3,Camera>(f,camera);
  EXPECT(assert_equal(numericalH, actualH, 1e-9));
}
*/

/* ************************************************************************* */
TEST( PinholePose, lookat)
{
  // Create a level camera, looking in Y-direction
  Point3 C(10,0,0);
  Camera camera = Camera::Lookat(C, Point3(0,0,0), Point3(0,0,1));

  // expected
  Point3 xc(0,1,0),yc(0,0,-1),zc(-1,0,0);
  Pose3 expected(Rot3(xc,yc,zc),C);
  EXPECT(assert_equal( camera.pose(), expected));

  Point3 C2(30,0,10);
  Camera camera2 = Camera::Lookat(C2, Point3(0,0,0), Point3(0,0,1));

  Matrix R = camera2.pose().rotation().matrix();
  Matrix I = trans(R)*R;
  EXPECT(assert_equal(I, I_3x3));
}

/* ************************************************************************* */
TEST( PinholePose, project)
{
  EXPECT(assert_equal( camera.project2(point1), Point2(-100,  100) ));
  EXPECT(assert_equal( camera.project2(point2), Point2(-100, -100) ));
  EXPECT(assert_equal( camera.project2(point3), Point2( 100, -100) ));
  EXPECT(assert_equal( camera.project2(point4), Point2( 100,  100) ));
}

/* ************************************************************************* */
TEST( PinholePose, backproject)
{
  EXPECT(assert_equal( camera.backproject(Point2(-100,  100), 0.5),  point1));
  EXPECT(assert_equal( camera.backproject(Point2(-100, -100), 0.5),  point2));
  EXPECT(assert_equal( camera.backproject(Point2( 100, -100), 0.5),  point3));
  EXPECT(assert_equal( camera.backproject(Point2( 100,  100), 0.5),  point4));
}

/* ************************************************************************* */
TEST( PinholePose, backprojectInfinity)
{
  EXPECT(assert_equal( camera.backprojectPointAtInfinity(Point2(-100,  100)),  point1_inf));
  EXPECT(assert_equal( camera.backprojectPointAtInfinity(Point2(-100, -100)),  point2_inf));
  EXPECT(assert_equal( camera.backprojectPointAtInfinity(Point2( 100, -100)),  point3_inf));
  EXPECT(assert_equal( camera.backprojectPointAtInfinity(Point2( 100,  100)),  point4_inf));
}

/* ************************************************************************* */
TEST( PinholePose, backproject2)
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
static Point2 project3(const Pose3& pose, const Point3& point,
    const Cal3_S2::shared_ptr& cal) {
  return Camera(pose, cal).project2(point);
}

/* ************************************************************************* */
TEST( PinholePose, Dproject)
{
  Matrix Dpose, Dpoint;
  Point2 result = camera.project2(point1, Dpose, Dpoint);
  Matrix expectedDcamera  = numericalDerivative31(project3, pose, point1, K);
  Matrix expectedDpoint = numericalDerivative32(project3, pose, point1, K);
  EXPECT(assert_equal(Point2(-100,  100), result));
  EXPECT(assert_equal(expectedDcamera,  Dpose,  1e-7));
  EXPECT(assert_equal(expectedDpoint, Dpoint, 1e-7));
}

/* ************************************************************************* */
static Point2 project4(const Camera& camera, const Point3& point) {
  return camera.project2(point);
}

/* ************************************************************************* */
TEST( PinholePose, Dproject2)
{
  Matrix Dcamera, Dpoint;
  Point2 result = camera.project2(point1, Dcamera, Dpoint);
  Matrix expectedDcamera = numericalDerivative21(project4, camera, point1);
  Matrix expectedDpoint  = numericalDerivative22(project4, camera, point1);
  EXPECT(assert_equal(result, Point2(-100,  100) ));
  EXPECT(assert_equal(expectedDcamera, Dcamera, 1e-7));
  EXPECT(assert_equal(expectedDpoint,  Dpoint,  1e-7));
}

/* ************************************************************************* */
// Add a test with more arbitrary rotation
TEST( CalibratedCamera, Dproject3)
{
  static const Pose3 pose1(Rot3::Ypr(0.1, -0.1, 0.4), Point3(0, 0, -10));
  static const Camera camera(pose1);
  Matrix Dpose, Dpoint;
  camera.project2(point1, Dpose, Dpoint);
  Matrix expectedDcamera  = numericalDerivative21(project4, camera, point1);
  Matrix numerical_point = numericalDerivative22(project4, camera, point1);
  CHECK(assert_equal(expectedDcamera,  Dpose, 1e-7));
  CHECK(assert_equal(numerical_point, Dpoint, 1e-7));
}

/* ************************************************************************* */
static Point2 project(const Pose3& pose, const Unit3& pointAtInfinity,
    const Cal3_S2::shared_ptr& cal) {
  return Camera(pose, cal).project(pointAtInfinity);
}

/* ************************************************************************* */
TEST( PinholePose, DprojectAtInfinity2)
{
  Unit3 pointAtInfinity(0,0,-1000);
  Matrix Dpose, Dpoint;
  Point2 result = camera.project2(pointAtInfinity, Dpose, Dpoint);
  Matrix expectedDcamera  = numericalDerivative31(project, pose, pointAtInfinity, K);
  Matrix expectedDpoint = numericalDerivative32(project, pose, pointAtInfinity, K);
  EXPECT(assert_equal(Point2(0,0), result));
  EXPECT(assert_equal(expectedDcamera,  Dpose,  1e-7));
  EXPECT(assert_equal(expectedDpoint, Dpoint, 1e-7));
}


static Point3 backproject(const Pose3& pose, const Cal3_S2& cal,
                          const Point2& p, const double& depth) {
  return Camera(pose, cal.vector()).backproject(p, depth);
}

TEST( PinholePose, DbackprojectRegCamera)
{
  Matrix36 Dpose;
  Matrix31 Ddepth;
  Matrix32 Dpoint;
  Matrix Dcal;
  const Point2 point(-100, 100);
  const double depth(10);
  camera.backproject(point, depth, Dpose, Dpoint, Ddepth, Dcal);
  Matrix expectedDpose = numericalDerivative41(backproject, pose, *K,  point, depth);
  Matrix expectedDcal = numericalDerivative42(backproject, pose, *K,  point, depth);
  Matrix expectedDpoint = numericalDerivative43(backproject, pose, *K, point, depth);
  Matrix expectedDdepth = numericalDerivative44(backproject, pose, *K,  point, depth);

  EXPECT(assert_equal(expectedDpose,   Dpose,   1e-7));
  EXPECT(assert_equal(expectedDcal,    Dcal,    1e-7));
  EXPECT(assert_equal(expectedDpoint,  Dpoint,  1e-7));
  EXPECT(assert_equal(expectedDdepth,  Ddepth,  1e-7));
}

/* ************************************************************************* */
static double range0(const Camera& camera, const Point3& point) {
  return camera.range(point);
}

/* ************************************************************************* */
TEST( PinholePose, range0) {
  Matrix D1; Matrix D2;
  double result = camera.range(point1, D1, D2);
  Matrix expectedDcamera = numericalDerivative21(range0, camera, point1);
  Matrix expectedDpoint = numericalDerivative22(range0, camera, point1);
  EXPECT_DOUBLES_EQUAL(distance3(point1, camera.pose().translation()), result, 1e-9);
  EXPECT(assert_equal(expectedDcamera, D1, 1e-7));
  EXPECT(assert_equal(expectedDpoint, D2, 1e-7));
}

/* ************************************************************************* */
static double range1(const Camera& camera, const Pose3& pose) {
  return camera.range(pose);
}

/* ************************************************************************* */
TEST( PinholePose, range1) {
  Matrix D1; Matrix D2;
  double result = camera.range(pose1, D1, D2);
  Matrix expectedDcamera = numericalDerivative21(range1, camera, pose1);
  Matrix expectedDpoint = numericalDerivative22(range1, camera, pose1);
  EXPECT_DOUBLES_EQUAL(1, result, 1e-9);
  EXPECT(assert_equal(expectedDcamera, D1, 1e-7));
  EXPECT(assert_equal(expectedDpoint, D2, 1e-7));
}

/* ************************************************************************* */
typedef PinholePose<Cal3Bundler> Camera2;
static const boost::shared_ptr<Cal3Bundler> K2 =
    boost::make_shared<Cal3Bundler>(625, 1e-3, 1e-3);
static const Camera2 camera2(pose1, K2);
static double range2(const Camera& camera, const Camera2& camera2) {
  return camera.range<Cal3Bundler>(camera2);
}

/* ************************************************************************* */
TEST( PinholePose, range2) {
  Matrix D1; Matrix D2;
  double result = camera.range<Cal3Bundler>(camera2, D1, D2);
  Matrix expectedDcamera = numericalDerivative21(range2, camera, camera2);
  Matrix expectedDpoint = numericalDerivative22(range2, camera, camera2);
  EXPECT_DOUBLES_EQUAL(1, result, 1e-9);
  EXPECT(assert_equal(expectedDcamera, D1, 1e-7));
  EXPECT(assert_equal(expectedDpoint, D2, 1e-7));
}

/* ************************************************************************* */
static const CalibratedCamera camera3(pose1);
static double range3(const Camera& camera, const CalibratedCamera& camera3) {
  return camera.range(camera3);
}

/* ************************************************************************* */
TEST( PinholePose, range3) {
  Matrix D1; Matrix D2;
  double result = camera.range(camera3, D1, D2);
  Matrix expectedDcamera = numericalDerivative21(range3, camera, camera3);
  Matrix expectedDpoint = numericalDerivative22(range3, camera, camera3);
  EXPECT_DOUBLES_EQUAL(1, result, 1e-9);
  EXPECT(assert_equal(expectedDcamera, D1, 1e-7));
  EXPECT(assert_equal(expectedDpoint, D2, 1e-7));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */



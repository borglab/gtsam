/*
 * testInvDepthFactor.cpp
 *
 *  Created on: Apr 13, 2012
 *      Author: cbeall3
 */

#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/Testable.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Cal3_S2.h>

#include <gtsam_unstable/geometry/InvDepthCamera3.h>

using namespace std;
using namespace gtsam;

static Cal3_S2::shared_ptr K(new Cal3_S2(1500, 1200, 0, 640, 480));
Pose3 level_pose = Pose3(Rot3::Ypr(-M_PI/2, 0., -M_PI/2), gtsam::Point3(0,0,1));
PinholeCamera<Cal3_S2> level_camera(level_pose, *K);

/* ************************************************************************* */
TEST( InvDepthFactor, Project1) {

  // landmark 5 meters infront of camera
  Point3 landmark(5, 0, 1);

  Point2 expected_uv = level_camera.project(landmark);

  InvDepthCamera3<Cal3_S2> inv_camera(level_pose, K);
  Vector5 inv_landmark((Vector(5) << 1., 0., 1., 0., 0.).finished());
  double inv_depth(1./4);
  Point2 actual_uv = inv_camera.project(inv_landmark, inv_depth);
  EXPECT(assert_equal(expected_uv, actual_uv,1e-8));
  EXPECT(assert_equal(Point2(640,480), actual_uv));
}

/* ************************************************************************* */
TEST( InvDepthFactor, Project2) {

  // landmark 1m to the left and 1m up from camera
  // inv landmark xyz is same as camera xyz, so depth  actually doesn't matter
  Point3 landmark(1, 1, 2);
  Point2 expected = level_camera.project(landmark);

  InvDepthCamera3<Cal3_S2> inv_camera(level_pose, K);
  Vector5 diag_landmark((Vector(5) << 0., 0., 1., M_PI/4., atan(1.0/sqrt(2.0))).finished());
  double inv_depth(1/sqrt(3.0));
  Point2 actual = inv_camera.project(diag_landmark, inv_depth);
  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST( InvDepthFactor, Project3) {

  // landmark 1m to the left and 1m up from camera
  // inv depth landmark xyz at origion
  Point3 landmark(1, 1, 2);
  Point2 expected = level_camera.project(landmark);

  InvDepthCamera3<Cal3_S2> inv_camera(level_pose, K);
  Vector5 diag_landmark((Vector(5) << 0., 0., 0., M_PI/4., atan(2./sqrt(2.0))).finished());
  double inv_depth( 1./sqrt(1.0+1+4));
  Point2 actual = inv_camera.project(diag_landmark, inv_depth);
  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST( InvDepthFactor, Project4) {

  // landmark 4m to the left and 1m up from camera
  // inv depth landmark xyz at origion
  Point3 landmark(1, 4, 2);
  Point2 expected = level_camera.project(landmark);

  InvDepthCamera3<Cal3_S2> inv_camera(level_pose, K);
  Vector5 diag_landmark((Vector(5) << 0., 0., 0., atan(4.0/1), atan(2./sqrt(1.+16.))).finished());
  double inv_depth(1./sqrt(1.+16.+4.));
  Point2 actual = inv_camera.project(diag_landmark, inv_depth);
  EXPECT(assert_equal(expected, actual));
}


/* ************************************************************************* */
Point2 project_(const Pose3& pose, const Vector5& landmark, const double& inv_depth) {
  return InvDepthCamera3<Cal3_S2>(pose,K).project(landmark, inv_depth); }

TEST( InvDepthFactor, Dproject_pose)
{
  Vector5 landmark((Vector(5) << 0.1,0.2,0.3, 0.1,0.2).finished());
  double inv_depth(1./4);
  Matrix expected = numericalDerivative31(project_,level_pose, landmark, inv_depth);
  InvDepthCamera3<Cal3_S2> inv_camera(level_pose,K);
  Matrix actual;
  inv_camera.project(landmark, inv_depth, actual, {}, {});
  EXPECT(assert_equal(expected,actual,1e-6));
}

/* ************************************************************************* */
TEST( InvDepthFactor, Dproject_landmark)
{
  Vector5 landmark((Vector(5) << 0.1,0.2,0.3, 0.1,0.2).finished());
  double inv_depth(1./4);
  Matrix expected = numericalDerivative32(project_,level_pose, landmark, inv_depth);
  InvDepthCamera3<Cal3_S2> inv_camera(level_pose,K);
  Matrix actual;
  inv_camera.project(landmark, inv_depth, {}, actual, {});
  EXPECT(assert_equal(expected,actual,1e-7));
}

/* ************************************************************************* */
TEST( InvDepthFactor, Dproject_inv_depth)
{
  Vector5 landmark((Vector(5) << 0.1,0.2,0.3, 0.1,0.2).finished());
  double inv_depth(1./4);
  Matrix expected = numericalDerivative33(project_,level_pose, landmark, inv_depth);
  InvDepthCamera3<Cal3_S2> inv_camera(level_pose,K);
  Matrix actual;
  inv_camera.project(landmark, inv_depth, {}, {}, actual);
  EXPECT(assert_equal(expected,actual,1e-7));
}

/* ************************************************************************* */
TEST(InvDepthFactor, backproject)
{
  Vector expected((Vector(5) << 0.,0.,1., 0.1,0.2).finished());
  double inv_depth(1./4);
  InvDepthCamera3<Cal3_S2> inv_camera(level_pose,K);
  Point2 z = inv_camera.project(expected, inv_depth);

  const auto [actual_vec, actual_inv] = inv_camera.backproject(z, 4);
  EXPECT(assert_equal(expected,actual_vec,1e-7));
  EXPECT_DOUBLES_EQUAL(inv_depth,actual_inv,1e-7);
}

/* ************************************************************************* */
TEST(InvDepthFactor, backproject2)
{
  // backwards facing camera
  Vector expected((Vector(5) << -5.,-5.,2., 3., -0.1).finished());
  double inv_depth(1./10);
  InvDepthCamera3<Cal3_S2> inv_camera(Pose3(Rot3::Ypr(1.5,0.1, -1.5), Point3(-5, -5, 2)),K);
  Point2 z = inv_camera.project(expected, inv_depth);

  const auto [actual_vec, actual_inv] = inv_camera.backproject(z, 10);
  EXPECT(assert_equal(expected,actual_vec,1e-7));
  EXPECT_DOUBLES_EQUAL(inv_depth,actual_inv,1e-7);
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */

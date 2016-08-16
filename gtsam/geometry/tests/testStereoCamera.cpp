/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testStereoCamera.cpp
 * @brief   Unit test for StereoCamera
 * single camera
 * @author  Chris Beall
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/StereoCamera.h>

using namespace std;
using namespace gtsam;

GTSAM_CONCEPT_TESTABLE_INST(StereoCamera)
GTSAM_CONCEPT_MANIFOLD_INST(StereoCamera)

/* ************************************************************************* */
TEST( StereoCamera, operators)
{
  StereoPoint2 a(1, 2, 3), b(4, 5, 6), c(5, 7, 9), d(3, 3, 3);
  CHECK(assert_equal(c,a+b));
  CHECK(assert_equal(d,b-a));
}

/* ************************************************************************* */
TEST( StereoCamera, project)
{
  // create a Stereo camera at the origin with focal length 1500, baseline 0.5m
  // and principal point 320, 240 (for a hypothetical 640x480 sensor)
  Cal3_S2Stereo::shared_ptr K(new Cal3_S2Stereo(1500, 1500, 0, 320, 240, 0.5));
  StereoCamera stereoCam(Pose3(), K);

  // point X Y Z in meters
  Point3 p(0, 0, 5);
  StereoPoint2 result = stereoCam.project(p);
  CHECK(assert_equal(StereoPoint2(320, 320 - 150, 240), result));

  // point X Y Z in meters
  Point3 p2(1, 1, 5);
  StereoPoint2 result2 = stereoCam.project(p2);
  CHECK(assert_equal(StereoPoint2(320.0+300.0,320.0+150.0,240.0+300),result2));

  // move forward 1 meter and try the same thing
  // point X Y Z in meters
  Point3 p3(1, 1, 6);
  Rot3 unit = Rot3();
  Point3 one_meter_z(0, 0, 1);
  Pose3 camPose3(unit, one_meter_z);
  StereoCamera stereoCam3(camPose3, K);
  StereoPoint2 result3 = stereoCam3.project(p3);
  CHECK(assert_equal(StereoPoint2(320.0+300.0, 320.0+150.0, 240.0+300),result3));

  // camera at (0,0,1) looking right (90deg)
  Point3 p4(5, 1, 0);
  Rot3 right = Rot3(0, 0, 1, 0, 1, 0, -1, 0, 0);
  Pose3 camPose4(right, one_meter_z);
  StereoCamera stereoCam4(camPose4, K);
  StereoPoint2 result4 = stereoCam4.project(p4);
  CHECK(assert_equal(StereoPoint2(320.0+300.0,320.0+150.0,240.0+300),result4));
}

/* ************************************************************************* */

static Pose3 camPose(Rot3((Matrix3() <<
           1., 0., 0.,
           0.,-1., 0.,
           0., 0.,-1.
           ).finished()),
        Point3(0,0,6.25));

static Cal3_S2Stereo::shared_ptr K(new Cal3_S2Stereo(1500, 1500, 0, 320, 240, 0.5));
static StereoCamera stereoCam(camPose, K);

// point X Y Z in meters
static Point3 landmark(0, 0, 5);

/* ************************************************************************* */
static StereoPoint2 project3(const Pose3& pose, const Point3& point, const Cal3_S2Stereo& K) {
  return StereoCamera(pose, boost::make_shared<Cal3_S2Stereo>(K)).project(point);
}

/* ************************************************************************* */
TEST( StereoCamera, Dproject)
{
  Matrix expected1 = numericalDerivative31(project3, camPose, landmark, *K);
  Matrix actual1; stereoCam.project2(landmark, actual1, boost::none);
  CHECK(assert_equal(expected1,actual1,1e-7));

  Matrix expected2 = numericalDerivative32(project3, camPose, landmark, *K);
  Matrix actual2; stereoCam.project2(landmark, boost::none, actual2);
  CHECK(assert_equal(expected2,actual2,1e-7));
}

/* ************************************************************************* */
TEST( StereoCamera, projectCheirality)
{
  // create a Stereo camera
  Cal3_S2Stereo::shared_ptr K(new Cal3_S2Stereo(1500, 1500, 0, 320, 240, 0.5));
  StereoCamera stereoCam(Pose3(), K);

  // point behind the camera
  Point3 p(0, 0, -5);
#ifdef GTSAM_THROW_CHEIRALITY_EXCEPTION
  CHECK_EXCEPTION(stereoCam.project2(p), StereoCheiralityException);
#else // otherwise project should not throw the exception
  StereoPoint2 expected = StereoPoint2(320, 470, 240);
  CHECK(assert_equal(expected,stereoCam.project2(p),1e-7));
#endif
}

/* ************************************************************************* */
TEST( StereoCamera, backproject_case1)
{
  Cal3_S2Stereo::shared_ptr K2(new Cal3_S2Stereo(1500, 1500, 0, 320, 240, 0.5));
  StereoCamera stereoCam2(Pose3(), K2);

  Point3 expected(1.2, 2.3, 4.5);
  StereoPoint2 stereo_point = stereoCam2.project(expected);
  Point3 actual = stereoCam2.backproject(stereo_point);
  CHECK(assert_equal(expected,actual,1e-8));
}

/* ************************************************************************* */
TEST( StereoCamera, backproject_case2)
{
  Rot3 R(0.589511291,  -0.804859792,  0.0683931805,
         -0.804435942,  -0.592650676,  -0.0405925523,
         0.0732045588,  -0.0310882277,  -0.996832359);
  Point3 t(53.5239823, 23.7866016, -4.42379876);
  Cal3_S2Stereo::shared_ptr K(new Cal3_S2Stereo(1733.75, 1733.75, 0, 689.645, 508.835, 0.0699612));
  StereoCamera camera(Pose3(R,t), K);
  StereoPoint2 z(184.812, 129.068, 714.768);

  Point3 l = camera.backproject(z);
  StereoPoint2 actual = camera.project(l);
  CHECK(assert_equal(z, actual, 1e-3));
}

static Point3 backproject3(const Pose3& pose, const StereoPoint2& point, const Cal3_S2Stereo& K) {
  return StereoCamera(pose, boost::make_shared<Cal3_S2Stereo>(K)).backproject(point);
}

/* ************************************************************************* */
TEST( StereoCamera, backproject2_case1)
{
  Cal3_S2Stereo::shared_ptr K2(new Cal3_S2Stereo(1500, 1500, 0, 320, 240, 0.5));
  StereoCamera stereoCam2(Pose3(), K2);

  Point3 expected_point(1.2, 2.3, 4.5);
  StereoPoint2 stereo_point = stereoCam2.project(expected_point);

  Matrix actual_jacobian_1, actual_jacobian_2;
  Point3 actual_point = stereoCam2.backproject2(stereo_point, actual_jacobian_1, actual_jacobian_2);
  CHECK(assert_equal(expected_point, actual_point, 1e-8));

  Matrix expected_jacobian_to_pose = numericalDerivative31(backproject3, Pose3(), stereo_point, *K2);
  CHECK(assert_equal(expected_jacobian_to_pose, actual_jacobian_1, 1e-6));

  Matrix expected_jacobian_to_point = numericalDerivative32(backproject3, Pose3(), stereo_point, *K2);
  CHECK(assert_equal(expected_jacobian_to_point, actual_jacobian_2, 1e-6));
}

TEST( StereoCamera, backproject2_case2)
{
  Rot3 R(0.589511291,  -0.804859792,  0.0683931805,
         -0.804435942,  -0.592650676,  -0.0405925523,
         0.0732045588,  -0.0310882277,  -0.996832359);
  Point3 t(53.5239823, 23.7866016, -4.42379876);
  Cal3_S2Stereo::shared_ptr K(new Cal3_S2Stereo(1733.75, 1733.75, 0, 689.645, 508.835, 0.0699612));
  StereoCamera camera(Pose3(R,t), K);
  StereoPoint2 z(184.812, 129.068, 714.768);

  Matrix actual_jacobian_1, actual_jacobian_2;
  Point3 l = camera.backproject2(z, actual_jacobian_1, actual_jacobian_2);

  StereoPoint2 actual = camera.project(l);
  CHECK(assert_equal(z, actual, 1e-3));

  Matrix expected_jacobian_to_pose = numericalDerivative31(backproject3, Pose3(R,t), z, *K);
  CHECK(assert_equal(expected_jacobian_to_pose, actual_jacobian_1, 1e-6));

  Matrix expected_jacobian_to_point = numericalDerivative32(backproject3, Pose3(R,t), z, *K);
  CHECK(assert_equal(expected_jacobian_to_point, actual_jacobian_2, 1e-6));
}

/* ************************************************************************* */
  int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */

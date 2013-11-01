/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * testTriangulation.cpp
 *
 *  Created on: July 30th, 2013
 *      Author: cbeall3
 */

#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/Testable.h>
#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Cal3Bundler.h>

#include <gtsam_unstable/geometry/InvDepthCamera3.h>
#include <gtsam_unstable/geometry/triangulation.h>

#include <boost/assign.hpp>
#include <boost/assign/std/vector.hpp>

using namespace std;
using namespace gtsam;
using namespace boost::assign;

/* ************************************************************************* */
TEST( triangulation, twoPosesBundler) {
  Cal3Bundler K(1500, 0, 0, 640, 480);
  // create first camera. Looking along X-axis, 1 meter above ground plane (x-y)
  Pose3 level_pose = Pose3(Rot3::ypr(-M_PI / 2, 0., -M_PI / 2),
      gtsam::Point3(0, 0, 1));
  PinholeCamera<Cal3Bundler> level_camera(level_pose, K);

  // create second camera 1 meter to the right of first camera
  Pose3 level_pose_right = level_pose * Pose3(Rot3(), Point3(1, 0, 0));
  PinholeCamera<Cal3Bundler> level_camera_right(level_pose_right, K);

  // landmark ~5 meters infront of camera
  Point3 landmark(5, 0.5, 1.2);

  // 1. Project two landmarks into two cameras and triangulate
  Point2 level_uv = level_camera.project(landmark);
  Point2 level_uv_right = level_camera_right.project(landmark);

  vector<Pose3> poses;
  vector<Point2> measurements;

  poses += level_pose, level_pose_right;
  measurements += level_uv, level_uv_right;

  bool optimize = true;
  double rank_tol = 1e-9;

  boost::optional<Point3> triangulated_landmark = triangulatePoint3(poses,
      measurements, K, rank_tol, optimize);
  EXPECT(assert_equal(landmark, *triangulated_landmark, 1e-2));

  // 2. Add some noise and try again: result should be ~ (4.995, 0.499167, 1.19814)
  measurements.at(0) += Point2(0.1, 0.5);
  measurements.at(1) += Point2(-0.2, 0.3);

  boost::optional<Point3> triangulated_landmark_noise = triangulatePoint3(poses,
      measurements, K, rank_tol, optimize);
  EXPECT(assert_equal(landmark, *triangulated_landmark_noise, 1e-2));
}

/* ************************************************************************* */
TEST( triangulation, fourPoses) {
  Cal3_S2 K(1500, 1200, 0, 640, 480);
  // create first camera. Looking along X-axis, 1 meter above ground plane (x-y)
  Pose3 level_pose = Pose3(Rot3::ypr(-M_PI / 2, 0., -M_PI / 2),
      gtsam::Point3(0, 0, 1));
  SimpleCamera level_camera(level_pose, K);

  // create second camera 1 meter to the right of first camera
  Pose3 level_pose_right = level_pose * Pose3(Rot3(), Point3(1, 0, 0));
  SimpleCamera level_camera_right(level_pose_right, K);

  // landmark ~5 meters infront of camera
  Point3 landmark(5, 0.5, 1.2);

  // 1. Project two landmarks into two cameras and triangulate
  Point2 level_uv = level_camera.project(landmark);
  Point2 level_uv_right = level_camera_right.project(landmark);

  vector<Pose3> poses;
  vector<Point2> measurements;

  poses += level_pose, level_pose_right;
  measurements += level_uv, level_uv_right;

  boost::optional<Point3> triangulated_landmark = triangulatePoint3(poses,
      measurements, K);
  EXPECT(assert_equal(landmark, *triangulated_landmark, 1e-2));

  // 2. Add some noise and try again: result should be ~ (4.995, 0.499167, 1.19814)
  measurements.at(0) += Point2(0.1, 0.5);
  measurements.at(1) += Point2(-0.2, 0.3);

  boost::optional<Point3> triangulated_landmark_noise = triangulatePoint3(poses,
      measurements, K);
  EXPECT(assert_equal(landmark, *triangulated_landmark_noise, 1e-2));

  // 3. Add a slightly rotated third camera above, again with measurement noise
  Pose3 pose_top = level_pose
      * Pose3(Rot3::ypr(0.1, 0.2, 0.1), Point3(0.1, -2, -.1));
  SimpleCamera camera_top(pose_top, K);
  Point2 top_uv = camera_top.project(landmark);

  poses += pose_top;
  measurements += top_uv + Point2(0.1, -0.1);

  boost::optional<Point3> triangulated_3cameras = triangulatePoint3(poses,
      measurements, K);
  EXPECT(assert_equal(landmark, *triangulated_3cameras, 1e-2));

  // Again with nonlinear optimization
  boost::optional<Point3> triangulated_3cameras_opt = triangulatePoint3(poses,
      measurements, K, 1e-9, true);
  EXPECT(assert_equal(landmark, *triangulated_3cameras_opt, 1e-2));

  // 4. Test failure: Add a 4th camera facing the wrong way
  Pose3 level_pose180 = Pose3(Rot3::ypr(M_PI / 2, 0., -M_PI / 2),
      Point3(0, 0, 1));
  SimpleCamera camera_180(level_pose180, K);

  CHECK_EXCEPTION(camera_180.project(landmark)
  ;, CheiralityException);

  poses += level_pose180;
  measurements += Point2(400, 400);

  CHECK_EXCEPTION(triangulatePoint3(poses, measurements, K),
      TriangulationCheiralityException);
}

/* ************************************************************************* */
TEST( triangulation, fourPoses_distinct_Ks) {
  Cal3_S2 K1(1500, 1200, 0, 640, 480);
  // create first camera. Looking along X-axis, 1 meter above ground plane (x-y)
  Pose3 level_pose = Pose3(Rot3::ypr(-M_PI / 2, 0., -M_PI / 2),
      gtsam::Point3(0, 0, 1));
  SimpleCamera level_camera(level_pose, K1);

  // create second camera 1 meter to the right of first camera
  Pose3 level_pose_right = level_pose * Pose3(Rot3(), Point3(1, 0, 0));
  Cal3_S2 K2(1600, 1300, 0, 650, 440);
  SimpleCamera level_camera_right(level_pose_right, K2);

  // landmark ~5 meters infront of camera
  Point3 landmark(5, 0.5, 1.2);

  // 1. Project two landmarks into two cameras and triangulate
  Point2 level_uv = level_camera.project(landmark);
  Point2 level_uv_right = level_camera_right.project(landmark);

  vector<Pose3> poses;
  vector<Point2> measurements;

  poses += level_pose, level_pose_right;
  measurements += level_uv, level_uv_right;

  std::vector<boost::shared_ptr<Cal3_S2> > Ks;
  Ks.push_back(boost::make_shared<Cal3_S2>(K1));
  Ks.push_back(boost::make_shared<Cal3_S2>(K2));

  boost::optional<Point3> triangulated_landmark = triangulatePoint3(poses,
      measurements, Ks);
  EXPECT(assert_equal(landmark, *triangulated_landmark, 1e-2));

  // 2. Add some noise and try again: result should be ~ (4.995, 0.499167, 1.19814)
  measurements.at(0) += Point2(0.1, 0.5);
  measurements.at(1) += Point2(-0.2, 0.3);

  boost::optional<Point3> triangulated_landmark_noise = triangulatePoint3(poses,
      measurements, Ks);
  EXPECT(assert_equal(landmark, *triangulated_landmark_noise, 1e-2));

  // 3. Add a slightly rotated third camera above, again with measurement noise
  Pose3 pose_top = level_pose
      * Pose3(Rot3::ypr(0.1, 0.2, 0.1), Point3(0.1, -2, -.1));
  Cal3_S2 K3(700, 500, 0, 640, 480);
  SimpleCamera camera_top(pose_top, K3);
  Point2 top_uv = camera_top.project(landmark);

  poses += pose_top;
  measurements += top_uv + Point2(0.1, -0.1);
  Ks.push_back(boost::make_shared<Cal3_S2>(K3));

  boost::optional<Point3> triangulated_3cameras = triangulatePoint3(poses,
      measurements, Ks);
  EXPECT(assert_equal(landmark, *triangulated_3cameras, 1e-2));

  // Again with nonlinear optimization
  boost::optional<Point3> triangulated_3cameras_opt = triangulatePoint3(poses,
      measurements, Ks, 1e-9, true);
  EXPECT(assert_equal(landmark, *triangulated_3cameras_opt, 1e-2));

  // 4. Test failure: Add a 4th camera facing the wrong way
  Pose3 level_pose180 = Pose3(Rot3::ypr(M_PI / 2, 0., -M_PI / 2),
      Point3(0, 0, 1));
  Cal3_S2 K4(700, 500, 0, 640, 480);
  SimpleCamera camera_180(level_pose180, K4);

  CHECK_EXCEPTION(camera_180.project(landmark)
  ;, CheiralityException);

  poses += level_pose180;
  measurements += Point2(400, 400);
  Ks.push_back(boost::make_shared<Cal3_S2>(K4));

  CHECK_EXCEPTION(triangulatePoint3(poses, measurements, Ks),
      TriangulationCheiralityException);
}

/* ************************************************************************* */
TEST( triangulation, twoIdenticalPoses) {
  Cal3_S2 K(1500, 1200, 0, 640, 480);
  // create first camera. Looking along X-axis, 1 meter above ground plane (x-y)
  Pose3 level_pose = Pose3(Rot3::ypr(-M_PI / 2, 0., -M_PI / 2),
      gtsam::Point3(0, 0, 1));
  SimpleCamera level_camera(level_pose, K);

  // landmark ~5 meters infront of camera
  Point3 landmark(5, 0.5, 1.2);

  // 1. Project two landmarks into two cameras and triangulate
  Point2 level_uv = level_camera.project(landmark);

  vector<Pose3> poses;
  vector<Point2> measurements;

  poses += level_pose, level_pose;
  measurements += level_uv, level_uv;

  CHECK_EXCEPTION(triangulatePoint3(poses, measurements, K),
      TriangulationUnderconstrainedException);
}

/* ************************************************************************* */
TEST( triangulation, onePose) {
  // we expect this test to fail with a TriangulationUnderconstrainedException
  // because there's only one camera observation

  Cal3_S2 K(1500, 1200, 0, 640, 480);

  vector<Pose3> poses;
  vector<Point2> measurements;

  poses += Pose3();
  measurements += Point2();

  CHECK_EXCEPTION(triangulatePoint3(poses, measurements, K),
      TriangulationUnderconstrainedException);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

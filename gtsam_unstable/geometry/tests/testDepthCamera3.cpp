
/*
 * testDepthFactor.cpp
 *
 *  Created on: Nov 9, 2025
 *      Author: junlinp
 */

#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/Testable.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Cal3_S2.h>

#include <gtsam_unstable/geometry/DepthCamera3.h>

using namespace std;
using namespace gtsam;

static Cal3_S2::shared_ptr K(new Cal3_S2(1500, 1200, 0, 640, 480));
Pose3 level_pose = Pose3(Rot3::Ypr(0, 0, 0), gtsam::Point3(0,0,1));
PinholeCamera<Cal3_S2> level_camera(level_pose, *K);

/* ************************************************************************* */
TEST(DepthCamera3, Project1) {
  double depth = 3.0;
  // landmark directly in front of camera at depth
  Point3 landmark(0, 0, level_pose.z() + depth);
  Point2 expected_uv = level_camera.project(landmark);
  
  Point3 measurement(expected_uv.x(), expected_uv.y(), depth); // z = depth
  cout << "measurement: " << measurement.transpose() << endl;
  DepthCamera3<Cal3_S2> depth_camera(measurement, K);

  Point3 actual_error = depth_camera.project(level_pose, landmark);

  // When landmark matches the measurement, error should be approximately zero
  Vector3 expected_zero = Vector3::Zero();
  EXPECT(assert_equal(expected_zero, Vector3(actual_error), 1e-6));
}

/* ************************************************************************* */
TEST(DepthCamera3, ProjectWithJacobians) {
  double depth = 3.0;
  Point3 landmark(5, 3, level_pose.z() + depth);
  Point2 expected_uv = level_camera.project(landmark);
  Point3 measurement(expected_uv.x(), expected_uv.y(), depth);
  
  DepthCamera3<Cal3_S2> depth_camera(measurement, K);

  // Compute analytic Jacobians
  Matrix H1, H2;
  depth_camera.project(level_pose, landmark, H1, H2);

  // Numerical Jacobians
  auto f1 = [&](const Pose3& pose) {
    return depth_camera.project(pose, landmark);
  };
  auto f2 = [&](const Point3& lm) {
    return depth_camera.project(level_pose, lm);
  };

  Matrix H1_num = numericalDerivative11<Point3, Pose3>(f1, level_pose, 1e-6);
  Matrix H2_num = numericalDerivative11<Point3, Point3>(f2, landmark, 1e-6);

  EXPECT(assert_equal(H1_num, H1, 1e-5));
  EXPECT(assert_equal(H2_num, H2, 1e-5));
}

/* ************************************************************************* */
TEST(DepthCamera3, ProjectWithBodyPSensor) {
  // Create a body_P_sensor transform (rotation and translation)
  Pose3 body_P_sensor(Rot3::RzRyRx(-M_PI_2, 0.0, -M_PI_2), Point3(0.25, -0.10, 1.0));
  
  // Body pose in world frame
  Pose3 body_pose_world = level_pose;
  
  // Camera pose in world frame (body pose composed with body_P_sensor)
  Pose3 camera_pose_world = body_pose_world.compose(body_P_sensor);
  PinholeCamera<Cal3_S2> camera_with_transform(camera_pose_world, *K);
  
  double depth = 3.0;
  // landmark directly in front of camera at depth (in camera frame: (0, 0, depth))
  Point3 landmark_cam(0, 0, depth);
  // Transform to world frame
  Point3 landmark = camera_pose_world.transformFrom(landmark_cam);
  Point2 expected_uv = camera_with_transform.project(landmark);
  
  Point3 measurement(expected_uv.x(), expected_uv.y(), depth);
  
  // Create DepthCamera3 with body_P_sensor
  DepthCamera3<Cal3_S2> depth_camera(measurement, K, body_P_sensor);
  
  // Evaluate error using body pose (not camera pose)
  Point3 actual_error = depth_camera.project(body_pose_world, landmark);
  
  // When landmark matches the measurement, error should be approximately zero
  Vector3 expected_zero = Vector3::Zero();
  EXPECT(assert_equal(expected_zero, Vector3(actual_error), 1e-6));
}

/* ************************************************************************* */
TEST(DepthCamera3, ProjectWithBodyPSensorJacobians) {
  // Create a body_P_sensor transform
  Pose3 body_P_sensor(Rot3::RzRyRx(-M_PI_2, 0.0, -M_PI_2), Point3(0.25, -0.10, 1.0));
  
  // Body pose in world frame
  Pose3 body_pose_world = level_pose;
  
  // Camera pose in world frame
  Pose3 camera_pose_world = body_pose_world.compose(body_P_sensor);
  PinholeCamera<Cal3_S2> camera_with_transform(camera_pose_world, *K);
  
  double depth = 3.0;
  // landmark in camera frame: (5, 3, depth)
  Point3 landmark_cam(5, 3, depth);
  // Transform to world frame
  Point3 landmark = camera_pose_world.transformFrom(landmark_cam);
  Point2 expected_uv = camera_with_transform.project(landmark);
  Point3 measurement(expected_uv.x(), expected_uv.y(), depth);
  
  // Create DepthCamera3 with body_P_sensor
  DepthCamera3<Cal3_S2> depth_camera(measurement, K, body_P_sensor);

  // Compute analytic Jacobians (w.r.t. body pose)
  Matrix H1, H2;
  depth_camera.project(body_pose_world, landmark, H1, H2);

  // Numerical Jacobians
  auto f1 = [&](const Pose3& pose) {
    return depth_camera.project(pose, landmark);
  };
  auto f2 = [&](const Point3& lm) {
    return depth_camera.project(body_pose_world, lm);
  };

  Matrix H1_num = numericalDerivative11<Point3, Pose3>(f1, body_pose_world, 1e-6);
  Matrix H2_num = numericalDerivative11<Point3, Point3>(f2, landmark, 1e-6);

  EXPECT(assert_equal(H1_num, H1, 1e-5));
  EXPECT(assert_equal(H2_num, H2, 1e-5));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */

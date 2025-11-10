/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file  testDepthFactor3.cpp
 *  @brief Unit tests for depth factor with 3D landmarks
 *
 *  @author junlinp
 *  @date   Nov 9, 2025
 */

#include <CppUnitLite/TestHarness.h>

#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/numericalDerivative.h>

#include <gtsam/slam/DepthFactor3.h>
#include <gtsam/nonlinear/PriorFactor.h>

using namespace std;
using namespace gtsam;

static Cal3_S2::shared_ptr K(new Cal3_S2(1500, 1200, 0, 640, 480));
static SharedNoiseModel sigma(noiseModel::Unit::Create(3));
static SharedNoiseModel sigma_pose(noiseModel::Isotropic::Sigma(6, 1e-6));

// camera pose at (0,0,1) looking straight along the z-axis.
Pose3 level_pose = Pose3(Rot3::Ypr(0, 0, 0), gtsam::Point3(0,0,1));
PinholeCamera<Cal3_S2> level_camera(level_pose, *K);

typedef DepthFactor3<Pose3, Point3> DepthFactor;
typedef NonlinearEquality<Pose3> PoseConstraint;

Vector factorError(const Pose3& pose, const Point3& landmark,
                     const DepthFactor& factor) {
  return factor.evaluateError(pose, landmark);
}

/* ************************************************************************* */
TEST(DepthFactor3, zeroErrorWhenConsistent) {
  double depth = 3.0;
  // landmark directly in front of camera at depth
  Point3 landmark(0, 0, level_pose.z() + depth);
  Point2 expected_uv = level_camera.project(landmark);
  
  Point3 measurement(expected_uv.x(), expected_uv.y(), depth); // z = depth
  
  DepthFactor factor(measurement, sigma, Symbol('x',1), Symbol('l',1), K);
  
  Vector actual_error = factor.evaluateError(level_pose, landmark);
  
  // When landmark matches the measurement, error should be approximately zero
  EXPECT(assert_equal(Vector3::Zero(), actual_error, 1e-6));
}

/* ************************************************************************* */
TEST(DepthFactor3, Jacobians) {
  double depth = 3.0;
  Point3 landmark(5, 3, level_pose.z() + depth);
  Point2 expected_uv = level_camera.project(landmark);
  Point3 measurement(expected_uv.x(), expected_uv.y(), depth);
  
  DepthFactor factor(measurement, sigma, Symbol('x',1), Symbol('l',1), K);

  // Compute analytic Jacobians
  Matrix H1, H2;
  factor.evaluateError(level_pose, landmark, H1, H2);

  // Numerical Jacobians
  auto f1 = [&](const Pose3& pose) {
    return factor.evaluateError(pose, landmark);
  };
  auto f2 = [&](const Point3& lm) {
    return factor.evaluateError(level_pose, lm);
  };

  Matrix H1_num = numericalDerivative11<Vector, Pose3>(f1, level_pose, 1e-6);
  Matrix H2_num = numericalDerivative11<Vector, Point3>(f2, landmark, 1e-6);

  EXPECT(assert_equal(H1_num, H1, 1e-5));
  EXPECT(assert_equal(H2_num, H2, 1e-5));
}

/* ************************************************************************* */
TEST(DepthFactor3, optimize) {
  // landmark 5 meters in front of camera (camera center at (0,0,1), looking along +z)
  Point3 landmark(0, 0, level_pose.z() + 5.0); // 5 meters in front
  double depth = 5.0;

  // get expected projection using pinhole camera
  Point2 expected_uv = level_camera.project(landmark);
  Point3 measurement(expected_uv.x(), expected_uv.y(), depth);

  gtsam::NonlinearFactorGraph graph;
  Values initial;

  graph.emplace_shared<DepthFactor>(measurement, sigma,
      Symbol('x',1), Symbol('l',1), K);
  graph.emplace_shared<PoseConstraint>(Symbol('x', 1), level_pose);
  
  // Initialize with slightly perturbed landmark
  Point3 initial_landmark = Point3(0.1, 0.1, 0.1);
  initial.insert(Symbol('x',1), level_pose);
  initial.insert(Symbol('l',1), initial_landmark);

  LevenbergMarquardtParams lmParams;
  Values result = LevenbergMarquardtOptimizer(graph, initial, lmParams).optimize();

  // Verify that the landmark was optimized to the correct position
  Point3 result_landmark = result.at<Point3>(Symbol('l',1));
  EXPECT(assert_equal(landmark, result_landmark, 1e-6));
}

/* ************************************************************************* */
TEST(DepthFactor3, zeroErrorWhenConsistentWithBodyPSensor) {
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
  
  // Create factor with body_P_sensor
  DepthFactor factor(measurement, sigma, Symbol('x',1), Symbol('l',1), K, body_P_sensor);
  
  // Evaluate error using body pose (not camera pose)
  Vector actual_error = factor.evaluateError(body_pose_world, landmark);
  
  // When landmark matches the measurement, error should be approximately zero
  EXPECT(assert_equal(Vector3::Zero(), actual_error, 1e-6));
}

/* ************************************************************************* */
TEST(DepthFactor3, JacobiansWithBodyPSensor) {
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
  
  // Create factor with body_P_sensor
  DepthFactor factor(measurement, sigma, Symbol('x',1), Symbol('l',1), K, body_P_sensor);

  // Compute analytic Jacobians (w.r.t. body pose)
  Matrix H1, H2;
  factor.evaluateError(body_pose_world, landmark, H1, H2);

  // Numerical Jacobians
  auto f1 = [&](const Pose3& pose) {
    return factor.evaluateError(pose, landmark);
  };
  auto f2 = [&](const Point3& lm) {
    return factor.evaluateError(body_pose_world, lm);
  };

  Matrix H1_num = numericalDerivative11<Vector, Pose3>(f1, body_pose_world, 1e-6);
  Matrix H2_num = numericalDerivative11<Vector, Point3>(f2, landmark, 1e-6);

  EXPECT(assert_equal(H1_num, H1, 1e-5));
  EXPECT(assert_equal(H2_num, H2, 1e-5));
}

/* ************************************************************************* */
TEST(DepthFactor3, optimizeWithBodyPSensor) {
  // Create a body_P_sensor transform
  Pose3 body_P_sensor(Rot3::RzRyRx(-M_PI_2, 0.0, -M_PI_2), Point3(0.25, -0.10, 1.0));
  
  // Body pose in world frame
  Pose3 body_pose_world = level_pose;
  
  // Camera pose in world frame
  Pose3 camera_pose_world = body_pose_world.compose(body_P_sensor);
  PinholeCamera<Cal3_S2> camera_with_transform(camera_pose_world, *K);
  
  double depth = 5.0;
  // landmark 5 meters in front of camera (in camera frame: (0, 0, depth))
  Point3 landmark_cam(0, 0, depth);
  // Transform to world frame
  Point3 landmark = camera_pose_world.transformFrom(landmark_cam);

  // get expected projection using pinhole camera
  Point2 expected_uv = camera_with_transform.project(landmark);
  Point3 measurement(expected_uv.x(), expected_uv.y(), depth);

  gtsam::NonlinearFactorGraph graph;
  Values initial;

  // Create factor with body_P_sensor
  graph.emplace_shared<DepthFactor>(measurement, sigma,
      Symbol('x',1), Symbol('l',1), K, body_P_sensor);
  graph.emplace_shared<PoseConstraint>(Symbol('x', 1), body_pose_world);
  
  // Initialize with slightly perturbed landmark
  Point3 initial_landmark = landmark + Point3(0.1, 0.1, 0.1);
  initial.insert(Symbol('x',1), body_pose_world);
  initial.insert(Symbol('l',1), initial_landmark);

  LevenbergMarquardtParams lmParams;
  Values result = LevenbergMarquardtOptimizer(graph, initial, lmParams).optimize();

  // Verify that the landmark was optimized to the correct position
  Point3 result_landmark = result.at<Point3>(Symbol('l',1));
  EXPECT(assert_equal(landmark, result_landmark, 1e-6));
}

TEST(DepthFactor3, optimizeWithBodyPSensor_ba) {
  // Create a body_P_sensor transform
  Pose3 body_P_sensor(Rot3::RzRyRx(-M_PI_2, 0.0, -M_PI_2), Point3(0.0, 0.0, 0.0));
  
  // Define body poses in world frame
  Pose3 ground_truth_body1_pose = Pose3(Rot3::RzRyRx(0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));
  Pose3 ground_truth_body2_pose = Pose3(Rot3::RzRyRx(0, 0.0, 0.0), Point3(1.0, 0.0, 0.0));
  std::vector<Pose3> ground_truth_body_poses = {
    ground_truth_body1_pose,
    ground_truth_body2_pose,
  };
  
  // Compute camera poses from body poses
  std::vector<Pose3> ground_truth_camera_poses;
  for (const auto& body_pose : ground_truth_body_poses) {
    ground_truth_camera_poses.push_back(body_pose.compose(body_P_sensor));
  }
  
  std::vector<Point3> landmarks = {
    Point3(5, 0, 0),
    Point3(6, 0, 1),
    Point3(7, 1, 0),
    Point3(8, 0, 1),
  };

  gtsam::NonlinearFactorGraph graph;
  Values initial;

  // Insert body poses into initial values
  for (size_t j = 0; j < ground_truth_body_poses.size(); j++) {
    Key body_key = Symbol('x', j);
    // Use ground truth as initial (or add very small noise if needed)
    Pose3 noisy_body_pose = ground_truth_body_poses[j].compose(Pose3(Rot3::Ypr(0.001, 0.001, 0.001), Point3(0.001, 0.001, 0.001)));
    initial.insert(body_key, noisy_body_pose);
    if (j == 0) {
      graph.emplace_shared<gtsam::PriorFactor<Pose3>>(body_key, ground_truth_body_poses[j], sigma_pose);
    }
  }

  for (size_t i = 0; i < landmarks.size(); i++) {
    const auto& landmark = landmarks[i];
    Key landmark_key = Symbol('l', i);
    for (size_t j = 0; j < ground_truth_camera_poses.size(); j++) {
      Key body_key = Symbol('x', j);
      const auto& camera_pose = ground_truth_camera_poses[j];
      
      // Create camera for projection
      PinholeCamera<Cal3_S2> camera(camera_pose, *K);
      
      // Project landmark to get pixel coordinates
      Point2 expected_uv = camera.project(landmark);
      
      // Compute depth in camera frame
      Point3 landmark_cam = camera_pose.transformTo(landmark);
      double depth = landmark_cam.z();
      
      Point3 measurement(expected_uv.x(), expected_uv.y(), depth);
      
      // Create factor with body_P_sensor (using body key, not camera key)
      graph.emplace_shared<DepthFactor>(measurement, sigma, body_key, landmark_key, K, body_P_sensor);
    }
    Point3 noised_landmark = landmark + Point3(0.001, 0.001, 0.001);
    initial.insert(landmark_key, noised_landmark);
  }

  LevenbergMarquardtParams lmParams;
  Values result = LevenbergMarquardtOptimizer(graph, initial, lmParams).optimize();

  // Verify that the landmark was optimized to the correct position
  for (size_t i = 0; i < landmarks.size(); i++) {
    Point3 result_landmark = result.at<Point3>(Symbol('l', i));
    EXPECT(assert_equal(landmarks[i], result_landmark, 1e-6));
  }

  // Verify that the body poses were optimized to the correct position
  for (size_t i = 0; i < ground_truth_body_poses.size(); i++) {
    Key body_key = Symbol('x', i);
    Pose3 result_body_pose = result.at<Pose3>(body_key);
    EXPECT(assert_equal(ground_truth_body_poses[i], result_body_pose, 1e-6));
  }
}
/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */


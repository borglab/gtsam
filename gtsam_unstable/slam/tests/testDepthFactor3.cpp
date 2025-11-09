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

#include <gtsam_unstable/slam/DepthFactor3.h>

using namespace std;
using namespace gtsam;

static Cal3_S2::shared_ptr K(new Cal3_S2(1500, 1200, 0, 640, 480));
static SharedNoiseModel sigma(noiseModel::Unit::Create(3));

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
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */


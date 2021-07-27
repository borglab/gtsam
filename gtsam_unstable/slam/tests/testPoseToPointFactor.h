/**
 * @file   testPoseToPointFactor.cpp
 * @brief
 * @author David Wisth
 * @date   June 20, 2020
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam_unstable/slam/PoseToPointFactor.h>

using namespace gtsam;
using namespace gtsam::noiseModel;

/// Verify zero error when there is no noise
TEST(PoseToPointFactor, errorNoiseless) {
  Pose3 pose = Pose3::identity();
  Point3 point(1.0, 2.0, 3.0);
  Point3 noise(0.0, 0.0, 0.0);
  Point3 measured = t + noise;

  Key pose_key(1);
  Key point_key(2);
  PoseToPointFactor factor(pose_key, point_key, measured,
                           Isotropic::Sigma(3, 0.05));
  Vector expectedError = Vector3(0.0, 0.0, 0.0);
  Vector actualError = factor.evaluateError(pose, point);
  EXPECT(assert_equal(expectedError, actualError, 1E-5));
}

/// Verify expected error in test scenario
TEST(PoseToPointFactor, errorNoise) {
  Pose3 pose = Pose3::identity();
  Point3 point(1.0, 2.0, 3.0);
  Point3 noise(-1.0, 0.5, 0.3);
  Point3 measured = t + noise;

  Key pose_key(1);
  Key point_key(2);
  PoseToPointFactor factor(pose_key, point_key, measured,
                           Isotropic::Sigma(3, 0.05));
  Vector expectedError = noise;
  Vector actualError = factor.evaluateError(pose, point);
  EXPECT(assert_equal(expectedError, actualError, 1E-5));
}

/// Check Jacobians are correct
TEST(PoseToPointFactor, jacobian) {
  // Measurement
  gtsam::Point3 l_meas = gtsam::Point3(1, 2, 3);

  // Linearisation point
  gtsam::Point3 p_t = gtsam::Point3(-5, 12, 2);
  gtsam::Rot3 p_R = gtsam::Rot3::RzRyRx(1.5 * M_PI, -0.3 * M_PI, 0.4 * M_PI);
  Pose3 p(p_R, p_t);

  gtsam::Point3 l = gtsam::Point3(3, 0, 5);

  // Factor
  Key pose_key(1);
  Key point_key(2);
  SharedGaussian noise = noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.1));
  PoseToPointFactor factor(pose_key, point_key, l_meas, noise);

  // Calculate numerical derivatives
  auto f = boost::bind(&PoseToPointFactor::evaluateError, factor, _1, _2,
                       boost::none, boost::none);
  Matrix numerical_H1 = numericalDerivative21<Vector, Pose3, Point3>(f, p, l);
  Matrix numerical_H2 = numericalDerivative22<Vector, Pose3, Point3>(f, p, l);

  // Use the factor to calculate the derivative
  Matrix actual_H1;
  Matrix actual_H2;
  factor.evaluateError(p, l, actual_H1, actual_H2);

  // Verify we get the expected error
  EXPECT_TRUE(assert_equal(numerical_H1, actual_H1, 1e-8));
  EXPECT_TRUE(assert_equal(numerical_H2, actual_H2, 1e-8));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

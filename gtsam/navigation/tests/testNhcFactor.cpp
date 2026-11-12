/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   testNhcFactor.cpp
 *  @author inuex35
 *  @brief  Unit tests for the non-holonomic constraint (NHC) factors
 *  @date   July 4, 2026
 **/

#include <CppUnitLite/TestHarness.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/NhcFactor.h>
#include <gtsam/navigation/Scenario.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>

using namespace gtsam;
using symbol_shorthand::M;
using symbol_shorthand::V;
using symbol_shorthand::X;

/* ************************************************************************* */
namespace generic_state {

const Vector3 kAngularVelocity(0.01, -0.02, 0.05);
const Vector3 kLeverArm(-1.3, 0.1, 0.2);
const SharedNoiseModel kModel = noiseModel::Diagonal::Sigmas(Vector2(0.05, 0.05));
const Pose3 kPose(Rot3::RzRyRx(0.1, -0.2, 0.3), Point3(1, 2, 3));
const Vector3 kVelocity(2.0, 0.3, -0.1);
const Rot3 kMountRotation = Rot3::RzRyRx(0.02, -0.01, 0.03);

// Pure forward motion of the body has zero error at any speed.
TEST(NhcFactor, ZeroErrorForForwardMotion) {
  const NhcFactor factor(X(0), V(0), Vector3::Zero(), Vector3::Zero(), kModel);
  const Vector3 forwardVelocity = kPose.rotation().rotate(Vector3(7.0, 0, 0));
  EXPECT(assert_equal(Vector(Vector2::Zero()),
                      factor.evaluateError(kPose, forwardVelocity), 1e-9));
}

// The error is the lateral and vertical body velocity when frames are aligned.
TEST(NhcFactor, ErrorIsLateralAndVerticalVelocity) {
  const NhcFactor factor(X(0), V(0), Vector3::Zero(), Vector3::Zero(), kModel);
  const Vector3 worldVelocity = kPose.rotation().rotate(Vector3(5.0, 0.2, -0.3));
  EXPECT(assert_equal(Vector(Vector2(0.2, -0.3)),
                      factor.evaluateError(kPose, worldVelocity), 1e-9));
}

// Analytic Jacobians of NhcFactor match numerical derivatives.
TEST(NhcFactor, Jacobians) {
  const NhcFactor factor(X(0), V(0), kAngularVelocity, kLeverArm, kModel,
                         kMountRotation);
  Values values;
  values.insert(X(0), kPose);
  values.insert(V(0), kVelocity);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-5, 1e-5);
}

// Analytic Jacobians of NhcFactorCalib match numerical derivatives.
TEST(NhcFactorCalib, Jacobians) {
  const NhcFactorCalib factor(X(0), V(0), M(0), kAngularVelocity, kLeverArm,
                              kModel);
  Values values;
  values.insert(X(0), kPose);
  values.insert(V(0), kVelocity);
  values.insert(M(0), kMountRotation);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-5, 1e-5);
}

// NhcFactor with a fixed mounting rotation gives the same error as
// NhcFactorCalib evaluated at that rotation.
TEST(NhcFactor, MatchesCalibAtFixedMounting) {
  const NhcFactor fixedFactor(X(0), V(0), kAngularVelocity, kLeverArm, kModel,
                              kMountRotation);
  const NhcFactorCalib calibFactor(X(0), V(0), M(0), kAngularVelocity,
                                   kLeverArm, kModel);
  EXPECT(assert_equal(fixedFactor.evaluateError(kPose, kVelocity),
                      calibFactor.evaluateError(kPose, kVelocity,
                                                kMountRotation),
                      1e-12));
}

// When the constraint holds, rotation of the mounting about the forward axis
// (roll) does not change the error, so that Jacobian column vanishes.
TEST(NhcFactorCalib, ForwardAxisUnobservable) {
  const NhcFactorCalib factor(X(0), V(0), M(0), Vector3::Zero(),
                              Vector3::Zero(), kModel);
  const Vector3 forwardVelocity = kPose.rotation().rotate(Vector3(7.0, 0, 0));
  Matrix H_pose, H_velocity, H_mountRotation;
  factor.evaluateError(kPose, forwardVelocity, Rot3(), H_pose, H_velocity,
                       H_mountRotation);
  EXPECT(assert_equal(Vector(Vector2::Zero()), Vector(H_mountRotation.col(0)),
                      1e-9));
}

}  // namespace generic_state
/* ************************************************************************* */
namespace turning_vehicle {

// A vehicle whose reference point drives forward at 8 m/s while turning left.
const double kSpeed = 8.0;
const ConstantTwistScenario kScenario(Vector3(0.0, 0.0, 0.15),
                                      Vector3(kSpeed, 0.0, 0.0));
// The IMU sits ahead of, beside, and above the reference point, so the lever
// arm from the IMU to the reference point is the negation of this offset.
const Vector3 kImuOffset(1.3, 0.4, 0.5);

// With the IMU offset from the reference point, the IMU itself moves sideways
// during the turn, but the lever-arm correction makes the error vanish.
TEST(NhcFactor, LeverArmCompensatesTurn) {
  for (double t = 0.0; t < 4.0; t += 0.5) {
    const Vector3 omega = kScenario.omega_b(t);
    const NhcFactor factor(X(0), V(0), omega, -kImuOffset,
                           noiseModel::Isotropic::Sigma(2, 0.05));
    const Pose3 vehiclePose = kScenario.pose(t);
    const Pose3 imuPose = vehiclePose * Pose3(Rot3(), kImuOffset);
    const Vector3 imuVelocity = vehiclePose.rotation().rotate(
        Vector3(kSpeed, 0.0, 0.0) + omega.cross(kImuOffset));
    EXPECT(assert_equal(Vector(Vector2::Zero()),
                        factor.evaluateError(imuPose, imuVelocity), 1e-9));
  }
}

}  // namespace turning_vehicle
/* ************************************************************************* */
namespace optimization {

// With a fixed pose and a loose velocity prior, the NHC removes the lateral and
// vertical velocity while keeping the forward speed from the prior.
TEST(NhcFactor, ProjectsVelocityOntoForwardAxis) {
  const Rot3 wRb = Rot3::Yaw(30.0 * M_PI / 180.0);
  const Pose3 pose(wRb, Point3(10, 20, 0));
  const Vector3 measuredVelocity = wRb.rotate(Vector3(5.0, 1.0, 0.5));

  NonlinearFactorGraph graph;
  graph.addPrior(X(0), pose, noiseModel::Isotropic::Sigma(6, 1e-6));
  graph.addPrior(V(0), measuredVelocity, noiseModel::Isotropic::Sigma(3, 1.0));
  graph.emplace_shared<NhcFactor>(X(0), V(0), Vector3::Zero(), Vector3::Zero(),
                                  noiseModel::Isotropic::Sigma(2, 1e-3));

  Values initial;
  initial.insert(X(0), pose);
  initial.insert(V(0), measuredVelocity);
  const Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();

  EXPECT(assert_equal(Vector3(wRb.rotate(Vector3(5.0, 0, 0))),
                      result.at<Vector3>(V(0)), 1e-4));
}

// Epochs at several headings recover the yaw of the mounting rotation.
TEST(NhcFactorCalib, RecoverMountRotation) {
  const double speed = 4.0;
  const Rot3 trueMountRotation = Rot3::Yaw(0.08);

  NonlinearFactorGraph graph;
  Values initial;
  initial.insert(M(0), Rot3());
  // A loose prior keeps the unobservable roll finite.
  graph.addPrior(M(0), Rot3(), noiseModel::Isotropic::Sigma(3, 1.0));

  const double headings[] = {0.0, 0.5, 1.0, -0.7, 2.0};
  for (size_t i = 0; i < 5; ++i) {
    const Pose3 pose(Rot3::Yaw(headings[i]), Point3::Zero());
    const Vector3 bodyVelocity = trueMountRotation.rotate(Vector3(speed, 0, 0));
    const Vector3 worldVelocity = pose.rotation().rotate(bodyVelocity);

    graph.addPrior(X(i), pose, noiseModel::Isotropic::Sigma(6, 1e-6));
    graph.addPrior(V(i), worldVelocity, noiseModel::Isotropic::Sigma(3, 1e-6));
    graph.emplace_shared<NhcFactorCalib>(
        X(i), V(i), M(0), Vector3::Zero(), Vector3::Zero(),
        noiseModel::Isotropic::Sigma(2, 1e-2));
    initial.insert(X(i), pose);
    initial.insert(V(i), worldVelocity);
  }

  const Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();
  EXPECT_DOUBLES_EQUAL(trueMountRotation.yaw(), result.at<Rot3>(M(0)).yaw(),
                       1e-3);
}

}  // namespace optimization
/* ************************************************************************* */

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}

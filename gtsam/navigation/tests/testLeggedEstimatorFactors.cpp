/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file testLeggedEstimatorFactors.cpp
 * @date February 2026
 * @brief Unit tests for the legged estimator factor Jacobians.
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/navigation/LeggedEstimator.h>
#include <gtsam/navigation/LeggedEstimatorFactors.h>

using namespace gtsam;

namespace {

constexpr int kNumFeet = 2;
constexpr int kExtendedPoseDim = 3 + 3 * (2 + kNumFeet);

NavState sampleNavState() {
  return NavState(Rot3::RzRyRx(0.1, -0.2, 0.05), Point3(0.2, -0.4, 0.3),
                  Vector3(0.6, -0.1, 0.2));
}

Matrix sampleFootholds() {
  return (Matrix(3, 2) << 1.2, -0.3, 0.4, 0.8, -0.5, 0.1).finished();
}

ExtendedPose3d sampleExtendedPoseState() {
  Matrix blocks(3, 2 + kNumFeet);
  const NavState X = sampleNavState();
  blocks.col(0) = X.position();
  blocks.col(1) = X.velocity();
  blocks.rightCols(kNumFeet) = sampleFootholds();
  return ExtendedPose3d(X.attitude(), blocks);
}

}  // namespace

/* ************************************************************************* */
TEST(LeggedEstimatorFactors, ExtendedPoseContactFactorJacobian) {
  const ExtendedPose3d state = sampleExtendedPoseState();
  ExtendedPoseContactFactor factor(0, 3, Point3(0.15, 0.05, -0.2),
                                   noiseModel::Unit::Create(3));

  Matrix actual_H_state;
  factor.evaluateError(state, actual_H_state);
  const Matrix expected_H_state =
      numericalDerivative11<Vector, ExtendedPose3d, kExtendedPoseDim>(
          [&](const ExtendedPose3d& x) { return factor.evaluateError(x, {}); },
          state, 1e-6);

  EXPECT(assert_equal(expected_H_state, actual_H_state, 1e-6));
}

/* ************************************************************************* */
TEST(LeggedEstimatorFactors, ExtendedPoseHeightFactorJacobian) {
  const ExtendedPose3d state = sampleExtendedPoseState();
  ExtendedPoseHeightFactor factor(0, 2, -0.35, noiseModel::Unit::Create(1));

  Matrix actual_H_state;
  factor.evaluateError(state, actual_H_state);
  const Matrix expected_H_state =
      numericalDerivative11<Vector, ExtendedPose3d, kExtendedPoseDim>(
          [&](const ExtendedPose3d& x) { return factor.evaluateError(x, {}); },
          state, 1e-6);

  EXPECT(assert_equal(expected_H_state, actual_H_state, 1e-5));
}

/* ************************************************************************* */
TEST(LeggedEstimatorFactors, NavStatePointContactFactorJacobians) {
  const NavState state = sampleNavState();
  const Point3 foothold(1.2, 0.8, -0.5);
  NavStatePointContactFactor factor(0, 1, Point3(0.2, -0.1, 0.4),
                                    noiseModel::Unit::Create(3));

  Matrix actual_H_state, actual_H_foothold;
  factor.evaluateError(state, foothold, actual_H_state, actual_H_foothold);
  const auto error = [&](const NavState& x, const Point3& p) {
    return factor.evaluateError(x, p, {}, {});
  };
  const Matrix expected_H_state =
      numericalDerivative21<Vector, NavState, Point3>(error, state, foothold,
                                                      1e-6);
  const Matrix expected_H_foothold =
      numericalDerivative22<Vector, NavState, Point3>(error, state, foothold,
                                                      1e-6);

  EXPECT(assert_equal(expected_H_state, actual_H_state, 1e-6));
  EXPECT(assert_equal(expected_H_foothold, actual_H_foothold, 1e-6));
}

/* ************************************************************************* */
TEST(LeggedEstimatorFactors, Pose3PointContactFactorJacobians) {
  const Pose3 pose = sampleNavState().pose();
  const Point3 foothold(1.2, 0.8, -0.5);
  Pose3PointContactFactor factor(0, 1, Point3(0.2, -0.1, 0.4),
                                 noiseModel::Unit::Create(3));

  Matrix actual_H_pose, actual_H_foothold;
  factor.evaluateError(pose, foothold, actual_H_pose, actual_H_foothold);
  const auto error = [&](const Pose3& x, const Point3& p) {
    return factor.evaluateError(x, p, {}, {});
  };
  const Matrix expected_H_pose =
      numericalDerivative21<Vector, Pose3, Point3>(error, pose, foothold, 1e-6);
  const Matrix expected_H_foothold =
      numericalDerivative22<Vector, Pose3, Point3>(error, pose, foothold, 1e-6);

  EXPECT(assert_equal(expected_H_pose, actual_H_pose, 1e-6));
  EXPECT(assert_equal(expected_H_foothold, actual_H_foothold, 1e-6));
}

/* ************************************************************************* */
TEST(LeggedEstimatorFactors, PointHeightFactorJacobian) {
  const Point3 foothold(1.2, 0.8, -0.5);
  PointHeightFactor factor(0, -0.35, noiseModel::Unit::Create(1));

  Matrix actual_H_foothold;
  factor.evaluateError(foothold, actual_H_foothold);
  const Matrix expected_H_foothold = numericalDerivative11<Vector, Point3>(
      [&](const Point3& p) { return factor.evaluateError(p, {}); }, foothold,
      1e-6);

  EXPECT(assert_equal(expected_H_foothold, actual_H_foothold, 1e-6));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}

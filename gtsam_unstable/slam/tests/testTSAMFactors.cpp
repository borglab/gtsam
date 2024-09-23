/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  testTSAMFactors.cpp
 *  @brief Unit tests for TSAM 1 Factors
 *  @author Frank Dellaert
 *  @date May 2014
 */

#include <gtsam_unstable/slam/TSAMFactors.h>
#include <gtsam/base/numericalDerivative.h>

#include <CppUnitLite/TestHarness.h>
#include "gtsam/geometry/Point2.h"

using namespace std::placeholders;
using namespace std;
using namespace gtsam;

Key i(1), j(2); // Key for pose and point

//*************************************************************************
TEST( DeltaFactor, all ) {
  // Create a factor
  Point2 measurement(1, 1);
  static SharedNoiseModel model(noiseModel::Unit::Create(2));
  DeltaFactor factor(i, j, measurement, model);

  // Set the linearization point
  Pose2 pose(1, 2, 0);
  Point2 point(4, 11);
  Vector2 expected(4 - 1 - 1, 11 - 2 - 1);

  // Use the factor to calculate the Jacobians
  Matrix H1Actual, H2Actual;
  Vector actual = factor.evaluateError(pose, point, H1Actual, H2Actual);
  EXPECT(assert_equal(expected, actual, 1e-9));

  // Use numerical derivatives to calculate the Jacobians
  Matrix H1Expected, H2Expected;

  H1Expected = numericalDerivative11<Vector2, Pose2>(
      [&factor, &point](const Pose2& pose) { return factor.evaluateError(pose, point); }, pose);
  H2Expected = numericalDerivative11<Vector2, Point2>(
      [&factor, &pose](const Point2& point) { return factor.evaluateError(pose, point); }, point);

  // Verify the Jacobians are correct
  EXPECT(assert_equal(H1Expected, H1Actual, 1e-9));
  EXPECT(assert_equal(H2Expected, H2Actual, 1e-9));
}

//*************************************************************************
TEST( DeltaFactorBase, all ) {
  // Create a factor
  Key b1(10), b2(20);
  Point2 measurement(1, 1);
  static SharedNoiseModel model(noiseModel::Unit::Create(2));
  DeltaFactorBase factor(b1, i, b2, j, measurement, model);

  // Set the linearization point
  Pose2 base1, base2(1, 0, 0);
  Pose2 pose(1, 2, 0);
  Point2 point(4, 11);
  Vector2 expected(4 + 1 - 1 - 1, 11 - 2 - 1);

  // Use the factor to calculate the Jacobians
  Matrix H1Actual, H2Actual, H3Actual, H4Actual;
  Vector actual = factor.evaluateError(base1, pose, base2, point, H1Actual,
      H2Actual, H3Actual, H4Actual);
  EXPECT(assert_equal(expected, actual, 1e-9));

  // Use numerical derivatives to calculate the Jacobians
  Matrix H1Expected, H2Expected, H3Expected, H4Expected;
  H1Expected = numericalDerivative11<Vector2, Pose2>(
      [&factor, &pose, &base2, &point](const Pose2& pose_arg) {
        return factor.evaluateError(pose_arg, pose, base2, point);
      },
      base1);
  H2Expected = numericalDerivative11<Vector2, Pose2>(
      [&factor, &point, &base1, &base2](const Pose2& pose_arg) {
        return factor.evaluateError(base1, pose_arg, base2, point);
      },
      pose);
  H3Expected = numericalDerivative11<Vector2, Pose2>(
      [&factor, &pose, &base1, &point](const Pose2& pose_arg) {
        return factor.evaluateError(base1, pose, pose_arg, point);
      },
      base2);
  H4Expected = numericalDerivative11<Vector2, Point2>(
      [&factor, &pose, &base1, &base2](const Point2& point_arg) {
        return factor.evaluateError(base1, pose, base2, point_arg);
      },
      point);

  // Verify the Jacobians are correct
  EXPECT(assert_equal(H1Expected, H1Actual, 1e-9));
  EXPECT(assert_equal(H2Expected, H2Actual, 1e-9));
  EXPECT(assert_equal(H3Expected, H3Actual, 1e-9));
  EXPECT(assert_equal(H4Expected, H4Actual, 1e-9));
}

//*************************************************************************
TEST( OdometryFactorBase, all ) {
  // Create a factor
  Key b1(10), b2(20);
  Pose2 measurement(1, 1, 0);
  static SharedNoiseModel model(noiseModel::Unit::Create(2));
  OdometryFactorBase factor(b1, i, b2, j, measurement, model);

  // Set the linearization pose2
  Pose2 base1, base2(1, 0, 0);
  Pose2 pose1(1, 2, 0), pose2(4, 11, 0);
  Vector3 expected(4 + 1 - 1 - 1, 11 - 2 - 1, 0);

  // Use the factor to calculate the Jacobians
  Matrix H1Actual, H2Actual, H3Actual, H4Actual;
  Vector actual = factor.evaluateError(base1, pose1, base2, pose2, H1Actual,
      H2Actual, H3Actual, H4Actual);
  EXPECT(assert_equal(expected, actual, 1e-9));

  // Use numerical derivatives to calculate the Jacobians
  Matrix H1Expected, H2Expected, H3Expected, H4Expected;
  // using lambdas to replace bind
  H1Expected = numericalDerivative11<Vector3, Pose2>(
      [&factor, &pose1, &pose2, &base2](const Pose2& pose_arg) {
        return factor.evaluateError(pose_arg, pose1, base2, pose2);
      },
      base1);
  H2Expected = numericalDerivative11<Vector3, Pose2>(
      [&factor, &pose2, &base1, &base2](const Pose2& pose_arg) {
        return factor.evaluateError(base1, pose_arg, base2, pose2);
      },
      pose1);
  H3Expected = numericalDerivative11<Vector3, Pose2>(
      [&factor, &pose1, &base1, &pose2](const Pose2& pose_arg) {
        return factor.evaluateError(base1, pose1, pose_arg, pose2);
      },
      base2);
  H4Expected = numericalDerivative11<Vector3, Pose2>(
      [&factor, &pose1, &base1, &base2](const Pose2& pose_arg) {
        return factor.evaluateError(base1, pose1, base2, pose_arg);
      },
      pose2);

  // Verify the Jacobians are correct
  EXPECT(assert_equal(H1Expected, H1Actual, 1e-9));
  EXPECT(assert_equal(H2Expected, H2Actual, 1e-9));
  EXPECT(assert_equal(H3Expected, H3Actual, 1e-9));
  EXPECT(assert_equal(H4Expected, H4Actual, 1e-9));
}

//*************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//*************************************************************************


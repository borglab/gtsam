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
      std::bind(&DeltaFactor::evaluateError, &factor, std::placeholders::_1, point, boost::none,
          boost::none), pose);
  H2Expected = numericalDerivative11<Vector2, Point2>(
      std::bind(&DeltaFactor::evaluateError, &factor, pose, std::placeholders::_1, boost::none,
          boost::none), point);

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
      std::bind(&DeltaFactorBase::evaluateError, &factor, std::placeholders::_1, pose, base2,
          point, boost::none, boost::none, boost::none, boost::none), base1);
  H2Expected = numericalDerivative11<Vector2, Pose2>(
      std::bind(&DeltaFactorBase::evaluateError, &factor, base1, std::placeholders::_1, base2,
          point, boost::none, boost::none, boost::none, boost::none), pose);
  H3Expected = numericalDerivative11<Vector2, Pose2>(
      std::bind(&DeltaFactorBase::evaluateError, &factor, base1, pose, std::placeholders::_1,
          point, boost::none, boost::none, boost::none, boost::none), base2);
  H4Expected = numericalDerivative11<Vector2, Point2>(
      std::bind(&DeltaFactorBase::evaluateError, &factor, base1, pose, base2,
          std::placeholders::_1, boost::none, boost::none, boost::none, boost::none), point);

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
  H1Expected = numericalDerivative11<Vector3, Pose2>(
      std::bind(&OdometryFactorBase::evaluateError, &factor, std::placeholders::_1, pose1, base2,
          pose2, boost::none, boost::none, boost::none, boost::none), base1);
  H2Expected = numericalDerivative11<Vector3, Pose2>(
      std::bind(&OdometryFactorBase::evaluateError, &factor, base1, std::placeholders::_1, base2,
          pose2, boost::none, boost::none, boost::none, boost::none), pose1);
  H3Expected = numericalDerivative11<Vector3, Pose2>(
      std::bind(&OdometryFactorBase::evaluateError, &factor, base1, pose1, std::placeholders::_1,
          pose2, boost::none, boost::none, boost::none, boost::none), base2);
  H4Expected = numericalDerivative11<Vector3, Pose2>(
      std::bind(&OdometryFactorBase::evaluateError, &factor, base1, pose1,
          base2, std::placeholders::_1, boost::none, boost::none, boost::none, boost::none),
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


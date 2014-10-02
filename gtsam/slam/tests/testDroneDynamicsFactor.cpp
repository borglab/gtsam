/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  testRangeFactor.cpp
 *  @brief Unit tests for DroneDynamicsFactor Class
 *  @author Stephen Williams
 *  @date Oct 2012
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/slam/DroneDynamicsFactor.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/TestableAssertions.h>
#include <boost/bind.hpp>

using namespace std;
using namespace gtsam;

// Create a noise model for the pixel error
static SharedNoiseModel model(noiseModel::Unit::Create(3));

/* ************************************************************************* */
LieVector factorError(const Pose3& pose, const LieVector& vel, const DroneDynamicsFactor& factor) {
  return factor.evaluateError(pose, vel);
}

/* ************************************************************************* */
TEST( DroneDynamicsFactor, Error) {
  // Create a factor
  Key poseKey(1);
  Key velKey(2);
  LieVector measurement((Vector(3)<<10.0, 1.5, 0.0));
  DroneDynamicsFactor factor(poseKey, velKey, measurement, model);

  // Set the linearization point
  Pose3 pose(Rot3::ypr(1.0, 2.0, 0.57), Point3());
  LieVector vel((Vector(3) <<
      -2.913425624770731,
      -2.200086236883632,
      -9.429823523226959));

  // Use the factor to calculate the error
  Matrix H1, H2;
  Vector actualError(factor.evaluateError(pose, vel, H1, H2));

  Vector expectedError = zero(3);

  // Verify we get the expected error
  CHECK(assert_equal(expectedError, actualError, 1e-9));


  // Use numerical derivatives to calculate the Jacobians
  Matrix H1Expected, H2Expected;
  H1Expected = numericalDerivative11<LieVector, Pose3>(boost::bind(&factorError, _1, vel, factor), pose);
  H2Expected = numericalDerivative11<LieVector, LieVector>(boost::bind(&factorError, pose, _1, factor), vel);

  // Verify the Jacobians are correct
  CHECK(assert_equal(H1Expected, H1, 1e-9));
  CHECK(assert_equal(H2Expected, H2, 1e-9));
}

/* *************************************************************************
TEST( DroneDynamicsFactor, Jacobian2D ) {
  // Create a factor
  Key poseKey(1);
  Key pointKey(2);
  double measurement(10.0);
  RangeFactor2D factor(poseKey, pointKey, measurement, model);

  // Set the linearization point
  Pose2 pose(1.0, 2.0, 0.57);
  Point2 point(-4.0, 11.0);

  // Use the factor to calculate the Jacobians
  Matrix H1Actual, H2Actual;
  factor.evaluateError(pose, point, H1Actual, H2Actual);

  // Use numerical derivatives to calculate the Jacobians
  Matrix H1Expected, H2Expected;
  H1Expected = numericalDerivative11<LieVector, Pose2>(boost::bind(&factorError2D, _1, point, factor), pose);
  H2Expected = numericalDerivative11<LieVector, Point2>(boost::bind(&factorError2D, pose, _1, factor), point);

  // Verify the Jacobians are correct
  CHECK(assert_equal(H1Expected, H1Actual, 1e-9));
  CHECK(assert_equal(H2Expected, H2Actual, 1e-9));
}

/* *************************************************************************

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */


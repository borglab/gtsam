/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  testRangeFactor.cpp
 *  @brief Unit tests for RangeFactor Class
 *  @author Stephen Williams
 *  @date Oct 2012
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/slam/RangeFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/LieVector.h>
#include <gtsam/base/TestableAssertions.h>
#include <boost/bind.hpp>

using namespace std;
using namespace gtsam;

// Create a noise model for the pixel error
static SharedNoiseModel model(noiseModel::Unit::Create(1));

typedef RangeFactor<Pose2, Point2> RangeFactor2D;
typedef RangeFactor<Pose3, Point3> RangeFactor3D;

/* ************************************************************************* */
LieVector factorError2D(const Pose2& pose, const Point2& point, const RangeFactor2D& factor) {
  return factor.evaluateError(pose, point);
}

/* ************************************************************************* */
LieVector factorError3D(const Pose3& pose, const Point3& point, const RangeFactor3D& factor) {
  return factor.evaluateError(pose, point);
}

/* ************************************************************************* */
TEST( RangeFactor, Constructor) {
  Key poseKey(1);
  Key pointKey(2);
  double measurement(10.0);

  RangeFactor2D factor2D(poseKey, pointKey, measurement, model);
  RangeFactor3D factor3D(poseKey, pointKey, measurement, model);
}

/* ************************************************************************* */
TEST( RangeFactor, ConstructorWithTransform) {
  Key poseKey(1);
  Key pointKey(2);
  double measurement(10.0);
  Pose2 body_P_sensor_2D(0.25, -0.10, -M_PI_2);
  Pose3 body_P_sensor_3D(Rot3::RzRyRx(-M_PI_2, 0.0, -M_PI_2), Point3(0.25, -0.10, 1.0));

  RangeFactor2D factor2D(poseKey, pointKey, measurement, model, body_P_sensor_2D);
  RangeFactor3D factor3D(poseKey, pointKey, measurement, model, body_P_sensor_3D);
}

/* ************************************************************************* */
TEST( RangeFactor, Equals ) {
  // Create two identical factors and make sure they're equal
  Key poseKey(1);
  Key pointKey(2);
  double measurement(10.0);

  RangeFactor2D factor2D_1(poseKey, pointKey, measurement, model);
  RangeFactor2D factor2D_2(poseKey, pointKey, measurement, model);
  CHECK(assert_equal(factor2D_1, factor2D_2));

  RangeFactor3D factor3D_1(poseKey, pointKey, measurement, model);
  RangeFactor3D factor3D_2(poseKey, pointKey, measurement, model);
  CHECK(assert_equal(factor3D_1, factor3D_2));
}

/* ************************************************************************* */
TEST( RangeFactor, EqualsWithTransform ) {
  // Create two identical factors and make sure they're equal
  Key poseKey(1);
  Key pointKey(2);
  double measurement(10.0);
  Pose2 body_P_sensor_2D(0.25, -0.10, -M_PI_2);
  Pose3 body_P_sensor_3D(Rot3::RzRyRx(-M_PI_2, 0.0, -M_PI_2), Point3(0.25, -0.10, 1.0));

  RangeFactor2D factor2D_1(poseKey, pointKey, measurement, model, body_P_sensor_2D);
  RangeFactor2D factor2D_2(poseKey, pointKey, measurement, model, body_P_sensor_2D);
  CHECK(assert_equal(factor2D_1, factor2D_2));

  RangeFactor3D factor3D_1(poseKey, pointKey, measurement, model, body_P_sensor_3D);
  RangeFactor3D factor3D_2(poseKey, pointKey, measurement, model, body_P_sensor_3D);
  CHECK(assert_equal(factor3D_1, factor3D_2));
}

/* ************************************************************************* */
TEST( RangeFactor, Error2D ) {
  // Create a factor
  Key poseKey(1);
  Key pointKey(2);
  double measurement(10.0);
  RangeFactor2D factor(poseKey, pointKey, measurement, model);

  // Set the linearization point
  Pose2 pose(1.0, 2.0, 0.57);
  Point2 point(-4.0, 11.0);

  // Use the factor to calculate the error
  Vector actualError(factor.evaluateError(pose, point));

  // The expected error is ||(5.0, 9.0)|| - 10.0 = 0.295630141 meter / UnitCovariance
  Vector expectedError = (Vector(1) << 0.295630141);

  // Verify we get the expected error
  CHECK(assert_equal(expectedError, actualError, 1e-9));
}

/* ************************************************************************* */
TEST( RangeFactor, Error2DWithTransform ) {
  // Create a factor
  Key poseKey(1);
  Key pointKey(2);
  double measurement(10.0);
  Pose2 body_P_sensor(0.25, -0.10, -M_PI_2);
  RangeFactor2D factor(poseKey, pointKey, measurement, model, body_P_sensor);

  // Set the linearization point
  Rot2 R(0.57);
  Point2 t = Point2(1.0, 2.0) - R.rotate(body_P_sensor.translation());
  Pose2 pose(R, t);
  Point2 point(-4.0, 11.0);

  // Use the factor to calculate the error
  Vector actualError(factor.evaluateError(pose, point));

  // The expected error is ||(5.0, 9.0)|| - 10.0 = 0.295630141 meter / UnitCovariance
  Vector expectedError = (Vector(1) << 0.295630141);

  // Verify we get the expected error
  CHECK(assert_equal(expectedError, actualError, 1e-9));
}

/* ************************************************************************* */
TEST( RangeFactor, Error3D ) {
  // Create a factor
  Key poseKey(1);
  Key pointKey(2);
  double measurement(10.0);
  RangeFactor3D factor(poseKey, pointKey, measurement, model);

  // Set the linearization point
  Pose3 pose(Rot3::RzRyRx(0.2, -0.3, 1.75), Point3(1.0, 2.0, -3.0));
  Point3 point(-2.0, 11.0, 1.0);

  // Use the factor to calculate the error
  Vector actualError(factor.evaluateError(pose, point));

  // The expected error is ||(3.0, 9.0, 4.0)|| - 10.0 = 0.295630141 meter / UnitCovariance
  Vector expectedError = (Vector(1) << 0.295630141);

  // Verify we get the expected error
  CHECK(assert_equal(expectedError, actualError, 1e-9));
}

/* ************************************************************************* */
TEST( RangeFactor, Error3DWithTransform ) {
  // Create a factor
  Key poseKey(1);
  Key pointKey(2);
  double measurement(10.0);
  Pose3 body_P_sensor(Rot3::RzRyRx(-M_PI_2, 0.0, -M_PI_2), Point3(0.25, -0.10, 1.0));
  RangeFactor3D factor(poseKey, pointKey, measurement, model, body_P_sensor);

  // Set the linearization point
  Rot3 R = Rot3::RzRyRx(0.2, -0.3, 1.75);
  Point3 t = Point3(1.0, 2.0, -3.0) - R.rotate(body_P_sensor.translation());
  Pose3 pose(R, t);
  Point3 point(-2.0, 11.0, 1.0);

  // Use the factor to calculate the error
  Vector actualError(factor.evaluateError(pose, point));

  // The expected error is ||(3.0, 9.0, 4.0)|| - 10.0 = 0.295630141 meter / UnitCovariance
  Vector expectedError = (Vector(1) << 0.295630141);

  // Verify we get the expected error
  CHECK(assert_equal(expectedError, actualError, 1e-9));
}

/* ************************************************************************* */
TEST( RangeFactor, Jacobian2D ) {
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

/* ************************************************************************* */
TEST( RangeFactor, Jacobian2DWithTransform ) {
  // Create a factor
  Key poseKey(1);
  Key pointKey(2);
  double measurement(10.0);
  Pose2 body_P_sensor(0.25, -0.10, -M_PI_2);
  RangeFactor2D factor(poseKey, pointKey, measurement, model, body_P_sensor);

  // Set the linearization point
  Rot2 R(0.57);
  Point2 t = Point2(1.0, 2.0) - R.rotate(body_P_sensor.translation());
  Pose2 pose(R, t);
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

/* ************************************************************************* */
TEST( RangeFactor, Jacobian3D ) {
  // Create a factor
  Key poseKey(1);
  Key pointKey(2);
  double measurement(10.0);
  RangeFactor3D factor(poseKey, pointKey, measurement, model);

  // Set the linearization point
  Pose3 pose(Rot3::RzRyRx(0.2, -0.3, 1.75), Point3(1.0, 2.0, -3.0));
  Point3 point(-2.0, 11.0, 1.0);

  // Use the factor to calculate the Jacobians
  Matrix H1Actual, H2Actual;
  factor.evaluateError(pose, point, H1Actual, H2Actual);

  // Use numerical derivatives to calculate the Jacobians
  Matrix H1Expected, H2Expected;
  H1Expected = numericalDerivative11<LieVector, Pose3>(boost::bind(&factorError3D, _1, point, factor), pose);
  H2Expected = numericalDerivative11<LieVector, Point3>(boost::bind(&factorError3D, pose, _1, factor), point);

  // Verify the Jacobians are correct
  CHECK(assert_equal(H1Expected, H1Actual, 1e-9));
  CHECK(assert_equal(H2Expected, H2Actual, 1e-9));
}

/* ************************************************************************* */
TEST( RangeFactor, Jacobian3DWithTransform ) {
  // Create a factor
  Key poseKey(1);
  Key pointKey(2);
  double measurement(10.0);
  Pose3 body_P_sensor(Rot3::RzRyRx(-M_PI_2, 0.0, -M_PI_2), Point3(0.25, -0.10, 1.0));
  RangeFactor3D factor(poseKey, pointKey, measurement, model, body_P_sensor);

  // Set the linearization point
  Rot3 R = Rot3::RzRyRx(0.2, -0.3, 1.75);
  Point3 t = Point3(1.0, 2.0, -3.0) - R.rotate(body_P_sensor.translation());
  Pose3 pose(R, t);
  Point3 point(-2.0, 11.0, 1.0);

  // Use the factor to calculate the Jacobians
  Matrix H1Actual, H2Actual;
  factor.evaluateError(pose, point, H1Actual, H2Actual);

  // Use numerical derivatives to calculate the Jacobians
  Matrix H1Expected, H2Expected;
  H1Expected = numericalDerivative11<LieVector, Pose3>(boost::bind(&factorError3D, _1, point, factor), pose);
  H2Expected = numericalDerivative11<LieVector, Point3>(boost::bind(&factorError3D, pose, _1, factor), point);

  // Verify the Jacobians are correct
  CHECK(assert_equal(H1Expected, H1Actual, 1e-9));
  CHECK(assert_equal(H2Expected, H2Actual, 1e-9));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */


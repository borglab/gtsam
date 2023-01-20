/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  testProjectionFactorPPPC.cpp
 *  @brief Unit tests for Pose+Transform+Calibration ProjectionFactor Class
 *  @author Chris Beall
 *  @date Jul 29, 2014
 */

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam_unstable/slam/ProjectionFactorPPPC.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Point2.h>

#include <CppUnitLite/TestHarness.h>

using namespace std::placeholders;
using namespace std;
using namespace gtsam;

// make a realistic calibration matrix
static double fov = 60; // degrees
static size_t w=640,h=480;
static Cal3_S2::shared_ptr K1(new Cal3_S2(fov,w,h));

// Create a noise model for the pixel error
static SharedNoiseModel model(noiseModel::Unit::Create(2));

// Convenience for named keys
using symbol_shorthand::X;
using symbol_shorthand::L;
using symbol_shorthand::T;
using symbol_shorthand::K;

typedef ProjectionFactorPPPC<Pose3, Point3, Cal3_S2> TestProjectionFactor;

/* ************************************************************************* */
TEST( ProjectionFactorPPPC, nonStandard ) {
  ProjectionFactorPPPC<Pose3, Point3, Cal3DS2> f;
}

/* ************************************************************************* */
TEST( ProjectionFactorPPPC, Constructor) {
  Point2 measurement(323.0, 240.0);
  TestProjectionFactor factor(measurement, model, X(1), T(1), L(1), K(1));
  // TODO: Actually check something
}

/* ************************************************************************* */
TEST( ProjectionFactorPPPC, Equals ) {
  // Create two identical factors and make sure they're equal
  Point2 measurement(323.0, 240.0);

  TestProjectionFactor factor1(measurement, model, X(1), T(1), L(1), K(1));
  TestProjectionFactor factor2(measurement, model, X(1), T(1), L(1), K(1));

  CHECK(assert_equal(factor1, factor2));
}

/* ************************************************************************* */
TEST( ProjectionFactorPPPC, Error ) {
  // Create the factor with a measurement that is 3 pixels off in x
  Point2 measurement(323.0, 240.0);
  TestProjectionFactor factor(measurement, model, X(1), T(1), L(1), K(1));

  // Set the linearization point
  Pose3 pose(Rot3(), Point3(0,0,-6));
  Point3 point(0.0, 0.0, 0.0);

  // Use the factor to calculate the error
  Vector actualError(factor.evaluateError(pose, Pose3(), point, *K1));

  // The expected error is (-3.0, 0.0) pixels / UnitCovariance
  Vector expectedError = Vector2(-3.0, 0.0);

  // Verify we get the expected error
  CHECK(assert_equal(expectedError, actualError, 1e-9));
}

/* ************************************************************************* */
TEST( ProjectionFactorPPPC, ErrorWithTransform ) {
  // Create the factor with a measurement that is 3 pixels off in x
  Point2 measurement(323.0, 240.0);
  Pose3 transform(Rot3::RzRyRx(-M_PI_2, 0.0, -M_PI_2), Point3(0.25, -0.10, 1.0));
  TestProjectionFactor factor(measurement, model, X(1),T(1), L(1), K(1));

  // Set the linearization point. The vehicle pose has been selected to put the camera at (-6, 0, 0)
  Pose3 pose(Rot3(), Point3(-6.25, 0.10 , -1.0));
  Point3 point(0.0, 0.0, 0.0);

  // Use the factor to calculate the error
  Vector actualError(factor.evaluateError(pose, transform, point, *K1));

  // The expected error is (-3.0, 0.0) pixels / UnitCovariance
  Vector expectedError = Vector2(-3.0, 0.0);

  // Verify we get the expected error
  CHECK(assert_equal(expectedError, actualError, 1e-9));
}

/* ************************************************************************* */
TEST( ProjectionFactorPPPC, Jacobian ) {
  // Create the factor with a measurement that is 3 pixels off in x
  Point2 measurement(323.0, 240.0);
  TestProjectionFactor factor(measurement, model, X(1), T(1), L(1), K(1));

  // Set the linearization point
  Pose3 pose(Rot3(), Point3(0,0,-6));
  Point3 point(0.0, 0.0, 0.0);

  // Use the factor to calculate the Jacobians
  Matrix H1Actual, H2Actual, H3Actual, H4Actual;
  factor.evaluateError(pose, Pose3(), point, *K1, H1Actual, H2Actual, H3Actual, H4Actual);

  // The expected Jacobians
  Matrix H1Expected = (Matrix(2, 6) << 0., -554.256, 0., -92.376, 0., 0., 554.256, 0., 0., 0., -92.376, 0.).finished();
  Matrix H3Expected = (Matrix(2, 3) << 92.376, 0., 0., 0., 92.376, 0.).finished();

  // Verify the Jacobians are correct
  CHECK(assert_equal(H1Expected, H1Actual, 1e-3));
  CHECK(assert_equal(H3Expected, H3Actual, 1e-3));

  // Verify H2 and H4 with numerical derivatives
  Matrix H2Expected = numericalDerivative11<Vector, Pose3>(
      std::bind(&TestProjectionFactor::evaluateError, &factor, pose,
                std::placeholders::_1, point, *K1, boost::none, boost::none,
                boost::none, boost::none),
      Pose3());

  Matrix H4Expected = numericalDerivative11<Vector, Cal3_S2>(
      std::bind(&TestProjectionFactor::evaluateError, &factor, pose, Pose3(),
                point, std::placeholders::_1, boost::none, boost::none,
                boost::none, boost::none),
      *K1);

  CHECK(assert_equal(H2Expected, H2Actual, 1e-5));
  CHECK(assert_equal(H4Expected, H4Actual, 1e-5));
}

/* ************************************************************************* */
TEST( ProjectionFactorPPPC, JacobianWithTransform ) {
  // Create the factor with a measurement that is 3 pixels off in x
  Point2 measurement(323.0, 240.0);
  Pose3 body_P_sensor(Rot3::RzRyRx(-M_PI_2, 0.0, -M_PI_2), Point3(0.25, -0.10, 1.0));
  TestProjectionFactor factor(measurement, model, X(1), T(1), L(1), K(1));

  // Set the linearization point. The vehicle pose has been selected to put the camera at (-6, 0, 0)
  Pose3 pose(Rot3(), Point3(-6.25, 0.10 , -1.0));
  Point3 point(0.0, 0.0, 0.0);

  // Use the factor to calculate the Jacobians
  Matrix H1Actual, H2Actual, H3Actual, H4Actual;
  factor.evaluateError(pose, body_P_sensor, point, *K1, H1Actual, H2Actual, H3Actual, H4Actual);

  // The expected Jacobians
  Matrix H1Expected = (Matrix(2, 6) << -92.376, 0., 577.350, 0., 92.376, 0., -9.2376, -577.350, 0., 0., 0., 92.376).finished();
  Matrix H3Expected = (Matrix(2, 3) << 0., -92.376, 0., 0., 0., -92.376).finished();

  // Verify the Jacobians are correct
  CHECK(assert_equal(H1Expected, H1Actual, 1e-3));
  CHECK(assert_equal(H3Expected, H3Actual, 1e-3));

  // Verify H2 and H4 with numerical derivatives
  Matrix H2Expected = numericalDerivative11<Vector, Pose3>(
      std::bind(&TestProjectionFactor::evaluateError, &factor, pose, std::placeholders::_1, point,
          *K1, boost::none, boost::none, boost::none, boost::none), body_P_sensor);

  Matrix H4Expected = numericalDerivative11<Vector, Cal3_S2>(
      std::bind(&TestProjectionFactor::evaluateError, &factor, pose, body_P_sensor, point,
          std::placeholders::_1, boost::none, boost::none, boost::none, boost::none), *K1);

  CHECK(assert_equal(H2Expected, H2Actual, 1e-5));
  CHECK(assert_equal(H4Expected, H4Actual, 1e-5));

}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */


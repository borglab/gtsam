/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testStereoFactor.cpp
 * @brief   Unit test for StereoFactor
 * @author  Chris Beall
 */

#include <gtsam/slam/StereoFactor.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/StereoCamera.h>
#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

static Pose3 camera1(Rot3(Vector3(1, -1, -1).asDiagonal()),
        Point3(0,0,6.25));

static std::shared_ptr<Cal3_S2Stereo> K(new Cal3_S2Stereo(625, 625, 0, 320, 240, 0.5));

// point X Y Z in meters
static Point3 p(0, 0, 5);
static SharedNoiseModel model(noiseModel::Unit::Create(3));

// Convenience for named keys
using symbol_shorthand::X;
using symbol_shorthand::L;

typedef GenericStereoFactor<Pose3, Point3> TestStereoFactor;

/* ************************************************************************* */
TEST( StereoFactor, Constructor) {
  StereoPoint2 measurement(323, 318-50, 241);

  TestStereoFactor factor(measurement, model, X(1), L(1), K);
}

/* ************************************************************************* */
TEST( StereoFactor, ConstructorWithTransform) {
  StereoPoint2 measurement(323, 318-50, 241);
  Pose3 body_P_sensor(Rot3::RzRyRx(-M_PI_2, 0.0, -M_PI_2), Point3(0.25, -0.10, 1.0));

  TestStereoFactor factor(measurement, model, X(1), L(1), K, body_P_sensor);
}

/* ************************************************************************* */
TEST( StereoFactor, Equals ) {
  // Create two identical factors and make sure they're equal
  StereoPoint2 measurement(323, 318-50, 241);

  TestStereoFactor factor1(measurement, model, X(1), L(1), K);
  TestStereoFactor factor2(measurement, model, X(1), L(1), K);

  CHECK(assert_equal(factor1, factor2));
}

/* ************************************************************************* */
TEST( StereoFactor, EqualsWithTransform ) {
  // Create two identical factors and make sure they're equal
  StereoPoint2 measurement(323, 318-50, 241);
  Pose3 body_P_sensor(Rot3::RzRyRx(-M_PI_2, 0.0, -M_PI_2), Point3(0.25, -0.10, 1.0));

  TestStereoFactor factor1(measurement, model, X(1), L(1), K, body_P_sensor);
  TestStereoFactor factor2(measurement, model, X(1), L(1), K, body_P_sensor);

  CHECK(assert_equal(factor1, factor2));
}

/* ************************************************************************* */
TEST( StereoFactor, Error ) {
  // Create the factor with a measurement that is 3 pixels off in x
  StereoPoint2 measurement(323, 318-50, 241);
  TestStereoFactor factor(measurement, model, X(1), L(1), K);

  // Set the linearization point
  Pose3 pose(Rot3(), Point3(0.0, 0.0, -6.25));
  Point3 point(0.0, 0.0, 0.0);

  // Use the factor to calculate the error
  Vector actualError(factor.evaluateError(pose, point));

  // The expected error is (-3.0, +2.0, -1.0) pixels / UnitCovariance
  Vector expectedError = Vector3(-3.0, +2.0, -1.0);

  // Verify we get the expected error
  CHECK(assert_equal(expectedError, actualError, 1e-9));
}

/* ************************************************************************* */
TEST( StereoFactor, ErrorWithTransform ) {
  // Create the factor with a measurement that is 3 pixels off in x
  StereoPoint2 measurement(323, 318-50, 241);
  Pose3 body_P_sensor(Rot3::RzRyRx(-M_PI_2, 0.0, -M_PI_2), Point3(0.25, -0.10, 1.0));
  TestStereoFactor factor(measurement, model, X(1), L(1), K, body_P_sensor);

  // Set the linearization point. The vehicle pose has been selected to put the camera at (-6, 0, 0)
  Pose3 pose(Rot3(), Point3(-6.50, 0.10 , -1.0));
  Point3 point(0.0, 0.0, 0.0);

  // Use the factor to calculate the error
  Vector actualError(factor.evaluateError(pose, point));

  // The expected error is (-3.0, +2.0, -1.0) pixels / UnitCovariance
  Vector expectedError = Vector3(-3.0, +2.0, -1.0);

  // Verify we get the expected error
  CHECK(assert_equal(expectedError, actualError, 1e-9));
}

/* ************************************************************************* */
TEST( StereoFactor, Jacobian ) {
  // Create the factor with a measurement that is 3 pixels off in x
  StereoPoint2 measurement(323, 318-50, 241);
  TestStereoFactor factor(measurement, model, X(1), L(1), K);

  // Set the linearization point
  Pose3 pose(Rot3(), Point3(0.0, 0.0, -6.25));
  Point3 point(0.0, 0.0, 0.0);

  // Use the factor to calculate the Jacobians
  Matrix H1Actual, H2Actual;
  factor.evaluateError(pose, point, H1Actual, H2Actual);

  // The expected Jacobians
  Matrix H1Expected = (Matrix(3, 6) << 0.0,  -625.0, 0.0, -100.0,    0.0,  0.0,
                                    0.0,  -625.0, 0.0, -100.0,    0.0, -8.0,
                                    625.0,   0.0, 0.0,    0.0, -100.0,  0.0).finished();
  Matrix H2Expected = (Matrix(3, 3) << 100.0,   0.0, 0.0,
                                    100.0,   0.0, 8.0,
                                    0.0,   100.0, 0.0).finished();

  // Verify the Jacobians are correct
  CHECK(assert_equal(H1Expected, H1Actual, 1e-3));
  CHECK(assert_equal(H2Expected, H2Actual, 1e-3));
}

/* ************************************************************************* */
TEST( StereoFactor, JacobianWithTransform ) {
  // Create the factor with a measurement that is 3 pixels off in x
  StereoPoint2 measurement(323, 318-50, 241);
  Pose3 body_P_sensor(Rot3::RzRyRx(-M_PI_2, 0.0, -M_PI_2), Point3(0.25, -0.10, 1.0));
  TestStereoFactor factor(measurement, model, X(1), L(1), K, body_P_sensor);

  // Set the linearization point. The vehicle pose has been selected to put the camera at (-6, 0, 0)
  Pose3 pose(Rot3(), Point3(-6.50, 0.10 , -1.0));
  Point3 point(0.0, 0.0, 0.0);

  // Use the factor to calculate the Jacobians
  Matrix H1Actual, H2Actual;
  factor.evaluateError(pose, point, H1Actual, H2Actual);

  // The expected Jacobians
  Matrix H1Expected = (Matrix(3, 6) << -100.0,    0.0,  650.0,   0.0,  100.0,    0.0,
                                    -100.0,   -8.0,  649.2,  -8.0,  100.0,    0.0,
                                     -10.0, -650.0,    0.0,   0.0,    0.0,  100.0).finished();
  Matrix H2Expected = (Matrix(3, 3) <<    0.0, -100.0,    0.0,
                                       8.0, -100.0,    0.0,
                                       0.0,    0.0, -100.0).finished();

  // Verify the Jacobians are correct
  CHECK(assert_equal(H1Expected, H1Actual, 1e-3));
  CHECK(assert_equal(H2Expected, H2Actual, 1e-3));
}

/* ************************************************************************* */
TEST( StereoFactor, singlePoint)
{
  NonlinearFactorGraph graph;

  graph.emplace_shared<NonlinearEquality<Pose3> >(X(1), camera1);

  StereoPoint2 measurement(320, 320.0-50, 240);
  // arguments: measurement, sigma, cam#, measurement #, K, baseline (m)
  graph.emplace_shared<GenericStereoFactor<Pose3, Point3> >(measurement, model, X(1), L(1), K);

  // Create a configuration corresponding to the ground truth
  Values values;
  values.insert(X(1), camera1); // add camera at z=6.25m looking towards origin

  Point3 l1(0, 0, 0);
  values.insert(L(1), l1);   // add point at origin;

  GaussNewtonOptimizer optimizer(graph, values);

  // We expect the initial to be zero because config is the ground truth
  DOUBLES_EQUAL(0.0, optimizer.error(), 1e-9);

  // Iterate once, and the config should not have changed
  optimizer.iterate();
  DOUBLES_EQUAL(0.0, optimizer.error(), 1e-9);

  // Complete solution
  optimizer.optimize();

  DOUBLES_EQUAL(0.0, optimizer.error(), 1e-6);
}

/* ************************************************************************* */
  int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */

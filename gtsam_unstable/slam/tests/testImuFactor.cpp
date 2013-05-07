/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testImuFactor.cpp
 * @brief   Unit test for StereoFactor
 * @author  Chris Beall
 */

#include <gtsam_unstable/slam/ImuFactor.h>
//#include <gtsam/nonlinear/NonlinearEquality.h>
//#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
//#include <gtsam/nonlinear/NonlinearFactorGraph.h>
//#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Symbol.h>
//#include <gtsam/geometry/StereoCamera.h>
//#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/LieVector.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <CppUnitLite/TestHarness.h>

#include <boost/bind.hpp>

using namespace std;
using namespace gtsam;

// Convenience for named keys
using symbol_shorthand::X;
using symbol_shorthand::V;

/* ************************************************************************* */
//TEST( ImuFactor, Constructor) {
//  Vector3 gravity; gravity << 0, 0, 9.81;
//  noiseModel::Diagonal::shared_ptr model = noiseModel::Diagonal::Sigmas(Vector_(9, 0.15, 0.15, 0.15, 1.5, 1.5, 1.5, 0.5, 0.5, 0.5));
//  ImuFactor factor(X(1), V(1), X(2), V(2), ImuFactor::PreintegratedMeasurements(),gravity, model);
//}

///* ************************************************************************* */
//TEST( ImuFactor, ConstructorWithTransform) {
//  StereoPoint2 measurement(323, 318-50, 241);
//  Pose3 body_P_sensor(Rot3::RzRyRx(-M_PI_2, 0.0, -M_PI_2), Point3(0.25, -0.10, 1.0));
//
//  TestStereoFactor factor(measurement, model, X(1), L(1), K, body_P_sensor);
//}
//
///* ************************************************************************* */
//TEST( StereoFactor, Equals ) {
//  // Create two identical factors and make sure they're equal
//  StereoPoint2 measurement(323, 318-50, 241);
//
//  TestStereoFactor factor1(measurement, model, X(1), L(1), K);
//  TestStereoFactor factor2(measurement, model, X(1), L(1), K);
//
//  CHECK(assert_equal(factor1, factor2));
//}
//
///* ************************************************************************* */
//TEST( StereoFactor, EqualsWithTransform ) {
//  // Create two identical factors and make sure they're equal
//  StereoPoint2 measurement(323, 318-50, 241);
//  Pose3 body_P_sensor(Rot3::RzRyRx(-M_PI_2, 0.0, -M_PI_2), Point3(0.25, -0.10, 1.0));
//
//  TestStereoFactor factor1(measurement, model, X(1), L(1), K, body_P_sensor);
//  TestStereoFactor factor2(measurement, model, X(1), L(1), K, body_P_sensor);
//
//  CHECK(assert_equal(factor1, factor2));
//}

/* ************************************************************************* */
TEST( ImuFactor, PreintegratedMeasurements )
{
  // Linearization point
  Vector3 biasOmega; biasOmega << 0,0,0; ///< Current estimate of rotation rate bias
  Vector3 biasAcc; biasAcc << 0,0,0; ///< Current estimate of acceleration bias

  // Measurements
  Vector3 measuredAcc(0.1, 0.0, 0.0);
  Vector3 measuredOmega(M_PI/100.0, 0.0, 0.0);
  double deltaT = 0.5;

  // Expected preintegrated values
  Vector3 expectedDeltaP1; expectedDeltaP1 << 0.5*0.1*0.5*0.5, 0, 0;
  Vector3 expectedDeltaV1(0.05, 0.0, 0.0);
  Rot3 expectedDeltaR1 = Rot3::RzRyRx(0.5 * M_PI/100.0, 0.0, 0.0);
  double expectedDeltaT1(0.5);

  // Actual preintegrated values
  ImuFactor::PreintegratedMeasurements actual1(biasAcc, biasOmega);
  actual1.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

  EXPECT(assert_equal(Vector(expectedDeltaP1), Vector(actual1.deltaPij), 1e-6));
  EXPECT(assert_equal(Vector(expectedDeltaV1), Vector(actual1.deltaVij), 1e-6));
  EXPECT(assert_equal(expectedDeltaR1, actual1.deltaRij, 1e-6));
  DOUBLES_EQUAL(expectedDeltaT1, actual1.deltaTij, 1e-6);

  // Integrate again
  Vector3 expectedDeltaP2; expectedDeltaP2 << 0.025 + expectedDeltaP1(0) + 0.5*0.1*0.5*0.5, 0, 0;
  Vector3 expectedDeltaV2 = Vector3(0.05, 0.0, 0.0) + expectedDeltaR1.matrix() * measuredAcc * 0.5;
  Rot3 expectedDeltaR2 = Rot3::RzRyRx(2.0 * 0.5 * M_PI/100.0, 0.0, 0.0);
  double expectedDeltaT2(1);

  // Actual preintegrated values
  ImuFactor::PreintegratedMeasurements actual2 = actual1;
  actual2.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

  EXPECT(assert_equal(Vector(expectedDeltaP2), Vector(actual2.deltaPij), 1e-6));
  EXPECT(assert_equal(Vector(expectedDeltaV2), Vector(actual2.deltaVij), 1e-6));
  EXPECT(assert_equal(expectedDeltaR2, actual2.deltaRij, 1e-6));
  DOUBLES_EQUAL(expectedDeltaT2, actual2.deltaTij, 1e-6);
}

/* ************************************************************************* */
TEST( ImuFactor, Error )
{
  // Linearization point
  Vector3 biasOmega; biasOmega << 0,0,0; ///< Current estimate of rotation rate bias
  Vector3 biasAcc; biasAcc << 0,0,0; ///< Current estimate of acceleration bias
  Pose3 x1(Rot3::RzRyRx(M_PI/12.0, M_PI/6.0, M_PI/4.0), Point3(5.0, 1.0, -50.0));
  LieVector v1(3, 0.5, 0.0, 0.0);
  Pose3 x2(Rot3::RzRyRx(M_PI/12.0 + M_PI/100.0, M_PI/6.0, M_PI/4.0), Point3(5.5, 1.0, -50.0));
  LieVector v2(3, 0.5, 0.0, 0.0);

  // Measurements
  Vector3 gravity; gravity << 0, 0, 9.81;
  Vector3 measuredOmega; measuredOmega << M_PI/100, 0, 0;
  Vector3 measuredAcc = x1.rotation().unrotate(-Point3(gravity)).vector();
  double deltaT = 1.0;
  ImuFactor::PreintegratedMeasurements pre_int_data(biasAcc, biasOmega);
  pre_int_data.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

  // Create factor
  noiseModel::Diagonal::shared_ptr model = noiseModel::Diagonal::Sigmas(Vector_(9, 0.15, 0.15, 0.15, 1.5, 1.5, 1.5, 0.5, 0.5, 0.5));
  ImuFactor factor(X(1), V(1), X(2), V(2), pre_int_data, gravity, model);

  Vector errorActual = factor.evaluateError(x1, v1, x2, v2);

  // Expected error
  Vector errorExpected(9); errorExpected << 0, 0, 0, 0, 0, 0, 0, 0, 0;
  EXPECT(assert_equal(errorExpected, errorActual, 1e-6));

  // Expected Jacobians
  Matrix H1e = numericalDerivative11<Pose3>(
      boost::bind(&ImuFactor::evaluateError, &factor, _1, v1, x2, v2,
          boost::none, boost::none, boost::none, boost::none), x1);
  Matrix H2e = numericalDerivative11<LieVector>(
      boost::bind(&ImuFactor::evaluateError, &factor, x1, _1, x2, v2,
          boost::none, boost::none, boost::none, boost::none), v1);
  Matrix H3e = numericalDerivative11<Pose3>(
      boost::bind(&ImuFactor::evaluateError, &factor, x1, v1, _1, v2,
          boost::none, boost::none, boost::none, boost::none), x2);
  Matrix H4e = numericalDerivative11<LieVector>(
      boost::bind(&ImuFactor::evaluateError, &factor, x1, v1, x2, _1,
          boost::none, boost::none, boost::none, boost::none), v2);

  // Actual Jacobians
  Matrix H1a, H2a, H3a, H4a;
  (void) factor.evaluateError(x1, v1, x2, v2, H1a, H2a, H3a, H4a);

  EXPECT(assert_equal(H1e, H1a));
  EXPECT(assert_equal(H2e, H2a));
  EXPECT(assert_equal(H3e, H3a));
  EXPECT(assert_equal(H4e, H4a));
}

/* ************************************************************************* */
TEST( ImuFactor, ErrorWithBiases )
{
  // Linearization poin
  Vector3 biasOmega; biasOmega << 0.1,0,0; ///< Current estimate of rotation rate bias
  Vector3 biasAcc; biasAcc << 0.2,0,0; ///< Current estimate of acceleration bias
  Pose3 x1(Rot3::RzRyRx(M_PI/12.0, M_PI/6.0, M_PI/4.0), Point3(5.0, 1.0, -50.0));
  LieVector v1(3, 0.5, 0.0, 0.0);
  Pose3 x2(Rot3::RzRyRx(M_PI/12.0 + M_PI/100.0, M_PI/6.0, M_PI/4.0), Point3(5.5, 1.0, -50.0));
  LieVector v2(3, 0.5, 0.0, 0.0);

  // Measurements
  Vector3 gravity; gravity << 0, 0, 9.81;
  Vector3 measuredOmega; measuredOmega << M_PI/100+0.1, 0, 0;
  Vector3 measuredAcc = x1.rotation().unrotate(-Point3(gravity)).vector() + Vector3(0.2,0.0,0.0);
  double deltaT = 1.0;

  ImuFactor::PreintegratedMeasurements pre_int_data(biasAcc, biasOmega);
    pre_int_data.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

    // Create factor
    noiseModel::Diagonal::shared_ptr model = noiseModel::Diagonal::Sigmas(Vector_(9, 0.15, 0.15, 0.15, 1.5, 1.5, 1.5, 0.5, 0.5, 0.5));
    ImuFactor factor(X(1), V(1), X(2), V(2), pre_int_data, gravity, model);

    Vector errorActual = factor.evaluateError(x1, v1, x2, v2);

    // Expected error
    Vector errorExpected(9); errorExpected << 0, 0, 0, 0, 0, 0, 0, 0, 0;
    EXPECT(assert_equal(errorExpected, errorActual, 1e-6));

    // Expected Jacobians
    Matrix H1e = numericalDerivative11<Pose3>(
        boost::bind(&ImuFactor::evaluateError, &factor, _1, v1, x2, v2,
            boost::none, boost::none, boost::none, boost::none), x1);
    Matrix H2e = numericalDerivative11<LieVector>(
        boost::bind(&ImuFactor::evaluateError, &factor, x1, _1, x2, v2,
            boost::none, boost::none, boost::none, boost::none), v1);
    Matrix H3e = numericalDerivative11<Pose3>(
        boost::bind(&ImuFactor::evaluateError, &factor, x1, v1, _1, v2,
            boost::none, boost::none, boost::none, boost::none), x2);
    Matrix H4e = numericalDerivative11<LieVector>(
        boost::bind(&ImuFactor::evaluateError, &factor, x1, v1, x2, _1,
            boost::none, boost::none, boost::none, boost::none), v2);

    // Actual Jacobians
    Matrix H1a, H2a, H3a, H4a;
    (void) factor.evaluateError(x1, v1, x2, v2, H1a, H2a, H3a, H4a);

    EXPECT(assert_equal(H1e, H1a));
    EXPECT(assert_equal(H2e, H2a));
    EXPECT(assert_equal(H3e, H3a));
    EXPECT(assert_equal(H4e, H4a));
}

///* ************************************************************************* */
//TEST( StereoFactor, ErrorWithTransform ) {
//  // Create the factor with a measurement that is 3 pixels off in x
//  StereoPoint2 measurement(323, 318-50, 241);
//  Pose3 body_P_sensor(Rot3::RzRyRx(-M_PI_2, 0.0, -M_PI_2), Point3(0.25, -0.10, 1.0));
//  TestStereoFactor factor(measurement, model, X(1), L(1), K, body_P_sensor);
//
//  // Set the linearization point. The vehicle pose has been selected to put the camera at (-6, 0, 0)
//  Pose3 pose(Rot3(), Point3(-6.50, 0.10 , -1.0));
//  Point3 point(0.0, 0.0, 0.0);
//
//  // Use the factor to calculate the error
//  Vector actualError(factor.evaluateError(pose, point));
//
//  // The expected error is (-3.0, +2.0, -1.0) pixels / UnitCovariance
//  Vector expectedError = Vector_(3, -3.0, +2.0, -1.0);
//
//  // Verify we get the expected error
//  CHECK(assert_equal(expectedError, actualError, 1e-9));
//}
//
///* ************************************************************************* */
//TEST( StereoFactor, Jacobian ) {
//  // Create the factor with a measurement that is 3 pixels off in x
//  StereoPoint2 measurement(323, 318-50, 241);
//  TestStereoFactor factor(measurement, model, X(1), L(1), K);
//
//  // Set the linearization point
//  Pose3 pose(Rot3(), Point3(0.0, 0.0, -6.25));
//  Point3 point(0.0, 0.0, 0.0);
//
//  // Use the factor to calculate the Jacobians
//  Matrix H1Actual, H2Actual;
//  factor.evaluateError(pose, point, H1Actual, H2Actual);
//
//  // The expected Jacobians
//  Matrix H1Expected = Matrix_(3, 6, 0.0,  -625.0, 0.0, -100.0,    0.0,  0.0,
//                                    0.0,  -625.0, 0.0, -100.0,    0.0, -8.0,
//                                    625.0,   0.0, 0.0,    0.0, -100.0,  0.0);
//  Matrix H2Expected = Matrix_(3, 3, 100.0,   0.0, 0.0,
//                                    100.0,   0.0, 8.0,
//                                    0.0,   100.0, 0.0);
//
//  // Verify the Jacobians are correct
//  CHECK(assert_equal(H1Expected, H1Actual, 1e-3));
//  CHECK(assert_equal(H2Expected, H2Actual, 1e-3));
//}
//
///* ************************************************************************* */
//TEST( StereoFactor, JacobianWithTransform ) {
//  // Create the factor with a measurement that is 3 pixels off in x
//  StereoPoint2 measurement(323, 318-50, 241);
//  Pose3 body_P_sensor(Rot3::RzRyRx(-M_PI_2, 0.0, -M_PI_2), Point3(0.25, -0.10, 1.0));
//  TestStereoFactor factor(measurement, model, X(1), L(1), K, body_P_sensor);
//
//  // Set the linearization point. The vehicle pose has been selected to put the camera at (-6, 0, 0)
//  Pose3 pose(Rot3(), Point3(-6.50, 0.10 , -1.0));
//  Point3 point(0.0, 0.0, 0.0);
//
//  // Use the factor to calculate the Jacobians
//  Matrix H1Actual, H2Actual;
//  factor.evaluateError(pose, point, H1Actual, H2Actual);
//
//  // The expected Jacobians
//  Matrix H1Expected = Matrix_(3, 6, -100.0,    0.0,  650.0,   0.0,  100.0,    0.0,
//                                    -100.0,   -8.0,  649.2,  -8.0,  100.0,    0.0,
//                                     -10.0, -650.0,    0.0,   0.0,    0.0,  100.0);
//  Matrix H2Expected = Matrix_(3, 3,    0.0, -100.0,    0.0,
//                                       8.0, -100.0,    0.0,
//                                       0.0,    0.0, -100.0);
//
//  // Verify the Jacobians are correct
//  CHECK(assert_equal(H1Expected, H1Actual, 1e-3));
//  CHECK(assert_equal(H2Expected, H2Actual, 1e-3));
//}
//
///* ************************************************************************* */
//TEST( StereoFactor, singlePoint)
//{
//  NonlinearFactorGraph graph;
//
//  graph.add(NonlinearEquality<Pose3>(X(1), camera1));
//
//  StereoPoint2 measurement(320, 320.0-50, 240);
//  // arguments: measurement, sigma, cam#, measurement #, K, baseline (m)
//  graph.add(GenericStereoFactor<Pose3, Point3>(measurement, model, X(1), L(1), K));
//
//  // Create a configuration corresponding to the ground truth
//  Values values;
//  values.insert(X(1), camera1); // add camera at z=6.25m looking towards origin
//
//  Point3 l1(0, 0, 0);
//  values.insert(L(1), l1);   // add point at origin;
//
//  GaussNewtonOptimizer optimizer(graph, values);
//
//  // We expect the initial to be zero because config is the ground truth
//  DOUBLES_EQUAL(0.0, optimizer.error(), 1e-9);
//
//  // Iterate once, and the config should not have changed
//  optimizer.iterate();
//  DOUBLES_EQUAL(0.0, optimizer.error(), 1e-9);
//
//  // Complete solution
//  optimizer.optimize();
//
//  DOUBLES_EQUAL(0.0, optimizer.error(), 1e-6);
//}

/* ************************************************************************* */
  int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */

/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testCombinedImuFactor.cpp
 * @brief   Unit test for Lupton-style combined IMU factor
 * @author  Luca Carlone
 * @author  Frank Dellaert
 * @author  Richard Roberts
 * @author  Stephen Williams
 */

#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>

#include <CppUnitLite/TestHarness.h>

#include <boost/bind.hpp>
#include <list>

#include "imuFactorTesting.h"

namespace {

// Auxiliary functions to test preintegrated Jacobians
// delPdelBiasAcc_ delPdelBiasOmega_ delVdelBiasAcc_ delVdelBiasOmega_ delRdelBiasOmega_
/* ************************************************************************* */
PreintegratedCombinedMeasurements evaluatePreintegratedMeasurements(
    const imuBias::ConstantBias& bias) {
  auto p = PreintegratedCombinedMeasurements::Params::MakeSharedD(9.81);
  PreintegratedCombinedMeasurements pim(p, bias);
  integrateMeasurements(testing::SomeMeasurements(), &pim);
  return pim;
}

Vector3 evaluatePreintegratedMeasurementsPosition(
    const imuBias::ConstantBias& bias) {
  return evaluatePreintegratedMeasurements(bias).deltaPij();
}

Vector3 evaluatePreintegratedMeasurementsVelocity(
    const imuBias::ConstantBias& bias) {
  return evaluatePreintegratedMeasurements(bias).deltaVij();
}
}

/* ************************************************************************* */
TEST( CombinedImuFactor, PreintegratedMeasurements ) {
  // Linearization point
  imuBias::ConstantBias bias(Vector3(0, 0, 0), Vector3(0, 0, 0)); ///< Current estimate of acceleration and angular rate biases

  // Measurements
  Vector3 measuredAcc(0.1, 0.0, 0.0);
  Vector3 measuredOmega(M_PI / 100.0, 0.0, 0.0);
  double deltaT = 0.5;
  double tol = 1e-6;

  auto p = PreintegratedCombinedMeasurements::Params::MakeSharedD(9.81);

  // Actual preintegrated values
  PreintegratedImuMeasurements expected1(p, bias);
  expected1.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

  PreintegratedCombinedMeasurements actual1(p, bias);

  actual1.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

  EXPECT(assert_equal(Vector(expected1.deltaPij()), actual1.deltaPij(), tol));
  EXPECT(assert_equal(Vector(expected1.deltaVij()), actual1.deltaVij(), tol));
  EXPECT(assert_equal(expected1.deltaRij(), actual1.deltaRij(), tol));
  DOUBLES_EQUAL(expected1.deltaTij(), actual1.deltaTij(), tol);
}

/* ************************************************************************* */
TEST( CombinedImuFactor, ErrorWithBiases ) {
  imuBias::ConstantBias bias(Vector3(0.2, 0, 0), Vector3(0, 0, 0.3)); // Biases (acc, rot)
  imuBias::ConstantBias bias2(Vector3(0.2, 0.2, 0), Vector3(1, 0, 0.3)); // Biases (acc, rot)
  Pose3 x1(Rot3::Expmap(Vector3(0, 0, M_PI / 4.0)), Point3(5.0, 1.0, -50.0));
  Vector3 v1(0.5, 0.0, 0.0);
  Pose3 x2(Rot3::Expmap(Vector3(0, 0, M_PI / 4.0 + M_PI / 10.0)),
      Point3(5.5, 1.0, -50.0));
  Vector3 v2(0.5, 0.0, 0.0);

  auto p = PreintegratedCombinedMeasurements::Params::MakeSharedD(9.81);
  p->omegaCoriolis = Vector3(0,0.1,0.1);
  PreintegratedImuMeasurements pim(
      p, imuBias::ConstantBias(Vector3(0.2, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)));

  // Measurements
  Vector3 measuredOmega;
  measuredOmega << 0, 0, M_PI / 10.0 + 0.3;
  Vector3 measuredAcc =
      x1.rotation().unrotate(-p->n_gravity) + Vector3(0.2, 0.0, 0.0);
  double deltaT = 1.0;
  double tol = 1e-6;

  pim.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

  PreintegratedCombinedMeasurements combined_pim(p,
      imuBias::ConstantBias(Vector3(0.2, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)));

  combined_pim.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

  // Create factor
  ImuFactor imuFactor(X(1), V(1), X(2), V(2), B(1), pim);

  noiseModel::Gaussian::shared_ptr Combinedmodel =
      noiseModel::Gaussian::Covariance(combined_pim.preintMeasCov());
  CombinedImuFactor combinedfactor(X(1), V(1), X(2), V(2), B(1), B(2),
                                   combined_pim);

  Vector errorExpected = imuFactor.evaluateError(x1, v1, x2, v2, bias);
  Vector errorActual = combinedfactor.evaluateError(x1, v1, x2, v2, bias,
      bias2);
  EXPECT(assert_equal(errorExpected, errorActual.head(9), tol));

  // Expected Jacobians
  Matrix H1e, H2e, H3e, H4e, H5e;
  (void) imuFactor.evaluateError(x1, v1, x2, v2, bias, H1e, H2e, H3e, H4e, H5e);

  // Actual Jacobians
  Matrix H1a, H2a, H3a, H4a, H5a, H6a;
  (void) combinedfactor.evaluateError(x1, v1, x2, v2, bias, bias2, H1a, H2a,
      H3a, H4a, H5a, H6a);

  EXPECT(assert_equal(H1e, H1a.topRows(9)));
  EXPECT(assert_equal(H2e, H2a.topRows(9)));
  EXPECT(assert_equal(H3e, H3a.topRows(9)));
  EXPECT(assert_equal(H4e, H4a.topRows(9)));
  EXPECT(assert_equal(H5e, H5a.topRows(9)));
}

/* ************************************************************************* */
TEST(CombinedImuFactor, FirstOrderPreIntegratedMeasurements) {
  // Actual preintegrated values
  PreintegratedCombinedMeasurements pim =
      evaluatePreintegratedMeasurements(kZeroBiasHat);

  // Check derivative of rotation
  boost::function<Vector3(const Vector3&, const Vector3&)> theta =
      [=](const Vector3& a, const Vector3& w) {
        return evaluatePreintegratedMeasurements(Bias(a, w)).theta();
      };
  EXPECT(
      assert_equal(numericalDerivative21(theta, Z_3x1, Z_3x1), Matrix(Z_3x3)));
  EXPECT(assert_equal(numericalDerivative22(theta, Z_3x1, Z_3x1),
                      pim.delRdelBiasOmega(), 1e-7));

  // Compute numerical derivatives
  Matrix expectedDelPdelBias =
      numericalDerivative11<Vector, imuBias::ConstantBias>(
          evaluatePreintegratedMeasurementsPosition, kZeroBiasHat);
  Matrix expectedDelPdelBiasAcc = expectedDelPdelBias.leftCols(3);
  Matrix expectedDelPdelBiasOmega = expectedDelPdelBias.rightCols(3);

  Matrix expectedDelVdelBias =
      numericalDerivative11<Vector, imuBias::ConstantBias>(
          &evaluatePreintegratedMeasurementsVelocity, kZeroBiasHat);
  Matrix expectedDelVdelBiasAcc = expectedDelVdelBias.leftCols(3);
  Matrix expectedDelVdelBiasOmega = expectedDelVdelBias.rightCols(3);

  // Compare Jacobians
  EXPECT(assert_equal(expectedDelPdelBiasAcc, pim.delPdelBiasAcc()));
  EXPECT(assert_equal(expectedDelPdelBiasOmega, pim.delPdelBiasOmega(), 1e-8));
  EXPECT(assert_equal(expectedDelVdelBiasAcc, pim.delVdelBiasAcc()));
  EXPECT(assert_equal(expectedDelVdelBiasOmega, pim.delVdelBiasOmega(), 1e-8));
}

/* ************************************************************************* */
TEST(CombinedImuFactor, PredictPositionAndVelocity) {
  imuBias::ConstantBias bias(Vector3(0, 0.1, 0), Vector3(0, 0.1, 0)); // Biases (acc, rot)

  auto p = PreintegratedCombinedMeasurements::Params::MakeSharedD(9.81);

  // Measurements
  Vector3 measuredOmega;
  measuredOmega << 0, 0.1, 0; //M_PI/10.0+0.3;
  Vector3 measuredAcc;
  measuredAcc << 0, 1.1, -9.81;
  double deltaT = 0.001;

  PreintegratedCombinedMeasurements pim(p, bias);

  for (int i = 0; i < 1000; ++i)
    pim.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

  // Create factor
  noiseModel::Gaussian::shared_ptr combinedmodel =
      noiseModel::Gaussian::Covariance(pim.preintMeasCov());
  CombinedImuFactor Combinedfactor(X(1), V(1), X(2), V(2), B(1), B(2), pim);

  // Predict
  NavState actual = pim.predict(NavState(), bias);
  Pose3 expectedPose(Rot3(), Point3(0, 0.5, 0));
  Vector3 expectedVelocity;
  expectedVelocity << 0, 1, 0;
  EXPECT(assert_equal(expectedPose, actual.pose()));
  EXPECT(
      assert_equal(Vector(expectedVelocity), Vector(actual.velocity())));
}

/* ************************************************************************* */
TEST(CombinedImuFactor, PredictRotation) {
  imuBias::ConstantBias bias(Vector3(0, 0, 0), Vector3(0, 0, 0)); // Biases (acc, rot)
  auto p = PreintegratedCombinedMeasurements::Params::MakeSharedD(9.81);
  PreintegratedCombinedMeasurements pim(p, bias);
  Vector3 measuredAcc;
  measuredAcc << 0, 0, -9.81;
  Vector3 measuredOmega;
  measuredOmega << 0, 0, M_PI / 10.0;
  double deltaT = 0.001;
  double tol = 1e-4;
  for (int i = 0; i < 1000; ++i)
    pim.integrateMeasurement(measuredAcc, measuredOmega, deltaT);
  CombinedImuFactor Combinedfactor(X(1), V(1), X(2), V(2), B(1), B(2), pim);

  // Predict
  Pose3 x(Rot3::Ypr(0, 0, 0), Point3(0, 0, 0)), x2;
  Vector3 v(0, 0, 0), v2;
  NavState actual = pim.predict(NavState(x, v), bias);
  Pose3 expectedPose(Rot3::Ypr(M_PI / 10, 0, 0), Point3(0, 0, 0));
  EXPECT(assert_equal(expectedPose, actual.pose(), tol));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

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
 * @author  Stephen Williams
 * @author  Richard Roberts
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

using namespace std;
using namespace gtsam;

// Convenience for named keys
using symbol_shorthand::X;
using symbol_shorthand::V;
using symbol_shorthand::B;

namespace {

// Auxiliary functions to test preintegrated Jacobians
// delPdelBiasAcc_ delPdelBiasOmega_ delVdelBiasAcc_ delVdelBiasOmega_ delRdelBiasOmega_
/* ************************************************************************* */
CombinedImuFactor::CombinedPreintegratedMeasurements evaluatePreintegratedMeasurements(
    const imuBias::ConstantBias& bias, const list<Vector3>& measuredAccs,
    const list<Vector3>& measuredOmegas, const list<double>& deltaTs) {
  CombinedImuFactor::CombinedPreintegratedMeasurements result(bias, I_3x3,
      I_3x3, I_3x3, I_3x3, I_3x3, I_6x6);

  list<Vector3>::const_iterator itAcc = measuredAccs.begin();
  list<Vector3>::const_iterator itOmega = measuredOmegas.begin();
  list<double>::const_iterator itDeltaT = deltaTs.begin();
  for (; itAcc != measuredAccs.end(); ++itAcc, ++itOmega, ++itDeltaT) {
    result.integrateMeasurement(*itAcc, *itOmega, *itDeltaT);
  }
  return result;
}

Vector3 evaluatePreintegratedMeasurementsPosition(
    const imuBias::ConstantBias& bias, const list<Vector3>& measuredAccs,
    const list<Vector3>& measuredOmegas, const list<double>& deltaTs) {
  return evaluatePreintegratedMeasurements(bias, measuredAccs, measuredOmegas,
      deltaTs).deltaPij();
}

Vector3 evaluatePreintegratedMeasurementsVelocity(
    const imuBias::ConstantBias& bias, const list<Vector3>& measuredAccs,
    const list<Vector3>& measuredOmegas, const list<double>& deltaTs) {
  return evaluatePreintegratedMeasurements(bias, measuredAccs, measuredOmegas,
      deltaTs).deltaVij();
}

Rot3 evaluatePreintegratedMeasurementsRotation(
    const imuBias::ConstantBias& bias, const list<Vector3>& measuredAccs,
    const list<Vector3>& measuredOmegas, const list<double>& deltaTs) {
  return Rot3(
      evaluatePreintegratedMeasurements(bias, measuredAccs, measuredOmegas,
          deltaTs).deltaRij());
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

  // Actual preintegrated values
  ImuFactor::PreintegratedMeasurements expected1(bias, Z_3x3, Z_3x3, Z_3x3);
  expected1.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

  CombinedImuFactor::CombinedPreintegratedMeasurements actual1(bias, Z_3x3,
      Z_3x3, Z_3x3, Z_3x3, Z_3x3, Z_6x6);

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

  // Measurements
  Vector3 gravity;
  gravity << 0, 0, 9.81;
  Vector3 omegaCoriolis;
  omegaCoriolis << 0, 0.1, 0.1;
  Vector3 measuredOmega;
  measuredOmega << 0, 0, M_PI / 10.0 + 0.3;
  Vector3 measuredAcc = x1.rotation().unrotate(-Point3(gravity)).vector()
      + Vector3(0.2, 0.0, 0.0);
  double deltaT = 1.0;
  double tol = 1e-6;

  ImuFactor::PreintegratedMeasurements pim(
      imuBias::ConstantBias(Vector3(0.2, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)),
      I_3x3, I_3x3, I_3x3);

  pim.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

  CombinedImuFactor::CombinedPreintegratedMeasurements combined_pim(
      imuBias::ConstantBias(Vector3(0.2, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)),
      I_3x3, I_3x3, I_3x3, I_3x3, 2 * I_3x3, I_6x6);

  combined_pim.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

  // Create factor
  ImuFactor imuFactor(X(1), V(1), X(2), V(2), B(1), pim, gravity,
      omegaCoriolis);

  noiseModel::Gaussian::shared_ptr Combinedmodel =
      noiseModel::Gaussian::Covariance(combined_pim.preintMeasCov());
  CombinedImuFactor combinedfactor(X(1), V(1), X(2), V(2), B(1), B(2),
      combined_pim, gravity, omegaCoriolis);

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
TEST( CombinedImuFactor, FirstOrderPreIntegratedMeasurements ) {
  // Linearization point
  imuBias::ConstantBias bias; ///< Current estimate of acceleration and rotation rate biases

  Pose3 body_P_sensor(Rot3::Expmap(Vector3(0, 0.1, 0.1)), Point3(1, 0, 1));

  // Measurements
  list<Vector3> measuredAccs, measuredOmegas;
  list<double> deltaTs;
  measuredAccs.push_back(Vector3(0.1, 0.0, 0.0));
  measuredOmegas.push_back(Vector3(M_PI / 100.0, 0.0, 0.0));
  deltaTs.push_back(0.01);
  measuredAccs.push_back(Vector3(0.1, 0.0, 0.0));
  measuredOmegas.push_back(Vector3(M_PI / 100.0, 0.0, 0.0));
  deltaTs.push_back(0.01);
  for (int i = 1; i < 100; i++) {
    measuredAccs.push_back(Vector3(0.05, 0.09, 0.01));
    measuredOmegas.push_back(
        Vector3(M_PI / 100.0, M_PI / 300.0, 2 * M_PI / 100.0));
    deltaTs.push_back(0.01);
  }

  // Actual preintegrated values
  CombinedImuFactor::CombinedPreintegratedMeasurements pim =
      evaluatePreintegratedMeasurements(bias, measuredAccs, measuredOmegas,
          deltaTs);

  // Compute numerical derivatives
  Matrix expectedDelPdelBias = numericalDerivative11<Vector,
      imuBias::ConstantBias>(
      boost::bind(&evaluatePreintegratedMeasurementsPosition, _1, measuredAccs,
          measuredOmegas, deltaTs), bias);
  Matrix expectedDelPdelBiasAcc = expectedDelPdelBias.leftCols(3);
  Matrix expectedDelPdelBiasOmega = expectedDelPdelBias.rightCols(3);

  Matrix expectedDelVdelBias = numericalDerivative11<Vector,
      imuBias::ConstantBias>(
      boost::bind(&evaluatePreintegratedMeasurementsVelocity, _1, measuredAccs,
          measuredOmegas, deltaTs), bias);
  Matrix expectedDelVdelBiasAcc = expectedDelVdelBias.leftCols(3);
  Matrix expectedDelVdelBiasOmega = expectedDelVdelBias.rightCols(3);

  Matrix expectedDelRdelBias =
      numericalDerivative11<Rot3, imuBias::ConstantBias>(
          boost::bind(&evaluatePreintegratedMeasurementsRotation, _1,
              measuredAccs, measuredOmegas, deltaTs), bias);
  Matrix expectedDelRdelBiasAcc = expectedDelRdelBias.leftCols(3);
  Matrix expectedDelRdelBiasOmega = expectedDelRdelBias.rightCols(3);

  // Compare Jacobians
  EXPECT(assert_equal(expectedDelPdelBiasAcc, pim.delPdelBiasAcc()));
  EXPECT(assert_equal(expectedDelPdelBiasOmega, pim.delPdelBiasOmega()));
  EXPECT(assert_equal(expectedDelVdelBiasAcc, pim.delVdelBiasAcc()));
  EXPECT(assert_equal(expectedDelVdelBiasOmega, pim.delVdelBiasOmega()));
  EXPECT(assert_equal(expectedDelRdelBiasAcc, Matrix::Zero(3, 3)));
  EXPECT(assert_equal(expectedDelRdelBiasOmega, pim.delRdelBiasOmega(), 1e-3)); // 1e-3 needs to be added only when using quaternions for rotations
}

/* ************************************************************************* */
TEST(CombinedImuFactor, PredictPositionAndVelocity) {
  imuBias::ConstantBias bias(Vector3(0, 0.1, 0), Vector3(0, 0.1, 0)); // Biases (acc, rot)

  // Measurements
  Vector3 gravity;
  gravity << 0, 0, 9.81;
  Vector3 omegaCoriolis;
  omegaCoriolis << 0, 0, 0;
  Vector3 measuredOmega;
  measuredOmega << 0, 0.1, 0; //M_PI/10.0+0.3;
  Vector3 measuredAcc;
  measuredAcc << 0, 1.1, -9.81;
  double deltaT = 0.001;

  CombinedImuFactor::CombinedPreintegratedMeasurements pim(bias, I_3x3, I_3x3,
      I_3x3, I_3x3, 2 * I_3x3, I_6x6);

  for (int i = 0; i < 1000; ++i)
    pim.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

  // Create factor
  noiseModel::Gaussian::shared_ptr combinedmodel =
      noiseModel::Gaussian::Covariance(pim.preintMeasCov());
  CombinedImuFactor Combinedfactor(X(1), V(1), X(2), V(2), B(1), B(2), pim,
      gravity, omegaCoriolis);

  // Predict
  Pose3 x1;
  Vector3 v1(0, 0.0, 0.0);
  PoseVelocityBias poseVelocityBias = pim.predict(x1, v1, bias, gravity,
      omegaCoriolis);
  Pose3 expectedPose(Rot3(), Point3(0, 0.5, 0));
  Vector3 expectedVelocity;
  expectedVelocity << 0, 1, 0;
  EXPECT(assert_equal(expectedPose, poseVelocityBias.pose));
  EXPECT(
      assert_equal(Vector(expectedVelocity), Vector(poseVelocityBias.velocity)));
}

/* ************************************************************************* */
TEST(CombinedImuFactor, PredictRotation) {
  imuBias::ConstantBias bias(Vector3(0, 0, 0), Vector3(0, 0, 0)); // Biases (acc, rot)
  CombinedImuFactor::CombinedPreintegratedMeasurements pim(bias, I_3x3, I_3x3,
      I_3x3, I_3x3, 2 * I_3x3, I_6x6);
  Vector3 measuredAcc;
  measuredAcc << 0, 0, -9.81;
  Vector3 gravity;
  gravity << 0, 0, 9.81;
  Vector3 omegaCoriolis;
  omegaCoriolis << 0, 0, 0;
  Vector3 measuredOmega;
  measuredOmega << 0, 0, M_PI / 10.0;
  double deltaT = 0.001;
  double tol = 1e-4;
  for (int i = 0; i < 1000; ++i)
    pim.integrateMeasurement(measuredAcc, measuredOmega, deltaT);
  CombinedImuFactor Combinedfactor(X(1), V(1), X(2), V(2), B(1), B(2), pim,
      gravity, omegaCoriolis);

  // Predict
  Pose3 x(Rot3().ypr(0, 0, 0), Point3(0, 0, 0)), x2;
  Vector3 v(0, 0, 0), v2;
  CombinedImuFactor::Predict(x, v, x2, v2, bias, pim, gravity, omegaCoriolis);
  Pose3 expectedPose(Rot3().ypr(M_PI / 10, 0, 0), Point3(0, 0, 0));
  EXPECT(assert_equal(expectedPose, x2, tol));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

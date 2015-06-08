/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testImuFactor.cpp
 * @brief   Unit test for ImuFactor
 * @author  Luca Carlone, Stephen Williams, Richard Roberts
 */

#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/geometry/Pose3.h>
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

/* ************************************************************************* */
namespace {
// Auxiliary functions to test evaluate error in ImuFactor
/* ************************************************************************* */
Vector callEvaluateError(const ImuFactor& factor, const Pose3& pose_i,
    const Vector3& vel_i, const Pose3& pose_j, const Vector3& vel_j,
    const imuBias::ConstantBias& bias) {
  return factor.evaluateError(pose_i, vel_i, pose_j, vel_j, bias);
}

Rot3 evaluateRotationError(const ImuFactor& factor, const Pose3& pose_i,
    const Vector3& vel_i, const Pose3& pose_j, const Vector3& vel_j,
    const imuBias::ConstantBias& bias) {
  return Rot3::Expmap(
      factor.evaluateError(pose_i, vel_i, pose_j, vel_j, bias).tail(3));
}

// Auxiliary functions to test Jacobians F and G used for
// covariance propagation during preintegration
/* ************************************************************************* */
Vector updatePreintegratedPosVel(const Vector3 deltaPij_old,
    const Vector3& deltaVij_old, const Rot3& deltaRij_old,
    const Vector3& correctedAcc, const Vector3& correctedOmega,
    const double deltaT, const bool use2ndOrderIntegration_) {

  Matrix3 dRij = deltaRij_old.matrix();
  Vector3 temp = dRij * correctedAcc * deltaT;
  Vector3 deltaPij_new;
  if (!use2ndOrderIntegration_) {
    deltaPij_new = deltaPij_old + deltaVij_old * deltaT;
  } else {
    deltaPij_new = deltaPij_old + deltaVij_old * deltaT + 0.5 * temp * deltaT;
  }
  Vector3 deltaVij_new = deltaVij_old + temp;

  Vector result(6);
  result << deltaPij_new, deltaVij_new;
  return result;
}

Rot3 updatePreintegratedRot(const Rot3& deltaRij_old,
    const Vector3& correctedOmega, const double deltaT) {
  Rot3 deltaRij_new = deltaRij_old * Rot3::Expmap(correctedOmega * deltaT);
  return deltaRij_new;
}

// Define covariance matrices
/* ************************************************************************* */
double accNoiseVar = 0.01;
double omegaNoiseVar = 0.03;
double intNoiseVar = 0.0001;
const Matrix3 kMeasuredAccCovariance = accNoiseVar * Matrix3::Identity();
const Matrix3 kMeasuredOmegaCovariance = omegaNoiseVar * Matrix3::Identity();
const Matrix3 kIntegrationErrorCovariance = intNoiseVar * Matrix3::Identity();

// Auxiliary functions to test preintegrated Jacobians
// delPdelBiasAcc_ delPdelBiasOmega_ delVdelBiasAcc_ delVdelBiasOmega_ delRdelBiasOmega_
/* ************************************************************************* */
ImuFactor::PreintegratedMeasurements evaluatePreintegratedMeasurements(
    const imuBias::ConstantBias& bias, const list<Vector3>& measuredAccs,
    const list<Vector3>& measuredOmegas, const list<double>& deltaTs,
    const bool use2ndOrderIntegration = false) {
  ImuFactor::PreintegratedMeasurements result(bias,
                                              kMeasuredAccCovariance,
                                              kMeasuredOmegaCovariance,
                                              kIntegrationErrorCovariance,
                                              use2ndOrderIntegration);

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

Rot3 evaluateRotation(const Vector3 measuredOmega, const Vector3 biasOmega,
    const double deltaT) {
  return Rot3::Expmap((measuredOmega - biasOmega) * deltaT);
}

Vector3 evaluateLogRotation(const Vector3 thetahat, const Vector3 deltatheta) {
  return Rot3::Logmap(Rot3::Expmap(thetahat).compose(Rot3::Expmap(deltatheta)));
}

}

/* ************************************************************************* */
TEST( ImuFactor, PreintegratedMeasurements ) {
  // Linearization point
  imuBias::ConstantBias bias(Vector3(0, 0, 0), Vector3(0, 0, 0)); ///< Current estimate of acceleration and angular rate biases

  // Measurements
  Vector3 measuredAcc(0.1, 0.0, 0.0);
  Vector3 measuredOmega(M_PI / 100.0, 0.0, 0.0);
  double deltaT = 0.5;

  // Expected preintegrated values
  Vector3 expectedDeltaP1;
  expectedDeltaP1 << 0.5 * 0.1 * 0.5 * 0.5, 0, 0;
  Vector3 expectedDeltaV1(0.05, 0.0, 0.0);
  Rot3 expectedDeltaR1 = Rot3::RzRyRx(0.5 * M_PI / 100.0, 0.0, 0.0);
  double expectedDeltaT1(0.5);

  bool use2ndOrderIntegration = true;
  // Actual preintegrated values
  ImuFactor::PreintegratedMeasurements actual1(bias,
                                               kMeasuredAccCovariance,
                                               kMeasuredOmegaCovariance,
                                               kIntegrationErrorCovariance,
                                               use2ndOrderIntegration);
  actual1.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

  EXPECT(
      assert_equal(Vector(expectedDeltaP1), Vector(actual1.deltaPij()), 1e-6));
  EXPECT(
      assert_equal(Vector(expectedDeltaV1), Vector(actual1.deltaVij()), 1e-6));
  EXPECT(assert_equal(expectedDeltaR1, Rot3(actual1.deltaRij()), 1e-6));
  DOUBLES_EQUAL(expectedDeltaT1, actual1.deltaTij(), 1e-6);

  // Integrate again
  Vector3 expectedDeltaP2;
  expectedDeltaP2 << 0.025 + expectedDeltaP1(0) + 0.5 * 0.1 * 0.5 * 0.5, 0, 0;
  Vector3 expectedDeltaV2 = Vector3(0.05, 0.0, 0.0)
      + expectedDeltaR1.matrix() * measuredAcc * 0.5;
  Rot3 expectedDeltaR2 = Rot3::RzRyRx(2.0 * 0.5 * M_PI / 100.0, 0.0, 0.0);
  double expectedDeltaT2(1);

  // Actual preintegrated values
  ImuFactor::PreintegratedMeasurements actual2 = actual1;
  actual2.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

  EXPECT(
      assert_equal(Vector(expectedDeltaP2), Vector(actual2.deltaPij()), 1e-6));
  EXPECT(
      assert_equal(Vector(expectedDeltaV2), Vector(actual2.deltaVij()), 1e-6));
  EXPECT(assert_equal(expectedDeltaR2, Rot3(actual2.deltaRij()), 1e-6));
  DOUBLES_EQUAL(expectedDeltaT2, actual2.deltaTij(), 1e-6);
}

/* ************************************************************************* */
TEST( ImuFactor, ErrorAndJacobians ) {
  // Linearization point
  imuBias::ConstantBias bias; // Bias
  Pose3 x1(Rot3::RzRyRx(M_PI / 12.0, M_PI / 6.0, M_PI / 4.0),
      Point3(5.0, 1.0, -50.0));
  Vector3 v1(Vector3(0.5, 0.0, 0.0));
  Pose3 x2(Rot3::RzRyRx(M_PI / 12.0 + M_PI / 100.0, M_PI / 6.0, M_PI / 4.0),
      Point3(5.5, 1.0, -50.0));
  Vector3 v2(Vector3(0.5, 0.0, 0.0));

  // Measurements
  Vector3 gravity;
  gravity << 0, 0, 9.81;
  Vector3 omegaCoriolis;
  omegaCoriolis << 0, 0, 0;
  Vector3 measuredOmega;
  measuredOmega << M_PI / 100, 0, 0;
  Vector3 measuredAcc = x1.rotation().unrotate(-Point3(gravity)).vector();
  double deltaT = 1.0;
  bool use2ndOrderIntegration = true;
  ImuFactor::PreintegratedMeasurements pre_int_data(bias,
                                                    kMeasuredAccCovariance,
                                                    kMeasuredOmegaCovariance,
                                                    kIntegrationErrorCovariance,
                                                    use2ndOrderIntegration);
  pre_int_data.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

  // Create factor
  ImuFactor factor(X(1), V(1), X(2), V(2), B(1), pre_int_data, gravity,
      omegaCoriolis);

  Vector errorActual = factor.evaluateError(x1, v1, x2, v2, bias);

  // Expected error
  Vector errorExpected(9);
  errorExpected << 0, 0, 0, 0, 0, 0, 0, 0, 0;
  EXPECT(assert_equal(errorExpected, errorActual, 1e-6));

  // Actual Jacobians
  Matrix H1a, H2a, H3a, H4a, H5a;
  (void) factor.evaluateError(x1, v1, x2, v2, bias, H1a, H2a, H3a, H4a, H5a);

  // Expected Jacobians
  /////////////////// H1 ///////////////////////////
  Matrix H1e = numericalDerivative11<Vector, Pose3>(
      boost::bind(&callEvaluateError, factor, _1, v1, x2, v2, bias), x1);
  // Jacobians are around zero, so the rotation part is the same as:
  Matrix H1Rot3 = numericalDerivative11<Rot3, Pose3>(
      boost::bind(&evaluateRotationError, factor, _1, v1, x2, v2, bias), x1);
  EXPECT(assert_equal(H1Rot3, H1e.bottomRows(3)));
  EXPECT(assert_equal(H1e, H1a));

  /////////////////// H2 ///////////////////////////
  Matrix H2e = numericalDerivative11<Vector, Vector3>(
      boost::bind(&callEvaluateError, factor, x1, _1, x2, v2, bias), v1);
  EXPECT(assert_equal(H2e, H2a));

  /////////////////// H3 ///////////////////////////
  Matrix H3e = numericalDerivative11<Vector, Pose3>(
      boost::bind(&callEvaluateError, factor, x1, v1, _1, v2, bias), x2);
  // Jacobians are around zero, so the rotation part is the same as:
  Matrix H3Rot3 = numericalDerivative11<Rot3, Pose3>(
      boost::bind(&evaluateRotationError, factor, x1, v1, _1, v2, bias), x2);
  EXPECT(assert_equal(H3Rot3, H3e.bottomRows(3)));
  EXPECT(assert_equal(H3e, H3a));

  /////////////////// H4 ///////////////////////////
  Matrix H4e = numericalDerivative11<Vector, Vector3>(
      boost::bind(&callEvaluateError, factor, x1, v1, x2, _1, bias), v2);
  EXPECT(assert_equal(H4e, H4a));

  /////////////////// H5 ///////////////////////////
  Matrix H5e = numericalDerivative11<Vector, imuBias::ConstantBias>(
      boost::bind(&callEvaluateError, factor, x1, v1, x2, v2, _1), bias);
  EXPECT(assert_equal(H5e, H5a));

  ////////////////////////////////////////////////////////////////////////////
  // Evaluate error with wrong values
  Vector3 v2_wrong = v2 + Vector3(0.1, 0.1, 0.1);
  errorActual = factor.evaluateError(x1, v1, x2, v2_wrong, bias);
  errorExpected << 0, 0, 0, 0.0724744871, 0.040715657, 0.151952901, 0, 0, 0;
  EXPECT(assert_equal(errorExpected, errorActual, 1e-6));

  Values values;
  values.insert(X(1), x1);
  values.insert(V(1), v1);
  values.insert(X(2), x2);
  values.insert(V(2), v2_wrong);
  values.insert(B(1), bias);
  errorExpected << 0, 0, 0, 0.0724744871, 0.040715657, 0.151952901, 0, 0, 0;
  EXPECT(assert_equal(factor.unwhitenedError(values), errorActual, 1e-6));

  // Make sure the whitening is done correctly
  Matrix cov = pre_int_data.preintMeasCov();
  Matrix R = RtR(cov.inverse());
  Vector whitened = R * errorActual;
  EXPECT(assert_equal(0.5 * whitened.squaredNorm(), factor.error(values), 1e-6));

  ///////////////////////////////////////////////////////////////////////////////
  // Make sure linearization is correct
  // Create expected value by numerical differentiation
  JacobianFactor expected = linearizeNumerically(factor, values, 1e-8);

  // Create actual value by linearize
  GaussianFactor::shared_ptr linearized = factor.linearize(values);
  JacobianFactor* actual = dynamic_cast<JacobianFactor*>(linearized.get());

  // Check cast result and then equality
  EXPECT(assert_equal(expected, *actual, 1e-4));
}

/* ************************************************************************* */
TEST( ImuFactor, ErrorAndJacobianWithBiases ) {
  imuBias::ConstantBias bias(Vector3(0.2, 0, 0), Vector3(0.1, 0, 0.3)); // Biases (acc, rot)
  Pose3 x1(Rot3::RzRyRx(M_PI / 12.0, M_PI / 6.0, M_PI / 10.0),
      Point3(5.0, 1.0, -50.0));
  Vector3 v1(Vector3(0.5, 0.0, 0.0));
  Pose3 x2(Rot3::Expmap(Vector3(0, 0, M_PI / 10.0 + M_PI / 10.0)),
      Point3(5.5, 1.0, -50.0));
  Vector3 v2(Vector3(0.5, 0.0, 0.0));

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

  ImuFactor::PreintegratedMeasurements pre_int_data(
      imuBias::ConstantBias(Vector3(0.2, 0.0, 0.0), Vector3(0.0, 0.0, 0.1)), kMeasuredAccCovariance,
      kMeasuredOmegaCovariance, kIntegrationErrorCovariance);
  pre_int_data.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

  // Create factor
  ImuFactor factor(X(1), V(1), X(2), V(2), B(1), pre_int_data, gravity,
      omegaCoriolis);

  SETDEBUG("ImuFactor evaluateError", false);
  Vector errorActual = factor.evaluateError(x1, v1, x2, v2, bias);
  SETDEBUG("ImuFactor evaluateError", false);

  // Expected error (should not be zero in this test, as we want to evaluate Jacobians
  // at a nontrivial linearization point)
  // Vector errorExpected(9); errorExpected << 0, 0, 0, 0, 0, 0, 0, 0, 0;
  // EXPECT(assert_equal(errorExpected, errorActual, 1e-6));

  // Expected Jacobians
  Matrix H1e = numericalDerivative11<Vector, Pose3>(
      boost::bind(&callEvaluateError, factor, _1, v1, x2, v2, bias), x1);
  Matrix H2e = numericalDerivative11<Vector, Vector3>(
      boost::bind(&callEvaluateError, factor, x1, _1, x2, v2, bias), v1);
  Matrix H3e = numericalDerivative11<Vector, Pose3>(
      boost::bind(&callEvaluateError, factor, x1, v1, _1, v2, bias), x2);
  Matrix H4e = numericalDerivative11<Vector, Vector3>(
      boost::bind(&callEvaluateError, factor, x1, v1, x2, _1, bias), v2);
  Matrix H5e = numericalDerivative11<Vector, imuBias::ConstantBias>(
      boost::bind(&callEvaluateError, factor, x1, v1, x2, v2, _1), bias);

  // Check rotation Jacobians
  Matrix RH1e = numericalDerivative11<Rot3, Pose3>(
      boost::bind(&evaluateRotationError, factor, _1, v1, x2, v2, bias), x1);
  Matrix RH3e = numericalDerivative11<Rot3, Pose3>(
      boost::bind(&evaluateRotationError, factor, x1, v1, _1, v2, bias), x2);
  Matrix RH5e = numericalDerivative11<Rot3, imuBias::ConstantBias>(
      boost::bind(&evaluateRotationError, factor, x1, v1, x2, v2, _1), bias);

  // Actual Jacobians
  Matrix H1a, H2a, H3a, H4a, H5a;
  (void) factor.evaluateError(x1, v1, x2, v2, bias, H1a, H2a, H3a, H4a, H5a);

  EXPECT(assert_equal(H1e, H1a));
  EXPECT(assert_equal(H2e, H2a));
  EXPECT(assert_equal(H3e, H3a));
  EXPECT(assert_equal(H4e, H4a));
  EXPECT(assert_equal(H5e, H5a));
}

/* ************************************************************************* */
TEST( ImuFactor, ErrorAndJacobianWith2ndOrderCoriolis ) {
  imuBias::ConstantBias bias(Vector3(0.2, 0, 0), Vector3(0.1, 0, 0.3)); // Biases (acc, rot)
  Pose3 x1(Rot3::RzRyRx(M_PI / 12.0, M_PI / 6.0, M_PI / 10.0),
      Point3(5.0, 1.0, -50.0));
  Vector3 v1(Vector3(0.5, 0.0, 0.0));
  Pose3 x2(Rot3::Expmap(Vector3(0, 0, M_PI / 10.0 + M_PI / 10.0)),
      Point3(5.5, 1.0, -50.0));
  Vector3 v2(Vector3(0.5, 0.0, 0.0));

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

  ImuFactor::PreintegratedMeasurements pre_int_data(
      imuBias::ConstantBias(Vector3(0.2, 0.0, 0.0), Vector3(0.0, 0.0, 0.1)), kMeasuredAccCovariance,
      kMeasuredOmegaCovariance, kIntegrationErrorCovariance);
  pre_int_data.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

  // Create factor
  Pose3 bodyPsensor = Pose3();
  bool use2ndOrderCoriolis = true;
  ImuFactor factor(X(1), V(1), X(2), V(2), B(1), pre_int_data, gravity,
      omegaCoriolis, bodyPsensor, use2ndOrderCoriolis);

  SETDEBUG("ImuFactor evaluateError", false);
  Vector errorActual = factor.evaluateError(x1, v1, x2, v2, bias);
  SETDEBUG("ImuFactor evaluateError", false);

  // Expected error (should not be zero in this test, as we want to evaluate Jacobians
  // at a nontrivial linearization point)
  // Vector errorExpected(9); errorExpected << 0, 0, 0, 0, 0, 0, 0, 0, 0;
  // EXPECT(assert_equal(errorExpected, errorActual, 1e-6));

  // Expected Jacobians
  Matrix H1e = numericalDerivative11<Vector, Pose3>(
      boost::bind(&callEvaluateError, factor, _1, v1, x2, v2, bias), x1);
  Matrix H2e = numericalDerivative11<Vector, Vector3>(
      boost::bind(&callEvaluateError, factor, x1, _1, x2, v2, bias), v1);
  Matrix H3e = numericalDerivative11<Vector, Pose3>(
      boost::bind(&callEvaluateError, factor, x1, v1, _1, v2, bias), x2);
  Matrix H4e = numericalDerivative11<Vector, Vector3>(
      boost::bind(&callEvaluateError, factor, x1, v1, x2, _1, bias), v2);
  Matrix H5e = numericalDerivative11<Vector, imuBias::ConstantBias>(
      boost::bind(&callEvaluateError, factor, x1, v1, x2, v2, _1), bias);

  // Check rotation Jacobians
  Matrix RH1e = numericalDerivative11<Rot3, Pose3>(
      boost::bind(&evaluateRotationError, factor, _1, v1, x2, v2, bias), x1);
  Matrix RH3e = numericalDerivative11<Rot3, Pose3>(
      boost::bind(&evaluateRotationError, factor, x1, v1, _1, v2, bias), x2);
  Matrix RH5e = numericalDerivative11<Rot3, imuBias::ConstantBias>(
      boost::bind(&evaluateRotationError, factor, x1, v1, x2, v2, _1), bias);

  // Actual Jacobians
  Matrix H1a, H2a, H3a, H4a, H5a;
  (void) factor.evaluateError(x1, v1, x2, v2, bias, H1a, H2a, H3a, H4a, H5a);

  EXPECT(assert_equal(H1e, H1a));
  EXPECT(assert_equal(H2e, H2a));
  EXPECT(assert_equal(H3e, H3a));
  EXPECT(assert_equal(H4e, H4a));
  EXPECT(assert_equal(H5e, H5a));
}

/* ************************************************************************* */
TEST( ImuFactor, PartialDerivative_wrt_Bias ) {
  // Linearization point
  Vector3 biasOmega;
  biasOmega << 0, 0, 0; ///< Current estimate of rotation rate bias

  // Measurements
  Vector3 measuredOmega;
  measuredOmega << 0.1, 0, 0;
  double deltaT = 0.5;

  // Compute numerical derivatives
  Matrix expectedDelRdelBiasOmega = numericalDerivative11<Rot3, Vector3>(
      boost::bind(&evaluateRotation, measuredOmega, _1, deltaT),
      Vector3(biasOmega));

  const Matrix3 Jr = Rot3::ExpmapDerivative(
      (measuredOmega - biasOmega) * deltaT);

  Matrix3 actualdelRdelBiasOmega = -Jr * deltaT; // the delta bias appears with the minus sign

  // Compare Jacobians
  EXPECT(assert_equal(expectedDelRdelBiasOmega, actualdelRdelBiasOmega, 1e-3)); // 1e-3 needs to be added only when using quaternions for rotations
}

/* ************************************************************************* */
TEST( ImuFactor, PartialDerivativeLogmap ) {
  // Linearization point
  Vector3 thetahat;
  thetahat << 0.1, 0.1, 0; ///< Current estimate of rotation rate bias

  // Measurements
  Vector3 deltatheta;
  deltatheta << 0, 0, 0;

  // Compute numerical derivatives
  Matrix expectedDelFdeltheta = numericalDerivative11<Vector, Vector3>(
      boost::bind(&evaluateLogRotation, thetahat, _1), Vector3(deltatheta));

  Matrix3 actualDelFdeltheta = Rot3::LogmapDerivative(thetahat);

  // Compare Jacobians
  EXPECT(assert_equal(expectedDelFdeltheta, actualDelFdeltheta));
}

/* ************************************************************************* */
TEST( ImuFactor, fistOrderExponential ) {
  // Linearization point
  Vector3 biasOmega;
  biasOmega << 0, 0, 0; ///< Current estimate of rotation rate bias

  // Measurements
  Vector3 measuredOmega;
  measuredOmega << 0.1, 0, 0;
  double deltaT = 1.0;

  // change w.r.t. linearization point
  double alpha = 0.0;
  Vector3 deltabiasOmega;
  deltabiasOmega << alpha, alpha, alpha;

  const Matrix3 Jr = Rot3::ExpmapDerivative(
      (measuredOmega - biasOmega) * deltaT);

  Matrix3 delRdelBiasOmega = -Jr * deltaT; // the delta bias appears with the minus sign

  const Matrix expectedRot = Rot3::Expmap(
      (measuredOmega - biasOmega - deltabiasOmega) * deltaT).matrix();

  const Matrix3 hatRot =
      Rot3::Expmap((measuredOmega - biasOmega) * deltaT).matrix();
  const Matrix3 actualRot = hatRot
      * Rot3::Expmap(delRdelBiasOmega * deltabiasOmega).matrix();
  //hatRot * (Matrix3::Identity() + skewSymmetric(delRdelBiasOmega * deltabiasOmega));

  // This is a first order expansion so the equality is only an approximation
  EXPECT(assert_equal(expectedRot, actualRot));
}

/* ************************************************************************* */
TEST( ImuFactor, FirstOrderPreIntegratedMeasurements ) {
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
  ImuFactor::PreintegratedMeasurements preintegrated =
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
  EXPECT(assert_equal(expectedDelPdelBiasAcc, preintegrated.delPdelBiasAcc()));
  EXPECT(
      assert_equal(expectedDelPdelBiasOmega, preintegrated.delPdelBiasOmega()));
  EXPECT(assert_equal(expectedDelVdelBiasAcc, preintegrated.delVdelBiasAcc()));
  EXPECT(
      assert_equal(expectedDelVdelBiasOmega, preintegrated.delVdelBiasOmega()));
  EXPECT(assert_equal(expectedDelRdelBiasAcc, Matrix::Zero(3, 3)));
  EXPECT(
      assert_equal(expectedDelRdelBiasOmega, preintegrated.delRdelBiasOmega(),
          1e-3)); // 1e-3 needs to be added only when using quaternions for rotations
}

/* ************************************************************************* */
TEST( ImuFactor, JacobianPreintegratedCovariancePropagation ) {
  // Linearization point
  imuBias::ConstantBias bias; ///< Current estimate of acceleration and rotation rate biases
  Pose3 body_P_sensor = Pose3(); // (Rot3::Expmap(Vector3(0,0.1,0.1)), Point3(1, 0, 1));

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
  bool use2ndOrderIntegration = false;
  // Actual preintegrated values
  ImuFactor::PreintegratedMeasurements preintegrated =
      evaluatePreintegratedMeasurements(bias, measuredAccs, measuredOmegas,
          deltaTs, use2ndOrderIntegration);

  // so far we only created a nontrivial linearization point for the preintegrated measurements
  // Now we add a new measurement and ask for Jacobians
  const Vector3 newMeasuredAcc = Vector3(0.1, 0.0, 0.0);
  const Vector3 newMeasuredOmega = Vector3(M_PI / 100.0, 0.0, 0.0);
  const double newDeltaT = 0.01;
  const Rot3 deltaRij_old = preintegrated.deltaRij(); // before adding new measurement
  const Vector3 deltaVij_old = preintegrated.deltaVij(); // before adding new measurement
  const Vector3 deltaPij_old = preintegrated.deltaPij(); // before adding new measurement

  Matrix oldPreintCovariance = preintegrated.preintMeasCov();

  Matrix Factual, Gactual;
  preintegrated.integrateMeasurement(newMeasuredAcc, newMeasuredOmega,
      newDeltaT, body_P_sensor, Factual, Gactual);

  //////////////////////////////////////////////////////////////////////////////////////////////
  // COMPUTE NUMERICAL DERIVATIVES FOR F
  //////////////////////////////////////////////////////////////////////////////////////////////
  // Compute expected f_pos_vel wrt positions
  Matrix dfpv_dpos = numericalDerivative11<Vector, Vector3>(
      boost::bind(&updatePreintegratedPosVel, _1, deltaVij_old, deltaRij_old,
          newMeasuredAcc, newMeasuredOmega, newDeltaT, use2ndOrderIntegration),
      deltaPij_old);

  // Compute expected f_pos_vel wrt velocities
  Matrix dfpv_dvel = numericalDerivative11<Vector, Vector3>(
      boost::bind(&updatePreintegratedPosVel, deltaPij_old, _1, deltaRij_old,
          newMeasuredAcc, newMeasuredOmega, newDeltaT, use2ndOrderIntegration),
      deltaVij_old);

  // Compute expected f_pos_vel wrt angles
  Matrix dfpv_dangle = numericalDerivative11<Vector, Rot3>(
      boost::bind(&updatePreintegratedPosVel, deltaPij_old, deltaVij_old, _1,
          newMeasuredAcc, newMeasuredOmega, newDeltaT, use2ndOrderIntegration),
      deltaRij_old);

  Matrix FexpectedTop6(6, 9);
  FexpectedTop6 << dfpv_dpos, dfpv_dvel, dfpv_dangle;

  // Compute expected f_rot wrt angles
  Matrix dfr_dangle = numericalDerivative11<Rot3, Rot3>(
      boost::bind(&updatePreintegratedRot, _1, newMeasuredOmega, newDeltaT),
      deltaRij_old);

  Matrix FexpectedBottom3(3, 9);
  FexpectedBottom3 << Z_3x3, Z_3x3, dfr_dangle;
  Matrix Fexpected(9, 9);
  Fexpected << FexpectedTop6, FexpectedBottom3;

  EXPECT(assert_equal(Fexpected, Factual));

  //////////////////////////////////////////////////////////////////////////////////////////////
  // COMPUTE NUMERICAL DERIVATIVES FOR G
  //////////////////////////////////////////////////////////////////////////////////////////////
  // Compute jacobian wrt integration noise
  Matrix dgpv_dintNoise(6, 3);
  dgpv_dintNoise << I_3x3 * newDeltaT, Z_3x3;

  // Compute jacobian wrt acc noise
  Matrix dgpv_daccNoise = numericalDerivative11<Vector, Vector3>(
      boost::bind(&updatePreintegratedPosVel, deltaPij_old, deltaVij_old,
          deltaRij_old, _1, newMeasuredOmega, newDeltaT,
          use2ndOrderIntegration), newMeasuredAcc);

  // Compute expected F wrt gyro noise
  Matrix dgpv_domegaNoise = numericalDerivative11<Vector, Vector3>(
      boost::bind(&updatePreintegratedPosVel, deltaPij_old, deltaVij_old,
          deltaRij_old, newMeasuredAcc, _1, newDeltaT, use2ndOrderIntegration),
      newMeasuredOmega);
  Matrix GexpectedTop6(6, 9);
  GexpectedTop6 << dgpv_dintNoise, dgpv_daccNoise, dgpv_domegaNoise;

  // Compute expected f_rot wrt gyro noise
  Matrix dgr_dangle = numericalDerivative11<Rot3, Vector3>(
      boost::bind(&updatePreintegratedRot, deltaRij_old, _1, newDeltaT),
      newMeasuredOmega);

  Matrix GexpectedBottom3(3, 9);
  GexpectedBottom3 << Z_3x3, Z_3x3, dgr_dangle;
  Matrix Gexpected(9, 9);
  Gexpected << GexpectedTop6, GexpectedBottom3;

  EXPECT(assert_equal(Gexpected, Gactual));

  // Check covariance propagation
  Matrix9 measurementCovariance;
  measurementCovariance << intNoiseVar * I_3x3, Z_3x3, Z_3x3, Z_3x3, accNoiseVar
      * I_3x3, Z_3x3, Z_3x3, Z_3x3, omegaNoiseVar * I_3x3;

  Matrix newPreintCovarianceExpected = Factual * oldPreintCovariance
      * Factual.transpose()
      + (1 / newDeltaT) * Gactual * measurementCovariance * Gactual.transpose();

  Matrix newPreintCovarianceActual = preintegrated.preintMeasCov();
  EXPECT(assert_equal(newPreintCovarianceExpected, newPreintCovarianceActual));
}

/* ************************************************************************* */
TEST( ImuFactor, JacobianPreintegratedCovariancePropagation_2ndOrderInt ) {
  // Linearization point
  imuBias::ConstantBias bias; ///< Current estimate of acceleration and rotation rate biases
  Pose3 body_P_sensor = Pose3(); // (Rot3::Expmap(Vector3(0,0.1,0.1)), Point3(1, 0, 1));

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
  bool use2ndOrderIntegration = true;
  // Actual preintegrated values
  ImuFactor::PreintegratedMeasurements preintegrated =
      evaluatePreintegratedMeasurements(bias, measuredAccs, measuredOmegas,
          deltaTs, use2ndOrderIntegration);

  // so far we only created a nontrivial linearization point for the preintegrated measurements
  // Now we add a new measurement and ask for Jacobians
  const Vector3 newMeasuredAcc = Vector3(0.1, 0.0, 0.0);
  const Vector3 newMeasuredOmega = Vector3(M_PI / 100.0, 0.0, 0.0);
  const double newDeltaT = 0.01;
  const Rot3 deltaRij_old = preintegrated.deltaRij(); // before adding new measurement
  const Vector3 deltaVij_old = preintegrated.deltaVij(); // before adding new measurement
  const Vector3 deltaPij_old = preintegrated.deltaPij(); // before adding new measurement

  Matrix oldPreintCovariance = preintegrated.preintMeasCov();

  Matrix Factual, Gactual;
  preintegrated.integrateMeasurement(newMeasuredAcc, newMeasuredOmega,
      newDeltaT, body_P_sensor, Factual, Gactual);

  //////////////////////////////////////////////////////////////////////////////////////////////
  // COMPUTE NUMERICAL DERIVATIVES FOR F
  //////////////////////////////////////////////////////////////////////////////////////////////
  // Compute expected f_pos_vel wrt positions
  Matrix dfpv_dpos = numericalDerivative11<Vector, Vector3>(
      boost::bind(&updatePreintegratedPosVel, _1, deltaVij_old, deltaRij_old,
          newMeasuredAcc, newMeasuredOmega, newDeltaT, use2ndOrderIntegration),
      deltaPij_old);

  // Compute expected f_pos_vel wrt velocities
  Matrix dfpv_dvel = numericalDerivative11<Vector, Vector3>(
      boost::bind(&updatePreintegratedPosVel, deltaPij_old, _1, deltaRij_old,
          newMeasuredAcc, newMeasuredOmega, newDeltaT, use2ndOrderIntegration),
      deltaVij_old);

  // Compute expected f_pos_vel wrt angles
  Matrix dfpv_dangle = numericalDerivative11<Vector, Rot3>(
      boost::bind(&updatePreintegratedPosVel, deltaPij_old, deltaVij_old, _1,
          newMeasuredAcc, newMeasuredOmega, newDeltaT, use2ndOrderIntegration),
      deltaRij_old);

  Matrix FexpectedTop6(6, 9);
  FexpectedTop6 << dfpv_dpos, dfpv_dvel, dfpv_dangle;

  // Compute expected f_rot wrt angles
  Matrix dfr_dangle = numericalDerivative11<Rot3, Rot3>(
      boost::bind(&updatePreintegratedRot, _1, newMeasuredOmega, newDeltaT),
      deltaRij_old);

  Matrix FexpectedBottom3(3, 9);
  FexpectedBottom3 << Z_3x3, Z_3x3, dfr_dangle;
  Matrix Fexpected(9, 9);
  Fexpected << FexpectedTop6, FexpectedBottom3;

  EXPECT(assert_equal(Fexpected, Factual));

  //////////////////////////////////////////////////////////////////////////////////////////////
  // COMPUTE NUMERICAL DERIVATIVES FOR G
  //////////////////////////////////////////////////////////////////////////////////////////////
  // Compute jacobian wrt integration noise
  Matrix dgpv_dintNoise(6, 3);
  dgpv_dintNoise << I_3x3 * newDeltaT, Z_3x3;

  // Compute jacobian wrt acc noise
  Matrix dgpv_daccNoise = numericalDerivative11<Vector, Vector3>(
      boost::bind(&updatePreintegratedPosVel, deltaPij_old, deltaVij_old,
          deltaRij_old, _1, newMeasuredOmega, newDeltaT,
          use2ndOrderIntegration), newMeasuredAcc);

  // Compute expected F wrt gyro noise
  Matrix dgpv_domegaNoise = numericalDerivative11<Vector, Vector3>(
      boost::bind(&updatePreintegratedPosVel, deltaPij_old, deltaVij_old,
          deltaRij_old, newMeasuredAcc, _1, newDeltaT, use2ndOrderIntegration),
      newMeasuredOmega);
  Matrix GexpectedTop6(6, 9);
  GexpectedTop6 << dgpv_dintNoise, dgpv_daccNoise, dgpv_domegaNoise;

  // Compute expected f_rot wrt gyro noise
  Matrix dgr_dangle = numericalDerivative11<Rot3, Vector3>(
      boost::bind(&updatePreintegratedRot, deltaRij_old, _1, newDeltaT),
      newMeasuredOmega);

  Matrix GexpectedBottom3(3, 9);
  GexpectedBottom3 << Z_3x3, Z_3x3, dgr_dangle;
  Matrix Gexpected(9, 9);
  Gexpected << GexpectedTop6, GexpectedBottom3;

  EXPECT(assert_equal(Gexpected, Gactual));

  // Check covariance propagation
  Matrix9 measurementCovariance;
  measurementCovariance << intNoiseVar * I_3x3, Z_3x3, Z_3x3, Z_3x3, accNoiseVar
      * I_3x3, Z_3x3, Z_3x3, Z_3x3, omegaNoiseVar * I_3x3;

  Matrix newPreintCovarianceExpected = Factual * oldPreintCovariance
      * Factual.transpose()
      + (1 / newDeltaT) * Gactual * measurementCovariance * Gactual.transpose();

  Matrix newPreintCovarianceActual = preintegrated.preintMeasCov();
  EXPECT(assert_equal(newPreintCovarianceExpected, newPreintCovarianceActual));
}

//#include <gtsam/linear/GaussianFactorGraph.h>
///* ************************************************************************* */
//TEST( ImuFactor, LinearizeTiming)
//{
//  // Linearization point
//  Pose3 x1(Rot3::RzRyRx(M_PI/12.0, M_PI/6.0, M_PI/4.0), Point3(5.0, 1.0, -50.0));
//  Vector3 v1(Vector3(0.5, 0.0, 0.0));
//  Pose3 x2(Rot3::RzRyRx(M_PI/12.0 + M_PI/100.0, M_PI/6.0, M_PI/4.0), Point3(5.5, 1.0, -50.0));
//  Vector3 v2(Vector3(0.5, 0.0, 0.0));
//  imuBias::ConstantBias bias(Vector3(0.001, 0.002, 0.008), Vector3(0.002, 0.004, 0.012));
//
//  // Pre-integrator
//  imuBias::ConstantBias biasHat(Vector3(0, 0, 0.10), Vector3(0, 0, 0.10));
//  Vector3 gravity; gravity << 0, 0, 9.81;
//  Vector3 omegaCoriolis; omegaCoriolis << 0.0001, 0, 0.01;
//  ImuFactor::PreintegratedMeasurements pre_int_data(biasHat, Matrix3::Identity(), Matrix3::Identity(), Matrix3::Identity());
//
//  // Pre-integrate Measurements
//  Vector3 measuredAcc(0.1, 0.0, 0.0);
//  Vector3 measuredOmega(M_PI/100.0, 0.0, 0.0);
//  double deltaT = 0.5;
//  for(size_t i = 0; i < 50; ++i) {
//    pre_int_data.integrateMeasurement(measuredAcc, measuredOmega, deltaT);
//  }
//
//  // Create factor
//  noiseModel::Base::shared_ptr model = noiseModel::Gaussian::Covariance(pre_int_data.MeasurementCovariance());
//  ImuFactor factor(X(1), V(1), X(2), V(2), B(1), pre_int_data, gravity, omegaCoriolis);
//
//  Values values;
//  values.insert(X(1), x1);
//  values.insert(X(2), x2);
//  values.insert(V(1), v1);
//  values.insert(V(2), v2);
//  values.insert(B(1), bias);
//
//  Ordering ordering;
//  ordering.push_back(X(1));
//  ordering.push_back(V(1));
//  ordering.push_back(X(2));
//  ordering.push_back(V(2));
//  ordering.push_back(B(1));
//
//  GaussianFactorGraph graph;
//  gttic_(LinearizeTiming);
//  for(size_t i = 0; i < 100000; ++i) {
//    GaussianFactor::shared_ptr g = factor.linearize(values, ordering);
//    graph.push_back(g);
//  }
//  gttoc_(LinearizeTiming);
//  tictoc_finishedIteration_();
//  std::cout << "Linear Error: " << graph.error(values.zeroVectors(ordering)) << std::endl;
//  tictoc_print_();
//}

/* ************************************************************************* */
TEST( ImuFactor, ErrorWithBiasesAndSensorBodyDisplacement ) {

  imuBias::ConstantBias bias(Vector3(0.2, 0, 0), Vector3(0, 0, 0.3)); // Biases (acc, rot)
  Pose3 x1(Rot3::Expmap(Vector3(0, 0, M_PI / 4.0)), Point3(5.0, 1.0, -50.0));
  Vector3 v1(Vector3(0.5, 0.0, 0.0));
  Pose3 x2(Rot3::Expmap(Vector3(0, 0, M_PI / 4.0 + M_PI / 10.0)),
      Point3(5.5, 1.0, -50.0));
  Vector3 v2(Vector3(0.5, 0.0, 0.0));

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

  const Pose3 body_P_sensor(Rot3::Expmap(Vector3(0, 0.10, 0.10)),
      Point3(1, 0, 0));

  ImuFactor::PreintegratedMeasurements pre_int_data(
      imuBias::ConstantBias(Vector3(0.2, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)), kMeasuredAccCovariance,
      kMeasuredOmegaCovariance, kIntegrationErrorCovariance);

  pre_int_data.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

  // Create factor
  ImuFactor factor(X(1), V(1), X(2), V(2), B(1), pre_int_data, gravity,
      omegaCoriolis);

  // Expected Jacobians
  Matrix H1e = numericalDerivative11<Vector, Pose3>(
      boost::bind(&callEvaluateError, factor, _1, v1, x2, v2, bias), x1);
  Matrix H2e = numericalDerivative11<Vector, Vector3>(
      boost::bind(&callEvaluateError, factor, x1, _1, x2, v2, bias), v1);
  Matrix H3e = numericalDerivative11<Vector, Pose3>(
      boost::bind(&callEvaluateError, factor, x1, v1, _1, v2, bias), x2);
  Matrix H4e = numericalDerivative11<Vector, Vector3>(
      boost::bind(&callEvaluateError, factor, x1, v1, x2, _1, bias), v2);
  Matrix H5e = numericalDerivative11<Vector, imuBias::ConstantBias>(
      boost::bind(&callEvaluateError, factor, x1, v1, x2, v2, _1), bias);

  // Check rotation Jacobians
  Matrix RH1e = numericalDerivative11<Rot3, Pose3>(
      boost::bind(&evaluateRotationError, factor, _1, v1, x2, v2, bias), x1);
  Matrix RH3e = numericalDerivative11<Rot3, Pose3>(
      boost::bind(&evaluateRotationError, factor, x1, v1, _1, v2, bias), x2);
  Matrix RH5e = numericalDerivative11<Rot3, imuBias::ConstantBias>(
      boost::bind(&evaluateRotationError, factor, x1, v1, x2, v2, _1), bias);

  // Actual Jacobians
  Matrix H1a, H2a, H3a, H4a, H5a;
  (void) factor.evaluateError(x1, v1, x2, v2, bias, H1a, H2a, H3a, H4a, H5a);

  EXPECT(assert_equal(H1e, H1a));
  EXPECT(assert_equal(H2e, H2a));
  EXPECT(assert_equal(H3e, H3a));
  EXPECT(assert_equal(H4e, H4a));
  EXPECT(assert_equal(H5e, H5a));
}

/* ************************************************************************* */
TEST(ImuFactor, PredictPositionAndVelocity) {
  imuBias::ConstantBias bias(Vector3(0, 0, 0), Vector3(0, 0, 0)); // Biases (acc, rot)

  // Measurements
  Vector3 gravity;
  gravity << 0, 0, 9.81;
  Vector3 omegaCoriolis;
  omegaCoriolis << 0, 0, 0;
  Vector3 measuredOmega;
  measuredOmega << 0, 0, 0; //M_PI/10.0+0.3;
  Vector3 measuredAcc;
  measuredAcc << 0, 1, -9.81;
  double deltaT = 0.001;

  Matrix I6x6(6, 6);
  I6x6 = Matrix::Identity(6, 6);

  ImuFactor::PreintegratedMeasurements pre_int_data(
      imuBias::ConstantBias(Vector3(0.2, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)), kMeasuredAccCovariance,
      kMeasuredOmegaCovariance, kIntegrationErrorCovariance, true);

  for (int i = 0; i < 1000; ++i)
    pre_int_data.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

  // Create factor
  ImuFactor factor(X(1), V(1), X(2), V(2), B(1), pre_int_data, gravity,
      omegaCoriolis);

  // Predict
  Pose3 x1;
  Vector3 v1(0, 0.0, 0.0);
  PoseVelocityBias poseVelocity = pre_int_data.predict(x1, v1, bias, gravity,
      omegaCoriolis);
  Pose3 expectedPose(Rot3(), Point3(0, 0.5, 0));
  Vector3 expectedVelocity;
  expectedVelocity << 0, 1, 0;
  EXPECT(assert_equal(expectedPose, poseVelocity.pose));
  EXPECT(assert_equal(Vector(expectedVelocity), Vector(poseVelocity.velocity)));
}

/* ************************************************************************* */
TEST(ImuFactor, PredictRotation) {
  imuBias::ConstantBias bias(Vector3(0, 0, 0), Vector3(0, 0, 0)); // Biases (acc, rot)

  // Measurements
  Vector3 gravity;
  gravity << 0, 0, 9.81;
  Vector3 omegaCoriolis;
  omegaCoriolis << 0, 0, 0;
  Vector3 measuredOmega;
  measuredOmega << 0, 0, M_PI / 10; //M_PI/10.0+0.3;
  Vector3 measuredAcc;
  measuredAcc << 0, 0, -9.81;
  double deltaT = 0.001;

  Matrix I6x6(6, 6);
  I6x6 = Matrix::Identity(6, 6);

  ImuFactor::PreintegratedMeasurements pre_int_data(
      imuBias::ConstantBias(Vector3(0.2, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)), kMeasuredAccCovariance,
      kMeasuredOmegaCovariance, kIntegrationErrorCovariance, true);

  for (int i = 0; i < 1000; ++i)
    pre_int_data.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

  // Create factor
  ImuFactor factor(X(1), V(1), X(2), V(2), B(1), pre_int_data, gravity,
      omegaCoriolis);

  // Predict
  Pose3 x1, x2;
  Vector3 v1 = Vector3(0, 0.0, 0.0);
  Vector3 v2;
  ImuFactor::Predict(x1, v1, x2, v2, bias, factor.preintegratedMeasurements(),
      gravity, omegaCoriolis);
  Pose3 expectedPose(Rot3().ypr(M_PI / 10, 0, 0), Point3(0, 0, 0));
  Vector3 expectedVelocity;
  expectedVelocity << 0, 0, 0;
  EXPECT(assert_equal(expectedPose, x2));
  EXPECT(assert_equal(Vector(expectedVelocity), Vector(v2)));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

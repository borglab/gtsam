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
 * @author  Luca Carlone, Stephen Williams, Richard Roberts, Frank Dellaert
 */

#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ScenarioRunner.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/linear/Sampler.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>

#include <CppUnitLite/TestHarness.h>
#include <boost/bind.hpp>
#include <list>
#include <fstream>

using namespace std;
using namespace gtsam;

// Convenience for named keys
using symbol_shorthand::X;
using symbol_shorthand::V;
using symbol_shorthand::B;

static const double kGravity = 10;
static const Vector3 kGravityAlongNavZDown(0, 0, kGravity);
static const Vector3 kZeroOmegaCoriolis(0, 0, 0);
static const Vector3 kNonZeroOmegaCoriolis(0, 0.1, 0.1);
static const imuBias::ConstantBias kZeroBiasHat, kZeroBias;

/* ************************************************************************* */
namespace {
// Auxiliary functions to test evaluate error in ImuFactor
/* ************************************************************************* */
Rot3 evaluateRotationError(const ImuFactor& factor, const Pose3& pose_i,
    const Vector3& vel_i, const Pose3& pose_j, const Vector3& vel_j,
    const imuBias::ConstantBias& bias) {
  return Rot3::Expmap(
      factor.evaluateError(pose_i, vel_i, pose_j, vel_j, bias).head(3));
}

// Define covariance matrices
/* ************************************************************************* */
static const double kGyroSigma = 0.02;
static const double kAccelerometerSigma = 0.1;

// Create default parameters with Z-down and above noise paramaters
static boost::shared_ptr<PreintegrationParams> defaultParams() {
  auto p = PreintegrationParams::MakeSharedD(kGravity);
  p->gyroscopeCovariance = kGyroSigma * kGyroSigma * I_3x3;
  p->accelerometerCovariance = kAccelerometerSigma * kAccelerometerSigma
      * I_3x3;
  p->integrationCovariance = 0.0001 * I_3x3;
  return p;
}

// Auxiliary functions to test pre-integrated Jacobians
// delPdelBiasAcc_ delPdelBiasOmega_ delVdelBiasAcc_ delVdelBiasOmega_ delRdelBiasOmega_
/* ************************************************************************* */
PreintegratedImuMeasurements evaluatePreintegratedMeasurements(
    const imuBias::ConstantBias& bias, const list<Vector3>& measuredAccs,
    const list<Vector3>& measuredOmegas, const list<double>& deltaTs) {
  PreintegratedImuMeasurements result(defaultParams(), bias);

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

} // namespace

/* ************************************************************************* */
TEST(ImuFactor, Accelerating) {
  const double a = 0.2, v = 50;

  // Set up body pointing towards y axis, and start at 10,20,0 with velocity going in X
  // The body itself has Z axis pointing down
  const Rot3 nRb(Point3(0, 1, 0), Point3(1, 0, 0), Point3(0, 0, -1));
  const Point3 initial_position(10, 20, 0);
  const Vector3 initial_velocity(v, 0, 0);

  const AcceleratingScenario scenario(nRb, initial_position, initial_velocity,
      Vector3(a, 0, 0));

  const double T = 3.0; // seconds
  ScenarioRunner runner(&scenario, defaultParams(), T / 10);

  PreintegratedImuMeasurements pim = runner.integrate(T);
  EXPECT(assert_equal(scenario.pose(T), runner.predict(pim).pose(), 1e-9));

  Matrix9 estimatedCov = runner.estimateCovariance(T);
  EXPECT(assert_equal(estimatedCov, pim.preintMeasCov(), 0.1));
}

/* ************************************************************************* */
TEST(ImuFactor, PreintegratedMeasurements) {
  // Measurements
  Vector3 measuredAcc(0.1, 0.0, 0.0);
  Vector3 measuredOmega(M_PI / 100.0, 0.0, 0.0);
  double deltaT = 0.5;

  // Expected pre-integrated values
  Vector3 expectedDeltaP1;
  expectedDeltaP1 << 0.5 * 0.1 * 0.5 * 0.5, 0, 0;
  Vector3 expectedDeltaV1(0.05, 0.0, 0.0);
  Rot3 expectedDeltaR1 = Rot3::RzRyRx(0.5 * M_PI / 100.0, 0.0, 0.0);
  double expectedDeltaT1(0.5);

  // Actual pre-integrated values
  PreintegratedImuMeasurements actual1(defaultParams());
  actual1.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

  EXPECT(assert_equal(Vector(expectedDeltaP1), Vector(actual1.deltaPij())));
  EXPECT(assert_equal(Vector(expectedDeltaV1), Vector(actual1.deltaVij())));
  EXPECT(assert_equal(expectedDeltaR1, Rot3(actual1.deltaRij())));
  DOUBLES_EQUAL(expectedDeltaT1, actual1.deltaTij(), 1e-9);

  // Check derivatives of computeError
  imuBias::ConstantBias bias(Vector3(0.2, 0, 0), Vector3(0.1, 0, 0.3)); // Biases (acc, rot)
  NavState x1, x2 = actual1.predict(x1, bias);

  {
  Matrix9 aH1, aH2;
  Matrix96 aH3;
  actual1.computeError(x1, x2, bias, aH1, aH2, aH3);
  boost::function<Vector9(const NavState&, const NavState&,
                          const imuBias::ConstantBias&)> f =
      boost::bind(&PreintegrationBase::computeError, actual1, _1, _2, _3,
                  boost::none, boost::none, boost::none);
  // NOTE(frank): tolerance of 1e-3 on H1 because approximate away from 0
  EXPECT(assert_equal(numericalDerivative31(f, x1, x2, bias), aH1, 1e-9));
  EXPECT(assert_equal(numericalDerivative32(f, x1, x2, bias), aH2, 1e-9));
  EXPECT(assert_equal(numericalDerivative33(f, x1, x2, bias), aH3, 1e-9));
  }

  // Integrate again
  Vector3 expectedDeltaP2;
  expectedDeltaP2 << 0.025 + expectedDeltaP1(0) + 0.5 * 0.1 * 0.5 * 0.5, 0, 0;
  Vector3 expectedDeltaV2 = Vector3(0.05, 0.0, 0.0)
      + expectedDeltaR1.matrix() * measuredAcc * 0.5;
  Rot3 expectedDeltaR2 = Rot3::RzRyRx(2.0 * 0.5 * M_PI / 100.0, 0.0, 0.0);
  double expectedDeltaT2(1);

  // Actual pre-integrated values
  PreintegratedImuMeasurements actual2 = actual1;
  actual2.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

  EXPECT(assert_equal(Vector(expectedDeltaP2), Vector(actual2.deltaPij())));
  EXPECT(assert_equal(Vector(expectedDeltaV2), Vector(actual2.deltaVij())));
  EXPECT(assert_equal(expectedDeltaR2, Rot3(actual2.deltaRij())));
  DOUBLES_EQUAL(expectedDeltaT2, actual2.deltaTij(), 1e-9);
}
/* ************************************************************************* */
// Common linearization point and measurements for tests
namespace common {
static const Pose3 x1(Rot3::RzRyRx(M_PI / 12.0, M_PI / 6.0, M_PI / 4.0),
    Point3(5.0, 1.0, 0));
static const Vector3 v1(Vector3(0.5, 0.0, 0.0));
static const NavState state1(x1, v1);

// Measurements
static const double w = M_PI / 100;
static const Vector3 measuredOmega(w, 0, 0);
static const Vector3 measuredAcc = x1.rotation().unrotate(
    -kGravityAlongNavZDown);
static const double deltaT = 1.0;

static const Pose3 x2(Rot3::RzRyRx(M_PI / 12.0 + w, M_PI / 6.0, M_PI / 4.0),
    Point3(5.5, 1.0, 0));
static const Vector3 v2(Vector3(0.5, 0.0, 0.0));
static const NavState state2(x2, v2);
} // namespace common

/* ************************************************************************* */
TEST(ImuFactor, PreintegrationBaseMethods) {
  using namespace common;
  auto p = defaultParams();
  p->omegaCoriolis = Vector3(0.02, 0.03, 0.04);
  p->use2ndOrderCoriolis = true;

  PreintegratedImuMeasurements pim(p, kZeroBiasHat);
  pim.integrateMeasurement(measuredAcc, measuredOmega, deltaT);
  pim.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

  // biasCorrectedDelta
  Matrix96 actualH;
  pim.biasCorrectedDelta(kZeroBias, actualH);
  Matrix expectedH = numericalDerivative11<Vector9, imuBias::ConstantBias>(
      boost::bind(&PreintegrationBase::biasCorrectedDelta, pim, _1,
          boost::none), kZeroBias);
  EXPECT(assert_equal(expectedH, actualH));

  Matrix9 aH1;
  Matrix96 aH2;
  NavState predictedState = pim.predict(state1, kZeroBias, aH1, aH2);
  Matrix eH1 = numericalDerivative11<NavState, NavState>(
      boost::bind(&PreintegrationBase::predict, pim, _1, kZeroBias, boost::none,
          boost::none), state1);
  EXPECT(assert_equal(eH1, aH1));
  Matrix eH2 = numericalDerivative11<NavState, imuBias::ConstantBias>(
      boost::bind(&PreintegrationBase::predict, pim, state1, _1, boost::none,
          boost::none), kZeroBias);
  EXPECT(assert_equal(eH2, aH2));
  return;

}

/* ************************************************************************* */
TEST(ImuFactor, ErrorAndJacobians) {
  using namespace common;
  PreintegratedImuMeasurements pim(defaultParams());

  pim.integrateMeasurement(measuredAcc, measuredOmega, deltaT);
  EXPECT(assert_equal(state2, pim.predict(state1, kZeroBias)));

  // Create factor
  ImuFactor factor(X(1), V(1), X(2), V(2), B(1), pim);

  // Expected error
  Vector expectedError(9);
  expectedError << 0, 0, 0, 0, 0, 0, 0, 0, 0;
  EXPECT(
      assert_equal(expectedError,
          factor.evaluateError(x1, v1, x2, v2, kZeroBias)));

  Values values;
  values.insert(X(1), x1);
  values.insert(V(1), v1);
  values.insert(X(2), x2);
  values.insert(V(2), v2);
  values.insert(B(1), kZeroBias);
  EXPECT(assert_equal(expectedError, factor.unwhitenedError(values)));

  // Make sure linearization is correct
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);

  // Actual Jacobians
  Matrix H1a, H2a, H3a, H4a, H5a;
  (void) factor.evaluateError(x1, v1, x2, v2, kZeroBias, H1a, H2a, H3a, H4a,
      H5a);

  // Make sure rotation part is correct when error is interpreted as axis-angle
  // Jacobians are around zero, so the rotation part is the same as:
  Matrix H1Rot3 = numericalDerivative11<Rot3, Pose3>(
      boost::bind(&evaluateRotationError, factor, _1, v1, x2, v2, kZeroBias),
      x1);
  EXPECT(assert_equal(H1Rot3, H1a.topRows(3)));

  Matrix H3Rot3 = numericalDerivative11<Rot3, Pose3>(
      boost::bind(&evaluateRotationError, factor, x1, v1, _1, v2, kZeroBias),
      x2);
  EXPECT(assert_equal(H3Rot3, H3a.topRows(3)));

  // Evaluate error with wrong values
  Vector3 v2_wrong = v2 + Vector3(0.1, 0.1, 0.1);
  values.update(V(2), v2_wrong);
  expectedError << 0, 0, 0, 0, 0, 0, -0.0724744871, -0.040715657, -0.151952901;
  EXPECT(
      assert_equal(expectedError,
          factor.evaluateError(x1, v1, x2, v2_wrong, kZeroBias), 1e-2));
  EXPECT(assert_equal(expectedError, factor.unwhitenedError(values), 1e-2));

  // Make sure the whitening is done correctly
  Matrix cov = pim.preintMeasCov();
  Matrix R = RtR(cov.inverse());
  Vector whitened = R * expectedError;
  EXPECT(assert_equal(0.5 * whitened.squaredNorm(), factor.error(values), 1e-5));

  // Make sure linearization is correct
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

/* ************************************************************************* */
TEST(ImuFactor, ErrorAndJacobianWithBiases) {
  using common::x1;
  using common::v1;
  using common::v2;
  imuBias::ConstantBias bias(Vector3(0.2, 0, 0), Vector3(0.1, 0, 0.3)); // Biases (acc, rot)
  Pose3 x2(Rot3::Expmap(Vector3(0, 0, M_PI / 10.0 + M_PI / 10.0)),
      Point3(5.5, 1.0, -50.0));

  // Measurements
  Vector3 measuredOmega;
  measuredOmega << 0, 0, M_PI / 10.0 + 0.3;
  Vector3 measuredAcc = x1.rotation().unrotate(-kGravityAlongNavZDown)
      + Vector3(0.2, 0.0, 0.0);
  double deltaT = 1.0;

  auto p = defaultParams();
  p->omegaCoriolis = kNonZeroOmegaCoriolis;

  imuBias::ConstantBias biasHat(Vector3(0.2, 0.0, 0.0), Vector3(0.0, 0.0, 0.1));
  PreintegratedImuMeasurements pim(p, biasHat);
  pim.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

  // Make sure of biasCorrectedDelta
  Matrix96 actualH;
  pim.biasCorrectedDelta(bias, actualH);
  Matrix expectedH = numericalDerivative11<Vector9, imuBias::ConstantBias>(
      boost::bind(&PreintegrationBase::biasCorrectedDelta, pim, _1,
          boost::none), bias);
  EXPECT(assert_equal(expectedH, actualH));

  // Create factor
  ImuFactor factor(X(1), V(1), X(2), V(2), B(1), pim);

  Values values;
  values.insert(X(1), x1);
  values.insert(V(1), v1);
  values.insert(X(2), x2);
  values.insert(V(2), v2);
  values.insert(B(1), bias);

  // Make sure linearization is correct
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

/* ************************************************************************* */
TEST(ImuFactor, ErrorAndJacobianWith2ndOrderCoriolis) {
  using common::x1;
  using common::v1;
  using common::v2;
  imuBias::ConstantBias bias(Vector3(0.2, 0, 0), Vector3(0.1, 0, 0.3)); // Biases (acc, rot)
  Pose3 x2(Rot3::Expmap(Vector3(0, 0, M_PI / 10.0 + M_PI / 10.0)),
      Point3(5.5, 1.0, -50.0));

  // Measurements
  Vector3 measuredOmega;
  measuredOmega << 0, 0, M_PI / 10.0 + 0.3;
  Vector3 measuredAcc = x1.rotation().unrotate(-kGravityAlongNavZDown)
      + Vector3(0.2, 0.0, 0.0);
  double deltaT = 1.0;

  auto p = defaultParams();
  p->omegaCoriolis = kNonZeroOmegaCoriolis;
  p->use2ndOrderCoriolis = true;

  PreintegratedImuMeasurements pim(p,
      imuBias::ConstantBias(Vector3(0.2, 0.0, 0.0), Vector3(0.0, 0.0, 0.1)));
  pim.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

  // Create factor
  ImuFactor factor(X(1), V(1), X(2), V(2), B(1), pim);

  Values values;
  values.insert(X(1), x1);
  values.insert(V(1), v1);
  values.insert(X(2), x2);
  values.insert(V(2), v2);
  values.insert(B(1), bias);

  // Make sure linearization is correct
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

/* ************************************************************************* */
TEST(ImuFactor, PartialDerivative_wrt_Bias) {
  // Linearization point
  Vector3 biasOmega(0, 0, 0); // Current estimate of rotation rate bias

  // Measurements
  Vector3 measuredOmega(0.1, 0, 0);
  double deltaT = 0.5;

  // Compute numerical derivatives
  Matrix expectedDelRdelBiasOmega = numericalDerivative11<Rot3, Vector3>(
      boost::bind(&evaluateRotation, measuredOmega, _1, deltaT),
      Vector3(biasOmega));

  const Matrix3 Jr = Rot3::ExpmapDerivative(
      (measuredOmega - biasOmega) * deltaT);

  Matrix3 actualdelRdelBiasOmega = -Jr * deltaT; // the delta bias appears with the minus sign

  // Compare Jacobians
  EXPECT(assert_equal(expectedDelRdelBiasOmega, actualdelRdelBiasOmega, 1e-9));
}

/* ************************************************************************* */
TEST(ImuFactor, PartialDerivativeLogmap) {
  // Linearization point
  Vector3 thetahat(0.1, 0.1, 0); // Current estimate of rotation rate bias

  // Measurements
  Vector3 deltatheta(0, 0, 0);

  // Compute numerical derivatives
  Matrix expectedDelFdeltheta = numericalDerivative11<Vector, Vector3>(
      boost::bind(&evaluateLogRotation, thetahat, _1), Vector3(deltatheta));

  Matrix3 actualDelFdeltheta = Rot3::LogmapDerivative(thetahat);

  // Compare Jacobians
  EXPECT(assert_equal(expectedDelFdeltheta, actualDelFdeltheta));
}

/* ************************************************************************* */
TEST(ImuFactor, fistOrderExponential) {
  // Linearization point
  Vector3 biasOmega(0, 0, 0); // Current estimate of rotation rate bias

  // Measurements
  Vector3 measuredOmega(0.1, 0, 0);
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
  // hatRot * (I_3x3 + skewSymmetric(delRdelBiasOmega * deltabiasOmega));

  // This is a first order expansion so the equality is only an approximation
  EXPECT(assert_equal(expectedRot, actualRot));
}

/* ************************************************************************* */
TEST(ImuFactor, FirstOrderPreIntegratedMeasurements) {
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

  // Actual pre-integrated values
  PreintegratedImuMeasurements preintegrated =
      evaluatePreintegratedMeasurements(kZeroBias, measuredAccs, measuredOmegas,
          deltaTs);

  // Compute numerical derivatives
  Matrix expectedDelPdelBias = numericalDerivative11<Vector,
      imuBias::ConstantBias>(
      boost::bind(&evaluatePreintegratedMeasurementsPosition, _1, measuredAccs,
          measuredOmegas, deltaTs), kZeroBias);
  Matrix expectedDelPdelBiasAcc = expectedDelPdelBias.leftCols(3);
  Matrix expectedDelPdelBiasOmega = expectedDelPdelBias.rightCols(3);

  Matrix expectedDelVdelBias = numericalDerivative11<Vector,
      imuBias::ConstantBias>(
      boost::bind(&evaluatePreintegratedMeasurementsVelocity, _1, measuredAccs,
          measuredOmegas, deltaTs), kZeroBias);
  Matrix expectedDelVdelBiasAcc = expectedDelVdelBias.leftCols(3);
  Matrix expectedDelVdelBiasOmega = expectedDelVdelBias.rightCols(3);

  Matrix expectedDelRdelBias =
      numericalDerivative11<Rot3, imuBias::ConstantBias>(
          boost::bind(&evaluatePreintegratedMeasurementsRotation, _1,
              measuredAccs, measuredOmegas, deltaTs), kZeroBias);
  Matrix expectedDelRdelBiasAcc = expectedDelRdelBias.leftCols(3);
  Matrix expectedDelRdelBiasOmega = expectedDelRdelBias.rightCols(3);

  // Compare Jacobians
  EXPECT(assert_equal(expectedDelPdelBiasAcc, preintegrated.delPdelBiasAcc()));
  EXPECT(
      assert_equal(expectedDelPdelBiasOmega, preintegrated.delPdelBiasOmega(),1e-8));
  EXPECT(assert_equal(expectedDelVdelBiasAcc, preintegrated.delVdelBiasAcc()));
  EXPECT(
      assert_equal(expectedDelVdelBiasOmega, preintegrated.delVdelBiasOmega(),1e-8));
  EXPECT(assert_equal(expectedDelRdelBiasAcc, Matrix::Zero(3, 3)));
  EXPECT(
      assert_equal(expectedDelRdelBiasOmega, preintegrated.delRdelBiasOmega(),1e-7));
}

/* ************************************************************************* */
Vector3 correctedAcc(const PreintegratedImuMeasurements& pim,
    const Vector3& measuredAcc, const Vector3& measuredOmega) {
  return pim.correctMeasurementsByBiasAndSensorPose(measuredAcc, measuredOmega).first;
}

TEST(ImuFactor, ErrorWithBiasesAndSensorBodyDisplacement) {
  const Rot3 nRb = Rot3::Expmap(Vector3(0, 0, M_PI / 4.0));
  const Point3 p1(5.0, 1.0, -50.0);
  const Vector3 v1(0.5, 0.0, 0.0);

  const Vector3 a = nRb * Vector3(0.2, 0.0, 0.0);
  const AcceleratingScenario scenario(nRb, p1, v1, a,
      Vector3(0, 0, M_PI / 10.0 + 0.3));

  auto p = defaultParams();
  p->body_P_sensor = Pose3(Rot3::Expmap(Vector3(0, M_PI / 2, 0)),
      Point3(0.1, 0.05, 0.01));
  p->omegaCoriolis = kNonZeroOmegaCoriolis;

  imuBias::ConstantBias biasHat(Vector3(0.2, 0.0, 0.0), Vector3(0.0, 0.0, 0.0));

  const double T = 3.0; // seconds
  ScenarioRunner runner(&scenario, p, T / 10);

  //  PreintegratedImuMeasurements pim = runner.integrate(T);
  //  EXPECT(assert_equal(scenario.pose(T), runner.predict(pim).pose, 1e-9));
  //
  //  Matrix6 estimatedCov = runner.estimatePoseCovariance(T);
  //  EXPECT(assert_equal(estimatedCov, runner.poseCovariance(pim), 0.1));
  //
  ///////////////////////////////////////////////////////////////////////////////////////////
  Pose3 x1(nRb, p1);

  // Measurements
  Vector3 measuredOmega = runner.actualAngularVelocity(0);
  Vector3 measuredAcc = runner.actualSpecificForce(0);

  // Get mean prediction from "ground truth" measurements
  const Vector3 accNoiseVar2(0.01, 0.02, 0.03);
  const Vector3 omegaNoiseVar2(0.03, 0.01, 0.02);
  PreintegratedImuMeasurements pim(p, biasHat);

  // Check updatedDeltaXij derivatives
  Matrix3 D_correctedAcc_measuredOmega = Z_3x3;
  pim.correctMeasurementsByBiasAndSensorPose(measuredAcc, measuredOmega,
      boost::none, D_correctedAcc_measuredOmega, boost::none);
  Matrix3 expectedD = numericalDerivative11<Vector3, Vector3>(
      boost::bind(correctedAcc, pim, measuredAcc, _1), measuredOmega, 1e-6);
  EXPECT(assert_equal(expectedD, D_correctedAcc_measuredOmega, 1e-5));

  double dt = 0.1;

// TODO(frank): revive derivative tests
//  Matrix93 G1, G2;
//  PreintegrationBase::TangentVector preint =
//      pim.updatedDeltaXij(measuredAcc, measuredOmega, dt, boost::none, G1, G2);
//
//  Matrix93 expectedG1 = numericalDerivative21<NavState, Vector3, Vector3>(
//      boost::bind(&PreintegratedImuMeasurements::updatedDeltaXij, pim, _1, _2,
//          dt, boost::none, boost::none, boost::none), measuredAcc,
//      measuredOmega, 1e-6);
//  EXPECT(assert_equal(expectedG1, G1, 1e-5));
//
//  Matrix93 expectedG2 = numericalDerivative22<NavState, Vector3, Vector3>(
//      boost::bind(&PreintegratedImuMeasurements::updatedDeltaXij, pim, _1, _2,
//          dt, boost::none, boost::none, boost::none), measuredAcc,
//      measuredOmega, 1e-6);
//  EXPECT(assert_equal(expectedG2, G2, 1e-5));

  imuBias::ConstantBias bias(Vector3(0.2, 0, 0), Vector3(0, 0, 0.3)); // Biases (acc, rot)

  // integrate at least twice to get position information
  // otherwise factor cov noise from preint_cov is not positive definite
  pim.integrateMeasurement(measuredAcc, measuredOmega, dt);
  pim.integrateMeasurement(measuredAcc, measuredOmega, dt);

  // Create factor
  ImuFactor factor(X(1), V(1), X(2), V(2), B(1), pim);

  Pose3 x2(Rot3::Expmap(Vector3(0, 0, M_PI / 4.0 + M_PI / 10.0)),
      Point3(5.5, 1.0, -50.0));
  Vector3 v2(Vector3(0.5, 0.0, 0.0));

  Values values;
  values.insert(X(1), x1);
  values.insert(V(1), v1);
  values.insert(X(2), x2);
  values.insert(V(2), v2);
  values.insert(B(1), bias);

  // Make sure linearization is correct
  double diffDelta = 1e-8;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

/* ************************************************************************* */
TEST(ImuFactor, PredictPositionAndVelocity) {
  imuBias::ConstantBias bias(Vector3(0, 0, 0), Vector3(0, 0, 0)); // Biases (acc, rot)

  // Measurements
  Vector3 measuredOmega;
  measuredOmega << 0, 0, 0; // M_PI/10.0+0.3;
  Vector3 measuredAcc;
  measuredAcc << 0, 1, -kGravity;
  double deltaT = 0.001;

  PreintegratedImuMeasurements pim(defaultParams(),
      imuBias::ConstantBias(Vector3(0.2, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)));

  for (int i = 0; i < 1000; ++i)
    pim.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

  // Create factor
  ImuFactor factor(X(1), V(1), X(2), V(2), B(1), pim);

  // Predict
  Pose3 x1;
  Vector3 v1(0, 0.0, 0.0);
  NavState actual = pim.predict(NavState(x1, v1), bias);
  NavState expected(Rot3(), Point3(0, 0.5, 0), Vector3(0, 1, 0));
  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST(ImuFactor, PredictRotation) {
  imuBias::ConstantBias bias(Vector3(0, 0, 0), Vector3(0, 0, 0)); // Biases (acc, rot)

  // Measurements
  Vector3 measuredOmega;
  measuredOmega << 0, 0, M_PI / 10; // M_PI/10.0+0.3;
  Vector3 measuredAcc;
  measuredAcc << 0, 0, -kGravity;
  double deltaT = 0.001;

  PreintegratedImuMeasurements pim(defaultParams(),
      imuBias::ConstantBias(Vector3(0.2, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)));

  for (int i = 0; i < 1000; ++i)
    pim.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

  // Create factor
  ImuFactor factor(X(1), V(1), X(2), V(2), B(1), pim);

  // Predict
  NavState actual = pim.predict(NavState(), bias);
  NavState expected(Rot3().ypr(M_PI / 10, 0, 0), Point3(), Vector3::Zero());
  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST(ImuFactor, PredictArbitrary) {
  Pose3 x1;
  const Vector3 v1(0, 0, 0);

  const AcceleratingScenario scenario(x1.rotation(), x1.translation(), v1,
      Vector3(0.1, 0.2, 0), Vector3(M_PI / 10, M_PI / 10, M_PI / 10));

  const double T = 3.0; // seconds
  ScenarioRunner runner(&scenario, defaultParams(), T / 10);
  //
  //  PreintegratedImuMeasurements pim = runner.integrate(T);
  //  EXPECT(assert_equal(scenario.pose(T), runner.predict(pim).pose, 1e-9));
  //
  //  Matrix6 estimatedCov = runner.estimatePoseCovariance(T);
  //  EXPECT(assert_equal(estimatedCov, runner.poseCovariance(pim), 0.1));
  //////////////////////////////////////////////////////////////////////////////////

  imuBias::ConstantBias biasHat(Vector3(0.2, 0.0, 0.0), Vector3(0.0, 0.0, 0.0));

  // Measurements
  Vector3 measuredOmega = runner.actualAngularVelocity(0);
  Vector3 measuredAcc = runner.actualSpecificForce(0);

  auto p = defaultParams();
  p->integrationCovariance = Z_3x3; // MonteCarlo does not sample integration noise
  PreintegratedImuMeasurements pim(p, biasHat);
  imuBias::ConstantBias bias(Vector3(0, 0, 0), Vector3(0, 0, 0));
//  EXPECT(MonteCarlo(pim, NavState(x1, v1), bias, 0.1, boost::none, measuredAcc, measuredOmega,
//                    Vector3::Constant(accNoiseVar), Vector3::Constant(omegaNoiseVar), 100000));

  double dt = 0.001;
  for (int i = 0; i < 1000; ++i)
    pim.integrateMeasurement(measuredAcc, measuredOmega, dt);

  // Create factor
  ImuFactor factor(X(1), V(1), X(2), V(2), B(1), pim);

  // Predict
  NavState actual = pim.predict(NavState(x1, v1), bias);

  // Regression test for Imu Refactor
  Rot3 expectedR( //
      +0.903715275, -0.250741668, 0.347026393, //
      +0.347026393, 0.903715275, -0.250741668, //
      -0.250741668, 0.347026393, 0.903715275);
  Point3 expectedT(-0.516077031, 0.57842919, 0.0876478403);
  Vector3 expectedV(-1.62337767, 1.57954409, 0.343833571);
  NavState expected(expectedR, expectedT, expectedV);
  EXPECT(assert_equal(expected, actual, 1e-7));
}

/* ************************************************************************* */
TEST(ImuFactor, bodyPSensorNoBias) {
  imuBias::ConstantBias bias(Vector3(0, 0, 0), Vector3(0, 0.1, 0)); // Biases (acc, rot)

  // Rotate sensor (z-down) to body (same as navigation) i.e. z-up
  auto p = defaultParams();
  p->n_gravity = Vector3(0, 0, -kGravity); // z-up nav frame
  p->body_P_sensor = Pose3(Rot3::ypr(0, 0, M_PI), Point3(0, 0, 0));

  // Measurements
  // Gyroscope measurement is the angular velocity of sensor w.r.t nav frame in sensor frame
  Vector3 s_omegaMeas_ns(0, 0.1, M_PI / 10);
  // Acc measurement is acceleration of sensor in the sensor frame, when stationary,
  // table exerts an equal and opposite force w.r.t gravity
  Vector3 s_accMeas(0, 0, -kGravity);
  double dt = 0.001;

  PreintegratedImuMeasurements pim(p, bias);

  for (int i = 0; i < 1000; ++i)
    pim.integrateMeasurement(s_accMeas, s_omegaMeas_ns, dt);

  // Create factor
  ImuFactor factor(X(1), V(1), X(2), V(2), B(1), pim);

  // Predict
  NavState actual = pim.predict(NavState(), bias);

  Pose3 expectedPose(Rot3().ypr(-M_PI / 10, 0, 0), Point3(0, 0, 0));
  EXPECT(assert_equal(expectedPose, actual.pose()));

  Vector3 expectedVelocity(0, 0, 0);
  EXPECT(assert_equal(Vector(expectedVelocity), Vector(actual.velocity())));
}

/* ************************************************************************* */
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>

TEST(ImuFactor, bodyPSensorWithBias) {
  using noiseModel::Diagonal;
  typedef imuBias::ConstantBias Bias;

  int numFactors = 80;
  Vector6 noiseBetweenBiasSigma;
  noiseBetweenBiasSigma << Vector3(2.0e-5, 2.0e-5, 2.0e-5), Vector3(3.0e-6,
      3.0e-6, 3.0e-6);
  SharedDiagonal biasNoiseModel = Diagonal::Sigmas(noiseBetweenBiasSigma);

  // Measurements
  // Sensor frame is z-down
  // Gyroscope measurement is the angular velocity of sensor w.r.t nav frame in sensor frame
  Vector3 measuredOmega(0, 0.01, 0);
  // Acc measurement is acceleration of sensor in the sensor frame, when stationary,
  // table exerts an equal and opposite force w.r.t gravity
  Vector3 measuredAcc(0, 0, -kGravity);

  auto p = defaultParams();
  p->n_gravity = Vector3(0, 0, -kGravity);
  p->body_P_sensor = Pose3(Rot3::ypr(0, 0, M_PI), Point3());
  p->accelerometerCovariance = 1e-7 * I_3x3;
  p->gyroscopeCovariance = 1e-8 * I_3x3;
  p->integrationCovariance = 1e-9 * I_3x3;
  double deltaT = 0.005;

  //   Specify noise values on priors
  Vector6 priorNoisePoseSigmas(
      (Vector(6) << 0.001, 0.001, 0.001, 0.01, 0.01, 0.01).finished());
  Vector3 priorNoiseVelSigmas((Vector(3) << 0.1, 0.1, 0.1).finished());
  Vector6 priorNoiseBiasSigmas(
      (Vector(6) << 0.1, 0.1, 0.1, 0.5e-1, 0.5e-1, 0.5e-1).finished());
  SharedDiagonal priorNoisePose = Diagonal::Sigmas(priorNoisePoseSigmas);
  SharedDiagonal priorNoiseVel = Diagonal::Sigmas(priorNoiseVelSigmas);
  SharedDiagonal priorNoiseBias = Diagonal::Sigmas(priorNoiseBiasSigmas);
  Vector3 zeroVel(0, 0, 0);

  // Create a factor graph with priors on initial pose, vlocity and bias
  NonlinearFactorGraph graph;
  Values values;

  PriorFactor<Pose3> priorPose(X(0), Pose3(), priorNoisePose);
  graph.add(priorPose);
  values.insert(X(0), Pose3());

  PriorFactor<Vector3> priorVel(V(0), zeroVel, priorNoiseVel);
  graph.add(priorVel);
  values.insert(V(0), zeroVel);

  // The key to this test is that we specify the bias, in the sensor frame, as known a priori
  // We also create factors below that encode our assumption that this bias is constant over time
  // In theory, after optimization, we should recover that same bias estimate
  Bias priorBias(Vector3(0, 0, 0), Vector3(0, 0.01, 0)); // Biases (acc, rot)
  PriorFactor<Bias> priorBiasFactor(B(0), priorBias, priorNoiseBias);
  graph.add(priorBiasFactor);
  values.insert(B(0), priorBias);

  // Now add IMU factors and bias noise models
  Bias zeroBias(Vector3(0, 0, 0), Vector3(0, 0, 0));
  for (int i = 1; i < numFactors; i++) {
    PreintegratedImuMeasurements pim = PreintegratedImuMeasurements(p,
        priorBias);
    for (int j = 0; j < 200; ++j)
      pim.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

    // Create factors
    graph.add(ImuFactor(X(i - 1), V(i - 1), X(i), V(i), B(i - 1), pim));
    graph.add(BetweenFactor<Bias>(B(i - 1), B(i), zeroBias, biasNoiseModel));

    values.insert(X(i), Pose3());
    values.insert(V(i), zeroVel);
    values.insert(B(i), priorBias);
  }

  // Finally, optimize, and get bias at last time step
  Values results = LevenbergMarquardtOptimizer(graph, values).optimize();
  Bias biasActual = results.at<Bias>(B(numFactors - 1));

  // And compare it with expected value (our prior)
  Bias biasExpected(Vector3(0, 0, 0), Vector3(0, 0.01, 0));
  EXPECT(assert_equal(biasExpected, biasActual, 1e-3));
}

/* ************************************************************************** */
#include <gtsam/base/serializationTestHelpers.h>

BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Constrained,
    "gtsam_noiseModel_Constrained");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Diagonal,
    "gtsam_noiseModel_Diagonal");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Gaussian,
    "gtsam_noiseModel_Gaussian");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Unit, "gtsam_noiseModel_Unit");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Isotropic,
    "gtsam_noiseModel_Isotropic");
BOOST_CLASS_EXPORT_GUID(gtsam::SharedNoiseModel, "gtsam_SharedNoiseModel");
BOOST_CLASS_EXPORT_GUID(gtsam::SharedDiagonal, "gtsam_SharedDiagonal");

TEST(ImuFactor, serialization) {
  using namespace gtsam::serializationTestHelpers;

  auto p = defaultParams();
  p->n_gravity = Vector3(0, 0, -9.81);
  p->body_P_sensor = Pose3(Rot3::ypr(0, 0, M_PI), Point3());
  p->accelerometerCovariance = 1e-7 * I_3x3;
  p->gyroscopeCovariance = 1e-8 * I_3x3;
  p->integrationCovariance = 1e-9 * I_3x3;
  double deltaT = 0.005;
  imuBias::ConstantBias priorBias(Vector3(0, 0, 0), Vector3(0, 0.01, 0)); // Biases (acc, rot)

  PreintegratedImuMeasurements pim(p, priorBias);

  // measurements are needed for non-inf noise model, otherwise will throw err when deserialize

  // Sensor frame is z-down
  // Gyroscope measurement is the angular velocity of sensor w.r.t nav frame in sensor frame
  Vector3 measuredOmega(0, 0.01, 0);
  // Acc measurement is acceleration of sensor in the sensor frame, when stationary,
  // table exerts an equal and opposite force w.r.t gravity
  Vector3 measuredAcc(0, 0, -9.81);

  for (int j = 0; j < 200; ++j)
    pim.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

  ImuFactor factor(X(1), V(1), X(2), V(2), B(1), pim);

  EXPECT(equalsObj(factor));
  EXPECT(equalsXML(factor));
  EXPECT(equalsBinary(factor));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

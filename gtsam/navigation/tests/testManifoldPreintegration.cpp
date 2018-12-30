/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testManifoldPreintegration.cpp
 * @brief   Unit test for the ManifoldPreintegration
 * @author  Luca Carlone
 * @author  Frank Dellaert
 */

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/navigation/ManifoldPreintegration.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/expressionTesting.h>
#include <gtsam/nonlinear/expressions.h>

#include <CppUnitLite/TestHarness.h>
#include <boost/bind.hpp>

#include "imuFactorTesting.h"

static const double kDt = 0.1;
static const imuBias::ConstantBias kFixedBias(Vector3(0.1, 0.2, 0.1),
                                              Vector3(0.1, 0.1, 0.4));
static const Vector3 kAcc(0.1, 0.2, 10), kOmega(0.1, 0.2, 0.3);

namespace testing {
// Create default parameters with Z-down and above noise parameters
static boost::shared_ptr<PreintegrationParams> Params() {
  auto p = PreintegrationParams::MakeSharedD(kGravity);
  p->gyroscopeCovariance = kGyroSigma * kGyroSigma * I_3x3;
  p->accelerometerCovariance = kAccelSigma * kAccelSigma * I_3x3;
  p->integrationCovariance = 0.0001 * I_3x3;
  return p;
}
}  // namespace testing

/* ************************************************************************* */
TEST(ManifoldPreintegration, BiasCorrectionJacobians) {
  testing::SomeMeasurements measurements;

  boost::function<Rot3(const Vector3&, const Vector3&)> deltaRij =
      [=](const Vector3& a, const Vector3& w) {
        ManifoldPreintegration pim(testing::Params(), Bias(a, w));
        testing::integrateMeasurements(measurements, &pim);
        return pim.deltaRij();
      };

  boost::function<Point3(const Vector3&, const Vector3&)> deltaPij =
      [=](const Vector3& a, const Vector3& w) {
        ManifoldPreintegration pim(testing::Params(), Bias(a, w));
        testing::integrateMeasurements(measurements, &pim);
        return pim.deltaPij();
      };

  boost::function<Vector3(const Vector3&, const Vector3&)> deltaVij =
      [=](const Vector3& a, const Vector3& w) {
        ManifoldPreintegration pim(testing::Params(), Bias(a, w));
        testing::integrateMeasurements(measurements, &pim);
        return pim.deltaVij();
      };

  boost::function<NavState(const Vector3&, const Vector3&)> deltaXij =
      [=](const Vector3& a, const Vector3& w) {
        ManifoldPreintegration pim(testing::Params(), Bias(a, w));
        testing::integrateMeasurements(measurements, &pim);
        return pim.deltaXij();
      };

  // Actual pre-integrated values
  ManifoldPreintegration pim(testing::Params());
  testing::integrateMeasurements(measurements, &pim);

  Matrix93 oldWay;
  oldWay << numericalDerivative21(deltaRij, kZero, kZero),
      numericalDerivative21(deltaPij, kZero, kZero),
      numericalDerivative21(deltaVij, kZero, kZero);
  Matrix93 expected = numericalDerivative21(deltaXij, kZero, kZero);
  EXPECT(assert_equal(oldWay, expected));
  EXPECT(assert_equal(expected, pim.preintegrated_H_biasAcc()));

  oldWay << numericalDerivative22(deltaRij, kZero, kZero),
      numericalDerivative22(deltaPij, kZero, kZero),
      numericalDerivative22(deltaVij, kZero, kZero);
  expected = numericalDerivative22(deltaXij, kZero, kZero);
  EXPECT(assert_equal(oldWay, expected));
  EXPECT(assert_equal(expected, pim.preintegrated_H_biasOmega()));
}

/* ************************************************************************* */
NavState fupdate(const NavState& inputState, const Vector3& a,
                 const Vector3& w) {
  Matrix9 aH1;
  Matrix93 aH2, aH3;
  // Correct for bias in the sensor frame
  Vector3 a_cor = kFixedBias.correctAccelerometer(a);
  Vector3 w_cor = kFixedBias.correctGyroscope(w);
  NavState outputState = inputState.update(a_cor, w_cor, kDt, aH1, aH2, aH3);
  return outputState;
}

TEST(ManifoldPreintegration, UpdateEstimate) {
  ManifoldPreintegration pim(testing::Params(), kFixedBias);
  Matrix9 aH1;
  Matrix93 aH2, aH3;
  pim.update(kAcc, kOmega, kDt, &aH1, &aH2, &aH3);
  NavState state;

  EXPECT(
      assert_equal(numericalDerivative31(fupdate, state, kAcc, kOmega), aH1));
  EXPECT(
      assert_equal(numericalDerivative32(fupdate, state, kAcc, kOmega), aH2));
  EXPECT(
      assert_equal(numericalDerivative33(fupdate, state, kAcc, kOmega), aH3));
}

/* ************************************************************************* */
Vector9 fbias(const imuBias::ConstantBias& bias) {
  ManifoldPreintegration pim(testing::Params());
  pim.integrateMeasurement(kAcc, kOmega, kDt);
  return pim.biasCorrectedDelta(bias);
}

TEST(ManifoldPreintegration, biasCorrectedDelta1) {
  // regression values
  Vector9 expected;
  expected << -2.49966e-07, 0.00999825, -0.0099995, 0, 0, 0.0495, 0, 0, 0.99;
  ManifoldPreintegration pim(testing::Params());
  pim.integrateMeasurement(kAcc, kOmega, kDt);
  Matrix96 aH;
  Vector9 actual = pim.biasCorrectedDelta(kFixedBias, aH);
  EXPECT(assert_equal(expected, actual));
  EXPECT(assert_equal(numericalDerivative11(fbias, kFixedBias), aH));
}

/* ************************************************************************* */
TEST(ManifoldPreintegration, computeError) {
  // regression values
  Vector9 expected;
  expected << 0.01, 0.02, 0.03, 0.0005, 0.001, 0.1, 0.01, 0.02, 2;
  ManifoldPreintegration pim(testing::Params());
  pim.integrateMeasurement(kAcc, kOmega, kDt);
  NavState x1, x2;
  imuBias::ConstantBias bias;
  Matrix9 aH1, aH2;
  Matrix96 aH3;
  Vector9 actual = pim.computeError(x1, x2, bias, aH1, aH2, aH3);
  boost::function<Vector9(const NavState&, const NavState&,
                          const imuBias::ConstantBias&)>
      f = boost::bind(&ManifoldPreintegration::computeError, pim, _1, _2, _3,
                      boost::none, boost::none, boost::none);
  EXPECT(assert_equal(expected, actual));
  EXPECT(assert_equal(numericalDerivative31(f, x1, x2, bias), aH1));
  EXPECT(assert_equal(numericalDerivative32(f, x1, x2, bias), aH2));
  EXPECT(assert_equal(numericalDerivative33(f, x1, x2, bias), aH3));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

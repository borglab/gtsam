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
 */

#include <gtsam/navigation/ManifoldPreintegration.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/expressionTesting.h>

#include <CppUnitLite/TestHarness.h>
#include <boost/bind.hpp>

#include "imuFactorTesting.h"

static const double kDt = 0.1;

const imuBias::ConstantBias fixedBias(Vector3(0.1,0.2,0.1),Vector3(0.1,0.1,0.4));

namespace testing {
// Create default parameters with Z-down and above noise parameters
static boost::shared_ptr<PreintegrationParams> Params() {
  auto p = PreintegrationParams::MakeSharedD(kGravity);
  p->gyroscopeCovariance = kGyroSigma * kGyroSigma * I_3x3;
  p->accelerometerCovariance = kAccelSigma * kAccelSigma * I_3x3;
  p->integrationCovariance = 0.0001 * I_3x3;
  return p;
}
}

NavState fupdate(const NavState& inputState, const Vector3& a, const Vector3& w) {
  Matrix9 aH1;
  Matrix93 aH2, aH3;
  // Correct for bias in the sensor frame
  Vector3 a_cor = fixedBias.correctAccelerometer(a);
  Vector3 w_cor = fixedBias.correctGyroscope(w);
  NavState outputState = inputState.update(a_cor, w_cor, kDt, aH1, aH2, aH3);
  return outputState;
}

Vector9 fbias(const imuBias::ConstantBias& bias) {
  ManifoldPreintegration pim0(testing::Params());
  return pim0.biasCorrectedDelta(bias);
}

/* ************************************************************************* */
TEST(ManifoldPreintegration, UpdateEstimate) {
  ManifoldPreintegration pim(testing::Params(),fixedBias);
  const Vector3 acc(0.1, 0.2, 10), omega(0.1, 0.2, 0.3);
  Matrix9 aH1;
  Matrix93 aH2, aH3;
  pim.update(acc, omega, kDt, &aH1, &aH2, &aH3);
  NavState state;

  EXPECT(assert_equal(numericalDerivative31(fupdate, state, acc, omega), aH1, 1e-9));
  EXPECT(assert_equal(numericalDerivative32(fupdate, state, acc, omega), aH2, 1e-9));
  EXPECT(assert_equal(numericalDerivative33(fupdate, state, acc, omega), aH3, 1e-9));
}

/* ************************************************************************* */
TEST(ManifoldPreintegration, biasCorrectedDelta1) {
  ManifoldPreintegration pim(testing::Params());
  Matrix96 aH;
  const imuBias::ConstantBias bias;
  pim.biasCorrectedDelta(bias, aH);
  EXPECT(assert_equal(numericalDerivative11(fbias, bias), aH, 1e-9));
}

/* ************************************************************************* */
TEST(ManifoldPreintegration, computeError) {
  ManifoldPreintegration pim(testing::Params());
  NavState x1, x2;
  imuBias::ConstantBias bias;
  Matrix9 aH1, aH2;
  Matrix96 aH3;
  pim.computeError(x1, x2, bias, aH1, aH2, aH3);
  boost::function<Vector9(const NavState&, const NavState&,
                          const imuBias::ConstantBias&)> f =
      boost::bind(&ManifoldPreintegration::computeError, pim, _1, _2, _3,
                  boost::none, boost::none, boost::none);
  // NOTE(frank): tolerance of 1e-3 on H1 because approximate away from 0
  EXPECT(assert_equal(numericalDerivative31(f, x1, x2, bias), aH1, 1e-9));
  EXPECT(assert_equal(numericalDerivative32(f, x1, x2, bias), aH2, 1e-9));
  EXPECT(assert_equal(numericalDerivative33(f, x1, x2, bias), aH3, 1e-9));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

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

#include "imuFactorTesting.h"

using namespace std::placeholders;

namespace testing {
// Create default parameters with Z-down and above noise parameters
static std::shared_ptr<PreintegrationParams> Params() {
  auto p = PreintegrationParams::MakeSharedD(kGravity);
  p->gyroscopeCovariance = kGyroSigma * kGyroSigma * I_3x3;
  p->accelerometerCovariance = kAccelSigma * kAccelSigma * I_3x3;
  p->integrationCovariance = 0.0001 * I_3x3;
  return p;
}
}

/* ************************************************************************* */
TEST(ManifoldPreintegration, BiasCorrectionJacobians) {
  testing::SomeMeasurements measurements;

  std::function<Rot3(const Vector3&, const Vector3&)> deltaRij =
      [=](const Vector3& a, const Vector3& w) {
        ManifoldPreintegration pim(testing::Params(), Bias(a, w));
        testing::integrateMeasurements(measurements, &pim);
        return pim.deltaRij();
      };

  std::function<Point3(const Vector3&, const Vector3&)> deltaPij =
      [=](const Vector3& a, const Vector3& w) {
        ManifoldPreintegration pim(testing::Params(), Bias(a, w));
        testing::integrateMeasurements(measurements, &pim);
        return pim.deltaPij();
      };

  std::function<Vector3(const Vector3&, const Vector3&)> deltaVij =
      [=](const Vector3& a, const Vector3& w) {
        ManifoldPreintegration pim(testing::Params(), Bias(a, w));
        testing::integrateMeasurements(measurements, &pim);
        return pim.deltaVij();
      };

  // Actual pre-integrated values
  ManifoldPreintegration pim(testing::Params());
  testing::integrateMeasurements(measurements, &pim);

  EXPECT(
      assert_equal(numericalDerivative21(deltaRij, kZero, kZero),
          Matrix3(Z_3x3)));
  EXPECT(
      assert_equal(numericalDerivative22(deltaRij, kZero, kZero),
          pim.delRdelBiasOmega(), 1e-3));

  EXPECT(
      assert_equal(numericalDerivative21(deltaPij, kZero, kZero),
          pim.delPdelBiasAcc()));
  EXPECT(
      assert_equal(numericalDerivative22(deltaPij, kZero, kZero),
          pim.delPdelBiasOmega(), 1e-3));

  EXPECT(
      assert_equal(numericalDerivative21(deltaVij, kZero, kZero),
          pim.delVdelBiasAcc()));
  EXPECT(
      assert_equal(numericalDerivative22(deltaVij, kZero, kZero),
          pim.delVdelBiasOmega(), 1e-3));
}

/* ************************************************************************* */
TEST(ManifoldPreintegration, computeError) {
  ManifoldPreintegration pim(testing::Params());
  NavState x1, x2;
  imuBias::ConstantBias bias;
  Matrix9 aH1, aH2;
  Matrix96 aH3;
  pim.computeError(x1, x2, bias, aH1, aH2, aH3);
  std::function<Vector9(const NavState&, const NavState&,
                        const imuBias::ConstantBias&)>
      f = std::bind(&ManifoldPreintegration::computeError, pim,
                    std::placeholders::_1, std::placeholders::_2,
                    std::placeholders::_3, nullptr, nullptr,
                    nullptr);
  // NOTE(frank): tolerance of 1e-3 on H1 because approximate away from 0
  EXPECT(assert_equal(numericalDerivative31(f, x1, x2, bias), aH1, 1e-9));
  EXPECT(assert_equal(numericalDerivative32(f, x1, x2, bias), aH2, 1e-9));
  EXPECT(assert_equal(numericalDerivative33(f, x1, x2, bias), aH3, 1e-9));
}

/* ************************************************************************* */
TEST(ManifoldPreintegration, CompareWithPreintegratedRotation) {
  // Create a PreintegratedRotation object
  auto p = std::make_shared<PreintegratedRotationParams>();
  PreintegratedRotation pim(p);

  // Integrate a single measurement
  const double omega = 0.1;
  const Vector3 trueOmega(omega, 0, 0);
  const Vector3 bias(1, 2, 3);
  const Vector3 measuredOmega = trueOmega + bias;
  const double deltaT = 0.5;

  // Check the accumulated rotation.
  Rot3 expected = Rot3::Roll(omega * deltaT);
  pim.integrateGyroMeasurement(measuredOmega, bias, deltaT);
  EXPECT(assert_equal(expected, pim.deltaRij(), 1e-9));

  // Now do the same for a ManifoldPreintegration object
  imuBias::ConstantBias biasHat {Z_3x1, bias};
  ManifoldPreintegration manifoldPim(testing::Params(), biasHat);
  manifoldPim.integrateMeasurement(Z_3x1, measuredOmega, deltaT);
  EXPECT(assert_equal(expected, manifoldPim.deltaRij(), 1e-9));

  // Check their internal Jacobians are the same:
  EXPECT(assert_equal(pim.delRdelBiasOmega(), manifoldPim.delRdelBiasOmega()));

  // Check PreintegratedRotation::biascorrectedDeltaRij.
  Matrix3 H;
  const double delta = 0.05;
  const Vector3 biasOmegaIncr(delta, 0, 0);
  Rot3 corrected = pim.biascorrectedDeltaRij(biasOmegaIncr, H);
  EQUALITY(Vector3(-deltaT * delta, 0, 0), expected.logmap(corrected));
  const Rot3 expected2 = Rot3::Roll((omega - delta) * deltaT);
  EXPECT(assert_equal(expected2, corrected, 1e-9));

  // Check ManifoldPreintegration::biasCorrectedDelta.
  imuBias::ConstantBias bias_i {Z_3x1, bias + biasOmegaIncr};
  Matrix96 H2;
  Vector9 biasCorrected = manifoldPim.biasCorrectedDelta(bias_i, H2);
  Vector9 expected3;
  expected3 << Rot3::Logmap(expected2), Z_3x1, Z_3x1;
  EXPECT(assert_equal(expected3, biasCorrected, 1e-9));
  
  // Check that this one is sane:
  auto g = [&](const Vector3& increment) {
    return manifoldPim.biasCorrectedDelta({Z_3x1, bias + increment}, {});
  };
  EXPECT(assert_equal<Matrix>(numericalDerivative11<Vector9, Vector3>(g, Z_3x1),
                              H2.rightCols<3>(),
                              1e-4));  // NOTE(frank): reduced tolerance
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

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

/* ************************************************************************* */
TEST(ManifoldPreintegration, BiasCorrectionJacobians) {
  testing::SomeMeasurements measurements;

  std::function<Rot3(const Vector3&, const Vector3&)> deltaRij =
      [&](const Vector3& a, const Vector3& w) {
        ManifoldPreintegration pim(testing::Params(), Bias(a, w));
        testing::integrateMeasurements(measurements, &pim);
        return pim.deltaRij();
      };

  std::function<Point3(const Vector3&, const Vector3&)> deltaPij =
      [&](const Vector3& a, const Vector3& w) {
        ManifoldPreintegration pim(testing::Params(), Bias(a, w));
        testing::integrateMeasurements(measurements, &pim);
        return pim.deltaPij();
      };

  std::function<Vector3(const Vector3&, const Vector3&)> deltaVij =
      [&](const Vector3& a, const Vector3& w) {
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
  const Vector3 biasOmega(1, 2, 3); 
  const Vector3 measuredOmega = trueOmega + biasOmega;
  const double deltaT = 0.5;

  // Check the accumulated rotation.
  Rot3 expected = Rot3::Roll(omega * deltaT);
  const Vector biasOmegaHat = biasOmega;
  pim.integrateGyroMeasurement(measuredOmega, biasOmegaHat, deltaT);
  EXPECT(assert_equal(expected, pim.deltaRij(), 1e-9));

  // Now do the same for a ManifoldPreintegration object
  imuBias::ConstantBias biasHat {Z_3x1, biasOmega};
  ManifoldPreintegration manifoldPim(testing::Params(), biasHat);
  manifoldPim.integrateMeasurement(Z_3x1, measuredOmega, deltaT);
  EXPECT(assert_equal(expected, manifoldPim.deltaRij(), 1e-9));

  // Check their internal Jacobians are the same:
  EXPECT(assert_equal(pim.delRdelBiasOmega(), manifoldPim.delRdelBiasOmega()));

  // Let's check the derivatives a delta away from the bias hat
  const double delta = 0.05;
  const Vector3 biasOmegaIncr(delta, 0, 0);
  imuBias::ConstantBias bias_i {Z_3x1, biasOmegaHat + biasOmegaIncr};

  // Check PreintegratedRotation::biascorrectedDeltaRij.
  // Note that biascorrectedDeltaRij expects the biasHat to be subtracted already
  Matrix3 H;
  Rot3 corrected = pim.biascorrectedDeltaRij(biasOmegaIncr, H);
  EQUALITY(Vector3(-deltaT * delta, 0, 0), expected.logmap(corrected));
  const Rot3 expected2 = Rot3::Roll((omega - delta) * deltaT);
  EXPECT(assert_equal(expected2, corrected, 1e-9));

  // Check ManifoldPreintegration::biasCorrectedDelta.
  // Note that, confusingly, biasCorrectedDelta will subtract biasHat inside 
  Matrix96 H2;
  Vector9 biasCorrected = manifoldPim.biasCorrectedDelta(bias_i, H2);
  Vector9 expected3;
  expected3 << Rot3::Logmap(expected2), Z_3x1, Z_3x1;
  EXPECT(assert_equal(expected3, biasCorrected, 1e-9));
  
  // Check that this one is sane:
  auto g = [&](const Vector3& b) {
    return manifoldPim.biasCorrectedDelta({Z_3x1, b}, {});
  };
  EXPECT(assert_equal<Matrix>(numericalDerivative11<Vector9, Vector3>(g, bias_i.gyroscope()),
                              H2.rightCols<3>()));                      
}

static NavState UpdatePreintegratedSlow(
    const Eigen::Vector3d& a_body,
    const Eigen::Vector3d& w_body,
    double dt,
    const NavState& X,
    gtsam::OptionalJacobian<9,9> A = {},
    gtsam::OptionalJacobian<9,3> B = {},
    gtsam::OptionalJacobian<9,3> C = {}) {
  const Eigen::Matrix3d Rm = X.rotation().matrix();

  Matrix9 D_dXn_dXt_jm1 = Matrix9::Identity();
  D_dXn_dXt_jm1.block<3,3>(3,3) = Rm.transpose();
  D_dXn_dXt_jm1.block<3,3>(6,6) = Rm.transpose();

  Matrix9  An;
  Matrix93 Bn, Cn;

  NavState Xn = X.update(a_body, w_body, dt,
                         A ? &An : nullptr,
                         B ? &Bn : nullptr,
                         C ? &Cn : nullptr);

  const Eigen::Matrix3d Rnm = Xn.rotation().matrix();

  Matrix9 D_dXt_dXn_j = Matrix9::Identity();
  D_dXt_dXn_j.block<3,3>(3,3) = Rnm;
  D_dXt_dXn_j.block<3,3>(6,6) = Rnm;

  if (A) *A = D_dXt_dXn_j * An * D_dXn_dXt_jm1;
  if (B) *B = D_dXt_dXn_j * Bn;
  if (C) *C = D_dXt_dXn_j * Cn;

  return Xn;
}

TEST(ManifoldPreintegration, UpdatePreintegratedFastMatchesSlow) {
  using namespace gtsam;

  const unsigned int seed_cfg  = 0;
  const unsigned int seed_meas = 1;

  ImuSimConfig cfg(seed_cfg);
  auto params = MakeParamsU(cfg);

  const double T = 5.0; // seconds
  const auto meas = MakeRandomImuMeasurements(cfg, T, seed_meas);

  NavState X_fast;
  NavState X_slow;

  Matrix9 P_fast = Matrix9::Zero();
  Matrix9 P_slow = Matrix9::Zero();

  const Matrix3 aCov = params->accelerometerCovariance;
  const Matrix3 wCov = params->gyroscopeCovariance;
  const Matrix3 iCov = params->integrationCovariance;

  ManifoldPreintegration mip(params, cfg.bias);

  for (const auto& s : meas) {
    Matrix9  A_f;
    Matrix93 B_f, C_f;
    NavState Xn_fast = mip.UpdatePreintegrated(
        s.measuredAcc, s.measuredOmega, s.dt, X_fast, &A_f, &B_f, &C_f);

    Matrix9  A_s;
    Matrix93 B_s, C_s;
    NavState Xn_slow = UpdatePreintegratedSlow(
        s.measuredAcc, s.measuredOmega, s.dt, X_slow, &A_s, &B_s, &C_s);

    EXPECT(assert_equal(Xn_fast, Xn_slow, 1e-9));

    EXPECT_MAT_NEAR(A_f, A_s, 1e-9, 1e-6);
    EXPECT_MAT_NEAR(B_f, B_s, 1e-9, 1e-6);
    EXPECT_MAT_NEAR(C_f, C_s, 1e-6, 1e-4);

    P_fast = A_f * P_fast * A_f.transpose();
    P_fast.noalias() += B_f * (aCov / s.dt) * B_f.transpose();
    P_fast.noalias() += C_f * (wCov / s.dt) * C_f.transpose();
    P_fast.block<3,3>(3,3).noalias() += iCov * s.dt;

    P_slow = A_s * P_slow * A_s.transpose();
    P_slow.noalias() += B_s * (aCov / s.dt) * B_s.transpose();
    P_slow.noalias() += C_s * (wCov / s.dt) * C_s.transpose();
    P_slow.block<3,3>(3,3).noalias() += iCov * s.dt;

    EXPECT_MAT_NEAR(P_fast, P_slow, 1e-5, 5e-3);
    X_fast = Xn_fast;
    X_slow = Xn_slow;
  }
}

// verify that B and C from update() are correct when body_P_sensor
// has both rotation and non-zero translation. The non-zero translation activates
// the cross-term (*C += *B * D_correctedAcc_omega).
TEST(ManifoldPreintegration, UpdateJacobiansWithSensorPose) {
  const Pose3 body_P_sensor(Rot3::RzRyRx(0.1, 0.2, -0.15), Point3(0.1, -0.05, 0.2));
  auto params = testing::Params();
  params->body_P_sensor = body_P_sensor;

  const imuBias::ConstantBias zeroBias;
  const Vector3 measuredAcc(0.3, -0.2, 9.91);
  const Vector3 measuredOmega(0.05, -0.1, 0.15);
  const double dt = 0.01;

  // Run one update step from the identity state
  auto tangentAfterUpdate = [&](const Vector3& acc, const Vector3& omega) -> Vector9 {
    ManifoldPreintegration pim(params, zeroBias);
    Matrix9 A; Matrix93 B, C;
    pim.update(acc, omega, dt, &A, &B, &C);
    const NavState& Xn = pim.deltaXij();
    Vector9 xi;
    xi.segment<3>(0) = Rot3::Logmap(Xn.attitude());
    xi.segment<3>(3) = Xn.position();
    xi.segment<3>(6) = Xn.velocity();
    return xi;
  };

  // Analytic Jacobians
  ManifoldPreintegration pim(params, zeroBias);
  Matrix9 A; Matrix93 B, C;
  pim.update(measuredAcc, measuredOmega, dt, &A, &B, &C);

  // Numerical Jacobians
  auto f_acc   = [&](const Vector3& a) { return tangentAfterUpdate(a, measuredOmega); };
  auto f_omega = [&](const Vector3& w) { return tangentAfterUpdate(measuredAcc, w); };
  const Matrix93 B_num = numericalDerivative11<Vector9, Vector3>(f_acc,   measuredAcc,   1e-5);
  const Matrix93 C_num = numericalDerivative11<Vector9, Vector3>(f_omega, measuredOmega, 1e-5);

  EXPECT_MAT_NEAR(B, B_num, 1e-7, 3e-5);
  EXPECT_MAT_NEAR(C, C_num, 1e-7, 3e-5);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

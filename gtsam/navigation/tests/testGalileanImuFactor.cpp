/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testGalileanImuFactor.cpp
 * @brief Tests for left-invariant Galilean IMU preintegration.
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/MatrixConstants.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/navigation/GalileanImuFactor.h>

using namespace gtsam;

/* ************************************************************************* */
namespace galilean_imu_factor {

using PIM = PreintegratedImuMeasurementsG;
using Bias = imuBias::ConstantBias;

class TestPIM : public PIM {
 public:
  using PIM::NavStateProjector;
  using PIM::PIM;

  const Gal3& preintegratedGal3() const { return preintMatrix_; }
  const Matrix106& biasJacobian() const { return biasJacobian_; }
  const Matrix10& gal3Covariance() const { return preintMeasCov_; }
};

Vector9 Components(const Gal3& g) {
  Vector9 result;
  NavState::dR(result) = Rot3::Logmap(g.rotation());
  NavState::dP(result) = g.translation();
  NavState::dV(result) = g.velocity();
  return result;
}

Vector10 Input(const Vector3& acc, const Vector3& omega, double dt) {
  Vector10 input = Vector10::Zero();
  input.head<3>() = omega * dt;
  input.segment<3>(3) = acc * dt;
  input(9) = dt;
  return input;
}

// The mean uses right composition, including Galilean time-position coupling.
TEST(GalileanImuFactor, MeanUsesRightComposition) {
  const auto params = PreintegrationParams::MakeSharedU(9.81);
  const Vector3 acc1(0.3, -0.2, 0.5), omega1(0.2, 0.1, -0.3);
  const Vector3 acc2(-0.1, 0.4, 0.2), omega2(-0.2, 0.3, 0.1);
  const double dt1 = 0.08, dt2 = 0.13;

  TestPIM pim(params);
  pim.integrateMeasurement(acc1, omega1, dt1);
  pim.integrateMeasurement(acc2, omega2, dt2);

  const Gal3 expected = Gal3::Expmap(Input(acc1, omega1, dt1))
                            .compose(Gal3::Expmap(Input(acc2, omega2, dt2)));
  EXPECT(assert_equal(expected, pim.preintegratedGal3(), 1e-12));
  DOUBLES_EQUAL(dt1 + dt2, pim.deltaTij(), 1e-12);
}

// update() returns the projected LI transition and raw sensor Jacobians.
TEST(GalileanImuFactor, UpdateJacobiansWithSensorPose) {
  const auto params = PreintegrationParams::MakeSharedU(9.81);
  params->body_P_sensor =
      Pose3(Rot3::RzRyRx(0.2, -0.1, 0.3), Point3(0.08, -0.04, 0.02));
  const Vector3 acc(0.4, -0.5, 9.6), omega(0.3, -0.2, 0.15);
  const double dt = 0.017;

  TestPIM pim(params);
  Matrix9 A;
  Matrix93 B, C;
  pim.update(acc, omega, dt, &A, &B, &C);

  PIM::Matrix10 Jr;
  Vector3 correctedAcc, correctedOmega;
  Matrix3 correctedAcc_H_acc, correctedAcc_H_omega, correctedOmega_H_omega;
  std::tie(correctedAcc, correctedOmega) = pim.correctMeasurementsBySensorPose(
      acc, omega, correctedAcc_H_acc, correctedAcc_H_omega,
      correctedOmega_H_omega);
  const Gal3 increment =
      Gal3::Expmap(Input(correctedAcc, correctedOmega, dt), Jr);
  const auto P = TestPIM::NavStateProjector();
  const Matrix9 expectedA =
      P * increment.inverse().AdjointMap() * P.transpose();
  EXPECT(assert_equal(expectedA, A, 1e-12));

  const auto integrateAcc = [&](const Vector3& measuredAcc) {
    TestPIM q(params);
    q.integrateMeasurement(measuredAcc, omega, dt);
    return q.deltaXij();
  };
  const auto integrateOmega = [&](const Vector3& measuredOmega) {
    TestPIM q(params);
    q.integrateMeasurement(acc, measuredOmega, dt);
    return q.deltaXij();
  };
  EXPECT(assert_equal(
      numericalDerivative11<NavState, Vector3>(integrateAcc, acc), B, 1e-7));
  EXPECT(assert_equal(
      numericalDerivative11<NavState, Vector3>(integrateOmega, omega), C,
      1e-7));
}

// Covariance is propagated in Gal3 order and projected to NavState order.
TEST(GalileanImuFactor, CovarianceProjection) {
  const auto params = PreintegrationParams::MakeSharedU(9.81);
  params->accelerometerCovariance = (Vector3(1e-3, 2e-3, 3e-3)).asDiagonal();
  params->gyroscopeCovariance = (Vector3(4e-4, 5e-4, 6e-4)).asDiagonal();
  params->integrationCovariance = (Vector3(7e-5, 8e-5, 9e-5)).asDiagonal();
  const Vector3 acc(0.2, -0.1, 0.4), omega(0.1, 0.3, -0.2);
  const double dt = 0.04;

  TestPIM pim(params);
  pim.integrateMeasurement(acc, omega, dt);

  PIM::Matrix10 Jr;
  Gal3::Expmap(Input(acc, omega, dt), Jr);
  const PIM::Matrix103 Ga = Jr.block<10, 3>(0, 3) * dt;
  const PIM::Matrix103 Gg = Jr.block<10, 3>(0, 0) * dt;
  PIM::Matrix10 expectedGal =
      Ga * (params->accelerometerCovariance / dt) * Ga.transpose() +
      Gg * (params->gyroscopeCovariance / dt) * Gg.transpose();
  expectedGal.block<3, 3>(6, 6) += params->integrationCovariance * dt;

  EXPECT(assert_equal(expectedGal, pim.gal3Covariance(), 1e-12));
  const auto P = TestPIM::NavStateProjector();
  const Matrix9 expectedNav = P * expectedGal * P.transpose();
  EXPECT(assert_equal(expectedNav, pim.preintMeasCov(), 1e-12));
  EXPECT(assert_equal(pim.preintMeasCov(), pim.residualCovariance(), 1e-12));
}

// Bias correction is applied on the right and exposes the complete Jacobian.
TEST(GalileanImuFactor, RightAppliedBiasCorrection) {
  const auto params = PreintegrationParams::MakeSharedU(9.81);
  const Bias biasHat(Vector3(0.02, -0.01, 0.03), Vector3(-0.01, 0.02, 0.01));
  TestPIM pim(params, biasHat);
  pim.integrateMeasurement(Vector3(0.4, -0.2, 9.7), Vector3(0.2, -0.3, 0.1),
                           0.03);
  pim.integrateMeasurement(Vector3(-0.1, 0.5, 9.6), Vector3(-0.2, 0.1, 0.3),
                           0.06);

  const Vector6 db =
      (Vector6() << 1e-4, -2e-4, 1.5e-4, -1e-4, 0.5e-4, 2e-4).finished();
  const Bias bias = biasHat + Bias(db.head<3>(), db.tail<3>());
  Matrix96 H;
  const Vector9 actual = pim.biasCorrectedDelta(bias, H);
  const Gal3 expectedGroup =
      pim.preintegratedGal3().compose(Gal3::Expmap(pim.biasJacobian() * db));
  EXPECT(assert_equal(Components(expectedGroup), actual, 1e-12));

  const Matrix numerical = numericalDerivative11<Vector9, Bias>(
      [&](const Bias& b) { return pim.biasCorrectedDelta(b); }, bias, 1e-6);
  EXPECT(assert_equal(numerical, H, 2e-8));

  TestPIM reintegrated(params, bias);
  reintegrated.integrateMeasurement(Vector3(0.4, -0.2, 9.7),
                                    Vector3(0.2, -0.3, 0.1), 0.03);
  reintegrated.integrateMeasurement(Vector3(-0.1, 0.5, 9.6),
                                    Vector3(-0.2, 0.1, 0.3), 0.06);
  EXPECT(assert_equal(reintegrated.biasCorrectedDelta(bias), actual, 1e-7));
}

// Reset restores every accumulated quantity and invalid time steps are
// rejected.
TEST(GalileanImuFactor, ResetAndTimeValidation) {
  const auto params = PreintegrationParams::MakeSharedU(9.81);
  TestPIM pim(params);
  pim.integrateMeasurement(Vector3(0.1, 0.2, 0.3), Vector3(-0.1, 0.05, 0.2),
                           0.1);
  pim.resetIntegration();
  EXPECT(pim.equals(TestPIM(params)));
  EXPECT_LONGS_EQUAL(1, pim.preintMeasCov().isZero());
  CHECK_EXCEPTION(
      pim.integrateMeasurement(Vector3::Zero(), Vector3::Zero(), 0.0),
      std::runtime_error);
}

}  // namespace galilean_imu_factor
/* ************************************************************************* */

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}

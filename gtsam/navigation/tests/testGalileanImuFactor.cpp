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
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/GalileanImuFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>

using namespace gtsam;

/* ************************************************************************* */
namespace galilean_imu_factor {

using PIM = PreintegratedImuMeasurementsG;
using CombinedPIM = PreintegratedCombinedMeasurementsG;
using Bias = imuBias::ConstantBias;
using Matrix15 = Eigen::Matrix<double, 15, 15>;
using Matrix16 = Eigen::Matrix<double, 16, 16>;
using Matrix1516 = Eigen::Matrix<double, 15, 16>;

using symbol_shorthand::B;
using symbol_shorthand::V;
using symbol_shorthand::X;

static_assert(
    std::is_same_v<PIM,
                   PreintegratedImuMeasurementsT<GalileanPreintegration>>);
static_assert(
    std::is_same_v<CombinedPIM,
                   PreintegratedCombinedMeasurementsT<GalileanPreintegration>>);
static_assert(
    std::is_same_v<GalileanCombinedImuFactor,
                   CombinedImuFactorT<PreintegratedCombinedMeasurementsG>>);

class TestPIM : public PIM {
 public:
  using PIM::PIM;

  const Gal3& preintegratedGal3() const { return preintMatrix_; }
  const Matrix106& biasJacobian() const { return biasJacobian_; }
};

PIM::Matrix106 LiftJacobian() {
  PIM::Matrix106 lift = PIM::Matrix106::Zero();
  lift.block<3, 3>(0, 3) = I_3x3;
  lift.block<3, 3>(3, 0) = I_3x3;
  return lift;
}

PIM::Matrix910 NavStateProjector() {
  PIM::Matrix910 projector = PIM::Matrix910::Zero();
  projector.block<3, 3>(0, 0) = I_3x3;
  projector.block<3, 3>(3, 6) = I_3x3;
  projector.block<3, 3>(6, 3) = I_3x3;
  return projector;
}

Matrix1516 CombinedProjector() {
  Matrix1516 projector = Matrix1516::Zero();
  projector.block<9, 10>(0, 0) = NavStateProjector();
  projector.block<6, 6>(9, 10) = I_6x6;
  return projector;
}

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

// The historical zero-argument constructor owns usable default parameters.
TEST(GalileanImuFactor, DefaultConstruction) {
  PIM pim;
  EXPECT(pim.params() != nullptr);
  pim.integrateMeasurement(Vector3::Zero(), Vector3::Zero(), 0.01);
  DOUBLES_EQUAL(0.01, pim.deltaTij(), 1e-12);
}

// The physical input lift maps GTSAM (acceleration, angular rate) order into
// Gal3 (angular rate, acceleration, position rate, clock rate) order.
TEST(GalileanImuFactor, PhysicalInputLift) {
  const Vector3 acc(0.3, -0.2, 0.5), omega(0.2, 0.1, -0.3);
  const Vector6 input = (Vector6() << acc, omega).finished();
  Vector10 expected = Vector10::Zero();
  expected.head<3>() = omega;
  expected.segment<3>(3) = acc;
  const Vector10 actual = LiftJacobian() * input;
  EXPECT(assert_equal(expected, actual, 1e-12));
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
  EXPECT(assert_equal(Components(expected), pim.preintegrated(), 1e-12));
  DOUBLES_EQUAL(dt1 + dt2, pim.deltaTij(), 1e-12);
}

// A single step initializes the right-applied bias sensitivity as -J_R h E,
// with columns in GTSAM (accelerometer, gyroscope) order.
TEST(GalileanImuFactor, PhysicalBiasJacobian) {
  const auto params = PreintegrationParams::MakeSharedU(9.81);
  const Vector3 acc(0.3, -0.2, 0.5), omega(0.2, 0.1, -0.3);
  const double dt = 0.08;

  TestPIM pim(params);
  pim.integrateMeasurement(acc, omega, dt);

  PIM::Matrix10 Jr;
  Gal3::Expmap(Input(acc, omega, dt), Jr);
  const PIM::Matrix106 expected = -Jr * LiftJacobian() * dt;
  EXPECT(assert_equal(expected, pim.biasJacobian(), 1e-12));
}

// update() returns the projected LI transition and the two blocks of the
// physical six-dimensional input Jacobian.
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
  const auto P = NavStateProjector();
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

// Generic 9D covariance propagation equals full Gal3 propagation followed by
// removal of deterministic time, including across multiple measurements.
TEST(GalileanImuFactor, CovarianceProjection) {
  const auto params = PreintegrationParams::MakeSharedU(9.81);
  params->accelerometerCovariance = (Vector3(1e-3, 2e-3, 3e-3)).asDiagonal();
  params->gyroscopeCovariance = (Vector3(4e-4, 5e-4, 6e-4)).asDiagonal();
  params->integrationCovariance = (Vector3(7e-5, 8e-5, 9e-5)).asDiagonal();
  TestPIM pim(params);
  Matrix6 inputCovariance = Matrix6::Zero();
  inputCovariance.block<3, 3>(0, 0) = params->accelerometerCovariance;
  inputCovariance.block<3, 3>(3, 3) = params->gyroscopeCovariance;
  PIM::Matrix10 expectedGal = PIM::Matrix10::Zero();

  const auto integrate = [&](const Vector3& acc, const Vector3& omega,
                             double dt) {
    PIM::Matrix10 rightJacobian;
    const Gal3 increment =
        Gal3::Expmap(Input(acc, omega, dt), rightJacobian);
    const PIM::Matrix10 transition = increment.inverse().AdjointMap();
    const PIM::Matrix106 inputJacobian = rightJacobian * LiftJacobian() * dt;
    expectedGal = transition * expectedGal * transition.transpose();
    expectedGal.noalias() += inputJacobian * (inputCovariance / dt) *
                             inputJacobian.transpose();
    expectedGal.block<3, 3>(6, 6) += params->integrationCovariance * dt;
    pim.integrateMeasurement(acc, omega, dt);
  };

  integrate(Vector3(0.2, -0.1, 0.4), Vector3(0.1, 0.3, -0.2), 0.04);
  integrate(Vector3(-0.3, 0.5, 0.1), Vector3(-0.2, 0.1, 0.4), 0.07);

  const auto P = NavStateProjector();
  const Matrix9 expectedNav = P * expectedGal * P.transpose();
  EXPECT(assert_equal(expectedNav, pim.preintMeasCov(), 1e-12));
  EXPECT(assert_equal(pim.preintMeasCov(), pim.residualCovariance(), 1e-12));
}

// Combined propagation equals a full (Gal3,bias) covariance recursion followed
// by removal of deterministic time, including state-bias cross-covariances.
TEST(GalileanCombinedImuFactor, CovarianceProjection) {
  const auto params = PreintegrationCombinedParams::MakeSharedU(9.81);
  params->accelerometerCovariance = (Vector3(1e-3, 2e-3, 3e-3)).asDiagonal();
  params->gyroscopeCovariance = (Vector3(4e-4, 5e-4, 6e-4)).asDiagonal();
  params->integrationCovariance = (Vector3(7e-5, 8e-5, 9e-5)).asDiagonal();
  params->biasAccCovariance = (Vector3(2e-6, 3e-6, 4e-6)).asDiagonal();
  params->biasOmegaCovariance = (Vector3(5e-7, 6e-7, 7e-7)).asDiagonal();

  Matrix6 inputCovariance = Matrix6::Zero();
  inputCovariance.topLeftCorner<3, 3>() = params->accelerometerCovariance;
  inputCovariance.bottomRightCorner<3, 3>() = params->gyroscopeCovariance;

  CombinedPIM pim(params);
  Matrix16 expected = Matrix16::Zero();
  const auto integrate = [&](const Vector3& acc, const Vector3& omega,
                             double dt) {
    PIM::Matrix10 rightJacobian;
    const Gal3 increment = Gal3::Expmap(Input(acc, omega, dt), rightJacobian);
    const PIM::Matrix10 transition = increment.inverse().AdjointMap();
    const PIM::Matrix106 inputJacobian = rightJacobian * LiftJacobian() * dt;

    Matrix16 F = Matrix16::Identity();
    F.topLeftCorner<10, 10>() = transition;
    F.block<10, 6>(0, 10) = inputJacobian;
    expected = F * expected * F.transpose();
    expected.topLeftCorner<10, 10>().noalias() +=
        inputJacobian * (inputCovariance / dt) * inputJacobian.transpose();
    expected.block<3, 3>(6, 6).noalias() += params->integrationCovariance * dt;
    expected.block<3, 3>(10, 10).noalias() += params->biasAccCovariance * dt;
    expected.block<3, 3>(13, 13).noalias() += params->biasOmegaCovariance * dt;
    pim.integrateMeasurement(acc, omega, dt);
  };

  integrate(Vector3(0.2, -0.1, 0.4), Vector3(0.1, 0.3, -0.2), 0.04);
  integrate(Vector3(-0.3, 0.5, 0.1), Vector3(-0.2, 0.1, 0.4), 0.07);

  const Matrix15 projected =
      CombinedProjector() * expected * CombinedProjector().transpose();
  EXPECT(assert_equal(projected, pim.preintMeasCov(), 1e-12));
  const Matrix15 native = pim.preintMeasCov();
  const Matrix15 residual = pim.residualCovariance();
  EXPECT((native.block<9, 6>(0, 9).norm() > 0.0));
  EXPECT(assert_equal(Matrix9(residual.topLeftCorner<9, 9>()),
                      Matrix9(native.topLeftCorner<9, 9>()), 1e-12));
  EXPECT(assert_equal(Matrix6(residual.bottomRightCorner<6, 6>()),
                      Matrix6(native.bottomRightCorner<6, 6>()), 1e-12));
  EXPECT(assert_equal(Matrix96(residual.block<9, 6>(0, 9)),
                      Matrix96(-native.block<9, 6>(0, 9)), 1e-12));
}

// The public six-way alias gives zero navigation residual at prediction and
// exposes correct Jacobians for both endpoint states and both biases.
TEST(GalileanCombinedImuFactor, ErrorAndJacobians) {
  const auto params = PreintegrationCombinedParams::MakeSharedU(9.81);
  params->accelerometerCovariance = 1e-4 * I_3x3;
  params->gyroscopeCovariance = 1e-6 * I_3x3;
  params->integrationCovariance = 1e-8 * I_3x3;
  params->biasAccCovariance = 1e-7 * I_3x3;
  params->biasOmegaCovariance = 1e-8 * I_3x3;

  const Bias bias_i(Vector3(0.01, -0.02, 0.03), Vector3(-0.01, 0.02, 0.01));
  const Bias bias_j(Vector3(0.011, -0.018, 0.027),
                    Vector3(-0.012, 0.019, 0.013));
  CombinedPIM pim(params, bias_i);
  pim.integrateMeasurement(Vector3(0.2, -0.1, 9.7), Vector3(0.03, -0.02, 0.01),
                           0.01);
  pim.integrateMeasurement(Vector3(0.1, 0.2, 9.8), Vector3(-0.01, 0.04, 0.02),
                           0.02);

  const NavState state_i(Rot3::RzRyRx(0.1, -0.2, 0.3), Point3(1.0, -2.0, 0.5),
                         Vector3(0.4, -0.1, 0.2));
  const NavState state_j = pim.predict(state_i, bias_i);
  const GalileanCombinedImuFactor factor(X(0), V(0), X(1), V(1), B(0), B(1),
                                         pim);

  const Vector15 error =
      factor.evaluateError(state_i.pose(), state_i.velocity(), state_j.pose(),
                           state_j.velocity(), bias_i, bias_j);
  EXPECT(assert_equal(Vector(Vector9::Zero()), Vector(error.head<9>()), 1e-9));
  EXPECT(assert_equal(Vector((bias_i - bias_j).vector()),
                      Vector(error.tail<6>()), 1e-12));

  Values values;
  values.insert(X(0), state_i.pose());
  values.insert(V(0), state_i.velocity());
  values.insert(X(1), state_j.pose());
  values.insert(V(1), state_j.velocity());
  values.insert(B(0), bias_i);
  values.insert(B(1), bias_j);
  // This checks whitened Jacobians whose largest entries are about 5.8e4.
  // GCC's finite differences vary by roughly 2e-5 across platforms, so use a
  // still-tight absolute tolerance corresponding to about 2e-9 relatively.
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-6, 1e-4);
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

  Matrix96 zeroH;
  const Vector9 zeroCorrected = pim.biasCorrectedDelta(biasHat, zeroH);
  EXPECT(
      assert_equal(Components(pim.preintegratedGal3()), zeroCorrected, 1e-12));
  const Matrix numericalZeroH = numericalDerivative11<Vector9, Bias>(
      [&](const Bias& b) { return pim.biasCorrectedDelta(b); }, biasHat, 1e-6);
  EXPECT(assert_equal(numericalZeroH, zeroH, 2e-8));

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

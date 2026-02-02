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
 * @author  Varun Agrawal
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/NavStateImuEKF.h>
#include <gtsam/navigation/ScenarioRunner.h>
#include <gtsam/nonlinear/Values.h>

#include <list>

#include "imuFactorTesting.h"

namespace combined {
// Create default parameters with Z-down and above noise parameters
static std::shared_ptr<PreintegratedCombinedMeasurements::Params> Params(
    const Matrix3& biasAccCovariance = Matrix3::Zero(),
    const Matrix3& biasOmegaCovariance = Matrix3::Zero()) {
  auto p = PreintegratedCombinedMeasurements::Params::MakeSharedD(kGravity);
  p->gyroscopeCovariance = kGyroSigma * kGyroSigma * I_3x3;
  p->accelerometerCovariance = kAccelSigma * kAccelSigma * I_3x3;
  p->integrationCovariance = 0.0001 * I_3x3;
  p->biasAccCovariance = biasAccCovariance;
  p->biasOmegaCovariance = biasOmegaCovariance;
  return p;
}
}  // namespace combined

/* ************************************************************************* */
TEST_PIM(CombinedImuFactor, PreintegratedMeasurements ) {
  // Linearization point
  Bias bias(Vector3(0, 0, 0), Vector3(0, 0, 0)); ///< Current estimate of acceleration and angular rate biases

  // Measurements
  Vector3 measuredAcc(0.1, 0.0, 0.0);
  Vector3 measuredOmega(M_PI / 100.0, 0.0, 0.0);
  double deltaT = 0.5;
  double tol = 1e-6;

  auto p = combined::Params();

  // Actual preintegrated values
  PIM expected1(p, bias);
  expected1.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

  CombinedPIM actual1(p, bias);

  actual1.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

  EXPECT(assert_equal(Vector(expected1.deltaPij()), actual1.deltaPij(), tol));
  EXPECT(assert_equal(Vector(expected1.deltaVij()), actual1.deltaVij(), tol));
  EXPECT(assert_equal(expected1.deltaRij(), actual1.deltaRij(), tol));
  DOUBLES_EQUAL(expected1.deltaTij(), actual1.deltaTij(), tol);
}


/* ************************************************************************* */
TEST_PIM(CombinedImuFactor, ErrorWithBiases ) {
  Bias bias(Vector3(0.2, 0, 0), Vector3(0, 0, 0.3)); // Biases (acc, rot)
  Bias bias2(Vector3(0.2, 0.2, 0), Vector3(1, 0, 0.3)); // Biases (acc, rot)
  Pose3 x1(Rot3::Expmap(Vector3(0, 0, M_PI / 4.0)), Point3(5.0, 1.0, -50.0));
  Vector3 v1(0.5, 0.0, 0.0);
  Pose3 x2(Rot3::Expmap(Vector3(0, 0, M_PI / 4.0 + M_PI / 10.0)),
      Point3(5.5, 1.0, -50.0));
  Vector3 v2(0.5, 0.0, 0.0);

  auto p = combined::Params();
  p->omegaCoriolis = Vector3(0,0.1,0.1);
  PIM pim(p, Bias(Vector3(0.2, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)));

  // Measurements
  Vector3 measuredOmega;
  measuredOmega << 0, 0, M_PI / 10.0 + 0.3;
  Vector3 measuredAcc =
      x1.rotation().unrotate(-p->n_gravity) + Vector3(0.2, 0.0, 0.0);
  double deltaT = 1.0;
  double tol = 1e-6;

  pim.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

  CombinedPIM combined_pim(p, Bias(Vector3(0.2, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)));

  combined_pim.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

  // Create factor
  ImuFactorT<PIM> imuFactor(X(1), V(1), X(2), V(2), B(1), pim);

  CombinedImuFactorT<CombinedPIM> combinedFactor(X(1), V(1), X(2), V(2), B(1), B(2),
                                   combined_pim);

  Vector errorExpected = imuFactor.evaluateError(x1, v1, x2, v2, bias);
  Vector errorActual = combinedFactor.evaluateError(x1, v1, x2, v2, bias,
      bias2);
  EXPECT(assert_equal(errorExpected, errorActual.head(9), tol));

  // Expected Jacobians
  Matrix H1e, H2e, H3e, H4e, H5e;
  (void) imuFactor.evaluateError(x1, v1, x2, v2, bias, H1e, H2e, H3e, H4e, H5e);

  // Actual Jacobians
  Matrix H1a, H2a, H3a, H4a, H5a, H6a;
  (void) combinedFactor.evaluateError(x1, v1, x2, v2, bias, bias2, H1a, H2a,
      H3a, H4a, H5a, H6a);

  EXPECT(assert_equal(H1e, H1a.topRows(9)));
  EXPECT(assert_equal(H2e, H2a.topRows(9)));
  EXPECT(assert_equal(H3e, H3a.topRows(9)));
  EXPECT(assert_equal(H4e, H4a.topRows(9)));
  EXPECT(assert_equal(H5e, H5a.topRows(9)));
}

/* ************************************************************************* */
// This test only works with tangent preintegration.
TEST(CombinedImuFactor, FirstOrderPreIntegratedMeasurements) {
  auto p = combined::Params();
  testing::SomeMeasurements measurements;

  auto preintegrated = [&](const Vector3& a, const Vector3& w) {
    PreintegratedImuMeasurements pim(p, Bias(a, w));
    testing::integrateMeasurements(measurements, &pim);
    return pim.preintegrated();
  };

  // Actual pre-integrated values
  PreintegratedCombinedMeasurementsT<TangentPreintegration> pim(p);
  testing::integrateMeasurements(measurements, &pim);

  EXPECT(assert_equal(numericalDerivative21<Vector9, Vector3, Vector3>(preintegrated, Z_3x1, Z_3x1),
                      pim.preintegrated_H_biasAcc()));
  EXPECT(assert_equal(numericalDerivative22<Vector9, Vector3, Vector3>(preintegrated, Z_3x1, Z_3x1),
                      pim.preintegrated_H_biasOmega(), 1e-3));
}

/* ************************************************************************* */
TEST_PIM(CombinedImuFactor, PredictPositionAndVelocity) {
  const Bias bias(Vector3(0, 0.1, 0), Vector3(0, 0.1, 0));  // Biases (acc, rot)

  auto p = combined::Params();

  // Measurements
  const Vector3 measuredOmega(0, 0.1, 0);  // M_PI/10.0+0.3;
  const Vector3 measuredAcc(0, 1.1, -kGravity);
  const double deltaT = 0.01;

  CombinedPIM pim(p, bias);

  for (int i = 0; i < 100; ++i)
    pim.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

  // Create factor
  const noiseModel::Gaussian::shared_ptr combinedmodel =
      noiseModel::Gaussian::Covariance(pim.preintMeasCov());
  const CombinedImuFactorT<CombinedPIM> Combinedfactor(X(1), V(1), X(2), V(2), B(1), B(2), pim);

  // Predict
  const NavState actual = pim.predict(NavState(), bias);
  const Pose3 expectedPose(Rot3(), Point3(0, 0.5, 0));
  const Vector3 expectedVelocity(0, 1, 0);
  EXPECT(assert_equal(expectedPose, actual.pose()));
  EXPECT(assert_equal(Vector(expectedVelocity), Vector(actual.velocity())));
}

/* ************************************************************************* */
TEST_PIM(CombinedImuFactor, PredictRotation) {
  const Bias bias(Vector3(0, 0, 0), Vector3(0, 0, 0)); // Biases (acc, rot)
  auto p = combined::Params();
  CombinedPIM pim(p, bias);
  const Vector3 measuredAcc = - kGravityAlongNavZDown;
  const Vector3 measuredOmega(0, 0, M_PI / 10.0);
  const double deltaT = 0.01;
  const double tol = 1e-4;
  for (int i = 0; i < 100; ++i)
    pim.integrateMeasurement(measuredAcc, measuredOmega, deltaT);
  const CombinedImuFactorT<CombinedPIM> Combinedfactor(X(1), V(1), X(2), V(2), B(1), B(2), pim);

  // Predict
  const Pose3 x(Rot3::Ypr(0, 0, 0), Point3(0, 0, 0)), x2;
  const Vector3 v(0, 0, 0);
  const NavState actual = pim.predict(NavState(x, v), bias);
  const Pose3 expectedPose(Rot3::Ypr(M_PI / 10, 0, 0), Point3(0, 0, 0));
  EXPECT(assert_equal(expectedPose, actual.pose(), tol));
}

/* ************************************************************************* */
// Testing covariance to check if all the jacobians are accounted for.
TEST_PIM(CombinedImuFactor, CheckCovariance) {
  auto params = PreintegrationCombinedParams::MakeSharedU(9.81);

  params->setAccelerometerCovariance(pow(0.01, 2) * I_3x3);
  params->setGyroscopeCovariance(pow(1.75e-4, 2) * I_3x3);
  params->setIntegrationCovariance(pow(0.0, 2) * I_3x3);
  params->setOmegaCoriolis(Vector3::Zero());

  imuBias::ConstantBias currentBias;

  CombinedPIM actual(params, currentBias);

  // Measurements
  Vector3 measuredAcc(0.1577, -0.8251, 9.6111);
  Vector3 measuredOmega(-0.0210, 0.0311, 0.0145);
  double deltaT = 5;

  actual.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

  Eigen::Matrix<double, 15, 15> expected;
  expected << 1.53125e-07, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,          //
      0, 1.53125e-07, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                  //
      0, 0, 1.53125e-07, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                  //
      0, 0, 0, 0.003125, 0, 0, 0.00125, 0, 0, 0, 0, 0, 0, 0, 0,  //
      0, 0, 0, 0, 0.003125, 0, 0, 0.00125, 0, 0, 0, 0, 0, 0, 0,  //
      0, 0, 0, 0, 0, 0.003125, 0, 0, 0.00125, 0, 0, 0, 0, 0, 0,  //
      0, 0, 0, 0.00125, 0, 0, 0.0005, 0, 0, 0, 0, 0, 0, 0, 0,     //
      0, 0, 0, 0, 0.00125, 0, 0, 0.0005, 0, 0, 0, 0, 0, 0, 0,     //
      0, 0, 0, 0, 0, 0.00125, 0, 0, 0.0005, 0, 0, 0, 0, 0, 0,     //
      0, 0, 0, 0, 0, 0, 0, 0, 0, 5., 0, 0, 0, 0, 0,                  //
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5., 0, 0, 0, 0,                  //
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5., 0, 0, 0,                  //
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5., 0, 0,                  //
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5., 0,                  //
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5.;

  // regression
  EXPECT(assert_equal(expected, actual.preintMeasCov()));
}

/* ************************************************************************* */
// Test that the covariance values for the ImuFactor and the CombinedImuFactor
// (top-left 9x9) are the same
TEST_PIM(CombinedImuFactor, SameCovariance) {
  // IMU measurements and time delta
  Vector3 accMeas(0.1577, -0.8251, 9.6111);
  Vector3 omegaMeas(-0.0210, 0.0311, 0.0145);
  double deltaT = 0.01;

  // Assume zero bias
  imuBias::ConstantBias currentBias;

  // Define params for ImuFactor
  auto params = PreintegrationParams::MakeSharedU();
  params->setAccelerometerCovariance(pow(0.01, 2) * I_3x3);
  params->setGyroscopeCovariance(pow(1.75e-4, 2) * I_3x3);
  params->setIntegrationCovariance(pow(0, 2) * I_3x3);
  params->setOmegaCoriolis(Vector3::Zero());

  // The IMU preintegration object for ImuFactor
  PIM pim(params, currentBias);
  pim.integrateMeasurement(accMeas, omegaMeas, deltaT);

  // Define params for CombinedImuFactor
  auto combined_params = PreintegrationCombinedParams::MakeSharedU();
  combined_params->setAccelerometerCovariance(pow(0.01, 2) * I_3x3);
  combined_params->setGyroscopeCovariance(pow(1.75e-4, 2) * I_3x3);
  // Set bias integration covariance explicitly to zero
  combined_params->setIntegrationCovariance(Z_3x3);
  combined_params->setOmegaCoriolis(Z_3x1);

  // The IMU preintegration object for CombinedImuFactor
  CombinedPIM cpim(combined_params, currentBias);
  cpim.integrateMeasurement(accMeas, omegaMeas, deltaT);

  // Assert if the noise covariance
  EXPECT(assert_equal(pim.preintMeasCov(),
                      cpim.preintMeasCov().block(0, 0, 9, 9)));
}

/* ************************************************************************* */
// Runners currently still use the integration type based on the compile flag.
// So we use the default CombinedScenarioRunner below.
TEST(CombinedImuFactor, Accelerating) {
  const double a = 0.2, v = 50;

  // Set up body pointing towards y axis, and start at 10,20,0 with velocity
  // going in X The body itself has Z axis pointing down
  const Rot3 nRb(Point3(0, 1, 0), Point3(1, 0, 0), Point3(0, 0, -1));
  const Point3 initial_position(10, 20, 0);
  const Vector3 initial_velocity(v, 0, 0);

  const AcceleratingScenario scenario(nRb, initial_position, initial_velocity,
                                      Vector3(a, 0, 0));

  const double T = 3.0;  // seconds

  CombinedScenarioRunner runner(scenario, combined::Params(), T / 10);

  PreintegratedCombinedMeasurements pim = runner.integrate(T);
  EXPECT(assert_equal(scenario.pose(T), runner.predict(pim).pose(), 1e-9));

  auto estimatedCov = runner.estimateCovariance(T, 100);
  Eigen::Matrix<double, 15, 15> expected = pim.preintMeasCov();
  EXPECT(assert_equal(estimatedCov, expected, 0.1));
}

static inline std::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params>
MakeParamsU_Combined(const ImuSimConfig& cfg) {
  // Match MakeSharedU convention: Z-up nav frame => n_gravity = (0,0,-g)
  auto p = std::make_shared<gtsam::PreintegratedCombinedMeasurements::Params>(
      gtsam::Vector3(0.0, 0.0, -cfg.g));

  const gtsam::Matrix3 I = gtsam::Matrix3::Identity();
  p->gyroscopeCovariance      = (cfg.sigma_g_c * cfg.sigma_g_c) * I;
  p->accelerometerCovariance  = (cfg.sigma_a_c * cfg.sigma_a_c) * I;
  p->integrationCovariance    = 1e-12 * I;

  // Turn off bias random walk so we can compare to 9D EKF (no bias state)
  p->biasOmegaCovariance      = gtsam::Matrix3::Zero();
  p->biasAccCovariance        = gtsam::Matrix3::Zero();
  p->biasAccOmegaInt.setZero();

  p->use2ndOrderCoriolis      = false;
  return p;
}

// -------------------------- Maps15Nav + propagation --------------------------
struct Maps15Nav {
  Eigen::Matrix<double,15,15> F;     // dx_e / dx_s
  Eigen::Matrix<double,15,15> G;     // dx_e / dz   (residual/transition def)
  Eigen::Matrix<double,15,15> covG;  // dx_e / dz   (covariance def)
};

static inline Maps15Nav BuildMaps15Nav_Manifold(
    const Eigen::Matrix3d& dR,
    const Eigen::Vector3d& dP,
    const Eigen::Vector3d& dV,
    double dt) {

  Maps15Nav m;
  m.F.setZero();
  m.G.setZero();
  m.covG.setZero();

  const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  const Eigen::Matrix3d A = dR.transpose();

  // ---- F: NAV15 -> NAV15, error order [dtheta, dp, dv, dba, dbg] ----
  m.F.block<3,3>(0,0) = A;

  // dp_e = A*dp_s + dt*A*dv_s - A*skew(dP)*dtheta_s
  m.F.block<3,3>(3,0) = -A * gtsam::skewSymmetric(dP);
  m.F.block<3,3>(3,3) = A;
  m.F.block<3,3>(3,6) = dt * A;

  // dv_e = A*dv_s - A*skew(dV)*dtheta_s
  m.F.block<3,3>(6,0) = -A * gtsam::skewSymmetric(dV);
  m.F.block<3,3>(6,6) = A;

  // biases constant in this comparison
  m.F.block<3,3>(9,9)     = I; // dba
  m.F.block<3,3>(12,12)   = I; // dbg

  // ---- G: residual/transition definition ----
  // z = [dtheta, dp, dv, dba, dbg]
  // manifold correction: theta residual uses I (not A)
  m.G.block<3,3>(0,0)   = I;
  m.G.block<3,3>(3,3)   = A;
  m.G.block<3,3>(6,6)   = A;
  m.G.block<3,3>(9,9)   = -I;
  m.G.block<3,3>(12,12) = -I;

  // ---- covG: covariance-error definition ----
  // like 9D manifold: first 9 injected as identity in nav error coords
  m.covG.setIdentity();
  m.covG.block<3,3>(9,9)   = -I;
  m.covG.block<3,3>(12,12) = -I;

  return m;
}

static inline Maps15Nav BuildMaps15Nav_Tangent(
    const Eigen::Matrix3d& dR,
    const Eigen::Vector3d& dP,
    const Eigen::Vector3d& dV,
    double dt) {

  Maps15Nav m;
  m.F.setZero();
  m.G.setZero();
  m.covG.setZero();

  const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  const Eigen::Matrix3d A = dR.transpose();

  const gtsam::Vector3 phi = gtsam::Rot3::Logmap(gtsam::Rot3(dR));
  const Eigen::Matrix3d Jr = gtsam::so3::DexpFunctor(phi).rightJacobian();

  // ---- F ----
  m.F.block<3,3>(0,0) = A;

  m.F.block<3,3>(3,0) = -A * gtsam::skewSymmetric(dP);
  m.F.block<3,3>(3,3) = A;
  m.F.block<3,3>(3,6) = dt * A;

  m.F.block<3,3>(6,0) = -A * gtsam::skewSymmetric(dV);
  m.F.block<3,3>(6,6) = A;

  m.F.block<3,3>(9,9)     = I;
  m.F.block<3,3>(12,12)   = I;

  // ---- G: z = [dphi, dp, dv, dba, dbg] ----
  m.G.block<3,3>(0,0)   = Jr;
  m.G.block<3,3>(3,3)   = A;
  m.G.block<3,3>(6,6)   = A;
  m.G.block<3,3>(9,9)   = -I;
  m.G.block<3,3>(12,12) = -I;

  // Tangent: covG == G
  m.covG = m.G;

  return m;
}

static inline Eigen::Matrix<double,15,15> PropCov15Nav_Combined(
    const Maps15Nav& m,
    const Eigen::Matrix<double,15,15>& Sigma_s,
    const Eigen::Matrix<double,15,15>& Sigma_z) {

  Eigen::Matrix<double,15,15> Sigma_e;
  Sigma_e.noalias() = m.F * Sigma_s * m.F.transpose();
  Sigma_e.noalias() += m.covG * Sigma_z * m.covG.transpose();
  return Sigma_e;
}

template <class CombinedPimType, class BuildMapsFunc>
static void RunCombinedVsEkf9D(TestResult& result_, const std::string& name_,
                              const ImuSimConfig& cfg,
                              const std::vector<ImuRawSample>& meas,
                              BuildMapsFunc buildMaps,
                              const char* tag) {
  using namespace gtsam;
  std::cout << "\n--- " << tag << " ---\n";

  const NavState s0(cfg.Rws0, cfg.pws0, cfg.vws0);

  auto ekfParams = MakeParamsU(cfg);
  auto pimParams = MakeParamsU_Combined(cfg);

  // EKF (9D)
  NavStateImuEKF ekf(s0, cfg.P0_nav9, ekfParams);
  Eigen::Matrix<double,9,9> Phi_ekf = Eigen::Matrix<double,9,9>::Identity();

  // Combined PIM (15D)
  CombinedPimType pim(pimParams, cfg.bias);

  for (const auto& s : meas) {
    // Combined integrates RAW
    pim.integrateMeasurement(s.measuredAcc, s.measuredOmega, s.dt);

    // EKF integrates UNBIASED
    const Vector3 omega_unb = s.measuredOmega - cfg.bias.gyroscope();
    const Vector3 acc_unb   = s.measuredAcc   - cfg.bias.accelerometer();

    NavStateImuEKF::Jacobian A;
    ekf.Dynamics(ekfParams->n_gravity, ekf.state(), omega_unb, acc_unb, s.dt, A);
    Phi_ekf = A * Phi_ekf;
    ekf.predict(omega_unb, acc_unb, s.dt);
  }

  const NavState s_ekf = ekf.state();
  const Eigen::Matrix<double,9,9> P_ekf = ekf.covariance();

  // ------------------ (1) state compare ------------------
  const NavState s_pre = pim.predict(s0, cfg.bias);
  std::cout << "Compare state (combined preint.predict vs EKF)\n";
  EXPECT_ROT3_NEAR(s_pre.attitude(), s_ekf.attitude(), 5e-4);
  EXPECT_MAT_NEAR(s_pre.position(), s_ekf.position(), 1e-4, 1e-2);
  EXPECT_MAT_NEAR(s_pre.velocity(), s_ekf.velocity(), 1e-4, 1e-2);

  // Preint increments
  const Eigen::Matrix3d dR = pim.deltaRij().matrix();
  const Eigen::Vector3d dP = pim.deltaPij();
  const Eigen::Vector3d dV = pim.deltaVij();
  const double DT = pim.deltaTij();

  const Maps15Nav maps15 = buildMaps(dR, dP, dV, DT);

  // ------------------ (2) transition compare ------------------
  std::cout << "Compare transition\n";
  const Eigen::Matrix<double,9,9> F9 = maps15.F.topLeftCorner<9,9>();
  EXPECT_MAT_NEAR(Phi_ekf, F9, 1e-4, 2e-2);

  // ------------------ (3) covariance mapping sanity check ------------------
  // Qz15: covariance stored in pim
  // Qr15: covariance in residual convention (CombinedImuFactor uses this)
  const Eigen::Matrix<double, 15, 15> Qz15 = pim.preintMeasCov();
  const Eigen::Matrix<double, 15, 15> Qr15 = combinedImuFactorResidualCov(pim);

  // Compare: G * Qr * G'   vs   covG * Qz * covG'
  const Eigen::Matrix<double, 15, 15> GQrGt    = maps15.G    * Qr15 * maps15.G.transpose();
  const Eigen::Matrix<double, 15, 15> covGQzGt = maps15.covG * Qz15 * maps15.covG.transpose();
  EXPECT_MAT_NEAR(GQrGt, covGQzGt, 1e-8, 1e-6);

  // ------------------ (4) covariance compare to EKF (top-left 9x9) ------------------
  Eigen::Matrix<double,15,15> P0_nav15 = Eigen::Matrix<double,15,15>::Zero();
  P0_nav15.topLeftCorner<9,9>() = cfg.P0_nav9;

  const Eigen::Matrix<double,15,15> P15 =
      PropCov15Nav_Combined(maps15, P0_nav15, Qz15);

  std::cout << "Compare covariance (EKF P vs propagated P15 top-left)\n";
  const Eigen::Matrix<double,9,9> P9 = P15.topLeftCorner<9,9>();
  EXPECT_MAT_NEAR(P_ekf, P9, 1e-4, 2e-2);
}

TEST(CombinedImuFactor, CheckCovTangent) {
  const ImuSimConfig cfg(/*seed=*/0, /*zero_bw=*/true);
  const double T = 10.0;
  const auto meas = MakeRandomImuMeasurements(cfg, T, /*seed=*/1);

  using PimC_Tan = gtsam::PreintegratedCombinedMeasurementsT<gtsam::TangentPreintegration>;
  auto buildMaps = [](const Eigen::Matrix3d& dR,
                      const Eigen::Vector3d& dP,
                      const Eigen::Vector3d& dV,
                      double dt) {
    return BuildMaps15Nav_Tangent(dR, dP, dV, dt);
  };

  RunCombinedVsEkf9D<PimC_Tan>(result_, name_, cfg, meas, buildMaps,
      "Tangent Combined (15D) vs EKF (9D)");
}

TEST(CombinedImuFactor, CheckCovManifold) {
  const ImuSimConfig cfg(/*seed=*/0, /*zero_bw=*/true);
  const double T = 10.0;
  const auto meas = MakeRandomImuMeasurements(cfg, T, /*seed=*/1);

  using PimC_Man = gtsam::PreintegratedCombinedMeasurementsT<gtsam::ManifoldPreintegration>;
  auto buildMaps = [](const Eigen::Matrix3d& dR,
                      const Eigen::Vector3d& dP,
                      const Eigen::Vector3d& dV,
                      double dt) {
    return BuildMaps15Nav_Manifold(dR, dP, dV, dt);
  };

  RunCombinedVsEkf9D<PimC_Man>(result_, name_, cfg, meas, buildMaps,
      "Manifold Combined (15D) vs EKF (9D)");
}

TEST(ImuCombinedVsEKF, CheckResidualCov15D_TangentVsManifold) {
  using namespace gtsam;
  const unsigned int seed_cfg  = 0;
  const unsigned int seed_meas = 1;

  ImuSimConfig cfg(seed_cfg);

  auto pimParams = MakeParamsU_Combined(cfg);

  const double T = 10.0; // seconds
  const auto meas = MakeRandomImuMeasurements(cfg, T, seed_meas);

  using PimC_Tan = PreintegratedCombinedMeasurementsT<TangentPreintegration>;
  using PimC_Man = PreintegratedCombinedMeasurementsT<ManifoldPreintegration>;

  PimC_Tan pim_tan(pimParams, cfg.bias);
  PimC_Man pim_man(pimParams, cfg.bias);

  for (const auto& s : meas) {
    pim_tan.integrateMeasurement(s.measuredAcc, s.measuredOmega, s.dt);
    pim_man.integrateMeasurement(s.measuredAcc, s.measuredOmega, s.dt);
  }

  // Convert both to the *factor residual* covariance convention
  const Eigen::Matrix<double, 15, 15> Qr_tan = combinedImuFactorResidualCov(pim_tan);
  const Eigen::Matrix<double, 15, 15> Qr_man = combinedImuFactorResidualCov(pim_man);

  std::cout << "\n[CheckResidualCov15D] Compare tangent vs manifold residual covariance\n";
  // std::cout << "Tangent Qr (15x15):\n"  << Qr_tan << "\n";
  // std::cout << "Manifold Qr (15x15):\n" << Qr_man << "\n";

  EXPECT_MAT_NEAR(Qr_tan, Qr_man, 1e-4, 5e-3);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

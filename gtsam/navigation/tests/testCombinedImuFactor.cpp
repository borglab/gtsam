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
#include <gtsam/navigation/ScenarioRunner.h>
#include <gtsam/nonlinear/Values.h>

#include <list>

#include "imuFactorTesting.h"

namespace testing {
// Create default parameters with Z-down and above noise parameters
static std::shared_ptr<PreintegratedCombinedMeasurements::Params> Params(
    const Matrix3& biasAccCovariance = Matrix3::Zero(),
    const Matrix3& biasOmegaCovariance = Matrix3::Zero(),
    const Matrix6& biasAccOmegaInit = Matrix6::Zero()) {
  auto p = PreintegratedCombinedMeasurements::Params::MakeSharedD(kGravity);
  p->gyroscopeCovariance = kGyroSigma * kGyroSigma * I_3x3;
  p->accelerometerCovariance = kAccelSigma * kAccelSigma * I_3x3;
  p->integrationCovariance = 0.0001 * I_3x3;
  p->biasAccCovariance = biasAccCovariance;
  p->biasOmegaCovariance = biasOmegaCovariance;
  p->biasAccOmegaInit = biasAccOmegaInit;
  return p;
}
}  // namespace testing

/* ************************************************************************* */
TEST(CombinedImuFactor, PreintegratedMeasurements ) {
  // Linearization point
  Bias bias(Vector3(0, 0, 0), Vector3(0, 0, 0)); ///< Current estimate of acceleration and angular rate biases

  // Measurements
  Vector3 measuredAcc(0.1, 0.0, 0.0);
  Vector3 measuredOmega(M_PI / 100.0, 0.0, 0.0);
  double deltaT = 0.5;
  double tol = 1e-6;

  auto p = testing::Params();

  // Actual preintegrated values
  PreintegratedImuMeasurements expected1(p, bias);
  expected1.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

  PreintegratedCombinedMeasurements actual1(p, bias);

  actual1.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

  EXPECT(assert_equal(Vector(expected1.deltaPij()), actual1.deltaPij(), tol));
  EXPECT(assert_equal(Vector(expected1.deltaVij()), actual1.deltaVij(), tol));
  EXPECT(assert_equal(expected1.deltaRij(), actual1.deltaRij(), tol));
  DOUBLES_EQUAL(expected1.deltaTij(), actual1.deltaTij(), tol);
}


/* ************************************************************************* */
TEST(CombinedImuFactor, ErrorWithBiases ) {
  Bias bias(Vector3(0.2, 0, 0), Vector3(0, 0, 0.3)); // Biases (acc, rot)
  Bias bias2(Vector3(0.2, 0.2, 0), Vector3(1, 0, 0.3)); // Biases (acc, rot)
  Pose3 x1(Rot3::Expmap(Vector3(0, 0, M_PI / 4.0)), Point3(5.0, 1.0, -50.0));
  Vector3 v1(0.5, 0.0, 0.0);
  Pose3 x2(Rot3::Expmap(Vector3(0, 0, M_PI / 4.0 + M_PI / 10.0)),
      Point3(5.5, 1.0, -50.0));
  Vector3 v2(0.5, 0.0, 0.0);

  auto p = testing::Params();
  p->omegaCoriolis = Vector3(0,0.1,0.1);
  PreintegratedImuMeasurements pim(
      p, Bias(Vector3(0.2, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)));

  // Measurements
  Vector3 measuredOmega;
  measuredOmega << 0, 0, M_PI / 10.0 + 0.3;
  Vector3 measuredAcc =
      x1.rotation().unrotate(-p->n_gravity) + Vector3(0.2, 0.0, 0.0);
  double deltaT = 1.0;
  double tol = 1e-6;

  pim.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

  PreintegratedCombinedMeasurements combined_pim(p,
      Bias(Vector3(0.2, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)));

  combined_pim.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

  // Create factor
  ImuFactor imuFactor(X(1), V(1), X(2), V(2), B(1), pim);

  noiseModel::Gaussian::shared_ptr Combinedmodel =
      noiseModel::Gaussian::Covariance(combined_pim.preintMeasCov());
  CombinedImuFactor combinedfactor(X(1), V(1), X(2), V(2), B(1), B(2),
                                   combined_pim);

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
#ifdef GTSAM_TANGENT_PREINTEGRATION
TEST(CombinedImuFactor, FirstOrderPreIntegratedMeasurements) {
  auto p = testing::Params();
  testing::SomeMeasurements measurements;

  auto preintegrated = [=](const Vector3& a, const Vector3& w) {
    PreintegratedImuMeasurements pim(p, Bias(a, w));
    testing::integrateMeasurements(measurements, &pim);
    return pim.preintegrated();
  };

  // Actual pre-integrated values
  PreintegratedCombinedMeasurements pim(p);
  testing::integrateMeasurements(measurements, &pim);

  EXPECT(assert_equal(numericalDerivative21<Vector9, Vector3, Vector3>(preintegrated, Z_3x1, Z_3x1),
                      pim.preintegrated_H_biasAcc()));
  EXPECT(assert_equal(numericalDerivative22<Vector9, Vector3, Vector3>(preintegrated, Z_3x1, Z_3x1),
                      pim.preintegrated_H_biasOmega(), 1e-3));
}
#endif

/* ************************************************************************* */
TEST(CombinedImuFactor, PredictPositionAndVelocity) {
  const Bias bias(Vector3(0, 0.1, 0), Vector3(0, 0.1, 0));  // Biases (acc, rot)

  auto p = testing::Params();

  // Measurements
  const Vector3 measuredOmega(0, 0.1, 0);  // M_PI/10.0+0.3;
  const Vector3 measuredAcc(0, 1.1, -kGravity);
  const double deltaT = 0.01;

  PreintegratedCombinedMeasurements pim(p, bias);

  for (int i = 0; i < 100; ++i)
    pim.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

  // Create factor
  const noiseModel::Gaussian::shared_ptr combinedmodel =
      noiseModel::Gaussian::Covariance(pim.preintMeasCov());
  const CombinedImuFactor Combinedfactor(X(1), V(1), X(2), V(2), B(1), B(2), pim);

  // Predict
  const NavState actual = pim.predict(NavState(), bias);
  const Pose3 expectedPose(Rot3(), Point3(0, 0.5, 0));
  const Vector3 expectedVelocity(0, 1, 0);
  EXPECT(assert_equal(expectedPose, actual.pose()));
  EXPECT(assert_equal(Vector(expectedVelocity), Vector(actual.velocity())));
}

/* ************************************************************************* */
TEST(CombinedImuFactor, PredictRotation) {
  const Bias bias(Vector3(0, 0, 0), Vector3(0, 0, 0)); // Biases (acc, rot)
  auto p = testing::Params();
  PreintegratedCombinedMeasurements pim(p, bias);
  const Vector3 measuredAcc = - kGravityAlongNavZDown;
  const Vector3 measuredOmega(0, 0, M_PI / 10.0);
  const double deltaT = 0.01;
  const double tol = 1e-4;
  for (int i = 0; i < 100; ++i)
    pim.integrateMeasurement(measuredAcc, measuredOmega, deltaT);
  const CombinedImuFactor Combinedfactor(X(1), V(1), X(2), V(2), B(1), B(2), pim);

  // Predict
  const Pose3 x(Rot3::Ypr(0, 0, 0), Point3(0, 0, 0)), x2;
  const Vector3 v(0, 0, 0), v2;
  const NavState actual = pim.predict(NavState(x, v), bias);
  const Pose3 expectedPose(Rot3::Ypr(M_PI / 10, 0, 0), Point3(0, 0, 0));
  EXPECT(assert_equal(expectedPose, actual.pose(), tol));
}

/* ************************************************************************* */
// Testing covariance to check if all the jacobians are accounted for.
TEST(CombinedImuFactor, CheckCovariance) {
  auto params = PreintegrationCombinedParams::MakeSharedU(9.81);

  params->setAccelerometerCovariance(pow(0.01, 2) * I_3x3);
  params->setGyroscopeCovariance(pow(1.75e-4, 2) * I_3x3);
  params->setIntegrationCovariance(pow(0.0, 2) * I_3x3);
  params->setOmegaCoriolis(Vector3::Zero());

  imuBias::ConstantBias currentBias;

  PreintegratedCombinedMeasurements actual(params, currentBias);

  // Measurements
  Vector3 measuredAcc(0.1577, -0.8251, 9.6111);
  Vector3 measuredOmega(-0.0210, 0.0311, 0.0145);
  double deltaT = 0.01;

  actual.integrateMeasurement(measuredAcc, measuredOmega, deltaT);

  Eigen::Matrix<double, 15, 15> expected;
  expected << 0.01, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,          //
      0, 0.01, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                  //
      0, 0, 0.01, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                  //
      0, 0, 0, 2.50025e-07, 0, 0, 5.0005e-05, 0, 0, 0, 0, 0, 0, 0, 0,  //
      0, 0, 0, 0, 2.50025e-07, 0, 0, 5.0005e-05, 0, 0, 0, 0, 0, 0, 0,  //
      0, 0, 0, 0, 0, 2.50025e-07, 0, 0, 5.0005e-05, 0, 0, 0, 0, 0, 0,  //
      0, 0, 0, 5.0005e-05, 0, 0, 0.010001, 0, 0, 0, 0, 0, 0, 0, 0,     //
      0, 0, 0, 0, 5.0005e-05, 0, 0, 0.010001, 0, 0, 0, 0, 0, 0, 0,     //
      0, 0, 0, 0, 0, 5.0005e-05, 0, 0, 0.010001, 0, 0, 0, 0, 0, 0,     //
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0.01, 0, 0, 0, 0, 0,                  //
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.01, 0, 0, 0, 0,                  //
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.01, 0, 0, 0,                  //
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.01, 0, 0,                  //
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.01, 0,                  //
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.01;

  // regression
  EXPECT(assert_equal(expected, actual.preintMeasCov()));
}

/* ************************************************************************* */
// Test that the covariance values for the ImuFactor and the CombinedImuFactor
// (top-left 9x9) are the same
TEST(CombinedImuFactor, SameCovariance) {
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
  PreintegratedImuMeasurements pim(params, currentBias);
  pim.integrateMeasurement(accMeas, omegaMeas, deltaT);

  // Define params for CombinedImuFactor
  auto combined_params = PreintegrationCombinedParams::MakeSharedU();
  combined_params->setAccelerometerCovariance(pow(0.01, 2) * I_3x3);
  combined_params->setGyroscopeCovariance(pow(1.75e-4, 2) * I_3x3);
  // Set bias integration covariance explicitly to zero
  combined_params->setIntegrationCovariance(Z_3x3);
  combined_params->setOmegaCoriolis(Z_3x1);
  // Set bias initial covariance explicitly to zero
  combined_params->setBiasAccOmegaInit(Z_6x6);

  // The IMU preintegration object for CombinedImuFactor
  PreintegratedCombinedMeasurements cpim(combined_params, currentBias);
  cpim.integrateMeasurement(accMeas, omegaMeas, deltaT);

  // Assert if the noise covariance
  EXPECT(assert_equal(pim.preintMeasCov(),
                      cpim.preintMeasCov().block(0, 0, 9, 9)));
}

/* ************************************************************************* */
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

  CombinedScenarioRunner runner(scenario, testing::Params(), T / 10);

  PreintegratedCombinedMeasurements pim = runner.integrate(T);
  EXPECT(assert_equal(scenario.pose(T), runner.predict(pim).pose(), 1e-9));

  auto estimatedCov = runner.estimateCovariance(T, 100);
  Eigen::Matrix<double, 15, 15> expected = pim.preintMeasCov();
  EXPECT(assert_equal(estimatedCov, expected, 0.1));
}

/* ************************************************************************* */
TEST(CombinedImuFactor, ResetIntegration) {
  const double a = 0.2, v = 50;

  // Set up body pointing towards y axis, and start at 10,20,0 with velocity
  // going in X The body itself has Z axis pointing down
  const Rot3 nRb(Point3(0, 1, 0), Point3(1, 0, 0), Point3(0, 0, -1));
  const Point3 initial_position(10, 20, 0);
  const Vector3 initial_velocity(v, 0, 0);

  const AcceleratingScenario scenario(nRb, initial_position, initial_velocity,
                                      Vector3(a, 0, 0));

  const double T = 3.0;  // seconds

  auto preinMeasCov = 0.001 * Eigen::Matrix<double, 15, 15>::Identity();
  CombinedScenarioRunner runner(
      scenario,
      testing::Params(Matrix3::Zero(), Matrix3::Zero(),
                      0.1 * Matrix6::Identity()),
      T / 10, imuBias::ConstantBias(), preinMeasCov);

  PreintegratedCombinedMeasurements pim = runner.integrate(T);
  // Make copy for testing different conditions
  PreintegratedCombinedMeasurements pim2 = pim;

  // Test default method
  pim.resetIntegration();
  Matrix6 expected = 0.1 * I_6x6;
  EXPECT(assert_equal(expected, pim.p().biasAccOmegaInit, 1e-9));

  // Test method where Q_init is provided
  Matrix6 expected_Q_init = I_6x6 * 0.001;
  pim2.resetIntegration(expected_Q_init);
  EXPECT(assert_equal(expected_Q_init, pim.p().biasAccOmegaInit, 1e-9));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

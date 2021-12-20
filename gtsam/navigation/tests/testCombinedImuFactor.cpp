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
 */

#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>

#include <CppUnitLite/TestHarness.h>

#include <list>

#include "imuFactorTesting.h"

namespace testing {
// Create default parameters with Z-down and above noise parameters
static boost::shared_ptr<PreintegratedCombinedMeasurements::Params> Params() {
  auto p = PreintegratedCombinedMeasurements::Params::MakeSharedD(kGravity);
  p->gyroscopeCovariance = kGyroSigma * kGyroSigma * I_3x3;
  p->accelerometerCovariance = kAccelSigma * kAccelSigma * I_3x3;
  p->integrationCovariance = 0.0001 * I_3x3;
  return p;
}
}

/* ************************************************************************* */
TEST( CombinedImuFactor, PreintegratedMeasurements ) {
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
TEST( CombinedImuFactor, ErrorWithBiases ) {
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
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

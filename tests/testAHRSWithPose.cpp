/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testAHRSWithPose.cpp
 * @brief   Unit tests for AHRSPose3Factor
 */

#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/navigation/AHRSFactor.h>

using namespace gtsam;

/* ************************************************************************* */
TEST(AHRSPose3Factor, zeroErrorWhenConsistent) {
  // Create preintegration params with small gyro noise
  auto params = std::make_shared<PreintegratedAhrsMeasurements::Params>();
  params->gyroscopeCovariance = 1e-6 * I_3x3;

  // Zero bias used during preintegration
  Vector3 biasHat = Vector3::Zero();
  PreintegratedAhrsMeasurements pim(params, biasHat);

  // Integrate a constant angular velocity around z for dt seconds
  const double dt = 0.01;            // 10 ms
  const int steps = 100;             // 1.0 s total
  const Vector3 omega(0.0, 0.0, 1.0);  // rad/s
  for (int k = 0; k < steps; ++k) {
    pim.integrateMeasurement(omega, dt);
  }

  // Construct poses consistent with integrated rotation
  const Rot3 Ri = Rot3();
  const Rot3 Rj = pim.predict(Ri, /*bias=*/Vector3::Zero());
  const Pose3 Posei(Ri, Point3(0, 0, 0));
  const Pose3 Posej(Rj, Point3(0, 0, 0));

  // Build factor and evaluate error at consistent state
  AHRSPose3Factor factor(/*pose_i*/ 1, /*pose_j*/ 2, /*bias*/ 3, pim);
  Vector actual = factor.evaluateError(Posei, Posej, /*bias=*/Vector3::Zero());

  EXPECT(assert_equal(Z_3x1, actual, 1e-9));
}

/* ************************************************************************* */
TEST(AHRSPose3Factor, jacobiansAgreeWithNumerical) {
  // Params with small gyro noise
  auto params = std::make_shared<PreintegratedAhrsMeasurements::Params>();
  params->gyroscopeCovariance = 1e-6 * I_3x3;

  // Non-zero bias used during preintegration
  Vector3 biasHat(0.01, -0.02, 0.03);
  PreintegratedAhrsMeasurements pim(params, biasHat);

  // Integrate a short random motion
  const double dt = 0.005;
  const Vector3 omega1(0.2, -0.1, 0.3);
  const Vector3 omega2(-0.05, 0.4, -0.2);
  pim.integrateMeasurement(omega1, dt);
  pim.integrateMeasurement(omega2, dt);

  // States to linearize at
  const Pose3 Posei(Rot3::RzRyRx(0.1, -0.2, 0.05), Point3(0.0, 0.0, 0.0));
  const Pose3 Posej(Rot3::RzRyRx(-0.05, 0.1, -0.02), Point3(0.0, 0.0, 0.0));
  const Vector3 bias(0.02, -0.03, 0.01);

  // Factor
  AHRSPose3Factor factor(/*pose_i*/ 1, /*pose_j*/ 2, /*bias*/ 3, pim);

  // Compute analytic Jacobians
  Matrix H1, H2, H3;
  factor.evaluateError(Posei, Posej, bias, H1, H2, H3);

  // Numerical Jacobians using GTSAM helpers
  auto f1 = [&](const Pose3& Pi) {
    return factor.evaluateError(Pi, Posej, bias);
  };
  auto f2 = [&](const Pose3& Pj) {
    return factor.evaluateError(Posei, Pj, bias);
  };
  auto f3 = [&](const Vector3& b) {
    return factor.evaluateError(Posei, Posej, b);
  };

  Matrix H1_num = numericalDerivative11<Vector, Pose3>(f1, Posei, 1e-6);
  Matrix H2_num = numericalDerivative11<Vector, Pose3>(f2, Posej, 1e-6);
  Matrix H3_num = numericalDerivative11<Vector, Vector3>(f3, bias, 1e-6);

  EXPECT(assert_equal(H1_num, H1, 1e-5));
  EXPECT(assert_equal(H2_num, H2, 1e-5));
  EXPECT(assert_equal(H3_num, H3, 1e-5));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

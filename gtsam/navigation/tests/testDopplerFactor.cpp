/**
 * @file    testDopplerFactor.cpp
 * @brief   Unit tests for DopplerFactor and ClockDriftFactor
 * @date    June 17, 2026
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/navigation/DopplerFactor.h>
#include <gtsam/navigation/tests/gnssTestHelpers.h>
#include <gtsam/nonlinear/factorTesting.h>

#include <cmath>

using namespace gtsam;
using namespace gtsam::gnss_test;

// *************************************************************************
TEST(TestDopplerFactor, Model) {
  const Point3 satVel(-1200.0, 2400.0, 800.0);     // sat ECEF velocity [m/s]
  const Point3 rcvVel(0.3, -0.1, 0.05);            // rover velocity [m/s]
  const double measDoppler = -1500.0;              // [Hz]
  const double satClkDrift = 1.2e-9;               // [s/s]
  const double rcvClkDrift = 4.5e-9;               // [s/s]

  const auto factor = DopplerFactor(
      Key(0), Key(1), measDoppler, kLambdaL1, sample::kSatPos, satVel,
      sample::kReceiverPos, satClkDrift);

  const double error =
      factor.evaluateError((Vector3)rcvVel, rcvClkDrift)[0];

  // Reference: range rate via Sagnac-aware LOS, plus the earth-rotation
  // (Sagnac) rate term, minus the measured range rate.
  Point3 e;
  gnss::geodist(sample::kSatPos, sample::kReceiverPos, e);
  const double kSag = gnss::OMGE / kCLight;
  const double sagnacRate =
      kSag * (satVel.y() * sample::kReceiverPos.x() +
              sample::kSatPos.y() * rcvVel.x() -
              satVel.x() * sample::kReceiverPos.y() -
              sample::kSatPos.x() * rcvVel.y());
  const double rangeRate = e.dot(satVel - rcvVel) +
                           kCLight * (rcvClkDrift - satClkDrift) + sagnacRate;
  const double expected = rangeRate - (-kLambdaL1 * measDoppler);
  EXPECT_DOUBLES_EQUAL(expected, error, 1e-6);

  Values values;
  values.insert(Key(0), (Vector3)rcvVel);
  values.insert(Key(1), rcvClkDrift);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-3, 1e-5);
}

// *************************************************************************
TEST(TestDopplerFactor, equals) {
  const Point3 satVel(100, 200, 300);
  const auto f1 = DopplerFactor(0, 1, 10.0, kLambdaL1, sample::kSatPos, satVel,
                                sample::kReceiverPos, 0.0);
  const auto f2 = DopplerFactor(0, 1, 10.0, kLambdaL1, sample::kSatPos, satVel,
                                sample::kReceiverPos, 0.0);
  const auto f3 = DopplerFactor(0, 1, 99.0, kLambdaL1, sample::kSatPos, satVel,
                                sample::kReceiverPos, 0.0);
  CHECK(f1.equals(f2));
  CHECK(!f1.equals(f3));
  f1.print("doppler ");
}

// *************************************************************************
TEST(TestClockDriftFactor, Model) {
  const double dt = 0.2;
  const double biasPrev = 1.0e-6, drift = 4.5e-9;
  const double biasCurr = biasPrev + drift * dt;  // zero-error point

  const auto factor = ClockDriftFactor(Key(0), Key(1), Key(2), dt);
  const double error = factor.evaluateError(biasPrev, biasCurr, drift)[0];
  EXPECT_DOUBLES_EQUAL(0.0, error, 1e-15);

  // Off the constraint:
  const double error2 = factor.evaluateError(biasPrev, biasCurr + 1e-7, drift)[0];
  EXPECT_DOUBLES_EQUAL(1e-7, error2, 1e-15);

  Values values;
  values.insert(Key(0), biasPrev);
  values.insert(Key(1), biasCurr);
  values.insert(Key(2), drift);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-3, 1e-6);
}

// *************************************************************************
// DopplerFactorArm: with omega = 0 it must reduce to DopplerFactor at the same
// velocity, independent of the pose attitude (lever velocity omega x b = 0).
TEST(TestDopplerFactorArm, ReducesToBaseWhenNoRotationRate) {
  const Point3 satVel(-1200.0, 2400.0, 800.0);
  const Vector3 rcvVel(0.3, -0.1, 0.05);
  const double measDoppler = -1500.0, satClkDrift = 1.2e-9, rcvClkDrift = 4.5e-9;
  const Point3 lever(0.5, -0.3, 1.0), omega(0.0, 0.0, 0.0);
  const Pose3 pose(Rot3::RzRyRx(0.3, -0.2, 0.5), sample::kReceiverPos);

  const auto arm = DopplerFactorArm(0, 1, 2, measDoppler, kLambdaL1,
                                    sample::kSatPos, satVel,
                                    sample::kReceiverPos, lever, omega,
                                    satClkDrift);
  const auto base = DopplerFactor(1, 2, measDoppler, kLambdaL1, sample::kSatPos,
                                  satVel, sample::kReceiverPos, satClkDrift);
  EXPECT_DOUBLES_EQUAL(base.evaluateError((Vector3)rcvVel, rcvClkDrift)[0],
                       arm.evaluateError(pose, (Vector3)rcvVel, rcvClkDrift)[0],
                       1e-9);
}

// *************************************************************************
TEST(TestDopplerFactorArm, Model) {
  const Point3 satVel(-1200.0, 2400.0, 800.0);
  const Vector3 rcvVel(0.3, -0.1, 0.05);
  const double measDoppler = -1500.0, satClkDrift = 1.2e-9, rcvClkDrift = 4.5e-9;
  const Point3 lever(0.5, -0.3, 1.0), omega(0.02, -0.05, 0.1);
  const Pose3 pose(Rot3::RzRyRx(0.3, -0.2, 0.5), sample::kReceiverPos);

  const auto factor = DopplerFactorArm(0, 1, 2, measDoppler, kLambdaL1,
                                       sample::kSatPos, satVel,
                                       sample::kReceiverPos, lever, omega,
                                       satClkDrift);

  // Independent reference: antenna velocity v_ant = v + R*(omega x lever),
  // then the same range-rate model (incl. Sagnac) as DopplerFactor at v_ant.
  Point3 e;
  gnss::geodist(sample::kSatPos, sample::kReceiverPos, e);
  const Vector3 vAnt = (Vector3)rcvVel + pose.rotation().rotate(omega.cross(lever));
  const double kSag = gnss::OMGE / kCLight;
  const double sagnac =
      kSag * (satVel.y() * sample::kReceiverPos.x() +
              sample::kSatPos.y() * vAnt.x() -
              satVel.x() * sample::kReceiverPos.y() -
              sample::kSatPos.x() * vAnt.y());
  const double rangeRate = e.dot(satVel - Point3(vAnt)) +
                           kCLight * (rcvClkDrift - satClkDrift) + sagnac;
  const double expected = rangeRate - (-kLambdaL1 * measDoppler);
  EXPECT_DOUBLES_EQUAL(
      expected, factor.evaluateError(pose, (Vector3)rcvVel, rcvClkDrift)[0],
      1e-6);

  Values values;
  values.insert(0, pose);
  values.insert(1, (Vector3)rcvVel);
  values.insert(2, rcvClkDrift);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-5, 1e-6);
}

// *************************************************************************
// Local nav-frame pose overload (ecef_T_nav): checks the composed-rotation
// Jacobian path.
TEST(TestDopplerFactorArm, NavFrameJacobians) {
  const Point3 satVel(-1200.0, 2400.0, 800.0);
  const Vector3 rcvVel(0.3, -0.1, 0.05);
  const double measDoppler = -1500.0, rcvClkDrift = 4.5e-9;
  const Point3 lever(0.5, -0.3, 1.0), omega(0.02, -0.05, 0.1);
  const Pose3 ecef_T_nav(Rot3::RzRyRx(0.1, 0.4, -0.7), sample::kReceiverPos);
  const Pose3 navPose(Rot3::RzRyRx(0.3, -0.2, 0.5), Point3(0.0, 0.0, 0.0));

  const auto factor = DopplerFactorArm(0, 1, 2, measDoppler, kLambdaL1,
                                       sample::kSatPos, satVel,
                                       sample::kReceiverPos, lever, ecef_T_nav,
                                       omega, 0.0);
  Values values;
  values.insert(0, navPose);
  values.insert(1, (Vector3)rcvVel);
  values.insert(2, rcvClkDrift);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-5, 1e-6);
}

// *************************************************************************
TEST(TestDopplerFactorArm, equals) {
  const Point3 satVel(100, 200, 300), lever(0.5, -0.3, 1.0), omega(0.02, 0, 0.1);
  const auto f1 = DopplerFactorArm(0, 1, 2, 10.0, kLambdaL1, sample::kSatPos,
                                   satVel, sample::kReceiverPos, lever, omega, 0.0);
  const auto f2 = DopplerFactorArm(0, 1, 2, 10.0, kLambdaL1, sample::kSatPos,
                                   satVel, sample::kReceiverPos, lever, omega, 0.0);
  const auto f3 = DopplerFactorArm(0, 1, 2, 10.0, kLambdaL1, sample::kSatPos,
                                   satVel, sample::kReceiverPos,
                                   Point3(1.0, 0.0, 0.0), omega, 0.0);
  CHECK(f1.equals(f2));
  CHECK(!f1.equals(f3));  // differs only in the lever arm
  f1.print("dopplerArm ");
}

// *************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
// *************************************************************************

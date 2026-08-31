/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testDopplerFactor.cpp
 * @brief   Unit tests for DopplerFactor and DopplerFactorArm
 * @date    June 17, 2026
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/linear/FixedJacobianFactor.h>
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
  const double dt = 0.2;                           // epoch interval [s]
  const double biasPrev = 1.0e-6;                  // [s]
  const double biasCurr = biasPrev + rcvClkDrift * dt;

  const auto factor = DopplerFactor(
      Key(0), Key(1), Key(2), measDoppler, kLambdaL1, sample::kSatPos, satVel,
      sample::kReceiverPos, dt, satClkDrift);

  const double error =
      factor.evaluateError((Vector3)rcvVel, biasPrev, biasCurr)[0];

  // Reference: range rate via Sagnac-aware LOS, plus the earth-rotation
  // (Sagnac) rate term, minus the measured range rate.  The receiver clock
  // drift is the time-differenced bias, (biasCurr - biasPrev)/dt = rcvClkDrift.
  Point3 e;
  gnss::geodist(sample::kSatPos, sample::kReceiverPos, e);
  const double kSag = gnss::OMGE / kCLight;
  const double sagnacRate =
      kSag * (satVel.x() * sample::kReceiverPos.y() +
              sample::kSatPos.x() * rcvVel.y() -
              satVel.y() * sample::kReceiverPos.x() -
              sample::kSatPos.y() * rcvVel.x());
  const double rangeRate = e.dot(satVel - rcvVel) +
                           kCLight * (rcvClkDrift - satClkDrift) + sagnacRate;
  const double expected = rangeRate - (-kLambdaL1 * measDoppler);
  EXPECT_DOUBLES_EQUAL(expected, error, 1e-6);

  Values values;
  values.insert(Key(0), (Vector3)rcvVel);
  values.insert(Key(1), biasPrev);
  values.insert(Key(2), biasCurr);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-3, 1e-5);
}

// *************************************************************************
// Fixed residual and argument dimensions select the automatic ternary path.
TEST(TestDopplerFactor, TernaryLinearization) {
  const Vector3 velocity(0.3, -0.1, 0.05);
  const double biasPrev = 1.0e-6, biasCurr = 1.0045e-6;
  const DopplerFactor factor(0, 1, 2, -1500.0, kLambdaL1, sample::kSatPos,
                              Point3(-1200.0, 2400.0, 800.0),
                              sample::kReceiverPos, 1.0, 1.2e-9);
  const Values values{{0, genericValue(velocity)},
                      {1, genericValue(biasPrev)},
                      {2, genericValue(biasCurr)}};

  const auto generic = factor.NoiseModelFactor::linearize(values);
  const auto fixed = factor.linearize(values);
  CHECK((std::dynamic_pointer_cast<FixedJacobianFactor<1, 3, 1, 1>>(fixed)));
  EXPECT(assert_equal(*generic, *fixed, 1e-12));
}

// *************************************************************************
// The clock term must depend only on the bias difference: shifting both bias
// states by a common offset leaves the error unchanged (Doppler cannot
// observe the absolute bias, only its rate).
TEST(TestDopplerFactor, CommonBiasOffsetInvariance) {
  const Point3 satVel(-1200.0, 2400.0, 800.0);
  const Vector3 rcvVel(0.3, -0.1, 0.05);
  const double dt = 1.0, biasPrev = 1.0e-6, biasCurr = 1.0045e-6;

  const auto factor = DopplerFactor(0, 1, 2, -1500.0, kLambdaL1,
                                    sample::kSatPos, satVel,
                                    sample::kReceiverPos, dt, 1.2e-9);
  const double e1 = factor.evaluateError(rcvVel, biasPrev, biasCurr)[0];
  const double offset = 3.7e-4;  // common clock offset [s]
  const double e2 =
      factor.evaluateError(rcvVel, biasPrev + offset, biasCurr + offset)[0];
  EXPECT_DOUBLES_EQUAL(e1, e2, 1e-6);
}

// *************************************************************************
TEST(TestDopplerFactor, InvalidDtThrows) {
  const Point3 satVel(100, 200, 300);
  CHECK_EXCEPTION(DopplerFactor(0, 1, 2, 10.0, kLambdaL1, sample::kSatPos,
                                satVel, sample::kReceiverPos, 0.0),
                  std::invalid_argument);
  CHECK_EXCEPTION(DopplerFactor(0, 1, 2, 10.0, kLambdaL1, sample::kSatPos,
                                satVel, sample::kReceiverPos, -1.0),
                  std::invalid_argument);
}

// *************************************************************************
TEST(TestDopplerFactor, equals) {
  const Point3 satVel(100, 200, 300);
  const auto f1 = DopplerFactor(0, 1, 2, 10.0, kLambdaL1, sample::kSatPos,
                                satVel, sample::kReceiverPos, 1.0, 0.0);
  const auto f2 = DopplerFactor(0, 1, 2, 10.0, kLambdaL1, sample::kSatPos,
                                satVel, sample::kReceiverPos, 1.0, 0.0);
  const auto f3 = DopplerFactor(0, 1, 2, 99.0, kLambdaL1, sample::kSatPos,
                                satVel, sample::kReceiverPos, 1.0, 0.0);
  const auto f4 = DopplerFactor(0, 1, 2, 10.0, kLambdaL1, sample::kSatPos,
                                satVel, sample::kReceiverPos, 0.5, 0.0);
  CHECK(f1.equals(f2));
  CHECK(!f1.equals(f3));
  CHECK(!f1.equals(f4));  // differs only in dt
  f1.print("doppler ");
}

// *************************************************************************
// DopplerFactorArm: with omega = 0 it must reduce to DopplerFactor at the same
// velocity, independent of the pose attitude (lever velocity omega x b = 0).
TEST(TestDopplerFactorArm, ReducesToBaseWhenNoRotationRate) {
  const Point3 satVel(-1200.0, 2400.0, 800.0);
  const Vector3 rcvVel(0.3, -0.1, 0.05);
  const double measDoppler = -1500.0, satClkDrift = 1.2e-9;
  const double dt = 0.2, biasPrev = 1.0e-6, biasCurr = 1.0e-6 + 4.5e-9 * dt;
  const Point3 lever(0.5, -0.3, 1.0), omega(0.0, 0.0, 0.0);
  const Pose3 pose(Rot3::RzRyRx(0.3, -0.2, 0.5), sample::kReceiverPos);

  const auto arm = DopplerFactorArm(0, 1, 2, 3, measDoppler, kLambdaL1,
                                    sample::kSatPos, satVel,
                                    sample::kReceiverPos, lever, omega, dt,
                                    satClkDrift);
  const auto base = DopplerFactor(1, 2, 3, measDoppler, kLambdaL1,
                                  sample::kSatPos, satVel,
                                  sample::kReceiverPos, dt, satClkDrift);
  EXPECT_DOUBLES_EQUAL(
      base.evaluateError((Vector3)rcvVel, biasPrev, biasCurr)[0],
      arm.evaluateError(pose, (Vector3)rcvVel, biasPrev, biasCurr)[0], 1e-9);
}

// *************************************************************************
TEST(TestDopplerFactorArm, Model) {
  const Point3 satVel(-1200.0, 2400.0, 800.0);
  const Vector3 rcvVel(0.3, -0.1, 0.05);
  const double measDoppler = -1500.0, satClkDrift = 1.2e-9, rcvClkDrift = 4.5e-9;
  const double dt = 0.2, biasPrev = 1.0e-6;
  const double biasCurr = biasPrev + rcvClkDrift * dt;
  const Point3 lever(0.5, -0.3, 1.0), omega(0.02, -0.05, 0.1);
  const Pose3 pose(Rot3::RzRyRx(0.3, -0.2, 0.5), sample::kReceiverPos);

  const auto factor = DopplerFactorArm(0, 1, 2, 3, measDoppler, kLambdaL1,
                                       sample::kSatPos, satVel,
                                       sample::kReceiverPos, lever, omega, dt,
                                       satClkDrift);

  // Independent reference: antenna velocity v_ant = v + R*(omega x lever),
  // then the same range-rate model (incl. Sagnac) as DopplerFactor at v_ant.
  Point3 e;
  gnss::geodist(sample::kSatPos, sample::kReceiverPos, e);
  const Vector3 vAnt = (Vector3)rcvVel + pose.rotation().rotate(omega.cross(lever));
  const double kSag = gnss::OMGE / kCLight;
  const double sagnac =
      kSag * (satVel.x() * sample::kReceiverPos.y() +
              sample::kSatPos.x() * vAnt.y() -
              satVel.y() * sample::kReceiverPos.x() -
              sample::kSatPos.y() * vAnt.x());
  const double rangeRate = e.dot(satVel - Point3(vAnt)) +
                           kCLight * (rcvClkDrift - satClkDrift) + sagnac;
  const double expected = rangeRate - (-kLambdaL1 * measDoppler);
  EXPECT_DOUBLES_EQUAL(
      expected,
      factor.evaluateError(pose, (Vector3)rcvVel, biasPrev, biasCurr)[0],
      1e-6);

  Values values;
  values.insert(0, pose);
  values.insert(1, (Vector3)rcvVel);
  values.insert(2, biasPrev);
  values.insert(3, biasCurr);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-5, 1e-6);
}

// *************************************************************************
// Nav-frame overload (ecef_T_nav): value + Jacobians, with a nav-frame velocity.
TEST(TestDopplerFactorArm, NavFrame) {
  const Point3 satVel(-1200.0, 2400.0, 800.0);
  const Vector3 navVel(0.3, -0.1, 0.05);  // receiver velocity in the nav frame
  const double measDoppler = -1500.0, satClkDrift = 1.2e-9, rcvClkDrift = 4.5e-9;
  const double dt = 0.2, biasPrev = 1.0e-6;
  const double biasCurr = biasPrev + rcvClkDrift * dt;
  const Point3 lever(0.5, -0.3, 1.0), omega(0.02, -0.05, 0.1);
  const Pose3 ecef_T_nav(Rot3::RzRyRx(0.1, 0.4, -0.7), sample::kReceiverPos);
  const Pose3 navPose(Rot3::RzRyRx(0.3, -0.2, 0.5), Point3(0.0, 0.0, 0.0));

  const auto factor = DopplerFactorArm(0, 1, 2, 3, measDoppler, kLambdaL1,
                                       sample::kSatPos, satVel,
                                       sample::kReceiverPos, lever, ecef_T_nav,
                                       omega, dt, satClkDrift);

  // Antenna velocity: (nav vel + nav_R_body*(omega x lever)) rotated to ECEF.
  Point3 e;
  gnss::geodist(sample::kSatPos, sample::kReceiverPos, e);
  const Vector3 vAntNav =
      navVel + Vector3(navPose.rotation().rotate(omega.cross(lever)));
  const Vector3 vAnt = ecef_T_nav.rotation().rotate(Point3(vAntNav));
  const double kSag = gnss::OMGE / kCLight;
  const double sagnac =
      kSag * (satVel.x() * sample::kReceiverPos.y() +
              sample::kSatPos.x() * vAnt.y() -
              satVel.y() * sample::kReceiverPos.x() -
              sample::kSatPos.y() * vAnt.x());
  const double rangeRate = e.dot(satVel - Point3(vAnt)) +
                           kCLight * (rcvClkDrift - satClkDrift) + sagnac;
  const double expected = rangeRate - (-kLambdaL1 * measDoppler);
  EXPECT_DOUBLES_EQUAL(
      expected,
      factor.evaluateError(navPose, navVel, biasPrev, biasCurr)[0], 1e-6);

  Values values;
  values.insert(0, navPose);
  values.insert(1, navVel);
  values.insert(2, biasPrev);
  values.insert(3, biasCurr);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-5, 1e-6);
}

// *************************************************************************
TEST(TestDopplerFactorArm, equals) {
  const Point3 satVel(100, 200, 300), lever(0.5, -0.3, 1.0), omega(0.02, 0, 0.1);
  const auto f1 = DopplerFactorArm(0, 1, 2, 3, 10.0, kLambdaL1, sample::kSatPos,
                                   satVel, sample::kReceiverPos, lever, omega,
                                   1.0, 0.0);
  const auto f2 = DopplerFactorArm(0, 1, 2, 3, 10.0, kLambdaL1, sample::kSatPos,
                                   satVel, sample::kReceiverPos, lever, omega,
                                   1.0, 0.0);
  const auto f3 = DopplerFactorArm(0, 1, 2, 3, 10.0, kLambdaL1, sample::kSatPos,
                                   satVel, sample::kReceiverPos,
                                   Point3(1.0, 0.0, 0.0), omega, 1.0, 0.0);
  CHECK(f1.equals(f2));
  CHECK(!f1.equals(f3));  // differs only in the lever arm
  f1.print("dopplerArm ");
}

// *************************************************************************
TEST(TestDopplerFactorArm, InvalidDtThrows) {
  const Point3 satVel(100, 200, 300), lever(0.5, -0.3, 1.0), omega(0.02, 0, 0.1);
  CHECK_EXCEPTION(
      DopplerFactorArm(0, 1, 2, 3, 10.0, kLambdaL1, sample::kSatPos, satVel,
                       sample::kReceiverPos, lever, omega, 0.0),
      std::invalid_argument);
  CHECK_EXCEPTION(
      DopplerFactorArm(0, 1, 2, 3, 10.0, kLambdaL1, sample::kSatPos, satVel,
                       sample::kReceiverPos, lever, omega, -1.0),
      std::invalid_argument);
}

/* ************************************************************************* */
namespace sagnac_rate {

TEST(TestDopplerFactor, SagnacRateMatchesGeodistDerivative) {
  const Point3 satVel(-1500.0, 900.0, 2300.0);
  const Point3 rcvVel(7.0, -5.0, 3.0);

  // Zero Doppler and equal clock biases, so the error is the modelled rate.
  DopplerFactor factor(0, 1, 2, /*measuredDoppler=*/0.0, kLambdaL1,
                       sample::kSatPos, satVel, sample::kReceiverPos,
                       /*dt=*/1.0);
  const double modelled = factor.evaluateError((Vector3)rcvVel, 0.0, 0.0)[0];

  const double h = 0.1;
  Point3 e;
  const double rangePlus = gnss::geodist(sample::kSatPos + satVel * h,
                                         sample::kReceiverPos + rcvVel * h, e);
  const double rangeMinus = gnss::geodist(sample::kSatPos - satVel * h,
                                          sample::kReceiverPos - rcvVel * h, e);
  EXPECT_DOUBLES_EQUAL((rangePlus - rangeMinus) / (2.0 * h), modelled, 1e-6);
}

TEST(TestDopplerFactor, SagnacRateSignMatchesRangeDomain) {
  const Point3 satVel(-1500.0, 900.0, 2300.0);
  const Point3 rcvVel(7.0, -5.0, 3.0);
  const double kSag = gnss::OMGE / kCLight;

  DopplerFactor factor(0, 1, 2, 0.0, kLambdaL1, sample::kSatPos, satVel,
                       sample::kReceiverPos, 1.0);
  Point3 e;
  gnss::geodist(sample::kSatPos, sample::kReceiverPos, e);
  const double applied =
      factor.evaluateError((Vector3)rcvVel, 0.0, 0.0)[0] - e.dot(satVel - rcvVel);

  const double expected =
      kSag * (satVel.x() * sample::kReceiverPos.y() +
              sample::kSatPos.x() * rcvVel.y() -
              satVel.y() * sample::kReceiverPos.x() -
              sample::kSatPos.y() * rcvVel.x());

  EXPECT_DOUBLES_EQUAL(expected, applied, 1e-9);
  CHECK(expected * applied > 0.0);
}

}  // namespace sagnac_rate
/* ************************************************************************* */

// *************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
// *************************************************************************

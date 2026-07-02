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
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
// *************************************************************************

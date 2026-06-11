/**
 *  @file   testCarrierPhaseFactor.cpp
 *  @brief  Unit tests for CarrierPhaseFactor and CarrierPhaseFactorArm
 *  @date   March 23, 2026
 **/

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/navigation/CarrierPhaseFactor.h>
#include <gtsam/navigation/tests/gnssTestHelpers.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/factorTesting.h>

#include <cmath>

using namespace gtsam;
using namespace gtsam::gnss_test;

// *************************************************************************
// CarrierPhaseFactor tests (all quantities in meters)
// *************************************************************************
TEST(TestCarrierPhaseFactor, Constructor) {
  // All zeros: error = 0 + 0 - 0 + 0 - 0 = 0
  const auto factor =
      CarrierPhaseFactor(Key(0), Key(1), Key(2), 0.0, Point3::Zero(), 0.0);
  const double error = factor.evaluateError(Point3::Zero(), 0.0, 0.0)[0];
  EXPECT_DOUBLES_EQUAL(0.0, error, 1e-9);
}

// *************************************************************************
TEST(TestCarrierPhaseFactor, AmbiguityEffect) {
  // error = range + clock - sat_clock + ambiguity - measurement
  // With range = measurement and clock = sat_clock = 0:
  //   error = ambiguity
  const Point3 satPos(0.0, 0.0, 20200000.0);
  const Point3 recvPos(0.0, 0.0, 0.0);
  const double range = 20200000.0;

  const auto factor = CarrierPhaseFactor(
      Key(0), Key(1), Key(2), range, satPos, 0.0);

  const double e0 = factor.evaluateError(recvPos, 0.0, 0.0)[0];
  EXPECT_DOUBLES_EQUAL(0.0, e0, 1e-9);

  // Adding 1.0 meter ambiguity should change error by 1.0
  const double e1 = factor.evaluateError(recvPos, 0.0, 1.0)[0];
  EXPECT_DOUBLES_EQUAL(1.0, e1, 1e-9);

  // Adding one L1 cycle in meters
  const double e2 = factor.evaluateError(recvPos, 0.0, kLambdaL1)[0];
  EXPECT_DOUBLES_EQUAL(kLambdaL1, e2, 1e-9);
}

// *************************************************************************
TEST(TestCarrierPhaseFactor, Jacobians) {
  const auto factor = CarrierPhaseFactor(
      Key(0), Key(1), Key(2), sample::kPseudorange, sample::kSatPos,
      sample::kSatClkBias);

  Values values;
  values.insert(Key(0), sample::kReceiverPos);
  values.insert(Key(1), sample::kReceiverClock);
  values.insert(Key(2), kLambdaL1 * 1000.0);  // ambiguity in meters
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-3, 1e-5);
}

// *************************************************************************
TEST(TestCarrierPhaseFactor, print) {
  const auto factor =
      CarrierPhaseFactor(Key(0), Key(1), Key(2), 0.0, Point3::Zero(), 0.0);
  factor.print("test ");
}

// *************************************************************************
TEST(TestCarrierPhaseFactor, equals) {
  const auto f1 = CarrierPhaseFactor(1, 2, 3, 100.0, Point3(1, 2, 3), 0.0);
  const auto f2 = CarrierPhaseFactor(1, 2, 3, 100.0, Point3(1, 2, 3), 0.0);
  const auto f3 = CarrierPhaseFactor(1, 2, 3, 200.0, Point3(1, 2, 3), 0.0);

  CHECK(f1.equals(f2));
  CHECK(!f1.equals(f3));
}

// *************************************************************************
// CarrierPhaseFactorArm tests
// *************************************************************************
TEST(TestCarrierPhaseFactorArm, Constructor) {
  const Point3 leverArm(0.5, -0.3, 1.0);
  const auto factor = CarrierPhaseFactorArm(
      Key(0), Key(1), Key(2), 0.0, Point3::Zero(), leverArm, 0.0);

  const Pose3 pose(Rot3::Identity(), Point3::Zero());
  const double error = factor.evaluateError(pose, 0.0, 0.0)[0];
  // Error should be range from lever arm to origin = ||leverArm||.
  EXPECT_DOUBLES_EQUAL(leverArm.norm(), error, 1e-9);
}

// *************************************************************************
TEST(TestCarrierPhaseFactorArm, Jacobians) {
  const Point3 leverArm(0.1, 0.0, -0.5);
  const auto factor = CarrierPhaseFactorArm(
      Key(0), Key(1), Key(2), sample::kPseudorange, sample::kSatPos, leverArm,
      sample::kSatClkBias);

  Values values;
  values.insert(Key(0),
                Pose3(Rot3::RzRyRx(0.1, 0.2, 0.3), sample::kReceiverPos));
  values.insert(Key(1), sample::kReceiverClock);
  values.insert(Key(2), kLambdaL1 * 1000.0);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-3, 1e-5);
}

// *************************************************************************
TEST(TestCarrierPhaseFactorArm, ZeroLeverArm) {
  const double clock = 1e-7;
  const double amb_m = kLambdaL1 * 500.0;

  const auto factorPt = CarrierPhaseFactor(
      Key(0), Key(1), Key(2), sample::kPseudorange, sample::kSatPos,
      sample::kSatClkBias);
  const auto factorArm = CarrierPhaseFactorArm(
      Key(0), Key(1), Key(2), sample::kPseudorange, sample::kSatPos,
      Point3::Zero(), sample::kSatClkBias);

  const double errorPt =
      factorPt.evaluateError(sample::kReceiverPos, clock, amb_m)[0];
  const double errorArm = factorArm.evaluateError(
      Pose3(Rot3::Identity(), sample::kReceiverPos), clock, amb_m)[0];
  EXPECT_DOUBLES_EQUAL(errorPt, errorArm, 1e-9);
}

// *************************************************************************
TEST(TestCarrierPhaseFactorArm, EcefTnavIdentity) {
  const Point3 leverArm(0.5, -0.3, 1.0);
  const Point3 satPos(0.0, 0.0, 3.0);
  const double phi = 4.0;

  const auto factorEcef = CarrierPhaseFactorArm(
      Key(0), Key(1), Key(2), phi, satPos, leverArm, 0.0);
  const auto factorNav = CarrierPhaseFactorArm(
      Key(0), Key(1), Key(2), phi, satPos, leverArm, Pose3::Identity(), 0.0);

  const Pose3 pose(Rot3::RzRyRx(0.1, 0.2, 0.3), Point3(1.0, 2.0, 3.0));
  const double errorEcef = factorEcef.evaluateError(pose, 1.0, 10.0)[0];
  const double errorNav = factorNav.evaluateError(pose, 1.0, 10.0)[0];
  EXPECT_DOUBLES_EQUAL(errorEcef, errorNav, 1e-9);
}

// *************************************************************************
TEST(TestCarrierPhaseFactorArm, EcefTnavIdentityJacobians) {
  const Point3 leverArm(0.5, -0.3, 1.0);
  const auto factor = CarrierPhaseFactorArm(
      Key(0), Key(1), Key(2), 4.0, Vector3(0.0, 0.0, 3.0), leverArm,
      Pose3::Identity(), 0.0);

  Values values;
  values.insert(Key(0), Pose3(Rot3::RzRyRx(0.1, 0.2, 0.3),
                               Point3(1.0, 2.0, 3.0)));
  values.insert(Key(1), 0.0);
  values.insert(Key(2), 10.0);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-3, 1e-5);
}

// *************************************************************************
TEST(TestCarrierPhaseFactorArm, EcefTnavENUJacobians) {
  const Pose3 ecef_T_nav = tokyo::ecefTnav();
  const Point3 leverArm(0.1, 0.0, -0.5);

  const auto factor = CarrierPhaseFactorArm(
      Key(0), Key(1), Key(2), sample::kPseudorange, sample::kSatPos, leverArm,
      ecef_T_nav, sample::kSatClkBias);

  Values values;
  values.insert(Key(0), Pose3(Rot3::RzRyRx(0.05, -0.03, 0.1),
                               Point3(10.0, 20.0, 5.0)));
  values.insert(Key(1), sample::kReceiverClock);
  values.insert(Key(2), kLambdaL1 * 1000.0);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-3, 1e-5);
}

// *************************************************************************
TEST(TestCarrierPhaseFactorArm, EcefTnavConsistency) {
  const Pose3 ecef_T_nav = tokyo::ecefTnav();
  const Pose3 nav_T_body(Rot3::RzRyRx(0.05, -0.03, 0.1),
                          Point3(10.0, 20.0, 5.0));
  const Pose3 ecef_T_body = ecef_T_nav.compose(nav_T_body);

  const Point3 leverArm(0.1, 0.0, -0.5);
  const double ambiguity_m = kLambdaL1 * 1000.0;

  const auto factorEcef = CarrierPhaseFactorArm(
      Key(0), Key(1), Key(2), sample::kPseudorange, sample::kSatPos, leverArm,
      sample::kSatClkBias);
  const auto factorNav = CarrierPhaseFactorArm(
      Key(0), Key(1), Key(2), sample::kPseudorange, sample::kSatPos, leverArm,
      ecef_T_nav, sample::kSatClkBias);

  const double errorEcef = factorEcef.evaluateError(
      ecef_T_body, sample::kReceiverClock, ambiguity_m)[0];
  const double errorNav = factorNav.evaluateError(
      nav_T_body, sample::kReceiverClock, ambiguity_m)[0];
  EXPECT_DOUBLES_EQUAL(errorEcef, errorNav, 1e-6);
}

// *************************************************************************
TEST(TestCarrierPhaseFactorArm, EcefTnavEquals) {
  const Point3 leverArm(0.1, 0.2, 0.3);
  const Pose3 ecef_T_nav = tokyo::ecefTnav();

  const auto f1 = CarrierPhaseFactorArm(
      1, 2, 3, 0.0, Point3::Zero(), leverArm, 0.0);
  const auto f2 = CarrierPhaseFactorArm(
      1, 2, 3, 0.0, Point3::Zero(), leverArm, ecef_T_nav, 0.0);

  CHECK(!f1.equals(f2));
  CHECK(f2.equals(f2));
}

// *************************************************************************
TEST(TestCarrierPhaseFactorArm, print) {
  const auto factor = CarrierPhaseFactorArm(
      Key(0), Key(1), Key(2), 0.0, Point3::Zero(), Point3(0.1, 0.2, 0.3),
      0.0);
  factor.print("test ");

  const auto factorNav = CarrierPhaseFactorArm(
      Key(0), Key(1), Key(2), 0.0, Point3::Zero(), Point3(0.1, 0.2, 0.3),
      Pose3::Identity(), 0.0);
  factorNav.print("test nav ");
}

// *************************************************************************
TEST(TestCarrierPhaseFactorArm, equals) {
  const Point3 leverArm(0.1, 0.2, 0.3);
  const auto f1 = CarrierPhaseFactorArm(
      1, 2, 3, 100.0, Point3(1, 2, 3), leverArm, 0.0);
  const auto f2 = CarrierPhaseFactorArm(
      1, 2, 3, 100.0, Point3(1, 2, 3), leverArm, 0.0);
  const auto f3 = CarrierPhaseFactorArm(
      1, 2, 3, 100.0, Point3(1, 2, 3), leverArm, 999.0);

  CHECK(f1.equals(f2));
  CHECK(!f1.equals(f3));  // different sat clock bias
}

// *************************************************************************
// DoubleDifferenceCarrierPhaseFactor tests
// *************************************************************************
TEST(TestDoubleDifferenceCarrierPhaseFactor, ZeroError) {
  // Build observations that give zero error at the true position.
  const double ambRef = 100.0;
  const double ambTarget = 200.0;

  const double rRovRef = geodistRange(dd::kSatRefRov, dd::kTruePos);
  const double rRovTarget = geodistRange(dd::kSatTargetRov, dd::kTruePos);
  const double rBaseRef = geodistRange(dd::kSatRefBase, dd::kBasePos);
  const double rBaseTarget = geodistRange(dd::kSatTargetBase, dd::kBasePos);

  const double ddModel = (rRovRef - rBaseRef) - (rRovTarget - rBaseTarget);
  // DD obs = ddModel + lam * (ambRef - ambTarget) => error = 0.
  const double ddObs = ddModel + kLambdaL1 * (ambRef - ambTarget);
  // Pick observations that recover this DD obs:
  //   (cpRovRef - cpBaseRef) - (cpRovTarget - cpBaseTarget) = ddObs
  const double cpRovRef = ddObs / 2.0 + 1000.0;
  const double cpBaseRef = -ddObs / 2.0 + 1000.0;
  const double cpRovTarget = 500.0;
  const double cpBaseTarget = 500.0;

  const auto factor = DoubleDifferenceCarrierPhaseFactor(
      Key(0), Key(1), Key(2),
      cpRovRef, cpBaseRef, cpRovTarget, cpBaseTarget,
      dd::kSatRefRov, dd::kSatTargetRov, dd::kSatRefBase, dd::kSatTargetBase,
      dd::kBasePos, kLambdaL1);

  const double error =
      factor.evaluateError(dd::kTruePos, ambRef, ambTarget)[0];
  EXPECT_DOUBLES_EQUAL(0.0, error, 1e-4);
}

// *************************************************************************
TEST(TestDoubleDifferenceCarrierPhaseFactor, Jacobians) {
  const auto factor = DoubleDifferenceCarrierPhaseFactor(
      Key(0), Key(1), Key(2),
      25000000.0, 24999500.0, 22000000.0, 21999800.0,
      dd::kSatRefRov, dd::kSatTargetRov, dd::kSatRefBase, dd::kSatTargetBase,
      dd::kBasePos, kLambdaL1);

  Values values;
  values.insert(Key(0), dd::kTruePos);
  values.insert(Key(1), 100.0);  // ambRef
  values.insert(Key(2), 200.0);  // ambTarget
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-3, 1e-5);
}

// *************************************************************************
TEST(TestDoubleDifferenceCarrierPhaseFactor, equals) {
  const auto f1 = DoubleDifferenceCarrierPhaseFactor(
      Key(0), Key(1), Key(2), 100.0, 99.0, 80.0, 79.0,
      dd::kSatRefRov, dd::kSatTargetRov, dd::kSatRefBase, dd::kSatTargetBase,
      dd::kBasePos, kLambdaL1);
  const auto f2 = DoubleDifferenceCarrierPhaseFactor(
      Key(0), Key(1), Key(2), 100.0, 99.0, 80.0, 79.0,
      dd::kSatRefRov, dd::kSatTargetRov, dd::kSatRefBase, dd::kSatTargetBase,
      dd::kBasePos, kLambdaL1);
  const auto f3 = DoubleDifferenceCarrierPhaseFactor(
      Key(0), Key(1), Key(2), 100.0, 99.0, 80.0, 79.0,
      dd::kSatRefRov, dd::kSatTargetRov, dd::kSatRefBase, dd::kSatTargetBase,
      dd::kBasePos, 0.25);  // different wavelength

  CHECK(f1.equals(f2));
  CHECK(!f1.equals(f3));
}

// *************************************************************************
TEST(TestDoubleDifferenceCarrierPhaseFactor, print) {
  const auto factor = DoubleDifferenceCarrierPhaseFactor(
      Key(0), Key(1), Key(2), 100.0, 99.0, 80.0, 79.0,
      dd::kSatRefRov, dd::kSatTargetRov, dd::kSatRefBase, dd::kSatTargetBase,
      dd::kBasePos, kLambdaL1);
  factor.print("test ");
}

// *************************************************************************
// DoubleDifferenceCarrierPhaseFactorArm tests
// *************************************************************************
TEST(TestDoubleDifferenceCarrierPhaseFactorArm, Jacobians) {
  const Point3 leverArm(0.1, 0.0, -0.5);
  const auto factor = DoubleDifferenceCarrierPhaseFactorArm(
      Key(0), Key(1), Key(2),
      25000000.0, 24999500.0, 22000000.0, 21999800.0,
      dd::kSatRefRov, dd::kSatTargetRov, dd::kSatRefBase, dd::kSatTargetBase,
      dd::kBasePos, kLambdaL1, leverArm);

  Values values;
  values.insert(Key(0), Pose3(Rot3::RzRyRx(0.1, 0.2, 0.3), dd::kTruePos));
  values.insert(Key(1), 100.0);
  values.insert(Key(2), 200.0);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-3, 1e-5);
}

// *************************************************************************
TEST(TestDoubleDifferenceCarrierPhaseFactorArm, EcefTnavJacobians) {
  const Pose3 ecef_T_nav = tokyo::ecefTnav();
  const Point3 leverArm(0.1, 0.0, -0.5);
  const auto factor = DoubleDifferenceCarrierPhaseFactorArm(
      Key(0), Key(1), Key(2),
      25000000.0, 24999500.0, 22000000.0, 21999800.0,
      dd::kSatRefRov, dd::kSatTargetRov, dd::kSatRefBase, dd::kSatTargetBase,
      dd::kBasePos, kLambdaL1, leverArm, ecef_T_nav);

  Values values;
  values.insert(Key(0), Pose3(Rot3::RzRyRx(0.05, -0.03, 0.1),
                               Point3(10.0, 20.0, 5.0)));
  values.insert(Key(1), 100.0);
  values.insert(Key(2), 200.0);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-3, 1e-5);
}

// *************************************************************************
TEST(TestDoubleDifferenceCarrierPhaseFactorArm, ZeroLeverArm) {
  const auto factorPt = DoubleDifferenceCarrierPhaseFactor(
      Key(0), Key(1), Key(2),
      25000000.0, 24999500.0, 22000000.0, 21999800.0,
      dd::kSatRefRov, dd::kSatTargetRov, dd::kSatRefBase, dd::kSatTargetBase,
      dd::kBasePos, kLambdaL1);
  const auto factorArm = DoubleDifferenceCarrierPhaseFactorArm(
      Key(0), Key(1), Key(2),
      25000000.0, 24999500.0, 22000000.0, 21999800.0,
      dd::kSatRefRov, dd::kSatTargetRov, dd::kSatRefBase, dd::kSatTargetBase,
      dd::kBasePos, kLambdaL1, Point3::Zero());

  const double errorPt = factorPt.evaluateError(dd::kTruePos, 100.0, 200.0)[0];
  const double errorArm = factorArm.evaluateError(
      Pose3(Rot3::Identity(), dd::kTruePos), 100.0, 200.0)[0];
  EXPECT_DOUBLES_EQUAL(errorPt, errorArm, 1e-9);
}

// *************************************************************************
TEST(TestDoubleDifferenceCarrierPhaseFactorArm, EcefTnavConsistency) {
  const Pose3 ecef_T_nav = tokyo::ecefTnav();
  const Pose3 nav_T_body(Rot3::RzRyRx(0.05, -0.03, 0.1),
                          Point3(10.0, 20.0, 5.0));
  const Pose3 ecef_T_body = ecef_T_nav.compose(nav_T_body);
  const Point3 leverArm(0.1, 0.0, -0.5);

  const auto factorEcef = DoubleDifferenceCarrierPhaseFactorArm(
      Key(0), Key(1), Key(2),
      25000000.0, 24999500.0, 22000000.0, 21999800.0,
      dd::kSatRefRov, dd::kSatTargetRov, dd::kSatRefBase, dd::kSatTargetBase,
      dd::kBasePos, kLambdaL1, leverArm);
  const auto factorNav = DoubleDifferenceCarrierPhaseFactorArm(
      Key(0), Key(1), Key(2),
      25000000.0, 24999500.0, 22000000.0, 21999800.0,
      dd::kSatRefRov, dd::kSatTargetRov, dd::kSatRefBase, dd::kSatTargetBase,
      dd::kBasePos, kLambdaL1, leverArm, ecef_T_nav);

  const double errorEcef =
      factorEcef.evaluateError(ecef_T_body, 100.0, 200.0)[0];
  const double errorNav =
      factorNav.evaluateError(nav_T_body, 100.0, 200.0)[0];
  EXPECT_DOUBLES_EQUAL(errorEcef, errorNav, 1e-6);
}

// *************************************************************************
TEST(TestDoubleDifferenceCarrierPhaseFactorArm, equals) {
  const Point3 leverArm(0.1, 0.0, -0.5);
  const Pose3 ecef_T_nav = tokyo::ecefTnav();

  const auto f1 = DoubleDifferenceCarrierPhaseFactorArm(
      Key(0), Key(1), Key(2), 100.0, 99.0, 80.0, 79.0,
      dd::kSatRefRov, dd::kSatTargetRov, dd::kSatRefBase, dd::kSatTargetBase,
      dd::kBasePos, kLambdaL1, leverArm);
  const auto f2 = DoubleDifferenceCarrierPhaseFactorArm(
      Key(0), Key(1), Key(2), 100.0, 99.0, 80.0, 79.0,
      dd::kSatRefRov, dd::kSatTargetRov, dd::kSatRefBase, dd::kSatTargetBase,
      dd::kBasePos, kLambdaL1, leverArm, ecef_T_nav);

  CHECK(!f1.equals(f2));
  CHECK(f1.equals(f1));
  CHECK(f2.equals(f2));
}

// *************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
// *************************************************************************

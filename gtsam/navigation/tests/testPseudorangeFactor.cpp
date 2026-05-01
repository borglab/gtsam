/**
 * @file    testPseudorangeFactor.cpp
 * @brief   Unit test for PseudorangeFactor
 * @author  Sammy Guo
 * @date   January 18, 2026
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/navigation/PseudorangeFactor.h>
#include <gtsam/nonlinear/factorTesting.h>

#include <cmath>

using namespace gtsam;

/// Test helper: compute ECEF-to-ENU transform from geodetic origin (WGS84).
static Pose3 makeEcefTnav(double lat_deg, double lon_deg, double h) {
  constexpr double deg2rad = M_PI / 180.0;
  const double lat = lat_deg * deg2rad;
  const double lon = lon_deg * deg2rad;
  const double slat = std::sin(lat), clat = std::cos(lat);
  const double slon = std::sin(lon), clon = std::cos(lon);

  Matrix3 R;
  R.col(0) = Vector3(-slon, clon, 0.0);
  R.col(1) = Vector3(-slat * clon, -slat * slon, clat);
  R.col(2) = Vector3(clat * clon, clat * slon, slat);

  constexpr double a = 6378137.0;
  constexpr double f = 1.0 / 298.257223563;
  const double e2 = 2.0 * f - f * f;
  const double N = a / std::sqrt(1.0 - e2 * slat * slat);
  const Point3 t((N + h) * clat * clon,
                 (N + h) * clat * slon,
                 (N * (1.0 - e2) + h) * slat);
  return Pose3(Rot3(R), t);
}

// *************************************************************************
TEST(TestPseudorangeFactor, Constructor) {
  const auto factor =
      PseudorangeFactor(Key(0), Key(1), 0.0, Point3::Zero(), 0.0,
                        noiseModel::Isotropic::Sigma(1, 1.0));

  Matrix Hpos, Hbias;
  const double error =
      factor.evaluateError(Point3::Zero(), 0.0, Hpos, Hbias)[0];
  EXPECT_DOUBLES_EQUAL(0.0, error, 1e-9);

  // Derivatives are technically undefined if the receiver and satellite
  // positions are the same (hopefully that's never the case in reality). But
  // for all intents and purposes, zero-valued derivatives can substitute for
  // undefined gradient at that singularity. So make sure this corner-case does
  // not numerically explode:
  EXPECT(!Hpos.array().isNaN().any());
  EXPECT(!Hbias.array().isNaN().any());
  EXPECT_DOUBLES_EQUAL(Hpos.norm(), 0.0, 1e-9);
  // Clock bias derivative should always be speed-of-light in vacuum:
  EXPECT_DOUBLES_EQUAL(Hbias(0, 0), 299792458.0, 1e-9);
}

// *************************************************************************
TEST(TestPseudorangeFactor, Jacobians1) {
  // Synthetic example with exact error/derivatives:
  const auto factor = PseudorangeFactor(
      Key(0), Key(1),  // Receiver position and clock bias keys.
      4.0,             // Measured pseudorange.
      // Satellite position:
      Vector3(0.0, 0.0, 3.0),
      0.0  // Sat clock drift bias.
  );
  const double error = factor.evaluateError(Vector3::Zero(), 0.0)[0];
  EXPECT_DOUBLES_EQUAL(-1.0, error, 1e-6);

  Values values;
  values.insert(Key(0), Vector3(1.0, 2.0, 3.0));
  values.insert(Key(1), 0.0);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-3, 1e-5);
}

// *************************************************************************
TEST(TestPseudorangeFactor, Jacobians2) {
  // Example values borrowed from `SinglePointPositioningExample.ipynb`:
  const auto factor = PseudorangeFactor(
      Key(0), Key(1),  // Receiver position and clock bias keys.
      24874028.989,    // Measured pseudorange.
      // Satellite position:
      Vector3(-5824269.46342, -22935011.26952, -12195522.22428),
      -0.00022743876852667193  // Sat clock drift bias.
  );

  Values values;
  values.insert(
      Key(0), Vector3(-2684418.91084688, -4293361.08683296, 3865365.45451951));
  values.insert(Key(1), 5.377885093511699e-07);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-3, 1e-5);
}

// *************************************************************************
TEST(TestPseudorangeFactor, print) {
  // Just make sure `print()` doesn't throw errors
  // since there's no elegant way to check stdout.
  const auto factor = PseudorangeFactor();
  factor.print();
}

// *************************************************************************
TEST(TestPseudorangeFactor, equals) {
  const auto factor1 = PseudorangeFactor();
  const auto factor2 = PseudorangeFactor(1, 2, 0.0, Point3::Zero(), 0.0);
  const auto factor3 =
      PseudorangeFactor(1, 2, 10.0, Point3(1.0, 2.0, 3.0), 20.0);

  CHECK(factor1.equals(factor1));
  CHECK(factor2.equals(factor2));
  CHECK(!factor1.equals(factor2));
  CHECK(factor2.equals(factor3, 1e99));

  // Test print:
  factor2.print("factor2");
}

// *************************************************************************
TEST(TestDifferentialPseudorangeFactor, Constructor) {
  const auto factor =
      DifferentialPseudorangeFactor(Key(0), Key(1), Key(2), 0.0, Point3::Zero(),
                                    0.0, noiseModel::Isotropic::Sigma(1, 1.0));

  Matrix Hpos, Hbias, Hcorrection;
  const double error = factor.evaluateError(Point3::Zero(), 0.0, 0.0, Hpos,
                                            Hbias, Hcorrection)[0];
  EXPECT_DOUBLES_EQUAL(0.0, error, 1e-9);

  // Derivatives are technically undefined if the receiver and satellite
  // positions are the same (hopefully that's never the case in reality). But
  // for all intents and purposes, zero-valued derivatives can substitute for
  // undefined gradient at that singularity. So make sure this corner-case does
  // not numerically explode:
  EXPECT(!Hpos.array().isNaN().any());
  EXPECT(!Hbias.array().isNaN().any());
  EXPECT(!Hcorrection.array().isNaN().any());
  EXPECT_DOUBLES_EQUAL(Hpos.norm(), 0.0, 1e-9);
  // Clock bias derivative should always be speed-of-light in vacuum:
  EXPECT_DOUBLES_EQUAL(Hbias(0, 0), 299792458.0, 1e-9);
  // Correction derivative should be constant -1:
  EXPECT_DOUBLES_EQUAL(Hcorrection(0, 0), -1.0, 1e-9);
}

// *************************************************************************
TEST(TestDifferentialPseudorangeFactor, Jacobians) {
  // Synthetic example with exact error/derivatives:
  const auto factor = DifferentialPseudorangeFactor(
      Key(0), Key(1),  // Receiver position and clock bias keys.
      Key(2),          // Differential correction keys.
      4.0,             // Measured pseudorange.
      // Satellite position:
      Vector3(0.0, 0.0, 3.0),
      0.0  // Sat clock drift bias.
  );

  // Zero differential correction case:
  {
    const double error = factor.evaluateError(Vector3::Zero(), 0.0, 0.0)[0];
    EXPECT_DOUBLES_EQUAL(-1.0, error, 1e-6);
  }

  // Nontrivial differential correction:
  {
    const double error = factor.evaluateError(Vector3::Zero(), 0.0, 123.0)[0];
    EXPECT_DOUBLES_EQUAL(-124.0, error, 1e-6);
  }

  Values values;
  values.insert(Key(0), Vector3(1.0, 2.0, 3.0));
  values.insert(Key(1), 0.0);
  values.insert(Key(2), 0.0);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-3, 1e-5);
}

// *************************************************************************
TEST(TestDifferentialPseudorangeFactor, print) {
  // Just make sure `print()` doesn't throw errors
  // since there's no elegant way to check stdout.
  const auto factor = DifferentialPseudorangeFactor();
  factor.print();
}

// *************************************************************************
TEST(TestDifferentialPseudorangeFactor, equals) {
  const auto factor1 = DifferentialPseudorangeFactor();
  const auto factor2 =
      DifferentialPseudorangeFactor(1, 2, 3, 0.0, Point3::Zero(), 0.0);
  const auto factorCorr =
      DifferentialPseudorangeFactor(1, 2, 7, 0.0, Point3::Zero(), 0.0);
  const auto factor3 =
      DifferentialPseudorangeFactor(1, 2, 3, 10.0, Point3(1.0, 2.0, 3.0), 20.0);

  CHECK(factor1.equals(factor1));
  CHECK(factor2.equals(factor2));
  CHECK(!factor1.equals(factor2));
  CHECK(factor2.equals(factor3, 1e99));
  CHECK(!factor2.equals(factorCorr));

  // Test print:
  factor2.print("factor2");
}

// *************************************************************************
TEST(TestPseudorangeFactorArm, Constructor) {
  const Point3 leverArm(0.1, 0.2, 0.3);
  const auto factor = PseudorangeFactorArm(
      Key(0), Key(1), 0.0, Point3::Zero(), leverArm, 0.0,
      noiseModel::Isotropic::Sigma(1, 1.0));

  Matrix Hpose, Hbias;
  const double error =
      factor.evaluateError(Pose3::Identity(), 0.0, Hpose, Hbias)[0];

  // With identity pose and zero satellite position, the antenna is at the
  // lever arm position, so range = ||leverArm|| = sqrt(0.01+0.04+0.09)
  const double expectedRange = leverArm.norm();
  EXPECT_DOUBLES_EQUAL(expectedRange, error, 1e-9);

  // Check Jacobians are not NaN:
  EXPECT(!Hpose.array().isNaN().any());
  EXPECT(!Hbias.array().isNaN().any());
  // Clock bias derivative should always be speed-of-light in vacuum:
  EXPECT_DOUBLES_EQUAL(Hbias(0, 0), 299792458.0, 1e-9);
}

// *************************************************************************
TEST(TestPseudorangeFactorArm, Jacobians1) {
  const Point3 leverArm(0.5, -0.3, 1.0);
  const auto factor = PseudorangeFactorArm(
      Key(0), Key(1),  // Receiver pose and clock bias keys.
      4.0,             // Measured pseudorange.
      Vector3(0.0, 0.0, 3.0),  // Satellite position.
      leverArm,
      0.0  // Sat clock drift bias.
  );

  Values values;
  values.insert(Key(0), Pose3(Rot3::RzRyRx(0.1, 0.2, 0.3),
                               Point3(1.0, 2.0, 3.0)));
  values.insert(Key(1), 0.0);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-5, 1e-5);
}

// *************************************************************************
TEST(TestPseudorangeFactorArm, Jacobians2) {
  // Example values adapted from SinglePointPositioningExample:
  const Point3 leverArm(0.1, 0.0, -0.5);
  const auto factor = PseudorangeFactorArm(
      Key(0), Key(1),  // Receiver pose and clock bias keys.
      24874028.989,    // Measured pseudorange.
      Vector3(-5824269.46342, -22935011.26952, -12195522.22428),
      leverArm,
      -0.00022743876852667193  // Sat clock drift bias.
  );

  Values values;
  values.insert(Key(0),
                Pose3(Rot3::RzRyRx(0.05, -0.03, 0.1),
                      Point3(-2684418.91084688, -4293361.08683296,
                             3865365.45451951)));
  values.insert(Key(1), 5.377885093511699e-07);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-3, 1e-5);
}

// *************************************************************************
TEST(TestPseudorangeFactorArm, LeverArmZeroError) {
  // If the body position is set so that antenna_pos = body_pos + ecef_R_body * leverArm
  // gives the correct range, the error should be zero.
  // This mirrors the GPSFactorArm test pattern.
  const Point3 leverArm(0.5, -0.3, 1.0);
  const Rot3 ecef_R_body = Rot3::RzRyRx(0.15, -0.30, 0.45);
  const Point3 satPos(1000.0, 2000.0, 3000.0);

  // Choose antenna position, then back-compute body position:
  const Point3 antennaPos(10.0, 20.0, 30.0);
  const Point3 bodyPos = antennaPos - ecef_R_body.matrix() * leverArm;
  const Pose3 ecef_T_body(ecef_R_body, bodyPos);

  // The true range from antenna to satellite:
  const double trueRange = (antennaPos - satPos).norm();

  // Set pseudorange = trueRange + c * (clockBias - satClkBias)
  // so that the error is exactly zero:
  const double clockBias = 1e-7;
  const double satClkBias = 2e-8;
  const double pseudorange = trueRange + 299792458.0 * (clockBias - satClkBias);

  const auto factor = PseudorangeFactorArm(
      Key(0), Key(1), pseudorange, satPos, leverArm, satClkBias);

  const double error = factor.evaluateError(ecef_T_body, clockBias)[0];
  EXPECT_DOUBLES_EQUAL(0.0, error, 1e-6);
}

// *************************************************************************
TEST(TestPseudorangeFactorArm, ZeroLeverArm) {
  // With zero lever arm, PseudorangeFactorArm should produce the same error
  // as PseudorangeFactor at the same position:
  const Point3 satPos(100.0, 200.0, 300.0);
  const Point3 receiverPos(1.0, 2.0, 3.0);
  const double pseudorange = 350.0;
  const double clockBias = 1e-8;
  const double satClkBias = 1e-9;

  const auto factorPoint = PseudorangeFactor(
      Key(0), Key(1), pseudorange, satPos, satClkBias);
  const auto factorArm = PseudorangeFactorArm(
      Key(0), Key(1), pseudorange, satPos, Point3::Zero(), satClkBias);

  const double errorPoint =
      factorPoint.evaluateError(receiverPos, clockBias)[0];
  const double errorArm =
      factorArm.evaluateError(Pose3(Rot3::Identity(), receiverPos),
                              clockBias)[0];
  EXPECT_DOUBLES_EQUAL(errorPoint, errorArm, 1e-9);
}

// *************************************************************************
TEST(TestPseudorangeFactorArm, print) {
  const auto factor = PseudorangeFactorArm();
  factor.print();
}

// *************************************************************************
TEST(TestPseudorangeFactorArm, equals) {
  const Point3 leverArm(0.1, 0.2, 0.3);
  const auto factor1 = PseudorangeFactorArm();
  const auto factor2 =
      PseudorangeFactorArm(1, 2, 0.0, Point3::Zero(), leverArm, 0.0);
  const auto factor3 =
      PseudorangeFactorArm(1, 2, 10.0, Point3(1.0, 2.0, 3.0), leverArm, 20.0);

  CHECK(factor1.equals(factor1));
  CHECK(factor2.equals(factor2));
  CHECK(!factor1.equals(factor2));
  CHECK(factor2.equals(factor3, 1e99));

  // Different lever arm should not be equal:
  const auto factor4 =
      PseudorangeFactorArm(1, 2, 0.0, Point3::Zero(),
                           Point3(9.0, 8.0, 7.0), 0.0);
  CHECK(!factor2.equals(factor4));

  factor2.print("factor2");
}

// *************************************************************************
TEST(TestDifferentialPseudorangeFactorArm, Constructor) {
  const Point3 leverArm(0.1, 0.2, 0.3);
  const auto factor = DifferentialPseudorangeFactorArm(
      Key(0), Key(1), Key(2), 0.0, Point3::Zero(), leverArm, 0.0,
      noiseModel::Isotropic::Sigma(1, 1.0));

  Matrix Hpose, Hbias, Hcorrection;
  const double error = factor.evaluateError(Pose3::Identity(), 0.0, 0.0, Hpose,
                                            Hbias, Hcorrection)[0];

  const double expectedRange = leverArm.norm();
  EXPECT_DOUBLES_EQUAL(expectedRange, error, 1e-9);

  EXPECT(!Hpose.array().isNaN().any());
  EXPECT(!Hbias.array().isNaN().any());
  EXPECT(!Hcorrection.array().isNaN().any());
  EXPECT_DOUBLES_EQUAL(Hbias(0, 0), 299792458.0, 1e-9);
  EXPECT_DOUBLES_EQUAL(Hcorrection(0, 0), -1.0, 1e-9);
}

// *************************************************************************
TEST(TestDifferentialPseudorangeFactorArm, Jacobians1) {
  const Point3 leverArm(0.5, -0.3, 1.0);
  const auto factor = DifferentialPseudorangeFactorArm(
      Key(0), Key(1), Key(2),
      4.0,
      Vector3(0.0, 0.0, 3.0),
      leverArm,
      0.0
  );

  Values values;
  values.insert(Key(0), Pose3(Rot3::RzRyRx(0.1, 0.2, 0.3),
                               Point3(1.0, 2.0, 3.0)));
  values.insert(Key(1), 0.0);
  values.insert(Key(2), 0.0);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-5, 1e-5);
}

// *************************************************************************
TEST(TestDifferentialPseudorangeFactorArm, Jacobians2) {
  const Point3 leverArm(0.1, 0.0, -0.5);
  const auto factor = DifferentialPseudorangeFactorArm(
      Key(0), Key(1), Key(2),
      24874028.989,
      Vector3(-5824269.46342, -22935011.26952, -12195522.22428),
      leverArm,
      -0.00022743876852667193
  );

  Values values;
  values.insert(Key(0),
                Pose3(Rot3::RzRyRx(0.05, -0.03, 0.1),
                      Point3(-2684418.91084688, -4293361.08683296,
                             3865365.45451951)));
  values.insert(Key(1), 5.377885093511699e-07);
  values.insert(Key(2), 10.0);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-3, 1e-5);
}

// *************************************************************************
TEST(TestDifferentialPseudorangeFactorArm, LeverArmZeroError) {
  const Point3 leverArm(0.5, -0.3, 1.0);
  const Rot3 ecef_R_body = Rot3::RzRyRx(0.15, -0.30, 0.45);
  const Point3 satPos(1000.0, 2000.0, 3000.0);

  const Point3 antennaPos(10.0, 20.0, 30.0);
  const Point3 bodyPos = antennaPos - ecef_R_body.matrix() * leverArm;
  const Pose3 ecef_T_body(ecef_R_body, bodyPos);

  const double trueRange = (antennaPos - satPos).norm();
  const double clockBias = 1e-7;
  const double satClkBias = 2e-8;
  const double correction = 5.0;
  // pseudorange = trueRange + c*(clockBias - satClkBias) - correction
  // so that error = 0
  const double pseudorange =
      trueRange + 299792458.0 * (clockBias - satClkBias) - correction;

  const auto factor = DifferentialPseudorangeFactorArm(
      Key(0), Key(1), Key(2), pseudorange, satPos, leverArm, satClkBias);

  const double error =
      factor.evaluateError(ecef_T_body, clockBias, correction)[0];
  EXPECT_DOUBLES_EQUAL(0.0, error, 1e-6);
}

// *************************************************************************
TEST(TestDifferentialPseudorangeFactorArm, ZeroLeverArm) {
  // With zero lever arm, should produce the same error as
  // DifferentialPseudorangeFactor at the same position:
  const Point3 satPos(100.0, 200.0, 300.0);
  const Point3 receiverPos(1.0, 2.0, 3.0);
  const double pseudorange = 350.0;
  const double clockBias = 1e-8;
  const double satClkBias = 1e-9;
  const double correction = 5.0;

  const auto factorPoint = DifferentialPseudorangeFactor(
      Key(0), Key(1), Key(2), pseudorange, satPos, satClkBias);
  const auto factorArm = DifferentialPseudorangeFactorArm(
      Key(0), Key(1), Key(2), pseudorange, satPos, Point3::Zero(), satClkBias);

  const double errorPoint =
      factorPoint.evaluateError(receiverPos, clockBias, correction)[0];
  const double errorArm =
      factorArm.evaluateError(Pose3(Rot3::Identity(), receiverPos),
                              clockBias, correction)[0];
  EXPECT_DOUBLES_EQUAL(errorPoint, errorArm, 1e-9);
}

// *************************************************************************
TEST(TestDifferentialPseudorangeFactorArm, print) {
  const auto factor = DifferentialPseudorangeFactorArm();
  factor.print();
}

// *************************************************************************
TEST(TestDifferentialPseudorangeFactorArm, equals) {
  const Point3 leverArm(0.1, 0.2, 0.3);
  const auto factor1 = DifferentialPseudorangeFactorArm();
  const auto factor2 = DifferentialPseudorangeFactorArm(
      1, 2, 3, 0.0, Point3::Zero(), leverArm, 0.0);
  const auto factor3 = DifferentialPseudorangeFactorArm(
      1, 2, 3, 10.0, Point3(1.0, 2.0, 3.0), leverArm, 20.0);

  CHECK(factor1.equals(factor1));
  CHECK(factor2.equals(factor2));
  CHECK(!factor1.equals(factor2));
  CHECK(factor2.equals(factor3, 1e99));

  const auto factor4 = DifferentialPseudorangeFactorArm(
      1, 2, 3, 0.0, Point3::Zero(), Point3(9.0, 8.0, 7.0), 0.0);
  CHECK(!factor2.equals(factor4));

  factor2.print("factor2");
}

// *************************************************************************
// ecef_T_nav tests for PseudorangeFactorArm
// *************************************************************************
TEST(TestPseudorangeFactorArm, EcefTnavIdentity) {
  // With ecef_T_nav = identity, should produce the same result as without it.
  const Point3 leverArm(0.5, -0.3, 1.0);
  const Point3 satPos(0.0, 0.0, 3.0);
  const double pseudorange = 4.0;

  const auto factorEcef = PseudorangeFactorArm(
      Key(0), Key(1), pseudorange, satPos, leverArm, 0.0);
  const auto factorNav = PseudorangeFactorArm(
      Key(0), Key(1), pseudorange, satPos, leverArm, Pose3::Identity(), 0.0);

  const Pose3 pose(Rot3::RzRyRx(0.1, 0.2, 0.3), Point3(1.0, 2.0, 3.0));
  const double clockBias = 1e-8;

  const double errorEcef = factorEcef.evaluateError(pose, clockBias)[0];
  const double errorNav = factorNav.evaluateError(pose, clockBias)[0];
  EXPECT_DOUBLES_EQUAL(errorEcef, errorNav, 1e-9);
}

// *************************************************************************
TEST(TestPseudorangeFactorArm, EcefTnavIdentityJacobians) {
  const Point3 leverArm(0.5, -0.3, 1.0);
  const auto factor = PseudorangeFactorArm(
      Key(0), Key(1), 4.0, Vector3(0.0, 0.0, 3.0), leverArm,
      Pose3::Identity(), 0.0);

  Values values;
  values.insert(Key(0), Pose3(Rot3::RzRyRx(0.1, 0.2, 0.3),
                               Point3(1.0, 2.0, 3.0)));
  values.insert(Key(1), 0.0);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-5, 1e-5);
}

// *************************************************************************
TEST(TestPseudorangeFactorArm, EcefTnavENUJacobians) {
  // Use a realistic ENU origin near Tokyo (lat=35.578, lon=139.749, h=80)
  const Pose3 ecef_T_nav = makeEcefTnav(35.578, 139.749, 80.0);

  const Point3 leverArm(0.1, 0.0, -0.5);
  const Point3 satPos(-5824269.46342, -22935011.26952, -12195522.22428);
  const auto factor = PseudorangeFactorArm(
      Key(0), Key(1), 24874028.989, satPos, leverArm, ecef_T_nav,
      -0.00022743876852667193);

  // nav_T_body in ENU frame (small offsets from origin)
  Values values;
  values.insert(Key(0), Pose3(Rot3::RzRyRx(0.05, -0.03, 0.1),
                               Point3(10.0, 20.0, 5.0)));
  values.insert(Key(1), 5.377885093511699e-07);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-3, 1e-5);
}

// *************************************************************************
TEST(TestPseudorangeFactorArm, EcefTnavConsistency) {
  // Verify: factor with ecef_T_nav and nav_T_body should give the same error
  // as factor without ecef_T_nav using ecef_T_body = ecef_T_nav * nav_T_body.
  const Pose3 ecef_T_nav = makeEcefTnav(35.578, 139.749, 80.0);
  const Pose3 nav_T_body(Rot3::RzRyRx(0.05, -0.03, 0.1),
                          Point3(10.0, 20.0, 5.0));
  const Pose3 ecef_T_body = ecef_T_nav.compose(nav_T_body);

  const Point3 leverArm(0.1, 0.0, -0.5);
  const Point3 satPos(-5824269.46342, -22935011.26952, -12195522.22428);
  const double pseudorange = 24874028.989;
  const double satClkBias = -0.00022743876852667193;
  const double clockBias = 5.377885093511699e-07;

  const auto factorEcef = PseudorangeFactorArm(
      Key(0), Key(1), pseudorange, satPos, leverArm, satClkBias);
  const auto factorNav = PseudorangeFactorArm(
      Key(0), Key(1), pseudorange, satPos, leverArm, ecef_T_nav, satClkBias);

  const double errorEcef = factorEcef.evaluateError(ecef_T_body, clockBias)[0];
  const double errorNav = factorNav.evaluateError(nav_T_body, clockBias)[0];
  EXPECT_DOUBLES_EQUAL(errorEcef, errorNav, 1e-6);
}

// *************************************************************************
TEST(TestPseudorangeFactorArm, EcefTnavEquals) {
  const Point3 leverArm(0.1, 0.2, 0.3);
  const Pose3 ecef_T_nav = makeEcefTnav(35.578, 139.749, 80.0);

  const auto factor1 = PseudorangeFactorArm(
      1, 2, 0.0, Point3::Zero(), leverArm, 0.0);
  const auto factor2 = PseudorangeFactorArm(
      1, 2, 0.0, Point3::Zero(), leverArm, ecef_T_nav, 0.0);

  // Different ecef_T_nav should not be equal
  CHECK(!factor1.equals(factor2));
  CHECK(factor2.equals(factor2));
}

// *************************************************************************
// ecef_T_nav tests for DifferentialPseudorangeFactorArm
// *************************************************************************
TEST(TestDifferentialPseudorangeFactorArm, EcefTnavIdentity) {
  const Point3 leverArm(0.5, -0.3, 1.0);
  const Point3 satPos(0.0, 0.0, 3.0);
  const double pseudorange = 4.0;

  const auto factorEcef = DifferentialPseudorangeFactorArm(
      Key(0), Key(1), Key(2), pseudorange, satPos, leverArm, 0.0);
  const auto factorNav = DifferentialPseudorangeFactorArm(
      Key(0), Key(1), Key(2), pseudorange, satPos, leverArm,
      Pose3::Identity(), 0.0);

  const Pose3 pose(Rot3::RzRyRx(0.1, 0.2, 0.3), Point3(1.0, 2.0, 3.0));
  const double clockBias = 1e-8;
  const double correction = 5.0;

  const double errorEcef =
      factorEcef.evaluateError(pose, clockBias, correction)[0];
  const double errorNav =
      factorNav.evaluateError(pose, clockBias, correction)[0];
  EXPECT_DOUBLES_EQUAL(errorEcef, errorNav, 1e-9);
}

// *************************************************************************
TEST(TestDifferentialPseudorangeFactorArm, EcefTnavENUJacobians) {
  const Pose3 ecef_T_nav = makeEcefTnav(35.578, 139.749, 80.0);
  const Point3 leverArm(0.1, 0.0, -0.5);
  const Point3 satPos(-5824269.46342, -22935011.26952, -12195522.22428);

  const auto factor = DifferentialPseudorangeFactorArm(
      Key(0), Key(1), Key(2), 24874028.989, satPos, leverArm, ecef_T_nav,
      -0.00022743876852667193);

  Values values;
  values.insert(Key(0), Pose3(Rot3::RzRyRx(0.05, -0.03, 0.1),
                               Point3(10.0, 20.0, 5.0)));
  values.insert(Key(1), 5.377885093511699e-07);
  values.insert(Key(2), 10.0);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-3, 1e-5);
}

// *************************************************************************
TEST(TestDifferentialPseudorangeFactorArm, EcefTnavConsistency) {
  const Pose3 ecef_T_nav = makeEcefTnav(35.578, 139.749, 80.0);
  const Pose3 nav_T_body(Rot3::RzRyRx(0.05, -0.03, 0.1),
                          Point3(10.0, 20.0, 5.0));
  const Pose3 ecef_T_body = ecef_T_nav.compose(nav_T_body);

  const Point3 leverArm(0.1, 0.0, -0.5);
  const Point3 satPos(-5824269.46342, -22935011.26952, -12195522.22428);
  const double pseudorange = 24874028.989;
  const double satClkBias = -0.00022743876852667193;
  const double clockBias = 5.377885093511699e-07;
  const double correction = 10.0;

  const auto factorEcef = DifferentialPseudorangeFactorArm(
      Key(0), Key(1), Key(2), pseudorange, satPos, leverArm, satClkBias);
  const auto factorNav = DifferentialPseudorangeFactorArm(
      Key(0), Key(1), Key(2), pseudorange, satPos, leverArm, ecef_T_nav,
      satClkBias);

  const double errorEcef =
      factorEcef.evaluateError(ecef_T_body, clockBias, correction)[0];
  const double errorNav =
      factorNav.evaluateError(nav_T_body, clockBias, correction)[0];
  EXPECT_DOUBLES_EQUAL(errorEcef, errorNav, 1e-6);
}

// *************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
// *************************************************************************
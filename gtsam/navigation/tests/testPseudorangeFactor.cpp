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

using namespace gtsam;

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
  EXPECT_DOUBLES_EQUAL(Hbias.norm(), 299792458.0, 1e-9);
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
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
// *************************************************************************
/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   testSelfCalibrationFactor.cpp
 *  @brief  Unit tests for SelfCalibrationFactor (closed-form focal-length self-calibration).
 *  @author Kathir Gounder
 */

#include <gtsam/sfm/SelfCalibrationFactor.h>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/nonlinear/factorTesting.h>

#include <CppUnitLite/TestHarness.h>

using namespace gtsam;

namespace {

// Test fixture: two cameras with EQUAL true focal length and distinct principal
// points, related by a generic pose (15 deg about a tilted axis, with a
// translation whose components are all nonzero). The pixel fundamental matrix F
// (convention x_j^T F x_i = 0) is rank-2 and non-degenerate: its second singular
// value sits cleanly between the largest and the (numerically zero) smallest, so
// the Fetzer residual is well-conditioned across the focals tested here — the
// rank-deficiency pole (di/dj -> 0) is never approached.
const Key kKeyI(1), kKeyJ(2);
const double kFocalTrue = 1000.0;
const Vector2 kPpI(640.0, 480.0);
const Vector2 kPpJ(600.0, 512.0);

Matrix3 fundamentalMatrix() {
  return Matrix3{
      {6.542596836638142e-07, 2.152812306321877e-06, -0.00010645321401584966},
      {-1.6601544719750939e-06, 5.97568997068656e-07, 0.003719199236179684},
      {-0.0015826359339192358, -0.0039029900113792815, 1.0}};
}

SelfCalibrationFactor makeFactor() {
  return SelfCalibrationFactor(kKeyI, kKeyJ, fundamentalMatrix(), kPpI, kPpJ,
                      noiseModel::Isotropic::Sigma(2, 1.0));
}

}  // namespace

/* ************************************************************************* */
TEST(SelfCalibrationFactor, Constructor) {
  SelfCalibrationFactor factor = makeFactor();
  LONGS_EQUAL(2, (long)factor.size());  // binary factor (fi, fj)
  LONGS_EQUAL(2, (long)factor.dim());   // 2-dimensional residual
}

/* ************************************************************************* */
TEST(SelfCalibrationFactor, ZeroError) {
  // At the true (equal) focals the fundamental matrix is exactly consistent, so
  // the closed-form focal residual vanishes.
  SelfCalibrationFactor factor = makeFactor();
  double fi = kFocalTrue, fj = kFocalTrue;
  Vector actual = factor.evaluateError(fi, fj);
  EXPECT(assert_equal(Vector::Zero(2), actual, 1e-9));
}

/* ************************************************************************* */
TEST(SelfCalibrationFactor, NonZeroError) {
  // Away from the true focals the residual is nonzero and finite (well clear of
  // the rank-deficiency pole). Pinned to guard the residual formula.
  SelfCalibrationFactor factor = makeFactor();
  double fi = 1050.0, fj = 965.0;
  Vector actual = factor.evaluateError(fi, fj);
  Vector expected{{0.09297405130946702, -0.0738601361584992}};
  EXPECT(assert_equal(expected, actual, 1e-9));
}

/* ************************************************************************* */
TEST(SelfCalibrationFactor, Jacobian) {
  // Analytic Jacobians must match numerical differentiation of evaluateError,
  // evaluated away from the true focals where the map is smooth.
  SelfCalibrationFactor factor = makeFactor();
  double fi = 1050.0, fj = 965.0;

  Matrix H1, H2;
  factor.evaluateError(fi, fj, &H1, &H2);

  Matrix numericalH1 = numericalDerivative21<Vector, double, double>(
      [&factor](const double& a, const double& b) {
        return factor.evaluateError(a, b);
      },
      fi, fj, 1e-5);
  Matrix numericalH2 = numericalDerivative22<Vector, double, double>(
      [&factor](const double& a, const double& b) {
        return factor.evaluateError(a, b);
      },
      fi, fj, 1e-5);

  EXPECT(assert_equal(numericalH1, H1, 1e-5));
  EXPECT(assert_equal(numericalH2, H2, 1e-5));
}

/* ************************************************************************* */
TEST(SelfCalibrationFactor, FactorJacobians) {
  // Whole-factor (noise-model-aware) linearization vs finite differences.
  SelfCalibrationFactor factor = makeFactor();
  Values values;
  values.insert(kKeyI, 1050.0);
  values.insert(kKeyJ, 965.0);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-5, 1e-5);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

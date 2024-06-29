/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file  testCal3DS2_k3.cpp
 * @brief Unit tests for Cal3DS2_k3 calibration model.
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Cal3DS2_k3.h>

using namespace gtsam;

GTSAM_CONCEPT_TESTABLE_INST(Cal3DS2_k3)
GTSAM_CONCEPT_MANIFOLD_INST(Cal3DS2_k3)

static Cal3DS2_k3 K(500, 100, 0.1, 320, 240, 1e-3, 2.0 * 1e-3, 3.0 * 1e-3, 4.0 * 1e-3, 5.0 * 1e-3);
static Point2 p(2, 3);

/* ************************************************************************* */
TEST(Cal3DS2_k3, Uncalibrate) {
  Vector k = K.k();
  double r = p.x() * p.x() + p.y() * p.y();
  double g = 1 + k[0] * r + k[1] * r * r + k[4] * r * r * r;
  double tx = 2 * k[2] * p.x() * p.y() + k[3] * (r + 2 * p.x() * p.x());
  double ty = k[2] * (r + 2 * p.y() * p.y()) + 2 * k[3] * p.x() * p.y();
  Vector v_hat = (Vector(3) << g * p.x() + tx, g * p.y() + ty, 1.0).finished();
  Vector v_i = K.K() * v_hat;
  Point2 p_i(v_i(0) / v_i(2), v_i(1) / v_i(2));
  Point2 q = K.uncalibrate(p);
  CHECK(assert_equal(q, p_i));
}

TEST(Cal3DS2_k3, Calibrate) {
  Point2 pn(0.5, 0.5);
  Point2 pi = K.uncalibrate(pn);
  Point2 pn_hat = K.calibrate(pi);
  CHECK(traits<Point2>::Equals(pn, pn_hat, 1e-5));
}

Point2 uncalibrate_(const Cal3DS2_k3& k, const Point2& pt) { return k.uncalibrate(pt); }

/* ************************************************************************* */
TEST(Cal3DS2_k3, Duncalibrate1) {
  Matrix computed;
  K.uncalibrate(p, computed, {});
  Matrix numerical = numericalDerivative21(uncalibrate_, K, p, 1e-6);
  CHECK(assert_equal(numerical, computed, 1e-5));
  Matrix separate = K.D2d_calibration(p);
  CHECK(assert_equal(numerical, separate, 1e-5));
}

/* ************************************************************************* */
TEST(Cal3DS2_k3, Duncalibrate2) {
  Matrix computed;
  K.uncalibrate(p, {}, computed);
  Matrix numerical = numericalDerivative22(uncalibrate_, K, p, 1e-6);
  CHECK(assert_equal(numerical, computed, 1e-5));
  Matrix separate = K.D2d_intrinsic(p);
  CHECK(assert_equal(numerical, separate, 1e-5));
}

Point2 calibrate_(const Cal3DS2_k3& k, const Point2& pt) { return k.calibrate(pt); }

/* ************************************************************************* */
TEST(Cal3DS2_k3, Dcalibrate) {
  Point2 pn(0.5, 0.5);
  Point2 pi = K.uncalibrate(pn);
  Matrix Dcal, Dp;
  K.calibrate(pi, Dcal, Dp);
  Matrix numerical1 = numericalDerivative21(calibrate_, K, pi, 1e-7);
  CHECK(assert_equal(numerical1, Dcal, 1e-5));
  Matrix numerical2 = numericalDerivative22(calibrate_, K, pi, 1e-7);
  CHECK(assert_equal(numerical2, Dp, 1e-5));
}

/* ************************************************************************* */
TEST(Cal3DS2_k3, Equal) { CHECK(assert_equal(K, K, 1e-5)); }

/* ************************************************************************* */
TEST(Cal3DS2_k3, Retract) {
  Cal3DS2_k3 expected(500 + 1,
                      100 + 2,
                      0.1 + 3,
                      320 + 4,
                      240 + 5,
                      1e-3 + 6,
                      2.0 * 1e-3 + 7,
                      3.0 * 1e-3 + 8,
                      4.0 * 1e-3 + 9,
                      5.0 * 1e-3 + 10);

  EXPECT_LONGS_EQUAL(Cal3DS2_k3::Dim(), 10);
  EXPECT_LONGS_EQUAL(expected.dim(), 10);

  Vector10 d;
  d << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10;
  Cal3DS2_k3 actual = K.retract(d);
  CHECK(assert_equal(expected, actual, 1e-7));
  CHECK(assert_equal(d, K.localCoordinates(actual), 1e-7));
}

/* ************************************************************************* */
TEST(Cal3DS2_k3, Print) {
  Cal3DS2_k3 cal(1, 2, 3, 4, 5, 6, 7, 8, 9, 10);
  std::stringstream os;
  os << "fx: " << cal.fx() << ", fy: " << cal.fy() << ", s: " << cal.skew() << ", px: " << cal.px()
     << ", py: " << cal.py() << ", k1: " << cal.k1() << ", k2: " << cal.k2() << ", p1: " << cal.p1()
     << ", p2: " << cal.p2() << ", k3: " << cal.k3();

  EXPECT(assert_stdout_equal(os.str(), cal));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

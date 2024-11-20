/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */
/**
 * @file  testCal3F.cpp
 * @brief Unit tests for the Cal3f calibration model.
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Cal3f.h>

using namespace gtsam;

GTSAM_CONCEPT_TESTABLE_INST(Cal3f)
GTSAM_CONCEPT_MANIFOLD_INST(Cal3f)

static Cal3f K(500.0, 1000.0, 2000.0);  // f = 500, u0 = 1000, v0 = 2000
static Point2 p(2.0, 3.0);

/* ************************************************************************* */
TEST(Cal3f, Vector) {
  Cal3f K(1.0, 0.0, 0.0);
  Vector expected(1);
  expected << 1.0;
  CHECK(assert_equal(expected, K.vector()));
}

/* ************************************************************************* */
TEST(Cal3f, Uncalibrate) {
  // Expected output: apply the intrinsic calibration matrix to point p
  Matrix3 K_matrix = K.K();
  Vector3 p_homogeneous(p.x(), p.y(), 1.0);
  Vector3 expected_homogeneous = K_matrix * p_homogeneous;
  Point2 expected(expected_homogeneous.x() / expected_homogeneous.z(),
                  expected_homogeneous.y() / expected_homogeneous.z());

  Point2 actual = K.uncalibrate(p);
  CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST(Cal3f, Calibrate) {
  Point2 pi = K.uncalibrate(p);
  Point2 pn = K.calibrate(pi);
  CHECK(traits<Point2>::Equals(p, pn, 1e-9));
}

/* ************************************************************************* */
Point2 uncalibrate_(const Cal3f& k, const Point2& pt) {
  return k.uncalibrate(pt);
}

Point2 calibrate_(const Cal3f& k, const Point2& pt) { return k.calibrate(pt); }

/* ************************************************************************* */
TEST(Cal3f, DUncalibrate) {
  Cal3f K(500.0, 1000.0, 2000.0);
  Matrix Dcal, Dp;
  Point2 actual = K.uncalibrate(p, Dcal, Dp);

  // Expected value computed manually
  Point2 expected = Point2(K.px() + K.fx() * p.x(), K.py() + K.fx() * p.y());
  CHECK(assert_equal(expected, actual, 1e-9));

  // Compute numerical derivatives
  Matrix numerical1 = numericalDerivative21(uncalibrate_, K, p);
  Matrix numerical2 = numericalDerivative22(uncalibrate_, K, p);
  CHECK(assert_equal(numerical1, Dcal, 1e-6));
  CHECK(assert_equal(numerical2, Dp, 1e-6));
}

/* ************************************************************************* */
TEST(Cal3f, DCalibrate) {
  Point2 pi = K.uncalibrate(p);
  Matrix Dcal, Dp;
  Point2 actual = K.calibrate(pi, Dcal, Dp);
  CHECK(assert_equal(p, actual, 1e-9));

  // Compute numerical derivatives
  Matrix numerical1 = numericalDerivative21(calibrate_, K, pi);
  Matrix numerical2 = numericalDerivative22(calibrate_, K, pi);
  CHECK(assert_equal(numerical1, Dcal, 1e-6));
  CHECK(assert_equal(numerical2, Dp, 1e-6));
}

/* ************************************************************************* */
TEST(Cal3f, Manifold) {
  Cal3f K1(500.0, 1000.0, 2000.0);
  Vector1 delta;
  delta << 10.0;  // Increment focal length by 10

  Cal3f K2 = K1.retract(delta);
  CHECK(assert_equal(510.0, K2.fx(), 1e-9));
  CHECK(assert_equal(K1.px(), K2.px(), 1e-9));
  CHECK(assert_equal(K1.py(), K2.py(), 1e-9));

  Vector1 delta_computed = K1.localCoordinates(K2);
  CHECK(assert_equal(delta, delta_computed, 1e-9));
}

/* ************************************************************************* */
TEST(Cal3f, Assert_equal) {
  CHECK(assert_equal(K, K, 1e-9));
  Cal3f K2(500.0, 1000.0, 2000.0);
  CHECK(assert_equal(K, K2, 1e-9));
}

/* ************************************************************************* */
TEST(Cal3f, Print) {
  Cal3f cal(500.0, 1000.0, 2000.0);
  std::stringstream os;
  os << "f: " << cal.fx() << ", px: " << cal.px() << ", py: " << cal.py();

  EXPECT(assert_stdout_equal(os.str(), cal));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
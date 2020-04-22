/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file  testCal3Fisheye.cpp
 * @brief Unit tests for fisheye calibration class
 * @author ghaggin
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Cal3Fisheye.h>

using namespace gtsam;

GTSAM_CONCEPT_TESTABLE_INST(Cal3Fisheye)
GTSAM_CONCEPT_MANIFOLD_INST(Cal3Fisheye)

static const double fx = 250, fy = 260, s = 0.1, u0 = 320, v0 = 240;
static Cal3Fisheye K(fx, fy, s, u0, v0, -0.013721808247486035,
                     0.020727425669427896, -0.012786476702685545,
                     0.0025242267320687625);
static Point2 p(2, 3);

/* ************************************************************************* */
TEST(Cal3Fisheye, uncalibrate1) {
  // Calculate the solution
  const double xi = p.x(), yi = p.y();
  const double r = sqrt(xi * xi + yi * yi);
  const double t = atan(r);
  const double tt = t * t, t4 = tt * tt, t6 = tt * t4, t8 = t4 * t4;
  const double td =
      t * (1 + K.k1() * tt + K.k2() * t4 + K.k3() * t6 + K.k4() * t8);
  Vector3 pd(td / r * xi, td / r * yi, 1);
  Vector3 v = K.K() * pd;

  Point2 uv_sol(v[0] / v[2], v[1] / v[2]);

  Point2 uv = K.uncalibrate(p);
  CHECK(assert_equal(uv, uv_sol));
}

TEST(Cal3Fisheye, calibrate) {
  Point2 pi;
  Point2 uv;
  Point2 pi_hat;

  pi = Point2(0.5, 0.5);     // point in intrinsic coordinates
  uv = K.uncalibrate(pi);    // map intrinsic coord to image plane (pi)
  pi_hat = K.calibrate(uv);  // map image coords (pi) back to intrinsic coords
  CHECK(traits<Point2>::Equals(pi, pi_hat,
                               1e-5));  // check that the inv mapping works

  pi = Point2(-0.7, -1.2);
  uv = K.uncalibrate(pi);
  pi_hat = K.calibrate(uv);
  CHECK(traits<Point2>::Equals(pi, pi_hat, 1e-5));

  pi = Point2(-3, 5);
  uv = K.uncalibrate(pi);
  pi_hat = K.calibrate(uv);
  CHECK(traits<Point2>::Equals(pi, pi_hat, 1e-5));

  pi = Point2(7, -12);
  uv = K.uncalibrate(pi);
  pi_hat = K.calibrate(uv);
  CHECK(traits<Point2>::Equals(pi, pi_hat, 1e-5));
}

Point2 uncalibrate_(const Cal3Fisheye& k, const Point2& pt) {
  return k.uncalibrate(pt);
}

/* ************************************************************************* */
TEST(Cal3Fisheye, Duncalibrate1) {
  Matrix computed;
  K.uncalibrate(p, computed, boost::none);
  Matrix numerical = numericalDerivative21(uncalibrate_, K, p, 1e-7);
  CHECK(assert_equal(numerical, computed, 1e-5));
  Matrix separate = K.D2d_calibration(p);
  CHECK(assert_equal(numerical, separate, 1e-5));
}

/* ************************************************************************* */
TEST(Cal3Fisheye, Duncalibrate2) {
  Matrix computed;
  K.uncalibrate(p, boost::none, computed);
  Matrix numerical = numericalDerivative22(uncalibrate_, K, p, 1e-7);
  CHECK(assert_equal(numerical, computed, 1e-5));
  Matrix separate = K.D2d_intrinsic(p);
  CHECK(assert_equal(numerical, separate, 1e-5));
}

/* ************************************************************************* */
TEST(Cal3Fisheye, assert_equal) { CHECK(assert_equal(K, K, 1e-5)); }

/* ************************************************************************* */
TEST(Cal3Fisheye, retract) {
  Cal3Fisheye expected(K.fx() + 1, K.fy() + 2, K.skew() + 3, K.px() + 4,
                       K.py() + 5, K.k1() + 6, K.k2() + 7, K.k3() + 8,
                       K.k4() + 9);
  Vector d(9);
  d << 1, 2, 3, 4, 5, 6, 7, 8, 9;
  Cal3Fisheye actual = K.retract(d);
  CHECK(assert_equal(expected, actual, 1e-7));
  CHECK(assert_equal(d, K.localCoordinates(actual), 1e-7));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

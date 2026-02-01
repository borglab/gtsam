/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file  testCal3_S2Stereo.cpp
 * @brief Unit tests for stereo-rig calibration model.
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Cal3_S2Stereo.h>

using namespace gtsam;

GTSAM_CONCEPT_TESTABLE_INST(Cal3_S2Stereo)
GTSAM_CONCEPT_MANIFOLD_INST(Cal3_S2Stereo)

static Cal3_S2Stereo K(500, 500, 0.1, 640 / 2, 480 / 2, 1);
static Point2 p(1, -2);
static Point2 p_uv(1320.3, 1740);
static Point2 p_xy(2, 3);

/* ************************************************************************* */
TEST(Cal3_S2Stereo, Constructor) {
  Cal3_S2Stereo expected(554.256, 554.256, 0, 640 / 2, 480 / 2, 3);

  double fov = 60;  // degrees
  size_t w = 640, h = 480;
  Cal3_S2Stereo actual(fov, w, h, 3);

  CHECK(assert_equal(expected, actual, 1e-3));
}

/* ************************************************************************* */
TEST(Cal3_S2Stereo, Calibrate) {
  Point2 intrinsic(2, 3);
  Point2 expectedimage(1320.3, 1740);
  Point2 imagecoordinates = K.uncalibrate(intrinsic);
  CHECK(assert_equal(expectedimage, imagecoordinates));
  CHECK(assert_equal(intrinsic, K.calibrate(imagecoordinates)));
}

/* ************************************************************************* */
TEST(Cal3_S2Stereo, CalibrateHomogeneous) {
  Vector3 intrinsic(2, 3, 1);
  Vector3 image(1320.3, 1740, 1);
  CHECK(assert_equal(intrinsic, K.calibrate(image)));
}

/* ************************************************************************* */
Point2 uncalibrate_(const Cal3_S2Stereo& k, const Point2& pt) {
  return k.uncalibrate(pt);
}

TEST(Cal3_S2Stereo, Duncalibrate) {
  Matrix26 Dcal;
  Matrix22 Dp;
  K.uncalibrate(p, Dcal, Dp);

  Matrix numerical1 = numericalDerivative21(uncalibrate_, K, p);
  CHECK(assert_equal(numerical1, Dcal, 1e-8));
  Matrix numerical2 = numericalDerivative22(uncalibrate_, K, p);
  CHECK(assert_equal(numerical2, Dp, 1e-9));
}

Point2 calibrate_(const Cal3_S2Stereo& K, const Point2& pt) {
  return K.calibrate(pt);
}
/* ************************************************************************* */
TEST(Cal3_S2Stereo, Dcalibrate) {
  Matrix26 Dcal;
  Matrix22 Dp;
  Point2 expected = K.calibrate(p_uv, Dcal, Dp);
  CHECK(assert_equal(expected, p_xy, 1e-8));

  Matrix numerical1 = numericalDerivative21(calibrate_, K, p_uv);
  CHECK(assert_equal(numerical1, Dcal, 1e-8));
  Matrix numerical2 = numericalDerivative22(calibrate_, K, p_uv);
  CHECK(assert_equal(numerical2, Dp, 1e-8));
}

/* ************************************************************************* */
TEST(Cal3_S2Stereo, Equal) {
  CHECK(assert_equal(K, K, 1e-9));

  Cal3_S2Stereo K1(500, 500, 0.1, 640 / 2, 480 / 2, 1);
  CHECK(assert_equal(K, K1, 1e-9));
}

/* ************************************************************************* */
TEST(Cal3_S2Stereo, Retract) {
  Cal3_S2Stereo expected(500 + 1, 500 + 2, 0.1 + 3, 640 / 2 + 4, 480 / 2 + 5,
                         7);
  EXPECT_LONGS_EQUAL(Cal3_S2Stereo::Dim(), 6);
  EXPECT_LONGS_EQUAL(expected.dim(), 6);

  Vector6 d;
  d << 1, 2, 3, 4, 5, 6;
  Cal3_S2Stereo actual = K.retract(d);
  CHECK(assert_equal(expected, actual, 1e-7));
  CHECK(assert_equal(d, K.localCoordinates(actual), 1e-7));
}

/* ************************************************************************* */
TEST(Cal3_S2Stereo, Print) {
  Cal3_S2Stereo cal(5, 5, 5, 5, 5, 2);
  std::stringstream os;
  os << "fx: " << cal.fx() << ", fy: " << cal.fy() << ", s: " << cal.skew()
     << ", px: " << cal.px() << ", py: " << cal.py()
     << ", b: " << cal.baseline();
  EXPECT(assert_stdout_equal(os.str(), cal));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

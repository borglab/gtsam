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
TEST(Cal3_S2Stereo, easy_constructor) {
  Cal3_S2Stereo expected(554.256, 554.256, 0, 640 / 2, 480 / 2, 3);

  double fov = 60;  // degrees
  size_t w = 640, h = 480;
  Cal3_S2Stereo actual(fov, w, h, 3);

  CHECK(assert_equal(expected, actual, 1e-3));
}

/* ************************************************************************* */
TEST(Cal3_S2Stereo, calibrate) {
  Point2 intrinsic(2, 3);
  Point2 expectedimage(1320.3, 1740);
  Point2 imagecoordinates = K.uncalibrate(intrinsic);
  CHECK(assert_equal(expectedimage, imagecoordinates));
  CHECK(assert_equal(intrinsic, K.calibrate(imagecoordinates)));
}

/* ************************************************************************* */
TEST(Cal3_S2Stereo, calibrate_homogeneous) {
  Vector3 intrinsic(2, 3, 1);
  Vector3 image(1320.3, 1740, 1);
  CHECK(assert_equal((Vector)intrinsic, (Vector)K.calibrate(image)));
}

//TODO(Varun) Add calibrate and uncalibrate methods
// /* ************************************************************************* */
// Point2 uncalibrate_(const Cal3_S2Stereo& k, const Point2& pt) {
//   return k.uncalibrate(pt);
// }

// TEST(Cal3_S2Stereo, Duncalibrate1) {
//   Matrix26 computed;
//   K.uncalibrate(p, computed, boost::none);
//   Matrix numerical = numericalDerivative21(uncalibrate_, K, p);
//   CHECK(assert_equal(numerical, computed, 1e-8));
// }

// /* ************************************************************************* */
// TEST(Cal3_S2Stereo, Duncalibrate2) {
//   Matrix computed;
//   K.uncalibrate(p, boost::none, computed);
//   Matrix numerical = numericalDerivative22(uncalibrate_, K, p);
//   CHECK(assert_equal(numerical, computed, 1e-9));
// }

// Point2 calibrate_(const Cal3_S2Stereo& k, const Point2& pt) {
//   return k.calibrate(pt);
// }
// /* ************************************************************************* */
// TEST(Cal3_S2Stereo, Dcalibrate1) {
//   Matrix computed;
//   Point2 expected = K.calibrate(p_uv, computed, boost::none);
//   Matrix numerical = numericalDerivative21(calibrate_, K, p_uv);
//   CHECK(assert_equal(expected, p_xy, 1e-8));
//   CHECK(assert_equal(numerical, computed, 1e-8));
// }

// /* ************************************************************************* */
// TEST(Cal3_S2Stereo, Dcalibrate2) {
//   Matrix computed;
//   Point2 expected = K.calibrate(p_uv, boost::none, computed);
//   Matrix numerical = numericalDerivative22(calibrate_, K, p_uv);
//   CHECK(assert_equal(expected, p_xy, 1e-8));
//   CHECK(assert_equal(numerical, computed, 1e-8));
// }

/* ************************************************************************* */
TEST(Cal3_S2Stereo, assert_equal) {
  CHECK(assert_equal(K, K, 1e-9));

  Cal3_S2Stereo K1(500, 500, 0.1, 640 / 2, 480 / 2, 1);
  CHECK(assert_equal(K, K1, 1e-9));
}

/* ************************************************************************* */
TEST(Cal3_S2Stereo, retract) {
  Cal3_S2Stereo expected(500 + 1, 500 + 2, 0.1 + 3, 640 / 2 + 4, 480 / 2 + 5,
                         7);
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

/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file  testCal3Unified.cpp
 * @brief Unit tests for Cal3Unified calibration model.
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Cal3Unified.h>

#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/Values.h>

using namespace gtsam;

GTSAM_CONCEPT_TESTABLE_INST(Cal3Unified)
GTSAM_CONCEPT_MANIFOLD_INST(Cal3Unified)

/*
ground truth from matlab, code :
X = [0.5 0.7 1]';
V = [0.1, 1e-3, 2.0*1e-3, 3.0*1e-3, 4.0*1e-3, 0, 0, 100, 105, 320, 240];
[P, J] = spaceToImgPlane(X, V);
matlab toolbox available at http://homepages.laas.fr/~cmei/index.php/Toolbox
*/

static Cal3Unified K(100, 105, 0.0, 320, 240, 1e-3, 2.0 * 1e-3, 3.0 * 1e-3,
                     4.0 * 1e-3, 0.1);
static Point2 p(0.5, 0.7);

/* ************************************************************************* */
TEST(Cal3Unified, Uncalibrate) {
  Point2 p_i(364.7791831734982, 305.6677211952602);
  Point2 q = K.uncalibrate(p);
  CHECK(assert_equal(q, p_i));
}

/* ************************************************************************* */
TEST(Cal3Unified, SpaceNplane) {
  Point2 q = K.spaceToNPlane(p);
  CHECK(assert_equal(Point2(0.441731600049497, 0.618424240069295), q));
  CHECK(assert_equal(p, K.nPlaneToSpace(q)));
}

/* ************************************************************************* */
TEST(Cal3Unified, Calibrate) {
  Point2 pi = K.uncalibrate(p);
  Point2 pn_hat = K.calibrate(pi);
  CHECK(traits<Point2>::Equals(p, pn_hat, 1e-8));
}

Point2 uncalibrate_(const Cal3Unified& k, const Point2& pt) {
  return k.uncalibrate(pt);
}

/* ************************************************************************* */
TEST(Cal3Unified, Duncalibrate1) {
  Matrix computed;
  K.uncalibrate(p, computed, {});
  Matrix numerical = numericalDerivative21(uncalibrate_, K, p, 1e-7);
  CHECK(assert_equal(numerical, computed, 1e-6));
}

/* ************************************************************************* */
TEST(Cal3Unified, Duncalibrate2) {
  Matrix computed;
  K.uncalibrate(p, {}, computed);
  Matrix numerical = numericalDerivative22(uncalibrate_, K, p, 1e-7);
  CHECK(assert_equal(numerical, computed, 1e-6));
}

Point2 calibrate_(const Cal3Unified& k, const Point2& pt) {
  return k.calibrate(pt);
}

/* ************************************************************************* */
TEST(Cal3Unified, Dcalibrate) {
  Point2 pi = K.uncalibrate(p);
  Matrix Dcal, Dp;
  K.calibrate(pi, Dcal, Dp);
  Matrix numerical1 = numericalDerivative21(calibrate_, K, pi);
  CHECK(assert_equal(numerical1, Dcal, 1e-5));
  Matrix numerical2 = numericalDerivative22(calibrate_, K, pi);
  CHECK(assert_equal(numerical2, Dp, 1e-5));
}

/* ************************************************************************* */
TEST(Cal3Unified, Equal) { CHECK(assert_equal(K, K, 1e-9)); }

/* ************************************************************************* */
TEST(Cal3Unified, Retract) {
  Cal3Unified expected(100 + 2, 105 + 3, 0.0 + 4, 320 + 5, 240 + 6, 1e-3 + 7,
                       2.0 * 1e-3 + 8, 3.0 * 1e-3 + 9, 4.0 * 1e-3 + 10,
                       0.1 + 1);

  EXPECT_LONGS_EQUAL(Cal3Unified::Dim(), 10);
  EXPECT_LONGS_EQUAL(expected.dim(), 10);

  Vector10 d;
  d << 2, 3, 4, 5, 6, 7, 8, 9, 10, 1;
  Cal3Unified actual = K.retract(d);
  CHECK(assert_equal(expected, actual, 1e-9));
  CHECK(assert_equal(d, K.localCoordinates(actual), 1e-9));
}

/* ************************************************************************* */
TEST(Cal3Unified, DerivedValue) {
  Values values;
  Cal3Unified cal(1, 2, 3, 4, 5, 6, 7, 8, 9, 10);
  Key key = 1;
  values.insert(key, cal);

  Cal3Unified calafter = values.at<Cal3Unified>(key);

  CHECK(assert_equal(cal, calafter, 1e-9));
}

/* ************************************************************************* */
TEST(Cal3Unified, Print) {
  Cal3Unified cal(0, 1, 2, 3, 4, 5, 6, 7, 8, 9);
  std::stringstream os;
  os << "fx: " << cal.fx() << ", fy: " << cal.fy() << ", s: " << cal.skew()
     << ", px: " << cal.px() << ", py: " << cal.py() << ", k1: " << cal.k1()
     << ", k2: " << cal.k2() << ", p1: " << cal.p1() << ", p2: " << cal.p2()
     << ", xi: " << cal.xi();

  EXPECT(assert_stdout_equal(os.str(), cal));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

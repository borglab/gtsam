/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file  testCal3_S2.cpp
 * @brief Unit tests for transform derivatives
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Cal3_S2.h>

using namespace gtsam;

GTSAM_CONCEPT_TESTABLE_INST(Cal3_S2)
GTSAM_CONCEPT_MANIFOLD_INST(Cal3_S2)

static Cal3_S2 K(500, 500, 0.1, 640 / 2, 480 / 2);
static Point2 p(1, -2);
static Point2 p_uv(1320.3, 1740);
static Point2 p_xy(2, 3);

/* ************************************************************************* */
TEST( Cal3_S2, easy_constructor)
{
  Cal3_S2 expected(554.256, 554.256, 0, 640 / 2, 480 / 2);

  double fov = 60; // degrees
  size_t w=640,h=480;
  Cal3_S2 actual(fov,w,h);

  CHECK(assert_equal(expected,actual,1e-3));
}

/* ************************************************************************* */
TEST( Cal3_S2, calibrate)
{
  Point2 intrinsic(2,3);
  Point2 expectedimage(1320.3, 1740);
  Point2 imagecoordinates = K.uncalibrate(intrinsic);
  CHECK(assert_equal(expectedimage,imagecoordinates));
  CHECK(assert_equal(intrinsic,K.calibrate(imagecoordinates)));
}

/* ************************************************************************* */
TEST( Cal3_S2, calibrate_homogeneous) {
  Vector3 intrinsic(2, 3, 1);
  Vector3 image(1320.3, 1740, 1);
  CHECK(assert_equal((Vector)intrinsic,(Vector)K.calibrate(image)));
}

/* ************************************************************************* */
Point2 uncalibrate_(const Cal3_S2& k, const Point2& pt) { return k.uncalibrate(pt); }
TEST( Cal3_S2, Duncalibrate1)
{
  Matrix25 computed; K.uncalibrate(p, computed, boost::none);
  Matrix numerical = numericalDerivative21(uncalibrate_, K, p);
  CHECK(assert_equal(numerical,computed,1e-8));
}

/* ************************************************************************* */
TEST( Cal3_S2, Duncalibrate2)
{
  Matrix computed; K.uncalibrate(p, boost::none, computed);
  Matrix numerical = numericalDerivative22(uncalibrate_, K, p);
  CHECK(assert_equal(numerical,computed,1e-9));
}

Point2 calibrate_(const Cal3_S2& k,  const Point2& pt) {return k.calibrate(pt); }
/* ************************************************************************* */
TEST(Cal3_S2, Dcalibrate1)
{
    Matrix computed;
    Point2 expected = K.calibrate(p_uv, computed, boost::none);
    Matrix numerical = numericalDerivative21(calibrate_, K, p_uv);
    CHECK(assert_equal(expected, p_xy, 1e-8));
    CHECK(assert_equal(numerical, computed, 1e-8));
}

/* ************************************************************************* */
TEST(Cal3_S2, Dcalibrate2)
{
    Matrix computed;
    Point2 expected = K.calibrate(p_uv, boost::none, computed);
    Matrix numerical = numericalDerivative22(calibrate_, K, p_uv);
    CHECK(assert_equal(expected, p_xy, 1e-8));
    CHECK(assert_equal(numerical, computed, 1e-8));
}

/* ************************************************************************* */
TEST( Cal3_S2, assert_equal)
{
  CHECK(assert_equal(K,K,1e-9));

  Cal3_S2 K1(500, 500, 0.1, 640 / 2, 480 / 2);
  CHECK(assert_equal(K,K1,1e-9));
}

/* ************************************************************************* */
TEST( Cal3_S2, retract)
{
  Cal3_S2 expected(500+1, 500+2, 0.1+3, 640 / 2+4, 480 / 2+5);
  Vector d(5);
  d << 1,2,3,4,5;
  Cal3_S2 actual = K.retract(d);
  CHECK(assert_equal(expected,actual,1e-7));
  CHECK(assert_equal(d,K.localCoordinates(actual),1e-7));
}

/* ************************************************************************* */
TEST(Cal3_S2, between) {
  Cal3_S2 k1(5, 5, 5, 5, 5), k2(5, 6, 7, 8, 9);
  Matrix H1, H2;

  EXPECT(assert_equal(Cal3_S2(0,1,2,3,4), k1.between(k2, H1, H2)));
  EXPECT(assert_equal(-I_5x5, H1));
  EXPECT(assert_equal(I_5x5, H2));

}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */


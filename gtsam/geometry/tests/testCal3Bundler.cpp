/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file  testCal3Bundler.cpp
 * @brief Unit tests for transform derivatives
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Cal3Bundler.h>

using namespace gtsam;

GTSAM_CONCEPT_TESTABLE_INST(Cal3Bundler)
GTSAM_CONCEPT_MANIFOLD_INST(Cal3Bundler)

static Cal3Bundler K(500, 1e-3, 1e-3, 1000, 2000);
static Point2 p(2,3);

/* ************************************************************************* */
TEST( Cal3Bundler, vector)
{
  Cal3Bundler K;
  Vector expected(3);
  expected << 1, 0, 0;
  CHECK(assert_equal(expected,K.vector()));
}

/* ************************************************************************* */
TEST( Cal3Bundler, uncalibrate)
{
  Vector v = K.vector() ;
  double r = p.x()*p.x() + p.y()*p.y() ;
  double g = v[0]*(1+v[1]*r+v[2]*r*r) ;
  Point2 expected (1000+g*p.x(), 2000+g*p.y()) ;
  Point2 actual = K.uncalibrate(p);
  CHECK(assert_equal(expected,actual));
}

TEST( Cal3Bundler, calibrate )
{
  Point2 pn(0.5, 0.5);
  Point2 pi = K.uncalibrate(pn);
  Point2 pn_hat = K.calibrate(pi);
  CHECK( traits<Point2>::Equals(pn, pn_hat, 1e-5));
}

/* ************************************************************************* */
Point2 uncalibrate_(const Cal3Bundler& k, const Point2& pt) { return k.uncalibrate(pt); }

/* ************************************************************************* */
TEST( Cal3Bundler, Duncalibrate)
{
  Matrix Dcal, Dp;
  Point2 actual = K.uncalibrate(p, Dcal, Dp);
  Point2 expected(2182, 3773);
  CHECK(assert_equal(expected,actual,1e-7));
  Matrix numerical1 = numericalDerivative21(uncalibrate_, K, p);
  Matrix numerical2 = numericalDerivative22(uncalibrate_, K, p);
  CHECK(assert_equal(numerical1,Dcal,1e-7));
  CHECK(assert_equal(numerical2,Dp,1e-7));
  CHECK(assert_equal(numerical1,K.D2d_calibration(p),1e-7));
  CHECK(assert_equal(numerical2,K.D2d_intrinsic(p),1e-7));
  Matrix Dcombined(2,5);
  Dcombined << Dp, Dcal;
  CHECK(assert_equal(Dcombined,K.D2d_intrinsic_calibration(p),1e-7));
}

/* ************************************************************************* */
TEST( Cal3Bundler, assert_equal)
{
  CHECK(assert_equal(K,K,1e-7));
}

/* ************************************************************************* */
TEST( Cal3Bundler, retract)
{
  Cal3Bundler expected(510, 2e-3, 2e-3, 1000, 2000);
  Vector d(3);
  d << 10, 1e-3, 1e-3;
  Cal3Bundler actual = K.retract(d);
  CHECK(assert_equal(expected,actual,1e-7));
  CHECK(assert_equal(d,K.localCoordinates(actual),1e-7));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */

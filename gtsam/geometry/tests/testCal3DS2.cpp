/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file  testCal3DS2.cpp
 * @brief Unit tests for transform derivatives
 */


#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Cal3DS2.h>

using namespace gtsam;

GTSAM_CONCEPT_TESTABLE_INST(Cal3DS2)
GTSAM_CONCEPT_MANIFOLD_INST(Cal3DS2)

static Cal3DS2 K(500, 100, 0.1, 320, 240, 1e-3, 2.0*1e-3, 3.0*1e-3, 4.0*1e-3);
static Point2 p(2,3);

/* ************************************************************************* */
TEST( Cal3DS2, uncalibrate)
{
  Vector k = K.k() ;
  double r = p.x()*p.x() + p.y()*p.y() ;
  double g = 1+k[0]*r+k[1]*r*r ;
  double tx = 2*k[2]*p.x()*p.y()       +   k[3]*(r+2*p.x()*p.x()) ;
  double ty =   k[2]*(r+2*p.y()*p.y()) + 2*k[3]*p.x()*p.y() ;
  Vector v_hat = Vector_(3, g*p.x() + tx, g*p.y() + ty, 1.0) ;
  Vector v_i = K.K() * v_hat ;
  Point2 p_i(v_i(0)/v_i(2), v_i(1)/v_i(2)) ;
  Point2 q = K.uncalibrate(p);
  CHECK(assert_equal(q,p_i));
}

TEST( Cal3DS2, calibrate )
{
  Point2 pn(0.5, 0.5);
  Point2 pi = K.uncalibrate(pn);
  Point2 pn_hat = K.calibrate(pi);
  CHECK( pn.equals(pn_hat, 1e-5));
}

Point2 uncalibrate_(const Cal3DS2& k, const Point2& pt) { return k.uncalibrate(pt); }

/* ************************************************************************* */
TEST( Cal3DS2, Duncalibrate1)
{
  Matrix computed;
  K.uncalibrate(p, computed, boost::none);
  Matrix numerical = numericalDerivative21(uncalibrate_, K, p, 1e-7);
  CHECK(assert_equal(numerical,computed,1e-5));
}

/* ************************************************************************* */
TEST( Cal3DS2, Duncalibrate2)
{
  Matrix computed; K.uncalibrate(p, boost::none, computed);
  Matrix numerical = numericalDerivative22(uncalibrate_, K, p, 1e-7);
  CHECK(assert_equal(numerical,computed,1e-5));
}

/* ************************************************************************* */
TEST( Cal3DS2, assert_equal)
{
  CHECK(assert_equal(K,K,1e-5));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */

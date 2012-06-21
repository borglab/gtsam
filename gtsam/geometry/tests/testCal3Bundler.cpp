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

static Cal3Bundler K(500, 1e-3, 1e-3);
static Point2 p(2,3);

/* ************************************************************************* */
TEST( Cal3Bundler, calibrate)
{
  Vector v = K.vector() ;
  double r = p.x()*p.x() + p.y()*p.y() ;
  double g = v[0]*(1+v[1]*r+v[2]*r*r) ;
  Point2 q_hat (g*p.x(), g*p.y()) ;
  Point2 q = K.uncalibrate(p);
  CHECK(assert_equal(q,q_hat));
}


Point2 uncalibrate_(const Cal3Bundler& k, const Point2& pt) { return k.uncalibrate(pt); }

/* ************************************************************************* */
TEST( Cal3Bundler, Duncalibrate1)
{
  Matrix computed;
  K.uncalibrate(p, computed, boost::none);
  Matrix numerical = numericalDerivative21(uncalibrate_, K, p);
  CHECK(assert_equal(numerical,computed,1e-7));
}

/* ************************************************************************* */
TEST( Cal3Bundler, Duncalibrate2)
{
  Matrix computed; K.uncalibrate(p, boost::none, computed);
  Matrix numerical = numericalDerivative22(uncalibrate_, K, p);
  CHECK(assert_equal(numerical,computed,1e-7));
}

/* ************************************************************************* */
TEST( Cal3Bundler, assert_equal)
{
  CHECK(assert_equal(K,K,1e-7));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */

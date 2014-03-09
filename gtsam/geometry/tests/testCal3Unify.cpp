/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file  testCal3Unify.cpp
 * @brief Unit tests for transform derivatives
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Cal3Unify.h>

using namespace gtsam;

GTSAM_CONCEPT_TESTABLE_INST(Cal3Unify)
GTSAM_CONCEPT_MANIFOLD_INST(Cal3Unify)

/* ground truth data will from matlab, code :
X = [2 3 1]';
V = [0.01, 1e-3, 2.0*1e-3, 3.0*1e-3, 4.0*1e-3, 0, 0, 100, 105, 320, 240];
[P, J] = spaceToImgPlane(X, V);
matlab toolbox available at http://homepages.laas.fr/~cmei/index.php/Toolbox */

static Cal3Unify K(0.01, 100, 105, 0.0, 320, 240, 1e-3, 2.0*1e-3, 3.0*1e-3, 4.0*1e-3);
static Point2 p(2,3);

/* ************************************************************************* */
TEST( Cal3Unify, uncalibrate)
{
  Point2 p_i(582.5228344366194, 649.6685266099726) ;
  Point2 q = K.uncalibrate(p);
  CHECK(assert_equal(q,p_i));
}

TEST( Cal3Unify, calibrate )
{
  Point2 pi = K.uncalibrate(p);
  Point2 pn_hat = K.calibrate(pi);
  CHECK( p.equals(pn_hat, 1e-5));
}

Point2 uncalibrate_(const Cal3Unify& k, const Point2& pt) { return k.uncalibrate(pt); }

/* ************************************************************************* */
TEST( Cal3Unify, Duncalibrate1)
{
  Matrix computed;
  K.uncalibrate(p, computed, boost::none);
  Matrix numerical = numericalDerivative21(uncalibrate_, K, p, 1e-7);
  CHECK(assert_equal(numerical,computed,1e-5));
}

/* ************************************************************************* */
TEST( Cal3Unify, Duncalibrate2)
{
  Matrix computed;
  K.uncalibrate(p, boost::none, computed);
  Matrix numerical = numericalDerivative22(uncalibrate_, K, p, 1e-7);
  CHECK(assert_equal(numerical,computed,1e-5));
}

/* ************************************************************************* */
TEST( Cal3Unify, assert_equal)
{
  CHECK(assert_equal(K,K,1e-5));
}

/* ************************************************************************* */
TEST( Cal3Unify, retract)
{
  Cal3Unify expected(0.01 + 1, 100 + 2, 105 + 3, 0.0 + 4, 320 + 5, 240 + 6,
      1e-3 + 7, 2.0*1e-3 + 8, 3.0*1e-3 + 9, 4.0*1e-3 + 10);
  Vector d(10);
  d << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10;
  Cal3Unify actual = K.retract(d);
  CHECK(assert_equal(expected,actual,1e-7));
  CHECK(assert_equal(d,K.localCoordinates(actual),1e-7));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */

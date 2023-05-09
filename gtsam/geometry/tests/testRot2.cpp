/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testRot2.cpp
 * @brief   Unit tests for Rot2 class
 * @author  Frank Dellaert
 */

#include <gtsam/geometry/Rot2.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/testLie.h>
#include <CppUnitLite/TestHarness.h>

using namespace gtsam;

GTSAM_CONCEPT_TESTABLE_INST(Rot2)
GTSAM_CONCEPT_LIE_INST(Rot2)

Rot2 R(Rot2::fromAngle(0.1));
Point2 P(0.2, 0.7);

/* ************************************************************************* */
TEST( Rot2, constructors_and_angle)
{
  double c=cos(0.1), s=sin(0.1);
  DOUBLES_EQUAL(0.1,R.theta(),1e-9);
  CHECK(assert_equal(R,Rot2(0.1)));
  CHECK(assert_equal(R,Rot2::fromAngle(0.1)));
  CHECK(assert_equal(R,Rot2::fromCosSin(c,s)));
  CHECK(assert_equal(R,Rot2::atan2(s*5,c*5)));
}

/* ************************************************************************* */
TEST( Rot2, unit)
{
  EXPECT(assert_equal(Point2(1.0, 0.0), Rot2::fromAngle(0).unit()));
  EXPECT(assert_equal(Point2(0.0, 1.0), Rot2::fromAngle(M_PI/2.0).unit()));
}

/* ************************************************************************* */
TEST( Rot2, transpose)
{
  Matrix expected = R.inverse().matrix();
  Matrix actual = R.transpose();
  CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST( Rot2, compose)
{
  CHECK(assert_equal(Rot2::fromAngle(0.45), Rot2::fromAngle(0.2)*Rot2::fromAngle(0.25)));
  CHECK(assert_equal(Rot2::fromAngle(0.45), Rot2::fromAngle(0.25)*Rot2::fromAngle(0.2)));

  Matrix H1, H2;
  (void) Rot2::fromAngle(1.0).compose(Rot2::fromAngle(2.0), H1, H2);
  EXPECT(assert_equal(I_1x1, H1));
  EXPECT(assert_equal(I_1x1, H2));
}

/* ************************************************************************* */
TEST( Rot2, between)
{
  CHECK(assert_equal(Rot2::fromAngle(0.05), Rot2::fromAngle(0.2).between(Rot2::fromAngle(0.25))));
  CHECK(assert_equal(Rot2::fromAngle(-0.05), Rot2::fromAngle(0.25).between(Rot2::fromAngle(0.2))));

  Matrix H1, H2;
  (void) Rot2::fromAngle(1.0).between(Rot2::fromAngle(2.0), H1, H2);
  EXPECT(assert_equal(-I_1x1, H1));
  EXPECT(assert_equal(I_1x1, H2));
}

/* ************************************************************************* */
TEST( Rot2, equals)
{
  CHECK(R.equals(R));
  Rot2 zero;
  CHECK(!R.equals(zero));
}

/* ************************************************************************* */
TEST( Rot2, expmap)
{
  Vector v = Z_1x1;
  CHECK(assert_equal(R.retract(v), R));
}

/* ************************************************************************* */
TEST(Rot2, logmap)
{
  Rot2 rot0(Rot2::fromAngle(M_PI/2.0));
  Rot2 rot(Rot2::fromAngle(M_PI));
  Vector expected = (Vector(1) << M_PI/2.0).finished();
  Vector actual = rot0.localCoordinates(rot);
  CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */
// rotate and derivatives
inline Point2 rotate_(const Rot2 & R, const Point2& p) {return R.rotate(p);}
TEST( Rot2, rotate)
{
  Matrix H1, H2;
  Point2 actual = R.rotate(P, H1, H2);
  CHECK(assert_equal(actual,R*P));
  Matrix numerical1 = numericalDerivative21(rotate_, R, P);
  CHECK(assert_equal(numerical1,H1));
  Matrix numerical2 = numericalDerivative22(rotate_, R, P);
  CHECK(assert_equal(numerical2,H2));
}

/* ************************************************************************* */
// unrotate and derivatives
inline Point2 unrotate_(const Rot2& R, const Point2& p) {return R.unrotate(p);}
TEST( Rot2, unrotate)
{
  Matrix H1, H2;
  Point2 w = R * P, actual = R.unrotate(w, H1, H2);
  CHECK(assert_equal(actual,P));
  Matrix numerical1 = numericalDerivative21(unrotate_, R, w);
  CHECK(assert_equal(numerical1,H1));
  Matrix numerical2 = numericalDerivative22(unrotate_, R, w);
  CHECK(assert_equal(numerical2,H2));
}

/* ************************************************************************* */
inline Rot2 relativeBearing_(const Point2& pt) {return Rot2::relativeBearing(pt); }
TEST( Rot2, relativeBearing )
{
  Point2 l1(1, 0), l2(1, 1);
  Matrix expectedH, actualH;

  // establish relativeBearing is indeed zero
  Rot2 actual1 = Rot2::relativeBearing(l1, actualH);
  CHECK(assert_equal(Rot2(),actual1));

  // Check numerical derivative
  expectedH = numericalDerivative11(relativeBearing_, l1);
  CHECK(assert_equal(expectedH,actualH));

  // establish relativeBearing is indeed 45 degrees
  Rot2 actual2 = Rot2::relativeBearing(l2, actualH);
  CHECK(assert_equal(Rot2::fromAngle(M_PI/4.0),actual2));

  // Check numerical derivative
  expectedH = numericalDerivative11(relativeBearing_, l2);
  CHECK(assert_equal(expectedH,actualH));
}

//******************************************************************************
Rot2 T1(0.1);
Rot2 T2(0.2);

//******************************************************************************
TEST(Rot2 , Invariants) {
  Rot2 id;

  EXPECT(check_group_invariants(id,id));
  EXPECT(check_group_invariants(id,T1));
  EXPECT(check_group_invariants(T2,id));
  EXPECT(check_group_invariants(T2,T1));

  EXPECT(check_manifold_invariants(id,id));
  EXPECT(check_manifold_invariants(id,T1));
  EXPECT(check_manifold_invariants(T2,id));
  EXPECT(check_manifold_invariants(T2,T1));

}

//******************************************************************************
TEST(Rot2 , LieGroupDerivatives) {
  Rot2 id;

  CHECK_LIE_GROUP_DERIVATIVES(id,id);
  CHECK_LIE_GROUP_DERIVATIVES(id,T2);
  CHECK_LIE_GROUP_DERIVATIVES(T2,id);
  CHECK_LIE_GROUP_DERIVATIVES(T2,T1);

}

//******************************************************************************
TEST(Rot2 , ChartDerivatives) {
  Rot2 id;

  CHECK_CHART_DERIVATIVES(id,id);
  CHECK_CHART_DERIVATIVES(id,T2);
  CHECK_CHART_DERIVATIVES(T2,id);
  CHECK_CHART_DERIVATIVES(T2,T1);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */


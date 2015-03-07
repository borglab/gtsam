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

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Rot2.h>

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
	CHECK(assert_equal(R.inverse().matrix(),R.transpose()));
}

/* ************************************************************************* */
TEST( Rot2, compose)
{
	CHECK(assert_equal(Rot2::fromAngle(0.45), Rot2::fromAngle(0.2)*Rot2::fromAngle(0.25)));
	CHECK(assert_equal(Rot2::fromAngle(0.45), Rot2::fromAngle(0.25)*Rot2::fromAngle(0.2)));

	Matrix H1, H2;
	(void) Rot2::fromAngle(1.0).compose(Rot2::fromAngle(2.0), H1, H2);
	EXPECT(assert_equal(eye(1), H1));
  EXPECT(assert_equal(eye(1), H2));
}

/* ************************************************************************* */
TEST( Rot2, between)
{
  CHECK(assert_equal(Rot2::fromAngle(0.05), Rot2::fromAngle(0.2).between(Rot2::fromAngle(0.25))));
  CHECK(assert_equal(Rot2::fromAngle(-0.05), Rot2::fromAngle(0.25).between(Rot2::fromAngle(0.2))));

  Matrix H1, H2;
  (void) Rot2::fromAngle(1.0).between(Rot2::fromAngle(2.0), H1, H2);
  EXPECT(assert_equal(-eye(1), H1));
  EXPECT(assert_equal(eye(1), H2));
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
	Vector v = zero(1);
	CHECK(assert_equal(R.retract(v), R));
}

/* ************************************************************************* */
TEST(Rot2, logmap)
{
	Rot2 rot0(Rot2::fromAngle(M_PI/2.0));
	Rot2 rot(Rot2::fromAngle(M_PI));
	Vector expected = Vector_(1, M_PI/2.0);
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

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */


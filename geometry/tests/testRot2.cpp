/**
 * @file    testRot2.cpp
 * @brief   Unit tests for Rot2 class
 * @author  Frank Dellaert
 */

#include <gtsam/CppUnitLite/TestHarness.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Rot2.h>

using namespace gtsam;

Rot2 R(Rot2::fromAngle(0.1));
Point2 P(0.2, 0.7);

/* ************************************************************************* */
TEST( Rot2, constructors_and_angle)
{
	double c=cos(0.1), s=sin(0.1);
	DOUBLES_EQUAL(0.1,R.theta(),1e-9);
	CHECK(assert_equal(R,Rot2::fromCosSin(c,s)));
	CHECK(assert_equal(R,Rot2::atan2(s*5,c*5)));
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
	CHECK(assert_equal(R.expmap(v), R));
}

/* ************************************************************************* */
TEST(Rot2, logmap)
{
	Rot2 rot0(Rot2::fromAngle(M_PI_2));
	Rot2 rot(Rot2::fromAngle(M_PI));
	Vector expected = Vector_(1, M_PI_2);
	Vector actual = rot0.logmap(rot);
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
	expectedH = numericalDerivative11(relativeBearing_, l1, 1e-5);
	CHECK(assert_equal(expectedH,actualH));

	// establish relativeBearing is indeed 45 degrees
	Rot2 actual2 = Rot2::relativeBearing(l2, actualH);
	CHECK(assert_equal(Rot2::fromAngle(M_PI_4),actual2));

	// Check numerical derivative
	expectedH = numericalDerivative11(relativeBearing_, l2, 1e-5);
	CHECK(assert_equal(expectedH,actualH));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */


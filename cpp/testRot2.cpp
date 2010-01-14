/**
 * @file    testRot2.cpp
 * @brief   Unit tests for Rot2 class
 * @author  Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>
#include "numericalDerivative.h"
#include "Rot2.h"

using namespace gtsam;

Rot2 R(0.1);
Point2 P(0.2, 0.7);

/* ************************************************************************* */
TEST( Rot2, angle)
{
	DOUBLES_EQUAL(0.1,R.theta(),1e-9);
}

/* ************************************************************************* */
TEST( Rot2, transpose)
{
	CHECK(assert_equal(inverse(R).matrix(),R.transpose()));
}

/* ************************************************************************* */
TEST( Rot2, negtranspose)
{
	CHECK(assert_equal(-inverse(R).matrix(),R.negtranspose()));
}

/* ************************************************************************* */
TEST( Rot2, compose)
{
	CHECK(assert_equal(Rot2(0.45), Rot2(0.2)*Rot2(0.25)));
	CHECK(assert_equal(Rot2(0.45), Rot2(0.25)*Rot2(0.2)));
}

/* ************************************************************************* */
TEST( Rot2, invcompose)
{
	CHECK(assert_equal(Rot2(0.2), invcompose(Rot2(0.25),Rot2(0.45))));
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
	CHECK(assert_equal(expmap(R,v), R));
}

/* ************************************************************************* */
TEST(Rot2, logmap)
{
	Rot2 rot0(M_PI_2);
	Rot2 rot(M_PI);
	Vector expected = Vector_(1, M_PI_2);
	Vector actual = logmap(rot0, rot);
	CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */
// rotate and derivatives
inline Point2 rotate_(const Rot2 & R, const Point2& p) {return R.rotate(p);}
TEST( Rot2, rotate)
{
	Matrix H1, H2;
	Point2 actual = rotate(R, P, H1, H2);
	CHECK(assert_equal(actual,R*P));
	Matrix numerical1 = numericalDerivative21(rotate_, R, P);
	CHECK(assert_equal(numerical1,H1));
	Matrix numerical2 = numericalDerivative22(rotate_, R, P);
	CHECK(assert_equal(numerical2,H2));
}

/* ************************************************************************* */
// unrotate and derivatives
inline Point2 unrotate_(const Rot2 & R, const Point2& p) {return R.unrotate(p);}
TEST( Rot2, unrotate)
{
	Matrix H1, H2;
	Point2 w = R * P, actual = unrotate(R, w, H1, H2);
	CHECK(assert_equal(actual,P));
	Matrix numerical1 = numericalDerivative21(unrotate_, R, w);
	CHECK(assert_equal(numerical1,H1));
	Matrix numerical2 = numericalDerivative22(unrotate_, R, w);
	CHECK(assert_equal(numerical2,H2));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */


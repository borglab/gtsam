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
  Vector actual = logmap(rot0,rot);
  CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */
// rotate derivatives

TEST( Rot2, Drotate1)
{
	Matrix computed = Drotate1(R, P);
	Matrix numerical = numericalDerivative21(rotate, R, P);
	CHECK(assert_equal(numerical,computed));
}

TEST( Rot2, Drotate2_DNrotate2)
{
	Matrix computed = Drotate2(R);
	Matrix numerical = numericalDerivative22(rotate, R, P);
	CHECK(assert_equal(numerical,computed));
}

/* ************************************************************************* */
// unrotate 

TEST( Rot2, unrotate)
{
	Point2 w = R * P;
	CHECK(assert_equal(unrotate(R,w),P));
}

/* ************************************************************************* */
// unrotate derivatives

TEST( Rot2, Dunrotate1)
{
	Matrix computed = Dunrotate1(R, P);
	Matrix numerical = numericalDerivative21(unrotate, R, P);
	CHECK(assert_equal(numerical,computed));
}

TEST( Rot2, Dunrotate2_DNunrotate2)
{
	Matrix computed = Dunrotate2(R);
	Matrix numerical = numericalDerivative22(unrotate, R, P);
	CHECK(assert_equal(numerical,computed));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */


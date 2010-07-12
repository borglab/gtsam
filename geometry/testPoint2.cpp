/**
 * @file   testPoint2.cpp
 * @brief  Unit tests for Point2 class
 * @author Frank Dellaert
 **/

#include <CppUnitLite/TestHarness.h>
#include "Point2.h"

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST( Point2, expmap)
{
	Vector d(2);
	d(0) = 1;
	d(1) = -1;
	Point2 a(4, 5), b = expmap(a, d), c(5, 4);
	CHECK(assert_equal(b,c));
}

/* ************************************************************************* */
TEST( Point2, arithmetic)
{
	CHECK(assert_equal( Point2(-5,-6), -Point2(5,6) ));
	CHECK(assert_equal( Point2(5,6), Point2(4,5)+Point2(1,1)));
	CHECK(assert_equal( Point2(3,4), Point2(4,5)-Point2(1,1) ));
	CHECK(assert_equal( Point2(8,6), Point2(4,3)*2));
	CHECK(assert_equal( Point2(4,6), 2*Point2(2,3)));
	CHECK(assert_equal( Point2(2,3), Point2(4,6)/2));
}

/* ************************************************************************* */
TEST( Point2, norm)
{
	Point2 p0(cos(5), sin(5));
	DOUBLES_EQUAL(1,p0.norm(),1e-6);
	Point2 p1(4, 5), p2(1, 1);
	DOUBLES_EQUAL( 5,p1.dist(p2),1e-6);
	DOUBLES_EQUAL( 5,(p2-p1).norm(),1e-6);
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

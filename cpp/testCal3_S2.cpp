/**
 * @file  testCal3_S2.cpp
 * @brief Unit tests for transform derivatives
 */

#include <CppUnitLite/TestHarness.h>
#include "numericalDerivative.h"
#include "Cal3_S2.h"

using namespace gtsam;

Cal3_S2 K(500, 500, 0.1, 640 / 2, 480 / 2);
Point2 p(1, -2);

/* ************************************************************************* */
TEST( Cal3_S2, easy_constructor)
{
	Cal3_S2 expected(369.504, 369.504, 0, 640 / 2, 480 / 2);

	double fov = 60; // degrees
	size_t w=640,h=480;
	Cal3_S2 actual(fov,w,h);

	CHECK(assert_equal(expected,actual,1e-3));
}

/* ************************************************************************* */
TEST( Cal3_S2, Duncalibrate1)
{
	Matrix computed = Duncalibrate1(K, p);
	Matrix numerical = numericalDerivative21(uncalibrate, K, p);
	CHECK(assert_equal(numerical,computed,1e-8));
}

/* ************************************************************************* */
TEST( Cal3_S2, Duncalibrate2)
{
	Matrix computed = Duncalibrate2(K, p);
	Matrix numerical = numericalDerivative22(uncalibrate, K, p);
	CHECK(assert_equal(numerical,computed,1e-9));
}

/* ************************************************************************* */
TEST( Cal3_S2, assert_equal)
{
	CHECK(assert_equal(K,K,1e-9));

	Cal3_S2 K1(500, 500, 0.1, 640 / 2, 480 / 2);
	CHECK(assert_equal(K,K1,1e-9));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */


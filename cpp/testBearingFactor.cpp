/**
 *  @file  testBearingFactor.cpp
 *  @brief Unit tests for BearingFactor Class
 *  @authors Frank Dellaert, Viorela Ila
 **/

#include <boost/bind.hpp>
#include <CppUnitLite/TestHarness.h>
#include "numericalDerivative.h"
#include "BearingFactor.h"

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST( BearingFactor, relativeBearing )
{
	Matrix expectedH, actualH;

	// establish relativeBearing is indeed zero
	Point2 l1(1, 0);
	CHECK(assert_equal(Rot2(),relativeBearing(l1)));

	// Check numerical derivative
	expectedH = numericalDerivative11(relativeBearing, l1, 1e-5);
	actualH = DrelativeBearing(l1);
	CHECK(assert_equal(expectedH,actualH));

	// establish relativeBearing is indeed 45 degrees
	Point2 l2(1, 1);
	CHECK(assert_equal(Rot2(M_PI_4),relativeBearing(l2)));

	// Check numerical derivative
	expectedH = numericalDerivative11(relativeBearing, l2, 1e-5);
	actualH = DrelativeBearing(l2);
	CHECK(assert_equal(expectedH,actualH));
}

/* ************************************************************************* */
TEST( BearingFactor, bearing )
{
	Matrix expectedH1, actualH1, expectedH2, actualH2;

	// establish bearing is indeed zero
	Pose2 x1;
	Point2 l1(1, 0);
	CHECK(assert_equal(Rot2(),bearing(x1,l1)));

	// establish bearing is indeed 45 degrees
	Point2 l2(1, 1);
	CHECK(assert_equal(Rot2(M_PI_4),bearing(x1,l2)));

	// establish bearing is indeed 45 degrees even if shifted
	Pose2 x2(1, 1, 0);
	Point2 l3(2, 2);
	CHECK(assert_equal(Rot2(M_PI_4),bearing(x2,l3)));

	// Check numerical derivatives
	expectedH1 = numericalDerivative21(bearing, x2, l3, 1e-5);
	actualH1 = Dbearing1(x2, l3);
	CHECK(assert_equal(expectedH1,actualH1));
	expectedH2 = numericalDerivative22(bearing, x2, l3, 1e-5);
	actualH2 = Dbearing1(x2, l3);
	CHECK(assert_equal(expectedH1,actualH1));

	// establish bearing is indeed 45 degrees even if rotated
	Pose2 x3(1, 1, M_PI_4);
	Point2 l4(1, 3);
	CHECK(assert_equal(Rot2(M_PI_4),bearing(x3,l4)));

	// Check numerical derivatives, optional style
	expectedH1 = numericalDerivative21(bearing, x3, l4, 1e-5);
	expectedH2 = numericalDerivative22(bearing, x3, l4, 1e-5);
	bearing(x3, l4, actualH1, actualH2);
	CHECK(assert_equal(expectedH1,actualH1));
	CHECK(assert_equal(expectedH1,actualH1));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

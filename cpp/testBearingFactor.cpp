/**
 *  @file  testBearingFactor.cpp
 *  @brief Unit tests for BearingFactor Class
 *  @authors Frank Dellaert
 **/

#include <CppUnitLite/TestHarness.h>

#include "numericalDerivative.h"
#include "BearingFactor.h"
#include "TupleConfig.h"

using namespace std;
using namespace gtsam;

// typedefs
typedef Symbol<Pose2, 'x'> PoseKey;
typedef Symbol<Point2, 'l'> PointKey;
typedef PairConfig<PoseKey, Pose2, PointKey, Point2> Config;
typedef BearingFactor<Config, PoseKey, PointKey> MyFactor;

// some shared test values
Pose2 x1, x2(1, 1, 0), x3(1, 1, M_PI_4);
Point2 l1(1, 0), l2(1, 1), l3(2, 2), l4(1, 3);

/* ************************************************************************* */
TEST( BearingFactor, relativeBearing )
{
	Matrix expectedH, actualH;

	// establish relativeBearing is indeed zero
	Rot2 actual1 = relativeBearing(l1, actualH);
	CHECK(assert_equal(Rot2(),actual1));

	// Check numerical derivative
	expectedH = numericalDerivative11(relativeBearing, l1, 1e-5);
	CHECK(assert_equal(expectedH,actualH));

	// establish relativeBearing is indeed 45 degrees
	Rot2 actual2 = relativeBearing(l2, actualH);
	CHECK(assert_equal(Rot2(M_PI_4),actual2));

	// Check numerical derivative
	expectedH = numericalDerivative11(relativeBearing, l2, 1e-5);
	CHECK(assert_equal(expectedH,actualH));
}

/* ************************************************************************* */
TEST( BearingFactor, bearing )
{
	Matrix expectedH1, actualH1, expectedH2, actualH2;

	// establish bearing is indeed zero
	CHECK(assert_equal(Rot2(),bearing(x1,l1)));

	// establish bearing is indeed 45 degrees
	CHECK(assert_equal(Rot2(M_PI_4),bearing(x1,l2)));

	// establish bearing is indeed 45 degrees even if shifted
	Rot2 actual23 = bearing(x2, l3, actualH1, actualH2);
	CHECK(assert_equal(Rot2(M_PI_4),actual23));

	// Check numerical derivatives
	expectedH1 = numericalDerivative21(bearing, x2, l3, 1e-5);
	CHECK(assert_equal(expectedH1,actualH1));
	expectedH2 = numericalDerivative22(bearing, x2, l3, 1e-5);
	CHECK(assert_equal(expectedH1,actualH1));

	// establish bearing is indeed 45 degrees even if rotated
	Rot2 actual34 = bearing(x3, l4, actualH1, actualH2);
	CHECK(assert_equal(Rot2(M_PI_4),actual34));

	// Check numerical derivatives
	expectedH1 = numericalDerivative21(bearing, x3, l4, 1e-5);
	expectedH2 = numericalDerivative22(bearing, x3, l4, 1e-5);
	CHECK(assert_equal(expectedH1,actualH1));
	CHECK(assert_equal(expectedH1,actualH1));
}

/* ************************************************************************* */
TEST( BearingFactor, evaluateError )
{
	// Create factor
	Rot2 z(M_PI_4+0.1); // h(x) - z = -0.1
	double sigma = 0.1;
	MyFactor factor(z, sigma, 2, 3);

	// create config
	Config c;
	c.insert(2, x2);
	c.insert(3, l3);

	// Check error
	Vector actual = factor.error_vector(c);
	CHECK(assert_equal(Vector_(1,-0.1),actual));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

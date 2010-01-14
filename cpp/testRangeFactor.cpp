/**
 *  @file  testRangeFactor.cpp
 *  @brief Unit tests for RangeFactor Class
 *  @authors Frank Dellaert
 **/

#include <CppUnitLite/TestHarness.h>

#include "numericalDerivative.h"
#include "RangeFactor.h"
#include "TupleConfig.h"

using namespace std;
using namespace gtsam;

// typedefs
typedef Symbol<Pose2, 'x'> PoseKey;
typedef Symbol<Point2, 'l'> PointKey;
typedef PairConfig<PoseKey, Pose2, PointKey, Point2> Config;
typedef RangeFactor<Config, PoseKey, PointKey> MyFactor;

// some shared test values
Pose2 x1, x2(1, 1, 0), x3(1, 1, M_PI_4);
Point2 l1(1, 0), l2(1, 1), l3(2, 2), l4(1, 3);

/* ************************************************************************* */
TEST( RangeFactor, range )
{
	Matrix expectedH1, actualH1, expectedH2, actualH2;

	// establish range is indeed zero
	DOUBLES_EQUAL(1,gtsam::range(x1,l1),1e-9);

	// establish range is indeed 45 degrees
	DOUBLES_EQUAL(sqrt(2),gtsam::range(x1,l2),1e-9);

	// Another pair
	double actual23 = gtsam::range(x2, l3, actualH1, actualH2);
	DOUBLES_EQUAL(sqrt(2),actual23,1e-9);

	// Check numerical derivatives
	expectedH1 = numericalDerivative21(range, x2, l3, 1e-5);
	CHECK(assert_equal(expectedH1,actualH1));
	expectedH2 = numericalDerivative22(range, x2, l3, 1e-5);
	CHECK(assert_equal(expectedH1,actualH1));

	// Another test
	double actual34 = gtsam::range(x3, l4, actualH1, actualH2);
	DOUBLES_EQUAL(2,actual34,1e-9);

	// Check numerical derivatives
	expectedH1 = numericalDerivative21(range, x3, l4, 1e-5);
	expectedH2 = numericalDerivative22(range, x3, l4, 1e-5);
	CHECK(assert_equal(expectedH1,actualH1));
	CHECK(assert_equal(expectedH1,actualH1));
}

/* ************************************************************************* */
TEST( RangeFactor, evaluateError )
{
	// Create factor
	double z(sqrt(2) - 0.22); // h(x) - z = 0.22
	double sigma = 0.1;
	MyFactor factor(z, sigma, 2, 3);

	// create config
	Config c;
	c.insert(2, x2);
	c.insert(3, l3);

	// Check error
	Vector actual = factor.error_vector(c);
	CHECK(assert_equal(Vector_(1,0.22),actual));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

/**
 *  @file  testBearingRange.cpp
 *  @authors Frank Dellaert
 **/

#include <iostream>
#include <CppUnitLite/TestHarness.h>
#include <gtsam/BearingFactor.h>
#include <gtsam/RangeFactor.h>
#include <gtsam/TupleConfig.h>
#include <gtsam/FactorGraph-inl.h>
#include <gtsam/NonlinearFactorGraph-inl.h>

using namespace std;
using namespace gtsam;

// typedefs
typedef Symbol<Pose2, 'x'> PoseKey;
typedef Symbol<Point2, 'l'> PointKey;
typedef PairConfig<PoseKey, Pose2, PointKey, Point2> Config;
typedef BearingFactor<Config, PoseKey, PointKey> BearingMeasurement;
typedef RangeFactor<Config, PoseKey, PointKey> RangeMeasurement;
typedef NonlinearFactorGraph<Config> Graph;

// some shared test values
Pose2 x1, x2(1, 1, 0), x3(1, 1, M_PI_4);
Point2 l1(1, 0), l2(1, 1), l3(2, 2), l4(1, 3);

/* ************************************************************************* */
TEST( BearingRange, constructor )
{
	// create config
	Config c;
	c.insert(2, x2);
	c.insert(3, l3);

	// create graph
	Graph G;

	// Create bearing factor
	Rot2 z1(M_PI_4 + 0.1); // h(x) - z = -0.1
	double sigma1 = 0.1;
	Graph::sharedFactor factor1(new BearingMeasurement(z1, sigma1, 2, 3));
	CHECK(assert_equal(Vector_(1,-0.1),factor1->error_vector(c)));
	G.push_back(factor1);

	// Create range factor
	double z2(sqrt(2) - 0.22); // h(x) - z = 0.22
	double sigma2 = 0.1;
	Graph::sharedFactor factor2(new RangeMeasurement(z2, sigma2, 2, 3));
	CHECK(assert_equal(Vector_(1,0.22),factor2->error_vector(c)));
	G.push_back(factor2);

	CHECK(assert_equal(Vector_(2,-0.1,0.22),G.error_vector(c)));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

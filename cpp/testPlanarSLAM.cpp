/**
 *  @file  testPlanarSLAM.cpp
 *  @authors Frank Dellaert
 **/

#include <iostream>
#include <CppUnitLite/TestHarness.h>

#include "planarSLAM.h"

using namespace std;
using namespace gtsam;

// some shared test values
static Pose2 x1, x2(1, 1, 0), x3(1, 1, M_PI_4);
static Point2 l1(1, 0), l2(1, 1), l3(2, 2), l4(1, 3);

SharedGaussian
	sigma(noiseModel::Isotropic::Sigma(1,0.1)),
	I3(noiseModel::Unit::Create(3));

/* ************************************************************************* */
TEST( planarSLAM, BearingFactor )
{
	// Create factor
	Rot2 z = Rot2::fromAngle(M_PI_4 + 0.1); // h(x) - z = -0.1
	planarSLAM::Bearing factor(2, 3, z, sigma);

	// create config
	planarSLAM::Config c;
	c.insert(2, x2);
	c.insert(3, l3);

	// Check error
	Vector actual = factor.unwhitenedError(c);
	CHECK(assert_equal(Vector_(1,-0.1),actual));
}

/* ************************************************************************* */
TEST( planarSLAM, RangeFactor )
{
	// Create factor
	double z(sqrt(2) - 0.22); // h(x) - z = 0.22
	planarSLAM::Range factor(2, 3, z, sigma);

	// create config
	planarSLAM::Config c;
	c.insert(2, x2);
	c.insert(3, l3);

	// Check error
	Vector actual = factor.unwhitenedError(c);
	CHECK(assert_equal(Vector_(1,0.22),actual));
}

/* ************************************************************************* */
TEST( planarSLAM, constructor )
{
	// create config
	planarSLAM::Config c;
	c.insert(2, x2);
	c.insert(3, x3);
	c.insert(3, l3);

	// create graph
	planarSLAM::Graph G;

	// Add pose constraint
	G.addPoseConstraint(2, x2); // make it feasible :-)

	// Add odometry
	G.addOdometry(2, 3, Pose2(0, 0, M_PI_4), I3);

	// Create bearing factor
	Rot2 z1 = Rot2::fromAngle(M_PI_4 + 0.1); // h(x) - z = -0.1
	G.addBearing(2, 3, z1, sigma);

	// Create range factor
	double z2(sqrt(2) - 0.22); // h(x) - z = 0.22
	G.addRange(2, 3, z2, sigma);

	Vector expected = Vector_(8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.1, 0.22);
	CHECK(assert_equal(expected,G.unwhitenedError(c)));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

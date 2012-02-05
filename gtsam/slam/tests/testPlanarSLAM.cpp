/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  testPlanarSLAM.cpp
 *  @author Frank Dellaert
 **/

#include <iostream>
#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/TestableAssertions.h>
#include <gtsam/slam/planarSLAM.h>
#include <gtsam/slam/BearingRangeFactor.h>

using namespace std;
using namespace gtsam;
using namespace planarSLAM;

// some shared test values
static Pose2 x1, x2(1, 1, 0), x3(1, 1, M_PI_4);
static Point2 l1(1, 0), l2(1, 1), l3(2, 2), l4(1, 3);

SharedNoiseModel
	sigma(noiseModel::Isotropic::Sigma(1,0.1)),
	sigma2(noiseModel::Isotropic::Sigma(2,0.1)),
	I3(noiseModel::Unit::Create(3));

/* ************************************************************************* */
TEST( planarSLAM, PriorFactor_equals )
{
	planarSLAM::Prior factor1(2, x1, I3), factor2(2, x2, I3);
	EXPECT(assert_equal(factor1, factor1, 1e-5));
	EXPECT(assert_equal(factor2, factor2, 1e-5));
	EXPECT(assert_inequal(factor1, factor2, 1e-5));
}

/* ************************************************************************* */
TEST( planarSLAM, BearingFactor )
{
	// Create factor
	Rot2 z = Rot2::fromAngle(M_PI_4 + 0.1); // h(x) - z = -0.1
	planarSLAM::Bearing factor(2, 3, z, sigma);

	// create config
	planarSLAM::Values c;
	c.insert(PoseKey(2), x2);
	c.insert(PointKey(3), l3);

	// Check error
	Vector actual = factor.unwhitenedError(c);
	EXPECT(assert_equal(Vector_(1,-0.1),actual));
}

/* ************************************************************************* */
TEST( planarSLAM, BearingFactor_equals )
{
	planarSLAM::Bearing
		factor1(2, 3, Rot2::fromAngle(0.1), sigma),
		factor2(2, 3, Rot2::fromAngle(2.3), sigma);
	EXPECT(assert_equal(factor1, factor1, 1e-5));
	EXPECT(assert_equal(factor2, factor2, 1e-5));
	EXPECT(assert_inequal(factor1, factor2, 1e-5));
}

/* ************************************************************************* */
TEST( planarSLAM, RangeFactor )
{
	// Create factor
	double z(sqrt(2) - 0.22); // h(x) - z = 0.22
	planarSLAM::Range factor(2, 3, z, sigma);

	// create config
	planarSLAM::Values c;
	c.insert(PoseKey(2), x2);
	c.insert(PointKey(3), l3);

	// Check error
	Vector actual = factor.unwhitenedError(c);
	EXPECT(assert_equal(Vector_(1,0.22),actual));
}

/* ************************************************************************* */
TEST( planarSLAM, RangeFactor_equals )
{
	planarSLAM::Range factor1(2, 3, 1.2, sigma), factor2(2, 3, 7.2, sigma);
	EXPECT(assert_equal(factor1, factor1, 1e-5));
	EXPECT(assert_equal(factor2, factor2, 1e-5));
	EXPECT(assert_inequal(factor1, factor2, 1e-5));
}

/* ************************************************************************* */
TEST( planarSLAM, BearingRangeFactor )
{
	// Create factor
	Rot2 r = Rot2::fromAngle(M_PI_4 + 0.1); // h(x) - z = -0.1
	double b(sqrt(2) - 0.22); // h(x) - z = 0.22
	planarSLAM::BearingRange factor(2, 3, r, b, sigma2);

	// create config
	planarSLAM::Values c;
	c.insert(PoseKey(2), x2);
	c.insert(PointKey(3), l3);

	// Check error
	Vector actual = factor.unwhitenedError(c);
	EXPECT(assert_equal(Vector_(2,-0.1, 0.22),actual));
}

/* ************************************************************************* */
TEST( planarSLAM, BearingRangeFactor_equals )
{
	planarSLAM::BearingRange
		factor1(2, 3, Rot2::fromAngle(0.1), 7.3,  sigma2),
		factor2(2, 3, Rot2::fromAngle(3), 2.0, sigma2);
	EXPECT(assert_equal(factor1, factor1, 1e-5));
	EXPECT(assert_equal(factor2, factor2, 1e-5));
	EXPECT(assert_inequal(factor1, factor2, 1e-5));
}

/* ************************************************************************* */
TEST( planarSLAM, PoseConstraint_equals )
{
	planarSLAM::Constraint factor1(2, x2), factor2(2, x3);
	EXPECT(assert_equal(factor1, factor1, 1e-5));
	EXPECT(assert_equal(factor2, factor2, 1e-5));
	EXPECT(assert_inequal(factor1, factor2, 1e-5));
}

/* ************************************************************************* */
TEST( planarSLAM, constructor )
{
	// create config
	planarSLAM::Values c;
	c.insert(PoseKey(2), x2);
	c.insert(PoseKey(3), x3);
	c.insert(PointKey(3), l3);

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

	Vector expected0 = Vector_(3, 0.0, 0.0, 0.0);
	Vector expected1 = Vector_(3, 0.0, 0.0, 0.0);
	Vector expected2 = Vector_(1, -0.1);
	Vector expected3 = Vector_(1, 0.22);
	// Get NoiseModelFactors
	FactorGraph<NoiseModelFactor > GNM =
	    *G.dynamicCastFactors<FactorGraph<NoiseModelFactor > >();
	EXPECT(assert_equal(expected0, GNM[0]->unwhitenedError(c)));
  EXPECT(assert_equal(expected1, GNM[1]->unwhitenedError(c)));
  EXPECT(assert_equal(expected2, GNM[2]->unwhitenedError(c)));
  EXPECT(assert_equal(expected3, GNM[3]->unwhitenedError(c)));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

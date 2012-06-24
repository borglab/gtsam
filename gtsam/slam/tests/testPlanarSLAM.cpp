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

#include <gtsam/slam/planarSLAM.h>
#include <gtsam/slam/BearingRangeFactor.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/base/TestableAssertions.h>

#include <iostream>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

// some shared test values
static Pose2 x1, x2(1, 1, 0), x3(1, 1, M_PI_4);
static Point2 l1(1, 0), l2(1, 1), l3(2, 2), l4(1, 3);
static Symbol i2('x',2), i3('x',3), j3('l',3);

SharedNoiseModel
	sigma(noiseModel::Isotropic::Sigma(1,0.1)),
	sigma2(noiseModel::Isotropic::Sigma(2,0.1)),
	I3(noiseModel::Unit::Create(3));

/* ************************************************************************* */
TEST( planarSLAM, PriorFactor_equals )
{
	PriorFactor<Pose2> factor1(i2, x1, I3), factor2(i2, x2, I3);
	EXPECT(assert_equal(factor1, factor1, 1e-5));
	EXPECT(assert_equal(factor2, factor2, 1e-5));
	EXPECT(assert_inequal(factor1, factor2, 1e-5));
}

/* ************************************************************************* */
TEST( planarSLAM, BearingFactor )
{
	// Create factor
	Rot2 z = Rot2::fromAngle(M_PI_4 + 0.1); // h(x) - z = -0.1
	BearingFactor<Pose2, Point2> factor(i2, j3, z, sigma);

	// create config
	planarSLAM::Values c;
	c.insert(i2, x2);
	c.insert(j3, l3);

	// Check error
	Vector actual = factor.unwhitenedError(c);
	EXPECT(assert_equal(Vector_(1,-0.1),actual));
}

/* ************************************************************************* */
TEST( planarSLAM, BearingFactor_equals )
{
	BearingFactor<Pose2, Point2>
		factor1(i2, j3, Rot2::fromAngle(0.1), sigma),
		factor2(i2, j3, Rot2::fromAngle(2.3), sigma);
	EXPECT(assert_equal(factor1, factor1, 1e-5));
	EXPECT(assert_equal(factor2, factor2, 1e-5));
	EXPECT(assert_inequal(factor1, factor2, 1e-5));
}

/* ************************************************************************* */
TEST( planarSLAM, RangeFactor )
{
	// Create factor
	double z(sqrt(2.0) - 0.22); // h(x) - z = 0.22
	RangeFactor<Pose2, Point2> factor(i2, j3, z, sigma);

	// create config
	planarSLAM::Values c;
	c.insert(i2, x2);
	c.insert(j3, l3);

	// Check error
	Vector actual = factor.unwhitenedError(c);
	EXPECT(assert_equal(Vector_(1,0.22),actual));
}

/* ************************************************************************* */
TEST( planarSLAM, RangeFactor_equals )
{
	RangeFactor<Pose2, Point2> factor1(i2, j3, 1.2, sigma), factor2(i2, j3, 7.2, sigma);
	EXPECT(assert_equal(factor1, factor1, 1e-5));
	EXPECT(assert_equal(factor2, factor2, 1e-5));
	EXPECT(assert_inequal(factor1, factor2, 1e-5));
}

/* ************************************************************************* */
TEST( planarSLAM, BearingRangeFactor )
{
	// Create factor
	Rot2 r = Rot2::fromAngle(M_PI_4 + 0.1); // h(x) - z = -0.1
	double b(sqrt(2.0) - 0.22); // h(x) - z = 0.22
	BearingRangeFactor<Pose2, Point2> factor(i2, j3, r, b, sigma2);

	// create config
	planarSLAM::Values c;
	c.insert(i2, x2);
	c.insert(j3, l3);

	// Check error
	Vector actual = factor.unwhitenedError(c);
	EXPECT(assert_equal(Vector_(2,-0.1, 0.22),actual));
}

/* ************************************************************************* */
TEST( planarSLAM, BearingRangeFactor_equals )
{
	BearingRangeFactor<Pose2, Point2>
		factor1(i2, j3, Rot2::fromAngle(0.1), 7.3,  sigma2),
		factor2(i2, j3, Rot2::fromAngle(3), 2.0, sigma2);
	EXPECT(assert_equal(factor1, factor1, 1e-5));
	EXPECT(assert_equal(factor2, factor2, 1e-5));
	EXPECT(assert_inequal(factor1, factor2, 1e-5));
}

/* ************************************************************************* */
TEST( planarSLAM, BearingRangeFactor_poses )
{
	typedef BearingRangeFactor<Pose2,Pose2> PoseBearingRange;
	PoseBearingRange actual(2, 3, Rot2::fromDegrees(60.0), 12.3, sigma2);
}

/* ************************************************************************* */
TEST( planarSLAM, PoseConstraint_equals )
{
	NonlinearEquality<Pose2> factor1(i2, x2), factor2(i2, x3);
	EXPECT(assert_equal(factor1, factor1, 1e-5));
	EXPECT(assert_equal(factor2, factor2, 1e-5));
	EXPECT(assert_inequal(factor1, factor2, 1e-5));
}

/* ************************************************************************* */
TEST( planarSLAM, constructor )
{
	// create config
	planarSLAM::Values c;
	c.insert(i2, x2);
	c.insert(i3, x3);
	c.insert(j3, l3);

	// create graph
	planarSLAM::Graph G;

	// Add pose constraint
	G.addPoseConstraint(i2, x2); // make it feasible :-)

	// Add odometry
	G.addRelativePose(i2, i3, Pose2(0, 0, M_PI_4), I3);

	// Create bearing factor
	Rot2 z1 = Rot2::fromAngle(M_PI_4 + 0.1); // h(x) - z = -0.1
	G.addBearing(i2, j3, z1, sigma);

	// Create range factor
	double z2(sqrt(2.0) - 0.22); // h(x) - z = 0.22
	G.addRange(i2, j3, z2, sigma);

	Vector expected0 = Vector_(3, 0.0, 0.0, 0.0);
	Vector expected1 = Vector_(3, 0.0, 0.0, 0.0);
	Vector expected2 = Vector_(1, -0.1);
	Vector expected3 = Vector_(1, 0.22);
	// Get NoiseModelFactors
	EXPECT(assert_equal(expected0, boost::dynamic_pointer_cast<NoiseModelFactor>(G[0])->unwhitenedError(c)));
  EXPECT(assert_equal(expected1, boost::dynamic_pointer_cast<NoiseModelFactor>(G[1])->unwhitenedError(c)));
  EXPECT(assert_equal(expected2, boost::dynamic_pointer_cast<NoiseModelFactor>(G[2])->unwhitenedError(c)));
  EXPECT(assert_equal(expected3, boost::dynamic_pointer_cast<NoiseModelFactor>(G[3])->unwhitenedError(c)));
}

/* ************************************************************************* */
TEST( planarSLAM, keys_and_view )
{
	// create config
	planarSLAM::Values c;
	c.insert(i2, x2);
	c.insert(i3, x3);
	c.insert(j3, l3);
	LONGS_EQUAL(2,c.nrPoses());
	LONGS_EQUAL(1,c.nrPoints());
	{
	FastList<Key> expected, actual;
	expected += j3, i2, i3;
	actual = c.keys();
	CHECK(expected == actual);
	}
	{
	FastList<Key> expected, actual;
	expected += i2, i3;
	actual = c.poseKeys();
	CHECK(expected == actual);
	}
	{
	FastList<Key> expected, actual;
	expected += j3;
	actual = c.pointKeys();
	CHECK(expected == actual);
	}
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

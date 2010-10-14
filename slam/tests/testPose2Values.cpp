/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  testPose2Values.cpp
 *  @authors Frank Dellaert
 **/

#include <iostream>

#include <gtsam/CppUnitLite/TestHarness.h>
#include <gtsam/slam/pose2SLAM.h>

using namespace std;
using namespace gtsam;
using namespace gtsam::pose2SLAM;

/* ************************************************************************* */
TEST( Pose2Values, pose2Circle )
{
	// expected is 4 poses tangent to circle with radius 1m
	Pose2Values expected;
	expected.insert(0, Pose2( 1,  0,   M_PI_2));
	expected.insert(1, Pose2( 0,  1, - M_PI  ));
	expected.insert(2, Pose2(-1,  0, - M_PI_2));
	expected.insert(3, Pose2( 0, -1,   0     ));

	Pose2Values actual = pose2SLAM::circle(4,1.0);
	CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST( Pose2Values, expmap )
{
	// expected is circle shifted to right
	Pose2Values expected;
	expected.insert(0, Pose2( 1.1,  0,   M_PI_2));
	expected.insert(1, Pose2( 0.1,  1, - M_PI  ));
	expected.insert(2, Pose2(-0.9,  0, - M_PI_2));
	expected.insert(3, Pose2( 0.1, -1,   0     ));

	// Note expmap coordinates are in local coordinates, so shifting to right requires thought !!!
  Pose2Values circle(pose2SLAM::circle(4,1.0));
  Ordering ordering(*circle.orderingArbitrary());
	VectorValues delta(circle.dims(ordering));
	delta[ordering[Key(0)]] = Vector_(3, 0.0,-0.1,0.0);
	delta[ordering[Key(1)]] = Vector_(3, -0.1,0.0,0.0);
	delta[ordering[Key(2)]] = Vector_(3, 0.0,0.1,0.0);
	delta[ordering[Key(3)]] = Vector_(3, 0.1,0.0,0.0);
	Pose2Values actual = circle.expmap(delta, ordering);
	CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

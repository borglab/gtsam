/**
 *  @file  testPose2Config.cpp
 *  @authors Frank Dellaert
 **/

#include <iostream>

#include <CppUnitLite/TestHarness.h>
#include "Pose2Config.h"

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST( Pose2Config, pose2Circle )
{
	// expected is 4 poses tangent to circle with radius 1m
	Pose2Config expected;
	expected.insert("p0", Pose2( 1,  0,   M_PI_2));
	expected.insert("p1", Pose2( 0,  1, - M_PI  ));
	expected.insert("p2", Pose2(-1,  0, - M_PI_2));
	expected.insert("p3", Pose2( 0, -1,   0     ));

	Pose2Config actual = pose2Circle(4,1.0,'p');
	CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST( Pose2Config, expmap )
{
	// expected is circle shifted to right
	Pose2Config expected;
	expected.insert("p0", Pose2( 1.1,  0,   M_PI_2));
	expected.insert("p1", Pose2( 0.1,  1, - M_PI  ));
	expected.insert("p2", Pose2(-0.9,  0, - M_PI_2));
	expected.insert("p3", Pose2( 0.1, -1,   0     ));

	// Note expmap coordinates are in local coordinates, so shifting to right requires thought !!!
	Vector delta = Vector_(12, 0.0,-0.1,0.0, -0.1,0.0,0.0, 0.0,0.1,0.0, 0.1,0.0,0.0);
	Pose2Config actual = expmap(pose2Circle(4,1.0,'p'),delta);
	CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

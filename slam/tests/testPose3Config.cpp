/**
 *  @file  testPose3Config.cpp
 *  @authors Frank Dellaert
 **/

#include <iostream>

#include <CppUnitLite/TestHarness.h>
#include "pose3SLAM.h"

using namespace std;
using namespace gtsam;

// The world coordinate system has z pointing up, y north, x east
// The vehicle has X forward, Y right, Z down
Rot3 R1(Point3( 0, 1, 0), Point3( 1, 0, 0), Point3(0, 0, -1));
Rot3 R2(Point3(-1, 0, 0), Point3( 0, 1, 0), Point3(0, 0, -1));
Rot3 R3(Point3( 0,-1, 0), Point3(-1, 0, 0), Point3(0, 0, -1));
Rot3 R4(Point3( 1, 0, 0), Point3( 0,-1, 0), Point3(0, 0, -1));

/* ************************************************************************* */
TEST( Pose3Config, pose3Circle )
{
	// expected is 4 poses tangent to circle with radius 1m
	Pose3Config expected;
	expected.insert(0, Pose3(R1, Point3( 1, 0, 0)));
	expected.insert(1, Pose3(R2, Point3( 0, 1, 0)));
	expected.insert(2, Pose3(R3, Point3(-1, 0, 0)));
	expected.insert(3, Pose3(R4, Point3( 0,-1, 0)));

	Pose3Config actual = pose3SLAM::circle(4,1.0);
	CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST( Pose3Config, expmap )
{
	Pose3Config expected;
#ifdef CORRECT_POSE3_EXMAP
	expected.insert(0, Pose3(R1, Point3( 1.0, 0.1, 0)));
	expected.insert(1, Pose3(R2, Point3(-0.1, 1.0, 0)));
	expected.insert(2, Pose3(R3, Point3(-1.0,-0.1, 0)));
	expected.insert(3, Pose3(R4, Point3( 0.1,-1.0, 0)));
#else
	// expected is circle shifted to East
	expected.insert(0, Pose3(R1, Point3( 1.1, 0, 0)));
	expected.insert(1, Pose3(R2, Point3( 0.1, 1, 0)));
	expected.insert(2, Pose3(R3, Point3(-0.9, 0, 0)));
	expected.insert(3, Pose3(R4, Point3( 0.1,-1, 0)));
#endif

	// Note expmap coordinates are in global coordinates with non-compose expmap
	// so shifting to East requires little thought, different from with Pose2 !!!
	Vector delta = Vector_(24,
			0.0,0.0,0.0,  0.1, 0.0, 0.0,
			0.0,0.0,0.0,  0.1, 0.0, 0.0,
			0.0,0.0,0.0,  0.1, 0.0, 0.0,
			0.0,0.0,0.0,  0.1, 0.0, 0.0);
	Pose3Config actual = expmap(pose3SLAM::circle(4,1.0),delta);
	CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

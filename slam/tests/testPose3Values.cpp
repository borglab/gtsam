/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  testPose3Values.cpp
 *  @authors Frank Dellaert
 **/

#include <iostream>

#include <gtsam/CppUnitLite/TestHarness.h>
#include <gtsam/slam/pose3SLAM.h>

using namespace std;
using namespace gtsam;

// The world coordinate system has z pointing up, y north, x east
// The vehicle has X forward, Y right, Z down
Rot3 R1(Point3( 0, 1, 0), Point3( 1, 0, 0), Point3(0, 0, -1));
Rot3 R2(Point3(-1, 0, 0), Point3( 0, 1, 0), Point3(0, 0, -1));
Rot3 R3(Point3( 0,-1, 0), Point3(-1, 0, 0), Point3(0, 0, -1));
Rot3 R4(Point3( 1, 0, 0), Point3( 0,-1, 0), Point3(0, 0, -1));

/* ************************************************************************* */
TEST( Pose3Values, pose3Circle )
{
	// expected is 4 poses tangent to circle with radius 1m
	Pose3Values expected;
	expected.insert(0, Pose3(R1, Point3( 1, 0, 0)));
	expected.insert(1, Pose3(R2, Point3( 0, 1, 0)));
	expected.insert(2, Pose3(R3, Point3(-1, 0, 0)));
	expected.insert(3, Pose3(R4, Point3( 0,-1, 0)));

	Pose3Values actual = pose3SLAM::circle(4,1.0);
	CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST( Pose3Values, expmap )
{
	Pose3Values expected;
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

	Ordering ordering(*expected.orderingArbitrary());
	VectorValues delta(expected.dims(ordering), Vector_(24,
			0.0,0.0,0.0,  0.1, 0.0, 0.0,
			0.0,0.0,0.0,  0.1, 0.0, 0.0,
			0.0,0.0,0.0,  0.1, 0.0, 0.0,
			0.0,0.0,0.0,  0.1, 0.0, 0.0));
	Pose3Values actual = pose3SLAM::circle(4,1.0).expmap(delta, ordering);
	CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

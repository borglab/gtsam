/**
 * @file   testPose2.cpp
 * @brief  Unit tests for Pose2 class
 */

#include <CppUnitLite/TestHarness.h>
#include "Pose2.h"

using namespace gtsam;

/* ************************************************************************* */
TEST(Pose2, constructors) {
	Point2 p;
	Pose2 pose(p,0);
	Pose2 origin;
	assert_equal(pose,origin);
}

/* ************************************************************************* */
TEST(Pose2, rotate) {
	double theta = 0.1, c=cos(theta),s=sin(theta);
	Pose2 p1(1,0,0.2), p2(0,1,0.4);
	CHECK(assert_equal(Pose2( c,s,0.3),p1.rotate(theta)));
	CHECK(assert_equal(Pose2(-s,c,0.5),p2.rotate(theta)));
}

/* ************************************************************************* */
TEST(Pose2, operators) {
	CHECK(assert_equal(Pose2(2,2,2),Pose2(1,1,1)+Pose2(1,1,1)));
	CHECK(assert_equal(Pose2(0,0,0),Pose2(1,1,1)-Pose2(1,1,1)));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */


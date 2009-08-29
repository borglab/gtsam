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
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */


/**
 *  @file  testPose3Pose.cpp
 *  @brief Unit tests for Pose3Factor Class
 *  @authors Frank Dellaert
 **/

#include <iostream>
#include <CppUnitLite/TestHarness.h>
#include "Pose3Factor.h"

using namespace std;
using namespace gtsam;

TEST( Pose3Factor, constructor )
{
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

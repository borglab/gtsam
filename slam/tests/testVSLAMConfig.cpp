/*
 * @file testConfig.cpp
 * @brief Tests for the Visual SLAM configuration class
 * @author Alex Cunningham
 */

#include <CppUnitLite/TestHarness.h>

#define GTSAM_MAGIC_KEY

#include "VectorConfig.h"
#include "visualSLAM.h"

using namespace std;
using namespace gtsam;
using namespace gtsam::visualSLAM;

/* ************************************************************************* */
TEST( Config, update_with_large_delta) {
	// this test ensures that if the update for delta is larger than
	// the size of the config, it only updates existing variables
	Config init;
	init.insert(1, Pose3());
	init.insert(1, Point3(1.0, 2.0, 3.0));

	Config expected;
	expected.insert(1, Pose3(Rot3(), Point3(0.1, 0.1, 0.1)));
	expected.insert(1, Point3(1.1, 2.1, 3.1));

	VectorConfig delta;
	delta.insert("x1", Vector_(6, 0.0, 0.0, 0.0, 0.1, 0.1, 0.1));
	delta.insert("l1", Vector_(3, 0.1, 0.1, 0.1));
	delta.insert("x2", Vector_(6, 0.0, 0.0, 0.0, 100.1, 4.1, 9.1));
	Config actual = expmap(init, delta);

	CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */

/*
 * @file testVSLAMConfig.cpp
 * @brief Tests for the Visual SLAM configuration class
 * @author Alex Cunningham
 */

#include <CppUnitLite/TestHarness.h>
#include <VSLAMConfig.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST( VSLAMConfig, update_with_large_delta) {
	// this test ensures that if the update for delta is larger than
	// the size of the config, it only updates existing variables
	VSLAMConfig init;
	init.addCameraPose(1, Pose3());
	init.addLandmarkPoint(1, Point3(1.0, 2.0, 3.0));

	VectorConfig delta;
	delta.insert("x1", Vector_(6, 0.0, 0.0, 0.0, 0.1, 0.1, 0.1));
	delta.insert("l1", Vector_(3, 0.1, 0.1, 0.1));
	delta.insert("x2", Vector_(6, 0.0, 0.0, 0.0, 100.1, 4.1, 9.1));

	VSLAMConfig actual = init.exmap(delta);
	VSLAMConfig expected;
	expected.addCameraPose(1, Pose3(Rot3(), Point3(0.1, 0.1, 0.1)));
	expected.addLandmarkPoint(1, Point3(1.1, 2.1, 3.1));

	CHECK(assert_equal(actual, expected));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */

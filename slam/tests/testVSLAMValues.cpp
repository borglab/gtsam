/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file testValues.cpp
 * @brief Tests for the Visual SLAM values structure class
 * @author Alex Cunningham
 */

#include <gtsam/CppUnitLite/TestHarness.h>

#define GTSAM_MAGIC_KEY

#include <gtsam/linear/VectorValues.h>
#include <gtsam/slam/visualSLAM.h>

using namespace std;
using namespace gtsam;
using namespace gtsam::visualSLAM;

/* ************************************************************************* */
TEST( Values, update_with_large_delta) {
	// this test ensures that if the update for delta is larger than
	// the size of the config, it only updates existing variables
	Values init;
	init.insert(1, Pose3());
	init.insert(1, Point3(1.0, 2.0, 3.0));

	Values expected;
	expected.insert(1, Pose3(Rot3(), Point3(0.1, 0.1, 0.1)));
	expected.insert(1, Point3(1.1, 2.1, 3.1));

	Ordering largeOrdering;
	Values largeValues = init;
	largeValues.insert(2, Pose3());
	largeOrdering += "x1","l1","x2";
	VectorValues delta(largeValues.dims(largeOrdering));
	delta[largeOrdering["x1"]] = Vector_(6, 0.0, 0.0, 0.0, 0.1, 0.1, 0.1);
	delta[largeOrdering["l1"]] = Vector_(3, 0.1, 0.1, 0.1);
	delta[largeOrdering["x2"]] = Vector_(6, 0.0, 0.0, 0.0, 100.1, 4.1, 9.1);
	Values actual = init.expmap(delta, largeOrdering);

	CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */

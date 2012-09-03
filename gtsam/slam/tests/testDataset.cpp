/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testDataset.cpp
 * @brief   Unit test for dataset.cpp
 * @author  Richard Roberts
 */

#include <CppUnitLite/TestHarness.h>

#include <boost/algorithm/string.hpp>

#include <gtsam/base/TestableAssertions.h>
#include <gtsam/slam/dataset.h>

using namespace std;
using namespace gtsam;

TEST(dataSet, findExampleDataFile) {
	const string expected_end = "examples/Data/example.graph";
	const string actual = findExampleDataFile("example");
	string actual_end = actual.substr(actual.size() - expected_end.size(), expected_end.size());
	boost::replace_all(actual_end, "\\", "/"); // Convert directory separators to forward-slash
	EXPECT(assert_equal(expected_end, actual_end));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */

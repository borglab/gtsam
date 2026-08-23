/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file testDiscreteConstraintCompatibility.cpp
 * @brief Test the deprecated GTSAM 4.3 discrete constraint include paths.
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam_unstable/discrete/AllDiff.h>
#include <gtsam_unstable/discrete/BinaryAllDiff.h>
#include <gtsam_unstable/discrete/Constraint.h>
#include <gtsam_unstable/discrete/Domain.h>
#include <gtsam_unstable/discrete/SingleValue.h>

using namespace gtsam;

// Verifies that every deprecated header exposes the canonical constraint types.
TEST(DiscreteConstraintCompatibility, DeprecatedHeaders) {
  const DiscreteKey first(0, 2), second(1, 2);
  const Domain domain(first);
  const SingleValue singleValue(first, 1);
  const BinaryAllDiff binaryAllDiff(first, second);
  const AllDiff allDiff(DiscreteKeys{first, second});
  const Constraint& constraint = allDiff;

  LONGS_EQUAL(2, domain.nrValues());
  DOUBLES_EQUAL(1.0, singleValue(DiscreteValues{{0, 1}}), 1e-9);
  DOUBLES_EQUAL(1.0, binaryAllDiff(DiscreteValues{{0, 0}, {1, 1}}), 1e-9);
  LONGS_EQUAL(2, constraint.size());
}

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}

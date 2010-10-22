/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testVariableIndex.cpp
 * @brief   
 * @author  Richard Roberts
 * @created Sep 26, 2010
 */

#include <gtsam/CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>

#include <gtsam/inference/VariableIndex.h>
#include <gtsam/inference/SymbolicFactorGraph.h>

using namespace gtsam;

/* ************************************************************************* */
TEST(VariableIndex, augment) {

  SymbolicFactorGraph fg1, fg2;
  fg1.push_factor(0, 1);
  fg1.push_factor(0, 2);
  fg1.push_factor(5, 9);
  fg1.push_factor(2, 3);
  fg2.push_factor(1, 3);
  fg2.push_factor(2, 4);
  fg2.push_factor(3, 5);
  fg2.push_factor(5, 6);

  SymbolicFactorGraph fgCombined; fgCombined.push_back(fg1); fgCombined.push_back(fg2);

  VariableIndex expected(fgCombined);
  VariableIndex actual(fg1);
  actual.augment(fg2);

  CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

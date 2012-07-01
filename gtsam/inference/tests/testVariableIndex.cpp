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
 * @date Sep 26, 2010
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>

#include <gtsam/inference/VariableIndex.h>
#include <gtsam/inference/SymbolicFactorGraph.h>

using namespace std;
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

	LONGS_EQUAL(16, actual.nEntries());
	LONGS_EQUAL(8, actual.nFactors());
  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST(VariableIndex, remove) {

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

  // Create a factor graph containing only the factors from fg2 and with null
  // factors in the place of those of fg1, so that the factor indices are correct.
  SymbolicFactorGraph fg2removed(fgCombined);
  fg2removed.remove(0); fg2removed.remove(1); fg2removed.remove(2); fg2removed.remove(3);

  // The expected VariableIndex has the same factor indices as fgCombined but
  // with entries from fg1 removed, and still has all 10 variables.
  VariableIndex expected(fg2removed, 10);
  VariableIndex actual(fgCombined);
  vector<size_t> indices;
  indices.push_back(0); indices.push_back(1); indices.push_back(2); indices.push_back(3);
  actual.remove(indices, fg1);

  CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST(VariableIndex, deep_copy) {

  SymbolicFactorGraph fg1, fg2;
  fg1.push_factor(0, 1);
  fg1.push_factor(0, 2);
  fg1.push_factor(5, 9);
  fg1.push_factor(2, 3);
  fg2.push_factor(1, 3);
  fg2.push_factor(2, 4);
  fg2.push_factor(3, 5);
  fg2.push_factor(5, 6);

  // Create original graph and VariableIndex
  SymbolicFactorGraph fgOriginal; fgOriginal.push_back(fg1); fgOriginal.push_back(fg2);
  VariableIndex original(fgOriginal);
  VariableIndex expectedOriginal(fgOriginal);

  // Create a factor graph containing only the factors from fg2 and with null
  // factors in the place of those of fg1, so that the factor indices are correct.
  SymbolicFactorGraph fg2removed(fgOriginal);
  fg2removed.remove(0); fg2removed.remove(1); fg2removed.remove(2); fg2removed.remove(3);
  VariableIndex expectedRemoved(fg2removed);

  // Create a clone and modify the clone - the original should not change
  VariableIndex clone(original);
  vector<size_t> indices;
  indices.push_back(0); indices.push_back(1); indices.push_back(2); indices.push_back(3);
  clone.remove(indices, fg1);

  // When modifying the clone, the original should have stayed the same
  EXPECT(assert_equal(expectedOriginal, original));
  EXPECT(assert_equal(expectedRemoved, clone));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

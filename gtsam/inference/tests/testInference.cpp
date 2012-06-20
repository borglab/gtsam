/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testInference.cpp
 * @author  Richard Roberts
 * @author  Alex Cunningham
 * @date Dec 6, 2010
 */

#include <CppUnitLite/TestHarness.h>

#include <gtsam/inference/inference.h>
#include <gtsam/inference/VariableIndex.h>
#include <gtsam/inference/SymbolicFactorGraph.h>

using namespace gtsam;

/* ************************************************************************* */
TEST(inference, UnobservedVariables) {
  SymbolicFactorGraph sfg;

  // Create a factor graph that skips some variables
  sfg.push_factor(0,1);
  sfg.push_factor(1,3);
  sfg.push_factor(3,5);

  VariableIndex variableIndex(sfg);

  // Computes a permutation with known variables first, skipped variables last
  // Actual 0 1 3 5 2 4
  Permutation::shared_ptr actual(inference::PermutationCOLAMD(variableIndex));
  Permutation expected(6);
  expected[0] = 0;
  expected[1] = 1;
  expected[2] = 3;
  expected[3] = 5;
  expected[4] = 2;
  expected[5] = 4;
  EXPECT(assert_equal(expected, *actual));
}

/* ************************************************************************* */
TEST(inference, constrained_ordering) {
  SymbolicFactorGraph sfg;

  // create graph with wanted variable set = 2, 4
  sfg.push_factor(0,1);
  sfg.push_factor(1,2);
  sfg.push_factor(2,3);
  sfg.push_factor(3,4);
  sfg.push_factor(4,5);

  VariableIndex variableIndex(sfg);

  // unconstrained version
  Permutation::shared_ptr actUnconstrained(inference::PermutationCOLAMD(variableIndex));
  Permutation expUnconstrained = Permutation::Identity(6);
  EXPECT(assert_equal(expUnconstrained, *actUnconstrained));

  // constrained version - push one set to the end
  std::vector<int> constrainLast;
  constrainLast.push_back(2);
  constrainLast.push_back(4);
  Permutation::shared_ptr actConstrained(inference::PermutationCOLAMD(variableIndex, constrainLast));
  Permutation expConstrained(6);
  expConstrained[0] = 0;
  expConstrained[1] = 1;
  expConstrained[2] = 5;
  expConstrained[3] = 3;
  expConstrained[4] = 4;
  expConstrained[5] = 2;
  EXPECT(assert_equal(expConstrained, *actConstrained));
}

/* ************************************************************************* */
TEST(inference, grouped_constrained_ordering) {
  SymbolicFactorGraph sfg;

  // create graph with constrained groups:
  // 1: 2, 4
  // 2: 5
  sfg.push_factor(0,1);
  sfg.push_factor(1,2);
  sfg.push_factor(2,3);
  sfg.push_factor(3,4);
  sfg.push_factor(4,5);

  VariableIndex variableIndex(sfg);

  // constrained version - push one set to the end
  FastMap<size_t, int> constraints;
  constraints[2] = 1;
  constraints[4] = 1;
  constraints[5] = 2;

  Permutation::shared_ptr actConstrained(inference::PermutationCOLAMDGrouped(variableIndex, constraints));
  Permutation expConstrained(6);
  expConstrained[0] = 0;
  expConstrained[1] = 1;
  expConstrained[2] = 3;
  expConstrained[3] = 2;
  expConstrained[4] = 4;
  expConstrained[5] = 5;
  EXPECT(assert_equal(expConstrained, *actConstrained));
}

/* ************************************************************************* */
int main() {  TestResult tr;  return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */

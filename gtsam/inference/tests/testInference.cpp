/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testInference.cpp
 * @brief   
 * @author  Richard Roberts
 * @created Dec 6, 2010
 */

#include <CppUnitLite/TestHarness.h>

#include <gtsam/inference/inference.h>
#include <gtsam/inference/VariableIndex.h>
#include <gtsam/inference/SymbolicFactorGraph.h>

using namespace gtsam;

/* ************************************************************************* */
TEST(Inference, UnobservedVariables) {
  SymbolicFactorGraph sfg;

  // Create a factor graph that skips some variables
  sfg.push_factor(0,1);
  sfg.push_factor(1,3);
  sfg.push_factor(3,5);

  VariableIndex variableIndex(sfg);

  Permutation::shared_ptr colamd(Inference::PermutationCOLAMD(variableIndex));
}

/* ************************************************************************* */
int main() {  TestResult tr;  return TestRegistry::runAllTests(tr); }

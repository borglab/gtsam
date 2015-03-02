/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testLinearInequalityFactorGraph.cpp
 * @author  Duy-Nguyen Ta
 * @author  Krunal Chande
 * @author  Luca Carlone
 * @date    Dec 15, 2014
 */

#include <gtsam/inference/Symbol.h>
#include <gtsam_unstable/nonlinear/LinearInequalityFactorGraph.h>
#include <CppUnitLite/TestHarness.h>
#include <iostream>

using namespace std;
using namespace gtsam::symbol_shorthand;
using namespace gtsam;
const double tol = 1e-10;

//******************************************************************************
TEST(LinearInequalityFactorGraph, constructor) {
  LinearInequalityFactorGraph linearInequalities;
  CHECK(linearInequalities.empty());
}


//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************

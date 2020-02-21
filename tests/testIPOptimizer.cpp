/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testIPOptimizer.cpp
 * @brief   Unit tests for Interior Point Optimizer
 * @author  Frank Dellaert
 * @author  Yetong Zhang
 */

#include <gtsam/nonlinear/IPOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <CppUnitLite/TestHarness.h>

#include <iostream>

using namespace std;
using namespace gtsam;

/**
 * IPOptimizer
 * Our version of IPOpt, https://github.com/coin-or/Ipopt
 * /

/* ************************************************************************* */
TEST(NonlinearOptimizer, iterateLM) { NonlinearFactorGraph fg; }

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

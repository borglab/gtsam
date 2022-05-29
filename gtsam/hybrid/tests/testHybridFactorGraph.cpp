/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file    testHybridFactorGraph.cpp
 * @brief   Unit tests for HybridFactorGraph
 * @author  Varun Agrawal
 * @date    May 2021
 */

#include <CppUnitLite/Test.h>
#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/utilities.h>
#include <gtsam/hybrid/HybridFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/PriorFactor.h>

using namespace std;
using namespace gtsam;
using noiseModel::Isotropic;
using symbol_shorthand::L;
using symbol_shorthand::M;
using symbol_shorthand::X;

/* ****************************************************************************
 * Test that any linearizedFactorGraph gaussian factors are appended to the
 * existing gaussian factor graph in the hybrid factor graph.
 */
TEST(HybridFactorGraph, Constructor) {
  // Initialize the hybrid factor graph
  HybridFactorGraph fg;
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

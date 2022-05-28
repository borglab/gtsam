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

#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/utilities.h>
#include <gtsam/hybrid/HybridFactorGraph.h>

/* ****************************************************************************
 * Test that any linearizedFactorGraph gaussian factors are appended to the
 * existing gaussian factor graph in the hybrid factor graph.
 */
TEST(HybridFactorGraph, GaussianFactorGraph) {
  // Initialize the hybrid factor graph
  HybridFactorGraph fg;

  // Add a simple prior factor to the nonlinear factor graph
  fg.emplace_shared<PriorFactor<double>>(X(0), 0, Isotropic::Sigma(1, 0.1));

  // Add a linear factor to the nonlinear factor graph
  fg.add(X(0), I_1x1, Vector1(5));

  // Linearization point
  Values linearizationPoint;
  linearizationPoint.insert<double>(X(0), 0);

  GaussianHybridFactorGraph ghfg = fg.linearize(linearizationPoint);

  // ghfg.push_back(ghfg.gaussianGraph().begin(), ghfg.gaussianGraph().end());

  // EXPECT_LONGS_EQUAL(2, dcmfg.gaussianGraph().size());
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

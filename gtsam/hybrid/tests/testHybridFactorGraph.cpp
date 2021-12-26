/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testDCFactorGraph.cpp
 * @brief   Unit tests for DCFactorGraph
 * @author  Varun Agrawal
 * @author  Fan Jiang
 * @author  Frank Dellaert
 * @date    December 2021
 */

#include <gtsam/hybrid/DCFactor.h>
#include <gtsam/hybrid/DCMixtureFactor.h>
#include <gtsam/hybrid/HybridFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include <cstdlib>

// Include for test suite
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;
using noiseModel::Isotropic;
using symbol_shorthand::M;
using symbol_shorthand::X;

/* ****************************************************************************
 * Test elimination on a switching-like hybrid factor graph.
 */
TEST(HybridFactorGraph, Switching) {
  // Number of time steps.
  const size_t K = 3;

  // Create DiscreteKeys for binary K modes, modes[0] will not be used.
  DiscreteKeys modes;
  for (size_t k = 0; k <= K; k++) {
    modes.emplace_back(M(k), 2);
  }

  // Create hybrid factor graph.
  HybridFactorGraph fg;

  // Add a prior on X(1).
  fg.emplace_shared<PriorFactor<double>>(X(1), 0, Isotropic::Sigma(1, 0.1));

  // Add "motion models".
  for (size_t k = 1; k < K; k++) {
    BetweenFactor<double> still(X(k), X(k + 1), 0.0, Isotropic::Sigma(1, 1.0)),
        moving(X(k), X(k + 1), 1.0, Isotropic::Sigma(1, 1.0));
    using MotionMixture = DCMixtureFactor<BetweenFactor<double>>;
    auto keys = {X(k), X(k + 1)};
    auto components = {still, moving};
    fg.emplace_shared<MotionMixture>(keys, modes[k], components);
  }

  // Add "mode chain": can only be done in HybridFactorGraph
  fg.push_discrete(DiscreteConditional(modes[1], {}, "1/1"));
  for (size_t k = 1; k < K; k++) {
    auto parents = {modes[k]};
    fg.emplace_shared<DiscreteConditional>(modes[k + 1], parents, "1/2 3/2");
  }

  GTSAM_PRINT(fg);

  Values values;
  // Add a bunch of values for the linearization point.
  for (size_t k = 1; k <= K; k++) {
    values.insert<double>(X(k), 0.0);
  }

  // TODO: create 4 linearization points.

  // There original hybrid factor graph should not have any Gaussian factors.
  // This ensures there are no unintentional factors being created.
  EXPECT(assert_equal(fg.gaussianGraph().size(), size_t(0)));

  // Linearize here:
  HybridFactorGraph dcmfg = fg.linearize(values);
  GTSAM_PRINT(dcmfg);

  // There should only be one linearized continuous factor corresponding to the
  // PriorFactor on X(1).
  EXPECT(assert_equal(dcmfg.gaussianGraph().size(), size_t(1)));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

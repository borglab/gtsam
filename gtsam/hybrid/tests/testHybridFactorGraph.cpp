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
#include <gtsam/hybrid/HybridGaussianFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/JacobianFactor.h>
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
// Test if methods to get keys work as expected.
TEST(HybridFactorGraph, Keys) {
  HybridGaussianFactorGraph hfg;

  // Add prior on x0
  hfg.add(JacobianFactor(X(0), I_3x3, Z_3x1));

  // Add factor between x0 and x1
  hfg.add(JacobianFactor(X(0), I_3x3, X(1), -I_3x3, Z_3x1));

  // Add a hybrid Gaussian factor ϕ(x1, c1)
  DiscreteKey m1(M(1), 2);
  std::vector<GaussianFactor::shared_ptr> components{
      std::make_shared<JacobianFactor>(X(1), I_3x3, Z_3x1),
      std::make_shared<JacobianFactor>(X(1), I_3x3, Vector3::Ones())};
  hfg.add(HybridGaussianFactor({X(1)}, m1, components));

  KeySet expected_continuous{X(0), X(1)};
  EXPECT(
      assert_container_equality(expected_continuous, hfg.continuousKeySet()));

  KeySet expected_discrete{M(1)};
  EXPECT(assert_container_equality(expected_discrete, hfg.discreteKeySet()));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

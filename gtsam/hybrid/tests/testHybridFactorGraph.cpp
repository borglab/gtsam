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

using MotionModel = BetweenFactor<double>;

/* ****************************************************************************
 * Test that any linearizedFactorGraph gaussian factors are appended to the
 * existing gaussian factor graph in the hybrid factor graph.
 */
TEST(HybridFactorGraph, GaussianFactorGraph) {
  NonlinearFactorGraph cfg;
  GaussianFactorGraph gfg;

  // Add a simple prior factor to the nonlinear factor graph
  cfg.emplace_shared<PriorFactor<double>>(X(0), 0, Isotropic::Sigma(1, 0.1));

  // Add a factor to the GaussianFactorGraph
  gfg.add(X(0), I_1x1, Vector1(5));

  // Initialize the hybrid factor graph
  HybridFactorGraph nonlinearFactorGraph(cfg, DiscreteFactorGraph(),
                                         DCFactorGraph(), gfg);

  // Linearization point
  Values linearizationPoint;
  linearizationPoint.insert<double>(X(0), 0);

  HybridFactorGraph dcmfg = nonlinearFactorGraph.linearize(linearizationPoint);

  EXPECT(dcmfg.gaussianGraph().size() == 2);
}

/**
 * Test fixture with switching network.
 */
using MotionMixture = DCMixtureFactor<MotionModel>;
struct Switching {
  size_t K;
  DiscreteKeys modes;
  HybridFactorGraph nonlinearFactorGraph;
  HybridFactorGraph linearizedFactorGraph;
  Values linearizationPoint;

  /// Create with given number of time steps.
  Switching(size_t K) : K(K) {
    // Create DiscreteKeys for binary K modes, modes[0] will not be used.
    for (size_t k = 0; k <= K; k++) {
      modes.emplace_back(M(k), 2);
    }

    // Create hybrid factor graph.
    // Add a prior on X(1).
    nonlinearFactorGraph.emplace_shared<PriorFactor<double>>(
        X(1), 0, Isotropic::Sigma(1, 0.1));

    // Add "motion models".
    for (size_t k = 1; k < K; k++) {
      using MotionMixture = DCMixtureFactor<MotionModel>;
      auto keys = {X(k), X(k + 1)};
      auto components = motionModels(k);
      nonlinearFactorGraph.emplace_shared<MotionMixture>(
          keys, DiscreteKeys{modes[k]}, components);
    }

    // Add "mode chain"
    addModeChain(&nonlinearFactorGraph);

    // Create the linearization point.
    for (size_t k = 1; k <= K; k++) {
      linearizationPoint.insert<double>(X(k), static_cast<double>(k));
    }

    // Create the linearizedFactorGraph hybrid factor graph.

    // Add a prior on X(1).
    PriorFactor<double> prior(X(1), 0, Isotropic::Sigma(1, 0.1));
    auto gaussian = prior.linearize(linearizationPoint);
    linearizedFactorGraph.push_gaussian(gaussian);

    // Add "motion models".
    for (size_t k = 1; k < K; k++) {
      MotionModel still(X(k), X(k + 1), 0.0, Isotropic::Sigma(1, 1.0)),
          moving(X(k), X(k + 1), 1.0, Isotropic::Sigma(1, 1.0));
      auto keys = {X(k), X(k + 1)};
      auto components = {still.linearize(linearizationPoint),
                         moving.linearize(linearizationPoint)};
      linearizedFactorGraph.emplace_shared<DCGaussianMixtureFactor>(
          keys, DiscreteKeys{modes[k]}, components);
    }

    // Add "mode chain"
    addModeChain(&linearizedFactorGraph);
  }

  std::vector<MotionModel::shared_ptr> motionModels(size_t k) {
    auto still = boost::make_shared<MotionModel>(X(k), X(k + 1), 0.0,
                                                 Isotropic::Sigma(1, 1.0)),
         moving = boost::make_shared<MotionModel>(X(k), X(k + 1), 1.0,
                                                  Isotropic::Sigma(1, 1.0));
    return {still, moving};
  }

  // Add "mode chain": can only be done in HybridFactorGraph
  void addModeChain(HybridFactorGraph* fg) {
    fg->push_discrete(DiscreteConditional(modes[1], {}, "1/1"));
    for (size_t k = 1; k < K; k++) {
      auto parents = {modes[k]};
      fg->emplace_shared<DiscreteConditional>(modes[k + 1], parents, "1/2 3/2");
    }
  }
};

/* ****************************************************************************
 * Test linearization on a switching-like hybrid factor graph.
 */
TEST(HybridFactorGraph, Linearization) {
  Switching self(3);

  // TODO: create 4 linearization points.

  // There original hybrid factor graph should not have any Gaussian factors.
  // This ensures there are no unintentional factors being created.
  EXPECT(self.nonlinearFactorGraph.gaussianGraph().size() == 0);

  // Linearize here:
  HybridFactorGraph actualLinearized =
      self.nonlinearFactorGraph.linearize(self.linearizationPoint);

  // There should only be one linearizedFactorGraph continuous factor
  // corresponding to the PriorFactor on X(1).
  EXPECT(actualLinearized.gaussianGraph().size() == 1);
  // There should be two linearizedFactorGraph DCGaussianMixtureFactors for each
  // DCMixtureFactor.
  EXPECT(actualLinearized.dcGraph().size() == 2);
}

/* ****************************************************************************/
// Test elimination
TEST(DCGaussianElimination, Switching) {
  Switching self(3);
  GTSAM_PRINT(self.linearizedFactorGraph);

  // Eliminate partially.
  Ordering ordering;
  for (size_t k = 1; k <= self.K; k++) ordering += X(k);
  auto result = self.linearizedFactorGraph.eliminatePartialSequential(ordering);
  GTSAM_PRINT(*result.first);   // HybridBayesNet
  GTSAM_PRINT(*result.second);  // HybridFactorGraph
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

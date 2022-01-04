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
#include <gtsam/hybrid/HybridEliminationTree.h>
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

  EXPECT_LONGS_EQUAL(2, dcmfg.gaussianGraph().size());
}

/* ****************************************************************************/
// Test fixture with switching network.
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
    auto prior = boost::make_shared<PriorFactor<double>>(
        X(1), 0, Isotropic::Sigma(1, 0.1));
    nonlinearFactorGraph.push_nonlinear(prior);

    // Add "motion models".
    for (size_t k = 1; k < K; k++) {
      using MotionMixture = DCMixtureFactor<MotionModel>;
      auto keys = {X(k), X(k + 1)};
      auto components = motionModels(k);
      nonlinearFactorGraph.emplace_dc<MotionMixture>(
          keys, DiscreteKeys{modes[k]}, components);
    }

    // Add "mode chain"
    addModeChain(&nonlinearFactorGraph);

    // Create the linearization point.
    for (size_t k = 1; k <= K; k++) {
      linearizationPoint.insert<double>(X(k), static_cast<double>(k));
    }

    // Create the linearized hybrid factor graph.
    // Add a prior on X(1).
    auto gaussian = prior->linearize(linearizationPoint);
    linearizedFactorGraph.push_gaussian(gaussian);

    // Add "motion models".
    for (size_t k = 1; k < K; k++) {
      auto components = motionModels(k);
      auto keys = {X(k), X(k + 1)};
      auto linearized = {components[0]->linearize(linearizationPoint),
                         components[1]->linearize(linearizationPoint)};
      linearizedFactorGraph.emplace_dc<DCGaussianMixtureFactor>(
          keys, DiscreteKeys{modes[k]}, linearized);
    }

    // Add "mode chain"
    addModeChain(&linearizedFactorGraph);
  }

  // Create motion models for a given time step
  std::vector<MotionModel::shared_ptr> motionModels(size_t k) {
    auto still = boost::make_shared<MotionModel>(X(k), X(k + 1), 0.0,
                                                 Isotropic::Sigma(1, 1.0)),
         moving = boost::make_shared<MotionModel>(X(k), X(k + 1), 1.0,
                                                  Isotropic::Sigma(1, 1.0));
    return {still, moving};
  }

  // Add "mode chain": can only be done in HybridFactorGraph
  void addModeChain(HybridFactorGraph* fg) {
    auto prior = boost::make_shared<DiscretePrior>(modes[1], "1/1");
    fg->push_discrete(prior);
    for (size_t k = 1; k < K; k++) {
      auto parents = {modes[k]};
      auto conditional = boost::make_shared<DiscreteConditional>(
          modes[k + 1], parents, "1/2 3/2");
      fg->push_discrete(conditional);
    }
  }
};

/* ****************************************************************************/
// Test construction of switching-like hybrid factor graph.
TEST(HybridFactorGraph, Switching) {
  Switching self(3);
  EXPECT_LONGS_EQUAL(6, self.nonlinearFactorGraph.size());
  EXPECT_LONGS_EQUAL(1, self.nonlinearFactorGraph.nonlinearGraph().size());
  EXPECT_LONGS_EQUAL(3, self.nonlinearFactorGraph.discreteGraph().size());
  EXPECT_LONGS_EQUAL(2, self.nonlinearFactorGraph.dcGraph().size());
  EXPECT_LONGS_EQUAL(0, self.nonlinearFactorGraph.gaussianGraph().size());

  EXPECT_LONGS_EQUAL(6, self.linearizedFactorGraph.size());
  EXPECT_LONGS_EQUAL(0, self.linearizedFactorGraph.nonlinearGraph().size());
  EXPECT_LONGS_EQUAL(3, self.linearizedFactorGraph.discreteGraph().size());
  EXPECT_LONGS_EQUAL(2, self.linearizedFactorGraph.dcGraph().size());
  EXPECT_LONGS_EQUAL(1, self.linearizedFactorGraph.gaussianGraph().size());
}

/* ****************************************************************************/
// Test linearization on a switching-like hybrid factor graph.
TEST(HybridFactorGraph, Linearization) {
  Switching self(3);
  // TODO: create 4 linearization points.

  // There original hybrid factor graph should not have any Gaussian factors.
  // This ensures there are no unintentional factors being created.
  EXPECT(self.nonlinearFactorGraph.gaussianGraph().size() == 0);

  // Linearize here:
  HybridFactorGraph actualLinearized =
      self.nonlinearFactorGraph.linearize(self.linearizationPoint);

  EXPECT_LONGS_EQUAL(6, actualLinearized.size());
  EXPECT_LONGS_EQUAL(0, actualLinearized.nonlinearGraph().size());
  EXPECT_LONGS_EQUAL(3, actualLinearized.discreteGraph().size());
  EXPECT_LONGS_EQUAL(2, actualLinearized.dcGraph().size());
  EXPECT_LONGS_EQUAL(1, actualLinearized.gaussianGraph().size());

  // TODO: fix this test, the graphs are equal !!!
  // EXPECT(assert_equal(self.linearizedFactorGraph, actualLinearized));
}

/* ****************************************************************************/
// Test elimination tree construction
TEST(HybridFactorGraph, EliminationTree) {
  Switching self(3);

  // Create ordering.
  Ordering ordering;
  for (size_t k = 1; k <= self.K; k++) ordering += X(k);

  // Create elimination tree.
  HybridEliminationTree etree(self.linearizedFactorGraph, ordering);
  EXPECT_LONGS_EQUAL(1, etree.roots().size())
}

/* ****************************************************************************/
// Test elimination function by eliminating x1 in *-x1-*-x2 graph.
TEST(DCGaussianElimination, Eliminate_x1) {
  Switching self(3);

  // Gather factors on x1, has a simple Gaussian and a mixture factor.
  HybridFactorGraph factors;
  factors.push_gaussian(self.linearizedFactorGraph.gaussianGraph()[0]);
  factors.push_dc(self.linearizedFactorGraph.dcGraph()[0]);

  // Check that sum works:
  auto sum = factors.sum();
  Assignment<Key> mode;
  mode[M(1)] = 1;
  auto actual = sum(mode);               // Selects one of 2 modes.
  EXPECT_LONGS_EQUAL(2, actual.size());  // Prior and motion model.

  // Eliminate x1
  Ordering ordering;
  ordering += X(1);

  auto result = EliminateHybrid(factors, ordering);
  CHECK(result.first);
  EXPECT_LONGS_EQUAL(1, result.first->nrFrontals());
  CHECK(result.second);
  EXPECT_LONGS_EQUAL(1, result.second->size());
}

/* ****************************************************************************/
// Test elimination function by eliminating x2 in x1-*-x2-*-x3 chain.
TEST(DCGaussianElimination, Eliminate_x2) {
  Switching self(3);

  // Gather factors on x2, will be two mixture factor (with x1 and x3, resp.).
  HybridFactorGraph factors;
  factors.push_dc(self.linearizedFactorGraph.dcGraph()[0]);
  factors.push_dc(self.linearizedFactorGraph.dcGraph()[1]);

  // Check that sum works:
  auto sum = factors.sum();
  Assignment<Key> mode;
  mode[M(1)] = 0;
  mode[M(2)] = 1;
  auto actual = sum(mode);               // Selects one of 4 mode combinations.
  EXPECT_LONGS_EQUAL(2, actual.size());  // 2 motion models.

  // Eliminate x2
  Ordering ordering;
  ordering += X(2);

  auto result = EliminateHybrid(factors, ordering);
  CHECK(result.first);
  EXPECT_LONGS_EQUAL(1, result.first->nrFrontals());
  CHECK(result.second);
  EXPECT_LONGS_EQUAL(2, result.second->size());
}

/* ****************************************************************************/
// Test elimination
TEST(HybridFactorGraph, Elimination) {
  Switching self(4);

  // Create ordering.
  Ordering ordering;
  for (size_t k = 1; k <= self.K; k++) ordering += X(k);

  // Eliminate partially.
  auto result = self.linearizedFactorGraph.eliminatePartialSequential(ordering);

  CHECK(result.first);
  GTSAM_PRINT(*result.first);   // HybridBayesNet
  EXPECT_LONGS_EQUAL(4, result.first->size())

  CHECK(result.second);
  GTSAM_PRINT(*result.second);  // HybridFactorGraph
  EXPECT_LONGS_EQUAL(4, result.second->size())
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
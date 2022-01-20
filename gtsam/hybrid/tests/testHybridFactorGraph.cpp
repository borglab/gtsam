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
#include <gtsam/hybrid/IncrementalHybrid.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/discrete/DiscreteDistribution.h>
#include <gtsam/discrete/DiscreteBayesNet.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/base/utilities.h>

#include <cstdlib>
#include <numeric>

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

/* ****************************************************************************
 * Test push_back on HFG makes the correct distinction.
 */
TEST(HybridFactorGraph, PushBack) {
  HybridFactorGraph fg;

  auto gaussianFactor = boost::make_shared<JacobianFactor>();
  fg.push_back(gaussianFactor);

  EXPECT_LONGS_EQUAL(fg.dcGraph().size(), 0);
  EXPECT_LONGS_EQUAL(fg.discreteGraph().size(), 0);
  EXPECT_LONGS_EQUAL(fg.nonlinearGraph().size(), 0);
  EXPECT_LONGS_EQUAL(fg.gaussianGraph().size(), 1);

  fg.clear();

  auto nonlinearFactor = boost::make_shared<BetweenFactor<double>>();
  fg.push_back(nonlinearFactor);

  EXPECT_LONGS_EQUAL(fg.dcGraph().size(), 0);
  EXPECT_LONGS_EQUAL(fg.discreteGraph().size(), 0);
  EXPECT_LONGS_EQUAL(fg.nonlinearGraph().size(), 1);
  EXPECT_LONGS_EQUAL(fg.gaussianGraph().size(), 0);

  fg.clear();

  auto discreteFactor = boost::make_shared<DecisionTreeFactor>();
  fg.push_back(discreteFactor);

  EXPECT_LONGS_EQUAL(fg.dcGraph().size(), 0);
  EXPECT_LONGS_EQUAL(fg.discreteGraph().size(), 1);
  EXPECT_LONGS_EQUAL(fg.nonlinearGraph().size(), 0);
  EXPECT_LONGS_EQUAL(fg.gaussianGraph().size(), 0);

  fg.clear();

  auto dcFactor = boost::make_shared<DCMixtureFactor<MotionModel>>();
  fg.push_back(dcFactor);

  EXPECT_LONGS_EQUAL(fg.dcGraph().size(), 1);
  EXPECT_LONGS_EQUAL(fg.discreteGraph().size(), 0);
  EXPECT_LONGS_EQUAL(fg.nonlinearGraph().size(), 0);
  EXPECT_LONGS_EQUAL(fg.gaussianGraph().size(), 0);
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
  Switching(size_t K, double between_sigma = 1.0, double prior_sigma = 0.1)
      : K(K) {
    // Create DiscreteKeys for binary K modes, modes[0] will not be used.
    for (size_t k = 0; k <= K; k++) {
      modes.emplace_back(M(k), 2);
    }

    // Create hybrid factor graph.
    // Add a prior on X(1).
    auto prior = boost::make_shared<PriorFactor<double>>(
        X(1), 0, Isotropic::Sigma(1, prior_sigma));
    nonlinearFactorGraph.push_nonlinear(prior);

    // Add "motion models".
    for (size_t k = 1; k < K; k++) {
      auto keys = {X(k), X(k + 1)};
      auto components = motionModels(k);
      nonlinearFactorGraph.emplace_dc<MotionMixture>(
          keys, DiscreteKeys{modes[k]}, components);
    }

    // Add measurement factors
    auto measurement_noise = noiseModel::Isotropic::Sigma(1, 0.1);
    for (size_t k = 1; k <= K; k++) {
      nonlinearFactorGraph.emplace_nonlinear<PriorFactor<double> >(
          X(k),
          1.0 * (k - 1),
          measurement_noise);
    }

    // Add "mode chain"
    addModeChain(&nonlinearFactorGraph);

    // Create the linearization point.
    for (size_t k = 1; k <= K; k++) {
      linearizationPoint.insert<double>(X(k), static_cast<double>(k));
    }

    linearizedFactorGraph = nonlinearFactorGraph.linearize(linearizationPoint);
  }

  // Create motion models for a given time step
  static std::vector<MotionModel::shared_ptr> motionModels(size_t k,
                                                           double sigma = 1.0) {
    auto noise_model = Isotropic::Sigma(1, sigma);
    auto still =
        boost::make_shared<MotionModel>(X(k), X(k + 1), 0.0, noise_model),
        moving =
        boost::make_shared<MotionModel>(X(k), X(k + 1), 1.0, noise_model);
    return {still, moving};
  }

  // Add "mode chain": can only be done in HybridFactorGraph
  void addModeChain(HybridFactorGraph *fg) {
    auto prior = boost::make_shared<DiscreteDistribution>(modes[1], "1/1");
    fg->push_discrete(prior);
    for (size_t k = 1; k < K - 1; k++) {
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
  EXPECT_LONGS_EQUAL(8, self.nonlinearFactorGraph.size());
  EXPECT_LONGS_EQUAL(4, self.nonlinearFactorGraph.nonlinearGraph().size());
  EXPECT_LONGS_EQUAL(2, self.nonlinearFactorGraph.discreteGraph().size());
  EXPECT_LONGS_EQUAL(2, self.nonlinearFactorGraph.dcGraph().size());
  EXPECT_LONGS_EQUAL(0, self.nonlinearFactorGraph.gaussianGraph().size());

  EXPECT_LONGS_EQUAL(8, self.linearizedFactorGraph.size());
  EXPECT_LONGS_EQUAL(0, self.linearizedFactorGraph.nonlinearGraph().size());
  EXPECT_LONGS_EQUAL(2, self.linearizedFactorGraph.discreteGraph().size());
  EXPECT_LONGS_EQUAL(2, self.linearizedFactorGraph.dcGraph().size());
  EXPECT_LONGS_EQUAL(4, self.linearizedFactorGraph.gaussianGraph().size());
}

/* ****************************************************************************/
// Test linearization on a switching-like hybrid factor graph.
TEST(HybridFactorGraph, Linearization) {
  Switching self(3);
  // TODO: create 4 linearization points.

  // There original hybrid factor graph should not have any Gaussian factors.
  // This ensures there are no unintentional factors being created.
  EXPECT(self.nonlinearFactorGraph.gaussianGraph().size() == 0);

  EXPECT_LONGS_EQUAL(8, self.linearizedFactorGraph.size());
  EXPECT_LONGS_EQUAL(0, self.linearizedFactorGraph.nonlinearGraph().size());
  EXPECT_LONGS_EQUAL(2, self.linearizedFactorGraph.discreteGraph().size());
  EXPECT_LONGS_EQUAL(2, self.linearizedFactorGraph.dcGraph().size());
  EXPECT_LONGS_EQUAL(4, self.linearizedFactorGraph.gaussianGraph().size());
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
  // Has two keys, x2 and m1
  EXPECT_LONGS_EQUAL(2, result.second->size());
}

/* ****************************************************************************/
// Test elimination function by eliminating x2 in x1-*-x2-*-x3 chain.
//                                                m1/      \m2
TEST(DCGaussianElimination, Eliminate_x2) {
  Switching self(3);

  // Gather factors on x2, will be two mixture factors (with x1 and x3, resp.).
  HybridFactorGraph factors;
  factors.push_dc(self.linearizedFactorGraph.dcGraph()[0]);  // involves m1
  factors.push_dc(self.linearizedFactorGraph.dcGraph()[1]);  // involves m2

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

  std::pair<GaussianMixture::shared_ptr, boost::shared_ptr<Factor>> result =
      EliminateHybrid(factors, ordering);
  CHECK(result.first);
  EXPECT_LONGS_EQUAL(1, result.first->nrFrontals());
  CHECK(result.second);
  // Note: separator keys should include m1, m2.
  EXPECT_LONGS_EQUAL(4, result.second->size());
}

/* ****************************************************************************/
// Helper method to generate gaussian factor graphs with a specific mode.
GaussianFactorGraph::shared_ptr batchGFG(double between,
                                         Values linearizationPoint) {
  NonlinearFactorGraph graph;
  graph.addPrior<double>(X(1), 0, Isotropic::Sigma(1, 0.1));

  auto between_x1_x2 = boost::make_shared<MotionModel>(
      X(1), X(2), between, Isotropic::Sigma(1, 1.0));

  graph.push_back(between_x1_x2);

  return graph.linearize(linearizationPoint);
}

/* ****************************************************************************/
// Test elimination function by eliminating x1 and x2 in graph.
TEST(DCGaussianElimination, EliminateHybrid_2_Variable) {
  Switching self(2);
  auto factors = self.linearizedFactorGraph;

  // Check that sum works:
  auto sum = factors.sum();
  Assignment<Key> mode;
  mode[M(1)] = 1;
  auto actual = sum(mode);               // Selects one of 2 modes.
  EXPECT_LONGS_EQUAL(4,
                     actual.size());  // Prior, 1 motion models, 2 measurements.

  // Eliminate x1
  Ordering ordering;
  ordering += X(1);
  ordering += X(2);

  GaussianMixture::shared_ptr gaussianConditionalMixture;
  boost::shared_ptr<Factor> factorOnModes;
  std::tie(gaussianConditionalMixture, factorOnModes) =
      EliminateHybrid(factors, ordering);

  CHECK(gaussianConditionalMixture);
  EXPECT_LONGS_EQUAL(2,
                     gaussianConditionalMixture->nrFrontals()); // Frontals = [x1, x2]
  EXPECT_LONGS_EQUAL(1,
                     gaussianConditionalMixture->nrParents()); // 1 parent, which is the mode

  auto discreteFactor = dynamic_pointer_cast<DecisionTreeFactor>(factorOnModes);
  CHECK(discreteFactor);
  EXPECT_LONGS_EQUAL(1, discreteFactor->discreteKeys().size());
  EXPECT(discreteFactor->root_->isLeaf() == false);
}

/* ****************************************************************************/
/// Test the toDecisionTreeFactor method
TEST(HybridFactorGraph, ToDecisionTreeFactor) {
  size_t K = 3;

  // Provide tight sigma values so that the errors are visibly different.
  double between_sigma = 5e-8, prior_sigma = 1e-7;

  Switching self(K, between_sigma, prior_sigma);

  // Clear out discrete factors since sum() cannot handle those
  HybridFactorGraph linearizedFactorGraph(
      NonlinearFactorGraph(), DiscreteFactorGraph(),
      self.linearizedFactorGraph.dcGraph(),
      self.linearizedFactorGraph.gaussianGraph());

  auto decisionTreeFactor = linearizedFactorGraph.toDecisionTreeFactor();

  auto allAssignments =
      DiscreteValues::CartesianProduct(linearizedFactorGraph.discreteKeys());

  // Get the error of the discrete assignment m1=0, m2=1.
  double actual = (*decisionTreeFactor)(allAssignments[1]);

  /********************************************/
  // Create equivalent factor graph for m1=0, m2=1
  GaussianFactorGraph graph = linearizedFactorGraph.gaussianGraph();

  for (auto &p : linearizedFactorGraph.dcGraph()) {
    if (auto
        mixture = boost::dynamic_pointer_cast<DCGaussianMixtureFactor>(p)) {
      graph.add((*mixture)(allAssignments[1]));
    }
  }

  VectorValues values = graph.optimize();
  double expected = graph.probPrime(values);
  /********************************************/
  EXPECT_DOUBLES_EQUAL(expected, actual, 1e-12);
  // REGRESSION:
  EXPECT_DOUBLES_EQUAL(0.6125, actual, 1e-4);
}

/* ****************************************************************************/
// Test elimination
TEST(HybridFactorGraph, Elimination) {
  Switching self(3);

  auto linearizedFactorGraph = self.linearizedFactorGraph;

  // Create ordering.
  Ordering ordering;
  for (size_t k = 1; k <= self.K; k++) ordering += X(k);

  // Eliminate partially.
  HybridBayesNet::shared_ptr hybridBayesNet;
  HybridFactorGraph::shared_ptr remainingFactorGraph;
  std::tie(hybridBayesNet, remainingFactorGraph) =
      linearizedFactorGraph.eliminatePartialSequential(ordering);

  CHECK(hybridBayesNet);
//  GTSAM_PRINT(*hybridBayesNet);  // HybridBayesNet
  EXPECT_LONGS_EQUAL(3, hybridBayesNet->size());
  EXPECT(hybridBayesNet->at(0)->frontals() == KeyVector{X(1)});
  EXPECT(hybridBayesNet->at(0)->parents() == KeyVector({X(2), M(1)}));
  EXPECT(hybridBayesNet->at(1)->frontals() == KeyVector{X(2)});
  EXPECT(hybridBayesNet->at(1)->parents() == KeyVector({X(3), M(2), M(1)}));
  EXPECT(hybridBayesNet->at(2)->frontals() == KeyVector{X(3)});
  EXPECT(hybridBayesNet->at(2)->parents() == KeyVector({M(2), M(1)}));

  CHECK(remainingFactorGraph);
//  GTSAM_PRINT(*remainingFactorGraph);  // HybridFactorGraph
  EXPECT_LONGS_EQUAL(3, remainingFactorGraph->size());
  EXPECT(
      remainingFactorGraph->discreteGraph().at(0)->keys() == KeyVector({M(1)}));
  EXPECT(remainingFactorGraph->discreteGraph().at(1)->keys()
             == KeyVector({M(2), M(1)}));
  EXPECT(remainingFactorGraph->discreteGraph().at(2)->keys()
             == KeyVector({M(2), M(1)}));
}

/* ****************************************************************************/
// Test if we can incrementally do the inference
TEST(DCGaussianElimination, Incremental_inference) {
  Switching switching(3);

  IncrementalHybrid incrementalHybrid;

  HybridFactorGraph graph1;

  graph1.push_back(switching.linearizedFactorGraph.dcGraph().at(0));
  graph1.push_back(switching.linearizedFactorGraph.gaussianGraph().at(0));
  graph1.push_back(switching.linearizedFactorGraph.gaussianGraph().at(1));
  graph1.push_back(switching.linearizedFactorGraph.gaussianGraph().at(2));

  // Create ordering.
  Ordering ordering;
  ordering += X(1);
  ordering += X(2);

  incrementalHybrid.update(graph1, ordering);

  auto hybridBayesNet = incrementalHybrid.hybridBayesNet_;
  CHECK(hybridBayesNet);
  EXPECT_LONGS_EQUAL(2, hybridBayesNet->size());
  EXPECT(hybridBayesNet->at(0)->frontals() == KeyVector{X(1)});
  EXPECT(hybridBayesNet->at(0)->parents() == KeyVector({X(2), M(1)}));
  EXPECT(hybridBayesNet->at(1)->frontals() == KeyVector{X(2)});
  EXPECT(hybridBayesNet->at(1)->parents() == KeyVector({M(1)}));

  auto remainingFactorGraph = incrementalHybrid.remainingFactorGraph_;
  CHECK(remainingFactorGraph);
  EXPECT_LONGS_EQUAL(1, remainingFactorGraph->size());

  auto discreteFactor_m1 =
      *dynamic_pointer_cast<DecisionTreeFactor>(remainingFactorGraph->discreteGraph().at(
          0));
  EXPECT(
      discreteFactor_m1.keys() == KeyVector({M(1)}));

  HybridFactorGraph graph2;

  graph2.push_back(switching.linearizedFactorGraph.dcGraph().at(1)); // p(x3 | x2, m2)
  graph2.push_back(switching.linearizedFactorGraph.gaussianGraph().at(3));

  // Create ordering.
  Ordering ordering2;
  ordering2 += X(2);
  ordering2 += X(3);

  incrementalHybrid.update(graph2, ordering2);

  auto hybridBayesNet2 = incrementalHybrid.hybridBayesNet_;
  CHECK(hybridBayesNet2);
  EXPECT_LONGS_EQUAL(2, hybridBayesNet2->size());
  EXPECT(hybridBayesNet2->at(0)->frontals() == KeyVector{X(2)});
  EXPECT(hybridBayesNet2->at(0)->parents() == KeyVector({X(3), M(2), M(1)}));
  EXPECT(hybridBayesNet2->at(1)->frontals() == KeyVector{X(3)});
  EXPECT(hybridBayesNet2->at(1)->parents() == KeyVector({M(2), M(1)}));

  auto remainingFactorGraph2 = incrementalHybrid.remainingFactorGraph_;
  CHECK(remainingFactorGraph2);
  EXPECT_LONGS_EQUAL(1, remainingFactorGraph2->size());

  auto discreteFactor =
      dynamic_pointer_cast<DecisionTreeFactor>(remainingFactorGraph2->discreteGraph().at(
          0));
  EXPECT(
      discreteFactor->keys()
          == KeyVector({M(2), M(1)}));

  ordering.clear();
  ordering += X(1);
  ordering += X(2);
  ordering += X(3);

  // Now we calculate the actual factors using full elimination
  HybridBayesNet::shared_ptr expectedHybridBayesNet;
  HybridFactorGraph::shared_ptr expectedRemainingGraph;
  std::tie(expectedHybridBayesNet, expectedRemainingGraph) =
      switching.linearizedFactorGraph.eliminatePartialSequential(ordering);

  // The densities on X(1) should be the same
  EXPECT(assert_equal(*(hybridBayesNet->at(0)),
                      *(expectedHybridBayesNet->at(0))));

  // The densities on X(2) should be the same
  EXPECT(assert_equal(*(hybridBayesNet2->at(0)),
                      *(expectedHybridBayesNet->at(1))));

  // The densities on X(3) should be the same
  EXPECT(assert_equal(*(hybridBayesNet2->at(1)),
                      *(expectedHybridBayesNet->at(2))));

  // we only do the manual continuous elimination for 0,0
  // the other discrete probabilities on M(2) are calculated the same way
  auto m00_prob = [&]() {
    GaussianFactorGraph gf;
    gf.add(switching.linearizedFactorGraph.gaussianGraph().at(3));

    Assignment<Key> m00;
    m00[M(1)] = 0, m00[M(2)] = 0;
    auto dcMixture =
        dynamic_pointer_cast<DCGaussianMixtureFactor>(graph2.dcGraph().at(0));
    gf.add(dcMixture->factors()(m00));
    auto x2_mixed = hybridBayesNet->at(1);
    gf.add(x2_mixed->factors()(m00));
    auto result_gf = gf.eliminateSequential();
    return gf.probPrime(result_gf->optimize());
  }();

  EXPECT(assert_equal(m00_prob, 0.60656, 1e-5));

  DiscreteValues assignment;
  assignment[M(1)] = 0;
  assignment[M(2)] = 0;
  EXPECT(assert_equal(m00_prob, (*discreteFactor)(assignment), 1e-5));
  assignment[M(1)] = 1;
  assignment[M(2)] = 0;
  EXPECT(assert_equal(0.612477, (*discreteFactor)(assignment), 1e-5));
  assignment[M(1)] = 0;
  assignment[M(2)] = 1;
  EXPECT(assert_equal(0.999952, (*discreteFactor)(assignment), 1e-5));
  assignment[M(1)] = 1;
  assignment[M(2)] = 1;
  EXPECT(assert_equal(1.0, (*discreteFactor)(assignment), 1e-5));

  DiscreteFactorGraph dfg;
  dfg.add(*discreteFactor);
  dfg.add(discreteFactor_m1);
  dfg.add_factors(switching.linearizedFactorGraph.discreteGraph());

  auto chordal = dfg.eliminateSequential();
  auto expectedChordal =
      expectedRemainingGraph->discreteGraph().eliminateSequential();

  EXPECT(assert_equal(*expectedChordal, *chordal, 1e-6));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

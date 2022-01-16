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
#include <gtsam/discrete/DiscretePrior.h>
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

    // Add "mode chain"
    addModeChain(&linearizedFactorGraph);

    // Add "motion models".
    for (size_t k = 1; k < K; k++) {
      auto components = motionModels(k, between_sigma);
      auto keys = {X(k), X(k + 1)};
      auto linearized = {components[0]->linearize(linearizationPoint),
                         components[1]->linearize(linearizationPoint)};
      linearizedFactorGraph.emplace_dc<DCGaussianMixtureFactor>(
          keys, DiscreteKeys{modes[k]}, linearized);
    }

    // Create the linearized hybrid factor graph.
    // Add a prior on X(1).
    auto gaussian = prior->linearize(linearizationPoint);
    linearizedFactorGraph.push_gaussian(gaussian);
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
  void addModeChain(HybridFactorGraph* fg) {
    auto prior = boost::make_shared<DiscretePrior>(modes[1], "1/1");
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
  EXPECT_LONGS_EQUAL(5, self.nonlinearFactorGraph.size());
  EXPECT_LONGS_EQUAL(1, self.nonlinearFactorGraph.nonlinearGraph().size());
  EXPECT_LONGS_EQUAL(2, self.nonlinearFactorGraph.discreteGraph().size());
  EXPECT_LONGS_EQUAL(2, self.nonlinearFactorGraph.dcGraph().size());
  EXPECT_LONGS_EQUAL(0, self.nonlinearFactorGraph.gaussianGraph().size());

  EXPECT_LONGS_EQUAL(5, self.linearizedFactorGraph.size());
  EXPECT_LONGS_EQUAL(0, self.linearizedFactorGraph.nonlinearGraph().size());
  EXPECT_LONGS_EQUAL(2, self.linearizedFactorGraph.discreteGraph().size());
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

  EXPECT_LONGS_EQUAL(5, actualLinearized.size());
  EXPECT_LONGS_EQUAL(0, actualLinearized.nonlinearGraph().size());
  EXPECT_LONGS_EQUAL(2, actualLinearized.discreteGraph().size());
  EXPECT_LONGS_EQUAL(2, actualLinearized.dcGraph().size());
  EXPECT_LONGS_EQUAL(1, actualLinearized.gaussianGraph().size());

  EXPECT(assert_equal(self.linearizedFactorGraph, actualLinearized));
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
// Test elimination function by eliminating x1 in *-x1-*-m1 graph.
TEST(DCGaussianElimination, Eliminate_fully) {
  Switching self(2);
  auto factors = self.linearizedFactorGraph;

  // Add measurement factors
  auto measurement_noise = noiseModel::Isotropic::Sigma(1, 0.1);
  factors.emplace_gaussian<JacobianFactor>(X(1), I_1x1, 1 * Vector1::Ones(), measurement_noise);
  factors.emplace_gaussian<JacobianFactor>(X(2), I_1x1, 2 * Vector1::Ones(), measurement_noise);

  // Check that sum works:
  auto sum = factors.sum();
  Assignment<Key> mode;
  mode[M(1)] = 1;
  auto actual = sum(mode);               // Selects one of 2 modes.
  EXPECT_LONGS_EQUAL(4, actual.size());  // Prior, motion model, 2 measurements.

  // Eliminate x1
  Ordering ordering;
  ordering += X(1);
  ordering += X(2);

  auto result = EliminateHybrid(factors, ordering);
  CHECK(result.first);

  EXPECT_LONGS_EQUAL(1, result.first->nrFrontals());
  CHECK(result.second);

  auto discreteFactor = dynamic_pointer_cast<DecisionTreeFactor>(result.second);
  CHECK(discreteFactor);
  EXPECT_LONGS_EQUAL(1, discreteFactor->discreteKeys().size());
  EXPECT(discreteFactor->root_->isLeaf() == false);
}

/* ****************************************************************************/
// Test if we can incrementally do the inference
TEST(DCGaussianElimination, Incremental_inference) {
  Switching three_step(3);

  // Add measurement factors
  auto measurement_noise = noiseModel::Isotropic::Sigma(1, 0.1);
  three_step.nonlinearFactorGraph.emplace_nonlinear<PriorFactor<double> >(X(1), 0.0, measurement_noise);
  three_step.nonlinearFactorGraph.emplace_nonlinear<PriorFactor<double> >(X(2), 1.0, measurement_noise);
  three_step.nonlinearFactorGraph.emplace_nonlinear<PriorFactor<double> >(X(3), 2.0, measurement_noise);

  Values linearizationPoint;
  for (size_t k = 1; k <= 4; k++) {
    linearizationPoint.insert<double>(X(k), static_cast<double>(k - 1));
  }

  auto factors = three_step.nonlinearFactorGraph.linearize(three_step.linearizationPoint);

  // Eliminate x1
  Ordering ordering;
  ordering += X(1);
  ordering += X(2);
  ordering += X(3);

  GaussianMixture::shared_ptr gm;
  boost::shared_ptr<Factor> dtf;
  std::tie(gm, dtf) = EliminateHybrid(factors, ordering);
  auto discreteFactor = dynamic_pointer_cast<DecisionTreeFactor>(dtf);

  Switching four_step(4);

  HybridFactorGraph hf;

  hf.push_dc(gm);

  auto dKeys = discreteFactor->keys();
  std::cout << std::accumulate(dKeys.begin(), dKeys.end(), std::string(),
                               [](const std::string& a, const Key& b) -> std::string {
                                 return a + (a.length() > 0 ? "," : "") + (boost::format("(%s)") % Symbol(b)).str();
                               } ) << "\n";

  // Look, ma, I indeed added the discrete result factor!
  hf.push_discrete(discreteFactor);

  auto conditional = boost::make_shared<DiscreteConditional>(
      DiscreteKey{M(3), 2}, DiscreteKeys{{M(2), 2}}, "1/2 3/2");
  hf.push_discrete(conditional);

  hf.push_dc(four_step.nonlinearFactorGraph.dcGraph().at(2));

  hf.emplace_nonlinear<PriorFactor<double> >(X(4), 2.0, measurement_noise);

  Values lp;
  lp.insert(X(3), linearizationPoint.at(X(3)));
  lp.insert(X(4), linearizationPoint.at(X(4)));
  auto lhf = hf.linearize(lp);

  GTSAM_PRINT(lhf);

  ordering += X(4);

  GaussianMixture::shared_ptr gm_4;
  boost::shared_ptr<Factor> dtf_4;
  std::tie(gm_4, dtf_4) = EliminateHybrid(lhf, ordering);
  gm_4->print("GM4");
  auto discreteFactor4 = dynamic_pointer_cast<DecisionTreeFactor>(dtf_4);
  discreteFactor4->print("DTF4");
  (*discreteFactor4 * *discreteFactor).print("Product");
  // So at this point we can see that the product seems correct, but why isn't the eliminated thing correct?

}

/* ****************************************************************************/
/// Test the toDecisionTreeFactor method
TEST(HybridFactorGraph, ToDecisionTreeFactor) {
  size_t K = 3;

  // Provide tight sigma values so that the errors are visibly different.
  double between_sigma = 5e-8, prior_sigma = 1e-7;

  Switching self(K, between_sigma, prior_sigma);

  // Clear out discrete factors since sum() cannot hanldle those
  HybridFactorGraph linearizedFactorGraph(
      NonlinearFactorGraph(), DiscreteFactorGraph(),
      self.linearizedFactorGraph.dcGraph(),
      self.linearizedFactorGraph.gaussianGraph());

  auto decisionTreeFactor = linearizedFactorGraph.toDecisionTreeFactor();

  auto allAssignments = DiscreteValues::CartesianProduct(linearizedFactorGraph.discreteKeys());

  // Get the error of the discrete assignment m1=0, m2=1.
  double actual = (*decisionTreeFactor)(allAssignments[1]);

  /********************************************/
  // Create equivalent factor graph for m1=0, m2=1
  GaussianFactorGraph graph;

  // Add a prior on X(1).
  auto prior = boost::make_shared<PriorFactor<double>>(
      X(1), 0, Isotropic::Sigma(1, prior_sigma));
  auto gaussian_prior = prior->linearize(self.linearizationPoint);
  graph.push_back(gaussian_prior);

  // Add "motion models".
  auto noise_model = Isotropic::Sigma(1, between_sigma);
  auto between_x1_x2 =
      boost::make_shared<MotionModel>(X(1), X(2), 0.0, noise_model)
          ->linearize(self.linearizationPoint);
  auto between_x2_x3 =
      boost::make_shared<MotionModel>(X(2), X(3), 1.0, noise_model)
          ->linearize(self.linearizationPoint);

  graph.push_back(between_x1_x2);
  graph.push_back(between_x2_x3);

  VectorValues values = graph.optimize();
  double expected = graph.probPrime(values);
  /********************************************/

  EXPECT_DOUBLES_EQUAL(expected, actual, 1e-12);
}

/* ****************************************************************************/
// Test elimination
TEST(HybridFactorGraph, Elimination) {
  Switching self(3);

  // Create ordering.
  Ordering ordering;
  for (size_t k = 1; k <= self.K; k++) ordering += X(k);

  // Eliminate partially.
  auto result = self.linearizedFactorGraph.eliminatePartialSequential(ordering);

  CHECK(result.first);
  //  GTSAM_PRINT(*result.first);  // HybridBayesNet
  EXPECT_LONGS_EQUAL(3, result.first->size());

  CHECK(result.second);
  //  GTSAM_PRINT(*result.second);  // HybridFactorGraph
  EXPECT_LONGS_EQUAL(3, result.second->size());
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testHybridBayesNet.cpp
 * @brief   Unit tests for HybridBayesNet
 * @author  Varun Agrawal
 * @author  Fan Jiang
 * @author  Frank Dellaert
 * @date    December 2021
 */

#include <gtsam/base/serializationTestHelpers.h>
#include <gtsam/hybrid/HybridBayesNet.h>
#include <gtsam/hybrid/HybridBayesTree.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include "Switching.h"

// Include for test suite
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;
using namespace gtsam::serializationTestHelpers;

using noiseModel::Isotropic;
using symbol_shorthand::M;
using symbol_shorthand::X;

static const DiscreteKey Asia(0, 2);

/* ****************************************************************************/
// Test creation
TEST(HybridBayesNet, Creation) {
  HybridBayesNet bayesNet;

  bayesNet.add(Asia, "99/1");

  DiscreteConditional expected(Asia, "99/1");

  CHECK(bayesNet.atDiscrete(0));
  auto& df = *bayesNet.atDiscrete(0);
  EXPECT(df.equals(expected));
}

/* ****************************************************************************/
// Test adding a bayes net to another one.
TEST(HybridBayesNet, Add) {
  HybridBayesNet bayesNet;

  bayesNet.add(Asia, "99/1");

  DiscreteConditional expected(Asia, "99/1");

  HybridBayesNet other;
  other.push_back(bayesNet);
  EXPECT(bayesNet.equals(other));
}

/* ****************************************************************************/
// Test choosing an assignment of conditionals
TEST(HybridBayesNet, Choose) {
  Switching s(4);

  Ordering ordering;
  for (auto&& kvp : s.linearizationPoint) {
    ordering += kvp.key;
  }

  HybridBayesNet::shared_ptr hybridBayesNet;
  HybridGaussianFactorGraph::shared_ptr remainingFactorGraph;
  std::tie(hybridBayesNet, remainingFactorGraph) =
      s.linearizedFactorGraph.eliminatePartialSequential(ordering);

  DiscreteValues assignment;
  assignment[M(0)] = 1;
  assignment[M(1)] = 1;
  assignment[M(2)] = 0;

  GaussianBayesNet gbn = hybridBayesNet->choose(assignment);

  EXPECT_LONGS_EQUAL(4, gbn.size());

  EXPECT(assert_equal(*(*boost::dynamic_pointer_cast<GaussianMixture>(
                          hybridBayesNet->atMixture(0)))(assignment),
                      *gbn.at(0)));
  EXPECT(assert_equal(*(*boost::dynamic_pointer_cast<GaussianMixture>(
                          hybridBayesNet->atMixture(1)))(assignment),
                      *gbn.at(1)));
  EXPECT(assert_equal(*(*boost::dynamic_pointer_cast<GaussianMixture>(
                          hybridBayesNet->atMixture(2)))(assignment),
                      *gbn.at(2)));
  EXPECT(assert_equal(*(*boost::dynamic_pointer_cast<GaussianMixture>(
                          hybridBayesNet->atMixture(3)))(assignment),
                      *gbn.at(3)));
}

/* ****************************************************************************/
// Test bayes net optimize
TEST(HybridBayesNet, OptimizeAssignment) {
  Switching s(4);

  Ordering ordering;
  for (auto&& kvp : s.linearizationPoint) {
    ordering += kvp.key;
  }

  HybridBayesNet::shared_ptr hybridBayesNet;
  HybridGaussianFactorGraph::shared_ptr remainingFactorGraph;
  std::tie(hybridBayesNet, remainingFactorGraph) =
      s.linearizedFactorGraph.eliminatePartialSequential(ordering);

  DiscreteValues assignment;
  assignment[M(0)] = 1;
  assignment[M(1)] = 1;
  assignment[M(2)] = 1;

  VectorValues delta = hybridBayesNet->optimize(assignment);

  // The linearization point has the same value as the key index,
  // e.g. X(0) = 1, X(1) = 2,
  // but the factors specify X(k) = k-1, so delta should be -1.
  VectorValues expected_delta;
  expected_delta.insert(make_pair(X(0), -Vector1::Ones()));
  expected_delta.insert(make_pair(X(1), -Vector1::Ones()));
  expected_delta.insert(make_pair(X(2), -Vector1::Ones()));
  expected_delta.insert(make_pair(X(3), -Vector1::Ones()));

  EXPECT(assert_equal(expected_delta, delta));
}

/* ****************************************************************************/
// Test bayes net optimize
TEST(HybridBayesNet, Optimize) {
  Switching s(4);

  Ordering hybridOrdering = s.linearizedFactorGraph.getHybridOrdering();
  HybridBayesNet::shared_ptr hybridBayesNet =
      s.linearizedFactorGraph.eliminateSequential(hybridOrdering);

  HybridValues delta = hybridBayesNet->optimize();

  DiscreteValues expectedAssignment;
  expectedAssignment[M(0)] = 1;
  expectedAssignment[M(1)] = 0;
  expectedAssignment[M(2)] = 1;
  EXPECT(assert_equal(expectedAssignment, delta.discrete()));

  VectorValues expectedValues;
  expectedValues.insert(X(0), -0.999904 * Vector1::Ones());
  expectedValues.insert(X(1), -0.99029 * Vector1::Ones());
  expectedValues.insert(X(2), -1.00971 * Vector1::Ones());
  expectedValues.insert(X(3), -1.0001 * Vector1::Ones());

  EXPECT(assert_equal(expectedValues, delta.continuous(), 1e-5));
}

/* ****************************************************************************/
// Test bayes net multifrontal optimize
TEST(HybridBayesNet, OptimizeMultifrontal) {
  Switching s(4);

  Ordering hybridOrdering = s.linearizedFactorGraph.getHybridOrdering();
  HybridBayesTree::shared_ptr hybridBayesTree =
      s.linearizedFactorGraph.eliminateMultifrontal(hybridOrdering);
  HybridValues delta = hybridBayesTree->optimize();

  VectorValues expectedValues;
  expectedValues.insert(X(0), -0.999904 * Vector1::Ones());
  expectedValues.insert(X(1), -0.99029 * Vector1::Ones());
  expectedValues.insert(X(2), -1.00971 * Vector1::Ones());
  expectedValues.insert(X(3), -1.0001 * Vector1::Ones());

  EXPECT(assert_equal(expectedValues, delta.continuous(), 1e-5));
}

/* ****************************************************************************/
// Test bayes net error
TEST(HybridBayesNet, Error) {
  Switching s(3);

  Ordering hybridOrdering = s.linearizedFactorGraph.getHybridOrdering();
  HybridBayesNet::shared_ptr hybridBayesNet =
      s.linearizedFactorGraph.eliminateSequential(hybridOrdering);

  HybridValues delta = hybridBayesNet->optimize();
  auto error_tree = hybridBayesNet->error(delta.continuous());

  std::vector<DiscreteKey> discrete_keys = {{M(0), 2}, {M(1), 2}};
  std::vector<double> leaves = {0.0097568009, 3.3973404e-31, 0.029126214,
                                0.0097568009};
  AlgebraicDecisionTree<Key> expected_error(discrete_keys, leaves);

  // regression
  EXPECT(assert_equal(expected_error, error_tree, 1e-9));

  // Error on pruned bayes net
  auto prunedBayesNet = hybridBayesNet->prune(2);
  auto pruned_error_tree = prunedBayesNet.error(delta.continuous());

  std::vector<double> pruned_leaves = {2e50, 3.3973404e-31, 2e50, 0.0097568009};
  AlgebraicDecisionTree<Key> expected_pruned_error(discrete_keys,
                                                   pruned_leaves);

  // regression
  EXPECT(assert_equal(expected_pruned_error, pruned_error_tree, 1e-9));

  // Verify error computation and check for specific error value
  DiscreteValues discrete_values;
  boost::assign::insert(discrete_values)(M(0), 1)(M(1), 1);

  double total_error = 0;
  for (size_t idx = 0; idx < hybridBayesNet->size(); idx++) {
    if (hybridBayesNet->at(idx)->isHybrid()) {
      double error = hybridBayesNet->atMixture(idx)->error(delta.continuous(),
                                                           discrete_values);
      total_error += error;
    } else if (hybridBayesNet->at(idx)->isContinuous()) {
      double error = hybridBayesNet->atGaussian(idx)->error(delta.continuous());
      total_error += error;
    }
  }

  EXPECT_DOUBLES_EQUAL(
      total_error, hybridBayesNet->error(delta.continuous(), discrete_values),
      1e-9);
  EXPECT_DOUBLES_EQUAL(total_error, error_tree(discrete_values), 1e-9);
  EXPECT_DOUBLES_EQUAL(total_error, pruned_error_tree(discrete_values), 1e-9);
}

/* ****************************************************************************/
// Test bayes net pruning
TEST(HybridBayesNet, Prune) {
  Switching s(4);

  Ordering hybridOrdering = s.linearizedFactorGraph.getHybridOrdering();
  HybridBayesNet::shared_ptr hybridBayesNet =
      s.linearizedFactorGraph.eliminateSequential(hybridOrdering);

  HybridValues delta = hybridBayesNet->optimize();

  auto prunedBayesNet = hybridBayesNet->prune(2);
  HybridValues pruned_delta = prunedBayesNet.optimize();

  EXPECT(assert_equal(delta.discrete(), pruned_delta.discrete()));
  EXPECT(assert_equal(delta.continuous(), pruned_delta.continuous()));
}

/* ****************************************************************************/
// Test bayes net updateDiscreteConditionals
TEST(HybridBayesNet, UpdateDiscreteConditionals) {
  Switching s(4);

  Ordering hybridOrdering = s.linearizedFactorGraph.getHybridOrdering();
  HybridBayesNet::shared_ptr hybridBayesNet =
      s.linearizedFactorGraph.eliminateSequential(hybridOrdering);

  size_t maxNrLeaves = 3;
  auto discreteConditionals = hybridBayesNet->discreteConditionals();
  const DecisionTreeFactor::shared_ptr prunedDecisionTree =
      boost::make_shared<DecisionTreeFactor>(
          discreteConditionals->prune(maxNrLeaves));

  EXPECT_LONGS_EQUAL(maxNrLeaves + 2 /*2 zero leaves*/,
                     prunedDecisionTree->nrLeaves());

  auto original_discrete_conditionals =
      *(hybridBayesNet->at(4)->asDiscreteConditional());

  // Prune!
  hybridBayesNet->prune(maxNrLeaves);

  // Functor to verify values against the original_discrete_conditionals
  auto checker = [&](const Assignment<Key>& assignment,
                     double probability) -> double {
    // typecast so we can use this to get probability value
    DiscreteValues choices(assignment);
    if (prunedDecisionTree->operator()(choices) == 0) {
      EXPECT_DOUBLES_EQUAL(0.0, probability, 1e-9);
    } else {
      EXPECT_DOUBLES_EQUAL(original_discrete_conditionals(choices), probability,
                           1e-9);
    }
    return 0.0;
  };

  // Get the pruned discrete conditionals as an AlgebraicDecisionTree
  auto pruned_discrete_conditionals =
      hybridBayesNet->at(4)->asDiscreteConditional();
  auto discrete_conditional_tree =
      boost::dynamic_pointer_cast<DecisionTreeFactor::ADT>(
          pruned_discrete_conditionals);

  // The checker functor verifies the values for us.
  discrete_conditional_tree->apply(checker);
}

/* ****************************************************************************/
// Test HybridBayesNet serialization.
TEST(HybridBayesNet, Serialization) {
  Switching s(4);
  Ordering ordering = s.linearizedFactorGraph.getHybridOrdering();
  HybridBayesNet hbn = *(s.linearizedFactorGraph.eliminateSequential(ordering));

  EXPECT(equalsObj<HybridBayesNet>(hbn));
  EXPECT(equalsXML<HybridBayesNet>(hbn));
  EXPECT(equalsBinary<HybridBayesNet>(hbn));
}

/* ****************************************************************************/
// Test HybridBayesNet sampling.
TEST(HybridBayesNet, Sampling) {
  HybridNonlinearFactorGraph nfg;

  auto noise_model = noiseModel::Diagonal::Sigmas(Vector1(1.0));
  auto zero_motion =
      boost::make_shared<BetweenFactor<double>>(X(0), X(1), 0, noise_model);
  auto one_motion =
      boost::make_shared<BetweenFactor<double>>(X(0), X(1), 1, noise_model);
  std::vector<NonlinearFactor::shared_ptr> factors = {zero_motion, one_motion};
  nfg.emplace_nonlinear<PriorFactor<double>>(X(0), 0.0, noise_model);
  nfg.emplace_hybrid<MixtureFactor>(
      KeyVector{X(0), X(1)}, DiscreteKeys{DiscreteKey(M(0), 2)}, factors);

  DiscreteKey mode(M(0), 2);
  auto discrete_prior = boost::make_shared<DiscreteDistribution>(mode, "1/1");
  nfg.push_discrete(discrete_prior);

  Values initial;
  double z0 = 0.0, z1 = 1.0;
  initial.insert<double>(X(0), z0);
  initial.insert<double>(X(1), z1);

  // Create the factor graph from the nonlinear factor graph.
  HybridGaussianFactorGraph::shared_ptr fg = nfg.linearize(initial);
  // Eliminate into BN
  Ordering ordering = fg->getHybridOrdering();
  HybridBayesNet::shared_ptr bn = fg->eliminateSequential(ordering);

  // Set up sampling
  std::mt19937_64 gen(11);

  // Initialize containers for computing the mean values.
  vector<double> discrete_samples;
  VectorValues average_continuous;

  size_t num_samples = 1000;
  for (size_t i = 0; i < num_samples; i++) {
    // Sample
    HybridValues sample = bn->sample(&gen);

    discrete_samples.push_back(sample.discrete()[M(0)]);

    if (i == 0) {
      average_continuous.insert(sample.continuous());
    } else {
      average_continuous += sample.continuous();
    }
  }

  EXPECT_LONGS_EQUAL(2, average_continuous.size());
  EXPECT_LONGS_EQUAL(num_samples, discrete_samples.size());

  // Regressions don't work across platforms :-(
  // // regression for specific RNG seed
  // double discrete_sum =
  //     std::accumulate(discrete_samples.begin(), discrete_samples.end(),
  //                     decltype(discrete_samples)::value_type(0));
  // EXPECT_DOUBLES_EQUAL(0.477, discrete_sum / num_samples, 1e-9);

  // VectorValues expected;
  // expected.insert({X(0), Vector1(-0.0131207162712)});
  // expected.insert({X(1), Vector1(-0.499026377568)});
  // // regression for specific RNG seed
  // EXPECT(assert_equal(expected, average_continuous.scale(1.0 /
  // num_samples)));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

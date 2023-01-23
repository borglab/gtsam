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

#include <gtsam/hybrid/HybridBayesNet.h>
#include <gtsam/hybrid/HybridBayesTree.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include "Switching.h"
#include "TinyHybridExample.h"

// Include for test suite
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

using noiseModel::Isotropic;
using symbol_shorthand::M;
using symbol_shorthand::X;
using symbol_shorthand::Z;

static const Key asiaKey = 0;
static const DiscreteKey Asia(asiaKey, 2);

/* ****************************************************************************/
// Test creation of a pure discrete Bayes net.
TEST(HybridBayesNet, Creation) {
  HybridBayesNet bayesNet;
  bayesNet.emplace_back(new DiscreteConditional(Asia, "99/1"));

  DiscreteConditional expected(Asia, "99/1");
  CHECK(bayesNet.at(0)->asDiscrete());
  EXPECT(assert_equal(expected, *bayesNet.at(0)->asDiscrete()));
}

/* ****************************************************************************/
// Test adding a Bayes net to another one.
TEST(HybridBayesNet, Add) {
  HybridBayesNet bayesNet;
  bayesNet.emplace_back(new DiscreteConditional(Asia, "99/1"));

  HybridBayesNet other;
  other.add(bayesNet);
  EXPECT(bayesNet.equals(other));
}

/* ****************************************************************************/
// Test evaluate for a pure discrete Bayes net P(Asia).
TEST(HybridBayesNet, EvaluatePureDiscrete) {
  HybridBayesNet bayesNet;
  bayesNet.emplace_back(new DiscreteConditional(Asia, "4/6"));
  HybridValues values;
  values.insert(asiaKey, 0);
  EXPECT_DOUBLES_EQUAL(0.4, bayesNet.evaluate(values), 1e-9);
}

/* ****************************************************************************/
// Test creation of a tiny hybrid Bayes net.
TEST(HybridBayesNet, Tiny) {
  auto bn = tiny::createHybridBayesNet();
  EXPECT_LONGS_EQUAL(3, bn.size());

  const VectorValues vv{{Z(0), Vector1(5.0)}, {X(0), Vector1(5.0)}};
  auto fg = bn.toFactorGraph(vv);
  EXPECT_LONGS_EQUAL(3, fg.size());

  // Check that the ratio of probPrime to evaluate is the same for all modes.
  std::vector<double> ratio(2);
  for (size_t mode : {0, 1}) {
    const HybridValues hv{vv, {{M(0), mode}}};
    ratio[mode] = std::exp(-fg.error(hv)) / bn.evaluate(hv);
  }
  EXPECT_DOUBLES_EQUAL(ratio[0], ratio[1], 1e-8);
}

/* ****************************************************************************/
// Test evaluate for a hybrid Bayes net P(X0|X1) P(X1|Asia) P(Asia).
TEST(HybridBayesNet, evaluateHybrid) {
  const auto continuousConditional = GaussianConditional::sharedMeanAndStddev(
      X(0), 2 * I_1x1, X(1), Vector1(-4.0), 5.0);

  const SharedDiagonal model0 = noiseModel::Diagonal::Sigmas(Vector1(2.0)),
                       model1 = noiseModel::Diagonal::Sigmas(Vector1(3.0));

  const auto conditional0 = std::make_shared<GaussianConditional>(
                 X(1), Vector1::Constant(5), I_1x1, model0),
             conditional1 = std::make_shared<GaussianConditional>(
                 X(1), Vector1::Constant(2), I_1x1, model1);

  // Create hybrid Bayes net.
  HybridBayesNet bayesNet;
  bayesNet.push_back(continuousConditional);
  bayesNet.emplace_back(
      new GaussianMixture({X(1)}, {}, {Asia}, {conditional0, conditional1}));
  bayesNet.emplace_back(new DiscreteConditional(Asia, "99/1"));

  // Create values at which to evaluate.
  HybridValues values;
  values.insert(asiaKey, 0);
  values.insert(X(0), Vector1(-6));
  values.insert(X(1), Vector1(1));

  const double conditionalProbability =
      continuousConditional->evaluate(values.continuous());
  const double mixtureProbability = conditional0->evaluate(values.continuous());
  EXPECT_DOUBLES_EQUAL(conditionalProbability * mixtureProbability * 0.99,
                       bayesNet.evaluate(values), 1e-9);
}

/* ****************************************************************************/
// Test choosing an assignment of conditionals
TEST(HybridBayesNet, Choose) {
  Switching s(4);

  const Ordering ordering(s.linearizationPoint.keys());

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

  EXPECT(assert_equal(*(*hybridBayesNet->at(0)->asMixture())(assignment),
                      *gbn.at(0)));
  EXPECT(assert_equal(*(*hybridBayesNet->at(1)->asMixture())(assignment),
                      *gbn.at(1)));
  EXPECT(assert_equal(*(*hybridBayesNet->at(2)->asMixture())(assignment),
                      *gbn.at(2)));
  EXPECT(assert_equal(*(*hybridBayesNet->at(3)->asMixture())(assignment),
                      *gbn.at(3)));
}

/* ****************************************************************************/
// Test Bayes net optimize
TEST(HybridBayesNet, OptimizeAssignment) {
  Switching s(4);

  const Ordering ordering(s.linearizationPoint.keys());

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
// Test Bayes net optimize
TEST(HybridBayesNet, Optimize) {
  Switching s(4, 1.0, 0.1, {0, 1, 2, 3}, "1/1 1/1");

  HybridBayesNet::shared_ptr hybridBayesNet =
      s.linearizedFactorGraph.eliminateSequential();

  HybridValues delta = hybridBayesNet->optimize();

  // NOTE: The true assignment is 111, but the discrete priors cause 101
  DiscreteValues expectedAssignment;
  expectedAssignment[M(0)] = 1;
  expectedAssignment[M(1)] = 1;
  expectedAssignment[M(2)] = 1;
  EXPECT(assert_equal(expectedAssignment, delta.discrete()));

  VectorValues expectedValues;
  expectedValues.insert(X(0), -Vector1::Ones());
  expectedValues.insert(X(1), -Vector1::Ones());
  expectedValues.insert(X(2), -Vector1::Ones());
  expectedValues.insert(X(3), -Vector1::Ones());

  EXPECT(assert_equal(expectedValues, delta.continuous(), 1e-5));
}

/* ****************************************************************************/
// Test Bayes net error
TEST(HybridBayesNet, Pruning) {
  Switching s(3);

  HybridBayesNet::shared_ptr posterior =
      s.linearizedFactorGraph.eliminateSequential();
  EXPECT_LONGS_EQUAL(5, posterior->size());

  HybridValues delta = posterior->optimize();
  auto actualTree = posterior->evaluate(delta.continuous());

  // Regression test on density tree.
  std::vector<DiscreteKey> discrete_keys = {{M(0), 2}, {M(1), 2}};
  std::vector<double> leaves = {6.1112424, 20.346113, 17.785849, 19.738098};
  AlgebraicDecisionTree<Key> expected(discrete_keys, leaves);
  EXPECT(assert_equal(expected, actualTree, 1e-6));

  // Prune and get probabilities
  auto prunedBayesNet = posterior->prune(2);
  auto prunedTree = prunedBayesNet.evaluate(delta.continuous());

  // Regression test on pruned logProbability tree
  std::vector<double> pruned_leaves = {0.0, 20.346113, 0.0, 19.738098};
  AlgebraicDecisionTree<Key> expected_pruned(discrete_keys, pruned_leaves);
  EXPECT(assert_equal(expected_pruned, prunedTree, 1e-6));

  // Verify logProbability computation and check specific logProbability value
  const DiscreteValues discrete_values{{M(0), 1}, {M(1), 1}};
  const HybridValues hybridValues{delta.continuous(), discrete_values};
  double logProbability = 0;
  logProbability += posterior->at(0)->asMixture()->logProbability(hybridValues);
  logProbability += posterior->at(1)->asMixture()->logProbability(hybridValues);
  logProbability += posterior->at(2)->asMixture()->logProbability(hybridValues);
  // NOTE(dellaert): the discrete errors were not added in logProbability tree!
  logProbability +=
      posterior->at(3)->asDiscrete()->logProbability(hybridValues);
  logProbability +=
      posterior->at(4)->asDiscrete()->logProbability(hybridValues);

  double density = exp(logProbability);
  EXPECT_DOUBLES_EQUAL(density, actualTree(discrete_values), 1e-9);
  EXPECT_DOUBLES_EQUAL(density, prunedTree(discrete_values), 1e-9);
  EXPECT_DOUBLES_EQUAL(logProbability, posterior->logProbability(hybridValues),
                       1e-9);
}

/* ****************************************************************************/
// Test Bayes net pruning
TEST(HybridBayesNet, Prune) {
  Switching s(4);

  HybridBayesNet::shared_ptr posterior =
      s.linearizedFactorGraph.eliminateSequential();
  EXPECT_LONGS_EQUAL(7, posterior->size());

  HybridValues delta = posterior->optimize();

  auto prunedBayesNet = posterior->prune(2);
  HybridValues pruned_delta = prunedBayesNet.optimize();

  EXPECT(assert_equal(delta.discrete(), pruned_delta.discrete()));
  EXPECT(assert_equal(delta.continuous(), pruned_delta.continuous()));
}

/* ****************************************************************************/
// Test Bayes net updateDiscreteConditionals
TEST(HybridBayesNet, UpdateDiscreteConditionals) {
  Switching s(4);

  HybridBayesNet::shared_ptr posterior =
      s.linearizedFactorGraph.eliminateSequential();
  EXPECT_LONGS_EQUAL(7, posterior->size());

  size_t maxNrLeaves = 3;
  auto discreteConditionals = posterior->discreteConditionals();
  const DecisionTreeFactor::shared_ptr prunedDecisionTree =
      std::make_shared<DecisionTreeFactor>(
          discreteConditionals->prune(maxNrLeaves));

  EXPECT_LONGS_EQUAL(maxNrLeaves + 2 /*2 zero leaves*/,
                     prunedDecisionTree->nrLeaves());

  auto original_discrete_conditionals = *(posterior->at(4)->asDiscrete());

  // Prune!
  posterior->prune(maxNrLeaves);

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
  auto pruned_discrete_conditionals = posterior->at(4)->asDiscrete();
  auto discrete_conditional_tree =
      std::dynamic_pointer_cast<DecisionTreeFactor::ADT>(
          pruned_discrete_conditionals);

  // The checker functor verifies the values for us.
  discrete_conditional_tree->apply(checker);
}

/* ****************************************************************************/
// Test HybridBayesNet sampling.
TEST(HybridBayesNet, Sampling) {
  HybridNonlinearFactorGraph nfg;

  auto noise_model = noiseModel::Diagonal::Sigmas(Vector1(1.0));
  auto zero_motion =
      std::make_shared<BetweenFactor<double>>(X(0), X(1), 0, noise_model);
  auto one_motion =
      std::make_shared<BetweenFactor<double>>(X(0), X(1), 1, noise_model);
  std::vector<NonlinearFactor::shared_ptr> factors = {zero_motion, one_motion};
  nfg.emplace_shared<PriorFactor<double>>(X(0), 0.0, noise_model);
  nfg.emplace_shared<MixtureFactor>(
      KeyVector{X(0), X(1)}, DiscreteKeys{DiscreteKey(M(0), 2)}, factors);

  DiscreteKey mode(M(0), 2);
  nfg.emplace_shared<DiscreteDistribution>(mode, "1/1");

  Values initial;
  double z0 = 0.0, z1 = 1.0;
  initial.insert<double>(X(0), z0);
  initial.insert<double>(X(1), z1);

  // Create the factor graph from the nonlinear factor graph.
  HybridGaussianFactorGraph::shared_ptr fg = nfg.linearize(initial);
  // Eliminate into BN
  HybridBayesNet::shared_ptr bn = fg->eliminateSequential();

  // Set up sampling
  std::mt19937_64 gen(11);

  // Initialize containers for computing the mean values.
  vector<double> discrete_samples;
  VectorValues average_continuous;

  size_t num_samples = 1000;
  for (size_t i = 0; i < num_samples; i++) {
    // Sample
    HybridValues sample = bn->sample(&gen);

    discrete_samples.push_back(sample.discrete().at(M(0)));

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

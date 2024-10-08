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

#include <gtsam/base/Testable.h>
#include <gtsam/discrete/DiscreteFactor.h>
#include <gtsam/hybrid/HybridBayesNet.h>
#include <gtsam/hybrid/HybridBayesTree.h>
#include <gtsam/hybrid/HybridConditional.h>
#include <gtsam/hybrid/HybridGaussianFactorGraph.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include "Switching.h"
#include "TinyHybridExample.h"

// Include for test suite
#include <CppUnitLite/TestHarness.h>
#include <memory>

using namespace std;
using namespace gtsam;

using symbol_shorthand::M;
using symbol_shorthand::X;
using symbol_shorthand::Z;

static const Key asiaKey = 0;
static const DiscreteKey Asia(asiaKey, 2);

/* ****************************************************************************/
// Test creation of a pure discrete Bayes net.
TEST(HybridBayesNet, Creation) {
  HybridBayesNet bayesNet;
  bayesNet.emplace_shared<DiscreteConditional>(Asia, "99/1");

  DiscreteConditional expected(Asia, "99/1");
  CHECK(bayesNet.at(0)->asDiscrete());
  EXPECT(assert_equal(expected, *bayesNet.at(0)->asDiscrete()));
}

/* ****************************************************************************/
// Test adding a Bayes net to another one.
TEST(HybridBayesNet, Add) {
  HybridBayesNet bayesNet;
  bayesNet.emplace_shared<DiscreteConditional>(Asia, "99/1");

  HybridBayesNet other;
  other.add(bayesNet);
  EXPECT(bayesNet.equals(other));
}

/* ****************************************************************************/
// Test API for a pure discrete Bayes net P(Asia).
TEST(HybridBayesNet, EvaluatePureDiscrete) {
  HybridBayesNet bayesNet;
  const auto pAsia = std::make_shared<DiscreteConditional>(Asia, "4/6");
  bayesNet.push_back(pAsia);
  HybridValues zero{{}, {{asiaKey, 0}}}, one{{}, {{asiaKey, 1}}};

  // choose
  GaussianBayesNet empty;
  EXPECT(assert_equal(empty, bayesNet.choose(zero.discrete()), 1e-9));

  // evaluate
  EXPECT_DOUBLES_EQUAL(0.4, bayesNet.evaluate(zero), 1e-9);
  EXPECT_DOUBLES_EQUAL(0.4, bayesNet(zero), 1e-9);

  // optimize
  EXPECT(assert_equal(one, bayesNet.optimize()));
  EXPECT(assert_equal(VectorValues{}, bayesNet.optimize(one.discrete())));

  // sample
  std::mt19937_64 rng(42);
  EXPECT(assert_equal(zero, bayesNet.sample(&rng)));
  EXPECT(assert_equal(one, bayesNet.sample(one, &rng)));
  EXPECT(assert_equal(zero, bayesNet.sample(zero, &rng)));

  // prune
  EXPECT(assert_equal(bayesNet, bayesNet.prune(2)));
  EXPECT_LONGS_EQUAL(1, bayesNet.prune(1).at(0)->size());

  // error
  EXPECT_DOUBLES_EQUAL(-log(0.4), bayesNet.error(zero), 1e-9);
  EXPECT_DOUBLES_EQUAL(-log(0.6), bayesNet.error(one), 1e-9);

  // errorTree
  AlgebraicDecisionTree<Key> expected(asiaKey, -log(0.4), -log(0.6));
  EXPECT(assert_equal(expected, bayesNet.errorTree({})));

  // logProbability
  EXPECT_DOUBLES_EQUAL(log(0.4), bayesNet.logProbability(zero), 1e-9);
  EXPECT_DOUBLES_EQUAL(log(0.6), bayesNet.logProbability(one), 1e-9);

  // discretePosterior
  AlgebraicDecisionTree<Key> expectedPosterior(asiaKey, 0.4, 0.6);
  EXPECT(assert_equal(expectedPosterior, bayesNet.discretePosterior({})));

  // toFactorGraph
  HybridGaussianFactorGraph expectedFG{pAsia}, fg = bayesNet.toFactorGraph({});
  EXPECT(assert_equal(expectedFG, fg));
}

/* ****************************************************************************/
// Test API for a tiny hybrid Bayes net.
TEST(HybridBayesNet, Tiny) {
  auto bayesNet = tiny::createHybridBayesNet();  // P(z|x,mode)P(x)P(mode)
  EXPECT_LONGS_EQUAL(3, bayesNet.size());

  const VectorValues vv{{Z(0), Vector1(5.0)}, {X(0), Vector1(5.0)}};
  HybridValues zero{vv, {{M(0), 0}}}, one{vv, {{M(0), 1}}};

  // Check Invariants for components
  HybridGaussianConditional::shared_ptr hgc = bayesNet.at(0)->asHybrid();
  GaussianConditional::shared_ptr gc0 = hgc->choose(zero.discrete()),
                                  gc1 = hgc->choose(one.discrete());
  GaussianConditional::shared_ptr px = bayesNet.at(1)->asGaussian();
  GaussianConditional::CheckInvariants(*gc0, vv);
  GaussianConditional::CheckInvariants(*gc1, vv);
  GaussianConditional::CheckInvariants(*px, vv);
  HybridGaussianConditional::CheckInvariants(*hgc, zero);
  HybridGaussianConditional::CheckInvariants(*hgc, one);

  // choose
  GaussianBayesNet expectedChosen;
  expectedChosen.push_back(gc0);
  expectedChosen.push_back(px);
  auto chosen0 = bayesNet.choose(zero.discrete());
  auto chosen1 = bayesNet.choose(one.discrete());
  EXPECT(assert_equal(expectedChosen, chosen0, 1e-9));

  // logProbability
  const double logP0 = chosen0.logProbability(vv) + log(0.4);  // 0.4 is prior
  const double logP1 = chosen1.logProbability(vv) + log(0.6);  // 0.6 is prior
  EXPECT_DOUBLES_EQUAL(logP0, bayesNet.logProbability(zero), 1e-9);
  EXPECT_DOUBLES_EQUAL(logP1, bayesNet.logProbability(one), 1e-9);

  // evaluate
  EXPECT_DOUBLES_EQUAL(exp(logP0), bayesNet.evaluate(zero), 1e-9);

  // optimize
  EXPECT(assert_equal(one, bayesNet.optimize()));
  EXPECT(assert_equal(chosen0.optimize(), bayesNet.optimize(zero.discrete())));

  // sample. Not deterministic !!! TODO(Frank): figure out why
  // std::mt19937_64 rng(42);
  // EXPECT(assert_equal({{M(0), 1}}, bayesNet.sample(&rng).discrete()));

  // prune
  auto pruned = bayesNet.prune(1);
  CHECK(pruned.at(1)->asHybrid());
  EXPECT_LONGS_EQUAL(1, pruned.at(1)->asHybrid()->nrComponents());
  EXPECT(!pruned.equals(bayesNet));

  // error
  const double error0 = chosen0.error(vv) + gc0->negLogConstant() -
                        px->negLogConstant() - log(0.4);
  const double error1 = chosen1.error(vv) + gc1->negLogConstant() -
                        px->negLogConstant() - log(0.6);
  // print errors:
  EXPECT_DOUBLES_EQUAL(error0, bayesNet.error(zero), 1e-9);
  EXPECT_DOUBLES_EQUAL(error1, bayesNet.error(one), 1e-9);
  EXPECT_DOUBLES_EQUAL(error0 + logP0, error1 + logP1, 1e-9);

  // errorTree
  AlgebraicDecisionTree<Key> expected(M(0), error0, error1);
  EXPECT(assert_equal(expected, bayesNet.errorTree(vv)));

  // discretePosterior
  // We have: P(z|x,mode)P(x)P(mode). When we condition on z and x, we get
  // P(mode|z,x) \propto P(z|x,mode)P(x)P(mode)
  // Normalizing this yields posterior P(mode|z,x) = {0.8, 0.2}
  double q0 = std::exp(logP0), q1 = std::exp(logP1), sum = q0 + q1;
  AlgebraicDecisionTree<Key> expectedPosterior(M(0), q0 / sum, q1 / sum);
  EXPECT(assert_equal(expectedPosterior, bayesNet.discretePosterior(vv)));

  // toFactorGraph
  auto fg = bayesNet.toFactorGraph({{Z(0), Vector1(5.0)}});
  EXPECT_LONGS_EQUAL(3, fg.size());

  // Create the product factor for eliminating x0:
  HybridGaussianFactorGraph factors_x0;
  factors_x0.push_back(fg.at(0));
  factors_x0.push_back(fg.at(1));
  auto productFactor = factors_x0.collectProductFactor();

  // Check that scalars are 0 and 1.79
  EXPECT_DOUBLES_EQUAL(0.0, productFactor({{M(0), 0}}).second, 1e-9);
  EXPECT_DOUBLES_EQUAL(1.791759, productFactor({{M(0), 1}}).second, 1e-5);

  // Call eliminate and check scalar:
  auto result = factors_x0.eliminate({X(0)});
  auto df = std::dynamic_pointer_cast<DecisionTreeFactor>(result.second);

  // Check that the ratio of probPrime to evaluate is the same for all modes.
  std::vector<double> ratio(2);
  ratio[0] = std::exp(-fg.error(zero)) / bayesNet.evaluate(zero);
  ratio[1] = std::exp(-fg.error(one)) / bayesNet.evaluate(one);
  EXPECT_DOUBLES_EQUAL(ratio[0], ratio[1], 1e-8);

  // Better and more general test:
  // Since ϕ(M, x) \propto P(M,x|z) the discretePosteriors should agree
  q0 = std::exp(-fg.error(zero));
  q1 = std::exp(-fg.error(one));
  sum = q0 + q1;
  EXPECT(assert_equal(expectedPosterior, {M(0), q0 / sum, q1 / sum}));
  VectorValues xv{{X(0), Vector1(5.0)}};
  auto fgPosterior = fg.discretePosterior(xv);
  EXPECT(assert_equal(expectedPosterior, fgPosterior));
}

/* ****************************************************************************/
// Hybrid Bayes net P(X0|X1) P(X1|Asia) P(Asia).
namespace different_sigmas {
const auto gc = GaussianConditional::sharedMeanAndStddev(X(0), 2 * I_1x1, X(1),
                                                         Vector1(-4.0), 5.0);

const std::vector<std::pair<Vector, double>> parms{{Vector1(5), 2.0},
                                                   {Vector1(2), 3.0}};
const auto hgc = std::make_shared<HybridGaussianConditional>(Asia, X(1), parms);

const auto prior = std::make_shared<DiscreteConditional>(Asia, "99/1");
auto wrap = [](const auto& c) {
  return std::make_shared<HybridConditional>(c);
};
const HybridBayesNet bayesNet{wrap(gc), wrap(hgc), wrap(prior)};

// Create values at which to evaluate.
HybridValues values{{{X(0), Vector1(-6)}, {X(1), Vector1(1)}}, {{asiaKey, 0}}};
}  // namespace different_sigmas

/* ****************************************************************************/
// Test evaluate for a hybrid Bayes net P(X0|X1) P(X1|Asia) P(Asia).
TEST(HybridBayesNet, evaluateHybrid) {
  using namespace different_sigmas;

  const double conditionalProbability = gc->evaluate(values.continuous());
  const double mixtureProbability = hgc->evaluate(values);
  EXPECT_DOUBLES_EQUAL(conditionalProbability * mixtureProbability * 0.99,
                       bayesNet.evaluate(values), 1e-9);
}

/* ****************************************************************************/
// Test choosing an assignment of conditionals
TEST(HybridBayesNet, Choose) {
  Switching s(4);

  const Ordering ordering(s.linearizationPoint.keys());

  const auto [hybridBayesNet, remainingFactorGraph] =
      s.linearizedFactorGraph.eliminatePartialSequential(ordering);

  DiscreteValues assignment;
  assignment[M(0)] = 1;
  assignment[M(1)] = 1;
  assignment[M(2)] = 0;

  GaussianBayesNet gbn = hybridBayesNet->choose(assignment);

  EXPECT_LONGS_EQUAL(4, gbn.size());

  EXPECT(assert_equal(*(*hybridBayesNet->at(0)->asHybrid())(assignment),
                      *gbn.at(0)));
  EXPECT(assert_equal(*(*hybridBayesNet->at(1)->asHybrid())(assignment),
                      *gbn.at(1)));
  EXPECT(assert_equal(*(*hybridBayesNet->at(2)->asHybrid())(assignment),
                      *gbn.at(2)));
  EXPECT(assert_equal(*(*hybridBayesNet->at(3)->asHybrid())(assignment),
                      *gbn.at(3)));
}

/* ****************************************************************************/
// Test Bayes net optimize
TEST(HybridBayesNet, OptimizeAssignment) {
  Switching s(4);

  const Ordering ordering(s.linearizationPoint.keys());

  const auto [hybridBayesNet, remainingFactorGraph] =
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
  // Create switching network with three continuous variables and two discrete:
  // ϕ(x0) ϕ(x0,x1,m0) ϕ(x1,x2,m1) ϕ(x0;z0) ϕ(x1;z1) ϕ(x2;z2) ϕ(m0) ϕ(m0,m1)
  Switching s(3);

  HybridBayesNet::shared_ptr posterior =
      s.linearizedFactorGraph.eliminateSequential();
  EXPECT_LONGS_EQUAL(5, posterior->size());

  // Optimize
  HybridValues delta = posterior->optimize();

  // Verify discrete posterior at optimal value sums to 1.
  auto discretePosterior = posterior->discretePosterior(delta.continuous());
  EXPECT_DOUBLES_EQUAL(1.0, discretePosterior.sum(), 1e-9);

  // Regression test on discrete posterior at optimal value.
  std::vector<double> leaves = {0.095516068, 0.31800092, 0.27798511, 0.3084979};
  AlgebraicDecisionTree<Key> expected(s.modes, leaves);
  EXPECT(assert_equal(expected, discretePosterior, 1e-6));

  // Prune and get probabilities
  auto prunedBayesNet = posterior->prune(2);
  auto prunedTree = prunedBayesNet.discretePosterior(delta.continuous());

  // Verify logProbability computation and check specific logProbability value
  const DiscreteValues discrete_values{{M(0), 1}, {M(1), 1}};
  const HybridValues hybridValues{delta.continuous(), discrete_values};
  double logProbability = 0;
  logProbability += posterior->at(0)->asHybrid()->logProbability(hybridValues);
  logProbability += posterior->at(1)->asHybrid()->logProbability(hybridValues);
  logProbability += posterior->at(2)->asHybrid()->logProbability(hybridValues);
  logProbability +=
      posterior->at(3)->asDiscrete()->logProbability(hybridValues);
  logProbability +=
      posterior->at(4)->asDiscrete()->logProbability(hybridValues);
  EXPECT_DOUBLES_EQUAL(logProbability, posterior->logProbability(hybridValues),
                       1e-9);

  // Check agreement with discrete posterior
  // double density = exp(logProbability);
  // FAILS: EXPECT_DOUBLES_EQUAL(density, discretePosterior(discrete_values),
  // 1e-6);

  // Regression test on pruned logProbability tree
  std::vector<double> pruned_leaves = {0.0, 0.50758422, 0.0, 0.49241578};
  AlgebraicDecisionTree<Key> expected_pruned(s.modes, pruned_leaves);
  EXPECT(assert_equal(expected_pruned, prunedTree, 1e-6));

  // Regression
  // FAILS: EXPECT_DOUBLES_EQUAL(density, prunedTree(discrete_values), 1e-9);
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

  DiscreteConditional joint;
  for (auto&& conditional : posterior->discreteMarginal()) {
    joint = joint * (*conditional);
  }

  size_t maxNrLeaves = 3;
  auto prunedDecisionTree = joint.prune(maxNrLeaves);

#ifdef GTSAM_DT_MERGING
  EXPECT_LONGS_EQUAL(maxNrLeaves + 2 /*2 zero leaves*/,
                     prunedDecisionTree.nrLeaves());
#else
  EXPECT_LONGS_EQUAL(8 /*full tree*/, prunedDecisionTree.nrLeaves());
#endif

  // regression
  // NOTE(Frank): I had to include *three* non-zeroes here now.
  DecisionTreeFactor::ADT potentials(
      s.modes,
      std::vector<double>{0, 0, 0, 0.28739288, 0, 0.43106901, 0, 0.2815381});
  DiscreteConditional expectedConditional(3, s.modes, potentials);

  // Prune!
  auto pruned = posterior->prune(maxNrLeaves);

  // Functor to verify values against the expectedConditional
  auto checker = [&](const Assignment<Key>& assignment,
                     double probability) -> double {
    // typecast so we can use this to get probability value
    DiscreteValues choices(assignment);
    if (prunedDecisionTree(choices) == 0) {
      EXPECT_DOUBLES_EQUAL(0.0, probability, 1e-9);
    } else {
      EXPECT_DOUBLES_EQUAL(expectedConditional(choices), probability, 1e-6);
    }
    return 0.0;
  };

  // Get the pruned discrete conditionals as an AlgebraicDecisionTree
  CHECK(pruned.at(0)->asDiscrete());
  auto pruned_discrete_conditionals = pruned.at(0)->asDiscrete();
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
  nfg.emplace_shared<PriorFactor<double>>(X(0), 0.0, noise_model);

  auto zero_motion =
      std::make_shared<BetweenFactor<double>>(X(0), X(1), 0, noise_model);
  auto one_motion =
      std::make_shared<BetweenFactor<double>>(X(0), X(1), 1, noise_model);
  nfg.emplace_shared<HybridNonlinearFactor>(
      DiscreteKey(M(0), 2),
      std::vector<NoiseModelFactor::shared_ptr>{zero_motion, one_motion});

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

/* ****************************************************************************/
// Test hybrid gaussian factor graph errorTree when
// there is a HybridConditional in the graph
TEST(HybridBayesNet, ErrorTreeWithConditional) {
  using symbol_shorthand::F;

  Key z0 = Z(0), f01 = F(0);
  Key x0 = X(0), x1 = X(1);

  HybridBayesNet hbn;

  auto prior_model = noiseModel::Isotropic::Sigma(1, 1e-1);
  auto measurement_model = noiseModel::Isotropic::Sigma(1, 2.0);

  // Set a prior P(x0) at x0=0
  hbn.emplace_shared<GaussianConditional>(x0, Vector1(0.0), I_1x1, prior_model);

  // Add measurement P(z0 | x0)
  hbn.emplace_shared<GaussianConditional>(z0, Vector1(0.0), -I_1x1, x0, I_1x1,
                                          measurement_model);

  // Add hybrid motion model
  double mu = 0.0;
  double sigma0 = 1e2, sigma1 = 1e-2;
  auto model0 = noiseModel::Isotropic::Sigma(1, sigma0);
  auto model1 = noiseModel::Isotropic::Sigma(1, sigma1);
  auto c0 = make_shared<GaussianConditional>(f01, Vector1(mu), I_1x1, x1, I_1x1,
                                             x0, -I_1x1, model0),
       c1 = make_shared<GaussianConditional>(f01, Vector1(mu), I_1x1, x1, I_1x1,
                                             x0, -I_1x1, model1);
  DiscreteKey m1(M(2), 2);
  hbn.emplace_shared<HybridGaussianConditional>(m1, std::vector{c0, c1});

  // Discrete uniform prior.
  hbn.emplace_shared<DiscreteConditional>(m1, "0.5/0.5");

  VectorValues given;
  given.insert(z0, Vector1(0.0));
  given.insert(f01, Vector1(0.0));
  auto gfg = hbn.toFactorGraph(given);

  VectorValues vv;
  vv.insert(x0, Vector1(1.0));
  vv.insert(x1, Vector1(2.0));
  AlgebraicDecisionTree<Key> errorTree = gfg.errorTree(vv);

  // regression
  AlgebraicDecisionTree<Key> expected(m1, 60.028538, 5050.8181);
  EXPECT(assert_equal(expected, errorTree, 1e-4));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 *  @file testHybridGaussianFactorGraph.cpp
 *  @date Mar 11, 2022
 *  @author Fan Jiang
 *  @author Varun Agrawal
 *  @author Frank Dellaert
 */

#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/Vector.h>
#include <gtsam/discrete/DecisionTreeFactor.h>
#include <gtsam/discrete/DiscreteKey.h>
#include <gtsam/discrete/DiscreteValues.h>
#include <gtsam/hybrid/HybridBayesNet.h>
#include <gtsam/hybrid/HybridConditional.h>
#include <gtsam/hybrid/HybridFactor.h>
#include <gtsam/hybrid/HybridGaussianConditional.h>
#include <gtsam/hybrid/HybridGaussianFactor.h>
#include <gtsam/hybrid/HybridGaussianFactorGraph.h>
#include <gtsam/hybrid/HybridGaussianProductFactor.h>
#include <gtsam/hybrid/HybridValues.h>
#include <gtsam/inference/BayesNet.h>
#include <gtsam/inference/Key.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/JacobianFactor.h>

#include <CppUnitLite/Test.h>
#include <CppUnitLite/TestHarness.h>

#include <cstddef>
#include <memory>
#include <vector>

#include "Switching.h"
#include "TinyHybridExample.h"
#include "gtsam/linear/GaussianFactorGraph.h"

using namespace std;
using namespace gtsam;

using gtsam::symbol_shorthand::M;
using gtsam::symbol_shorthand::N;
using gtsam::symbol_shorthand::X;
using gtsam::symbol_shorthand::Z;

// Set up sampling
std::mt19937_64 kRng(42);

static const DiscreteKey m0(M(0), 2), m1(M(1), 2), m2(M(2), 2);

/* ************************************************************************* */
TEST(HybridGaussianFactorGraph, Creation) {
  HybridConditional conditional;

  HybridGaussianFactorGraph hfg;

  hfg.emplace_shared<JacobianFactor>(X(0), I_3x3, Z_3x1);

  // Define a hybrid gaussian conditional P(x0|x1, c0)
  // and add it to the factor graph.
  HybridGaussianConditional gm(
      m0,
      {std::make_shared<GaussianConditional>(X(0), Z_3x1, I_3x3, X(1), I_3x3),
       std::make_shared<GaussianConditional>(X(0), Vector3::Ones(), I_3x3, X(1), I_3x3)});
  hfg.add(gm);

  EXPECT_LONGS_EQUAL(2, hfg.size());
}

/* ************************************************************************* */
TEST(HybridGaussianFactorGraph, EliminateSequential) {
  // Test elimination of a single variable.
  HybridGaussianFactorGraph hfg;

  hfg.emplace_shared<JacobianFactor>(0, I_3x3, Z_3x1);

  auto result = hfg.eliminatePartialSequential(KeyVector{0});

  EXPECT_LONGS_EQUAL(result.first->size(), 1);
}

/* ************************************************************************* */

namespace two {
std::vector<GaussianFactor::shared_ptr> components(Key key) {
  return {std::make_shared<JacobianFactor>(key, I_3x3, Z_3x1),
          std::make_shared<JacobianFactor>(key, I_3x3, Vector3::Ones())};
}
}  // namespace two

/* ************************************************************************* */
TEST(HybridGaussianFactorGraph, hybridEliminationOneFactor) {
  HybridGaussianFactorGraph hfg;
  hfg.add(HybridGaussianFactor(m1, two::components(X(1))));

  auto result = hfg.eliminate({X(1)});

  // Check that we have a valid Gaussian conditional.
  auto hgc = result.first->asHybrid();
  CHECK(hgc);
  const HybridValues values{{{X(1), Z_3x1}}, {{M(1), 1}}};
  EXPECT(HybridConditional::CheckInvariants(*result.first, values));

  // Check that factor is discrete and correct
  auto factor = std::dynamic_pointer_cast<DecisionTreeFactor>(result.second);
  CHECK(factor);
  // regression test
  EXPECT(assert_equal(DecisionTreeFactor{m1, "15.74961 15.74961"}, *factor, 1e-5));
}

/* ************************************************************************* */
TEST(HybridGaussianFactorGraph, eliminateFullSequentialEqualChance) {
  HybridGaussianFactorGraph hfg;

  // Add prior on x0
  hfg.add(JacobianFactor(X(0), I_3x3, Z_3x1));

  // Add factor between x0 and x1
  hfg.add(JacobianFactor(X(0), I_3x3, X(1), -I_3x3, Z_3x1));

  // Add a hybrid gaussian factor ϕ(x1, c1)
  hfg.add(HybridGaussianFactor(m1, two::components(X(1))));

  auto result = hfg.eliminateSequential();

  auto dc = result->at(2)->asDiscrete();
  CHECK(dc);
  DiscreteValues dv;
  dv[M(1)] = 0;
  // Regression test
  EXPECT_DOUBLES_EQUAL(0.62245933120185448, dc->operator()(dv), 1e-3);
}

/* ************************************************************************* */
TEST(HybridGaussianFactorGraph, eliminateFullSequentialSimple) {
  HybridGaussianFactorGraph hfg;

  // Add prior on x0
  hfg.add(JacobianFactor(X(0), I_3x3, Z_3x1));
  // Add factor between x0 and x1
  hfg.add(JacobianFactor(X(0), I_3x3, X(1), -I_3x3, Z_3x1));

  hfg.add(HybridGaussianFactor(m1, two::components(X(1))));

  // Discrete probability table for c1
  hfg.add(DecisionTreeFactor(m1, {2, 8}));
  // Joint discrete probability table for c1, c2
  hfg.add(DecisionTreeFactor({m1, m2}, "1 2 3 4"));

  HybridBayesNet::shared_ptr result = hfg.eliminateSequential();

  // There are 4 variables (2 continuous + 2 discrete) in the bayes net.
  EXPECT_LONGS_EQUAL(4, result->size());
}

/* ************************************************************************* */
// Test API for the smallest switching network.
// None of these are regression tests.
TEST(HybridBayesNet, Switching) {
  const double betweenSigma = 0.3, priorSigma = 0.1;
  Switching s(2, betweenSigma, priorSigma);
  const HybridGaussianFactorGraph& graph = s.linearizedFactorGraph;
  EXPECT_LONGS_EQUAL(4, graph.size());

  // Create some continuous and discrete values
  VectorValues continuousValues{{X(0), Vector1(0.1)}, {X(1), Vector1(1.2)}};
  DiscreteValues modeZero{{M(0), 0}}, modeOne{{M(0), 1}};

  // Get the hybrid gaussian factor and check it is as expected
  auto hgf = std::dynamic_pointer_cast<HybridGaussianFactor>(graph.at(1));
  CHECK(hgf);

  // Get factors and scalars for both modes
  auto [factor0, scalar0] = (*hgf)(modeZero);
  auto [factor1, scalar1] = (*hgf)(modeOne);
  CHECK(factor0);
  CHECK(factor1);

  // Check scalars against negLogConstant of noise model
  auto betweenModel = noiseModel::Isotropic::Sigma(1, betweenSigma);
  EXPECT_DOUBLES_EQUAL(betweenModel->negLogConstant(), scalar0, 1e-9);
  EXPECT_DOUBLES_EQUAL(betweenModel->negLogConstant(), scalar1, 1e-9);

  // Check error for M(0) = 0
  HybridValues values0{continuousValues, modeZero};
  double expectedError0 = 0;
  for (const auto& factor : graph) expectedError0 += factor->error(values0);
  EXPECT_DOUBLES_EQUAL(expectedError0, graph.error(values0), 1e-5);

  // Check error for M(0) = 1
  HybridValues values1{continuousValues, modeOne};
  double expectedError1 = 0;
  for (const auto& factor : graph) expectedError1 += factor->error(values1);
  EXPECT_DOUBLES_EQUAL(expectedError1, graph.error(values1), 1e-5);

  // Check errorTree
  AlgebraicDecisionTree<Key> actualErrors = graph.errorTree(continuousValues);
  // Create expected error tree
  AlgebraicDecisionTree<Key> expectedErrors(M(0), expectedError0, expectedError1);

  // Check that the actual error tree matches the expected one
  EXPECT(assert_equal(expectedErrors, actualErrors, 1e-5));

  // Check probPrime
  double probPrime0 = graph.probPrime(values0);
  EXPECT_DOUBLES_EQUAL(std::exp(-expectedError0), probPrime0, 1e-5);

  double probPrime1 = graph.probPrime(values1);
  EXPECT_DOUBLES_EQUAL(std::exp(-expectedError1), probPrime1, 1e-5);

  // Check discretePosterior
  AlgebraicDecisionTree<Key> graphPosterior = graph.discretePosterior(continuousValues);
  double sum = probPrime0 + probPrime1;
  AlgebraicDecisionTree<Key> expectedPosterior(M(0), probPrime0 / sum, probPrime1 / sum);
  EXPECT(assert_equal(expectedPosterior, graphPosterior, 1e-5));

  // Make the clique of factors connected to x0:
  HybridGaussianFactorGraph factors_x0;
  factors_x0.push_back(graph.at(0));
  factors_x0.push_back(hgf);

  // Test collectProductFactor
  auto productFactor = factors_x0.collectProductFactor();

  // For M(0) = 0
  auto [gaussianFactor0, actualScalar0] = productFactor(modeZero);
  EXPECT(gaussianFactor0.size() == 2);
  EXPECT_DOUBLES_EQUAL((*hgf)(modeZero).second, actualScalar0, 1e-5);

  // For M(0) = 1
  auto [gaussianFactor1, actualScalar1] = productFactor(modeOne);
  EXPECT(gaussianFactor1.size() == 2);
  EXPECT_DOUBLES_EQUAL((*hgf)(modeOne).second, actualScalar1, 1e-5);

  // Test eliminate x0
  Ordering ordering{X(0)};
  auto [conditional, factor] = factors_x0.eliminate(ordering);

  // Check the conditional
  CHECK(conditional);
  EXPECT(conditional->isHybrid());
  auto p_x0_given_x1_m = conditional->asHybrid();
  CHECK(p_x0_given_x1_m);
  EXPECT_LONGS_EQUAL(1, p_x0_given_x1_m->nrFrontals());  // x0
  EXPECT_LONGS_EQUAL(2, p_x0_given_x1_m->nrParents());   // x1, m0

  // Check the remaining factor
  EXPECT(factor);
  EXPECT(std::dynamic_pointer_cast<HybridGaussianFactor>(factor));
  auto phi_x1_m = std::dynamic_pointer_cast<HybridGaussianFactor>(factor);
  EXPECT_LONGS_EQUAL(2, phi_x1_m->keys().size());  // x1, m0
  // Check that the scalars incorporate the negative log constant of the conditional
  EXPECT_DOUBLES_EQUAL(
      scalar0 - (*p_x0_given_x1_m)(modeZero)->negLogConstant(), (*phi_x1_m)(modeZero).second, 1e-9);
  EXPECT_DOUBLES_EQUAL(
      scalar1 - (*p_x0_given_x1_m)(modeOne)->negLogConstant(), (*phi_x1_m)(modeOne).second, 1e-9);

  // Check that the conditional and remaining factor are consistent for both modes
  for (auto&& mode : {modeZero, modeOne}) {
    auto gc = (*p_x0_given_x1_m)(mode);
    auto [gf, scalar] = (*phi_x1_m)(mode);

    // The error of the original factors should equal the sum of errors of the conditional and
    // remaining factor, modulo the normalization constant of the conditional.
    double originalError = factors_x0.error({continuousValues, mode});
    const double actualError =
        gc->negLogConstant() + gc->error(continuousValues) + gf->error(continuousValues) + scalar;
    EXPECT_DOUBLES_EQUAL(originalError, actualError, 1e-9);
  }

  // Create a clique for x1
  HybridGaussianFactorGraph factors_x1;
  factors_x1.push_back(factor);       // Use the remaining factor from previous elimination
  factors_x1.push_back(graph.at(2));  // Add the factor for x1 from the original graph

  // Test collectProductFactor for x1 clique
  auto productFactor_x1 = factors_x1.collectProductFactor();

  // For M(0) = 0
  auto [gaussianFactor_x1_0, actualScalar_x1_0] = productFactor_x1(modeZero);
  EXPECT_LONGS_EQUAL(2, gaussianFactor_x1_0.size());
  // NOTE(Frank): prior on x1 does not contribute to the scalar
  EXPECT_DOUBLES_EQUAL((*phi_x1_m)(modeZero).second, actualScalar_x1_0, 1e-5);

  // For M(0) = 1
  auto [gaussianFactor_x1_1, actualScalar_x1_1] = productFactor_x1(modeOne);
  EXPECT_LONGS_EQUAL(2, gaussianFactor_x1_1.size());
  // NOTE(Frank): prior on x1 does not contribute to the scalar
  EXPECT_DOUBLES_EQUAL((*phi_x1_m)(modeOne).second, actualScalar_x1_1, 1e-5);

  // Test eliminate for x1 clique
  Ordering ordering_x1{X(1)};
  auto [conditional_x1, factor_x1] = factors_x1.eliminate(ordering_x1);

  // Check the conditional for x1
  CHECK(conditional_x1);
  EXPECT(conditional_x1->isHybrid());
  auto p_x1_given_m = conditional_x1->asHybrid();
  CHECK(p_x1_given_m);
  EXPECT_LONGS_EQUAL(1, p_x1_given_m->nrFrontals());  // x1
  EXPECT_LONGS_EQUAL(1, p_x1_given_m->nrParents());   // m0

  // Check the remaining factor for x1
  CHECK(factor_x1);
  auto phi_x1 = std::dynamic_pointer_cast<DecisionTreeFactor>(factor_x1);
  CHECK(phi_x1);
  EXPECT_LONGS_EQUAL(1, phi_x1->keys().size());  // m0
  // We can't really check the error of the decision tree factor phi_x1, because the continuos
  // factor whose error(kEmpty) we need is not available..

  // However, we can still check the total error for the clique factors_x1 and the elimination
  // results are equal, modulo -again- the negative log constant of the conditional.
  for (auto&& mode : {modeZero, modeOne}) {
    auto gc_x1 = (*p_x1_given_m)(mode);
    double originalError_x1 = factors_x1.error({continuousValues, mode});
    const double actualError =
        gc_x1->negLogConstant() + gc_x1->error(continuousValues) + phi_x1->error(mode);
    EXPECT_DOUBLES_EQUAL(originalError_x1, actualError, 1e-9);
  }

  // Now test full elimination of the graph:
  auto posterior = graph.eliminateSequential();
  CHECK(posterior);

  // Check that the posterior P(M|X=continuousValues) from the Bayes net is the same as the
  // same posterior from the graph. This is a sanity check that the elimination is done correctly.
  AlgebraicDecisionTree<Key> bnPosterior = graph.discretePosterior(continuousValues);
  EXPECT(assert_equal(graphPosterior, bnPosterior));
}

/* ************************************************************************* */
// Select a particular continuous factor graph given a discrete assignment
TEST(HybridGaussianFactorGraph, DiscreteSelection) {
  Switching s(3);

  HybridGaussianFactorGraph graph = s.linearizedFactorGraph;

  DiscreteValues dv00{{M(0), 0}, {M(1), 0}};
  GaussianFactorGraph continuous_00 = graph(dv00);
  GaussianFactorGraph expected_00;
  expected_00.push_back(JacobianFactor(X(0), I_1x1 * 10, Vector1(-10)));
  expected_00.push_back(JacobianFactor(X(0), -I_1x1, X(1), I_1x1, Vector1(-1)));
  expected_00.push_back(JacobianFactor(X(1), -I_1x1, X(2), I_1x1, Vector1(-1)));
  expected_00.push_back(JacobianFactor(X(1), I_1x1 * 10, Vector1(-10)));
  expected_00.push_back(JacobianFactor(X(2), I_1x1 * 10, Vector1(-10)));

  EXPECT(assert_equal(expected_00, continuous_00));

  DiscreteValues dv01{{M(0), 0}, {M(1), 1}};
  GaussianFactorGraph continuous_01 = graph(dv01);
  GaussianFactorGraph expected_01;
  expected_01.push_back(JacobianFactor(X(0), I_1x1 * 10, Vector1(-10)));
  expected_01.push_back(JacobianFactor(X(0), -I_1x1, X(1), I_1x1, Vector1(-1)));
  expected_01.push_back(JacobianFactor(X(1), -I_1x1, X(2), I_1x1, Vector1(-0)));
  expected_01.push_back(JacobianFactor(X(1), I_1x1 * 10, Vector1(-10)));
  expected_01.push_back(JacobianFactor(X(2), I_1x1 * 10, Vector1(-10)));

  EXPECT(assert_equal(expected_01, continuous_01));

  DiscreteValues dv10{{M(0), 1}, {M(1), 0}};
  GaussianFactorGraph continuous_10 = graph(dv10);
  GaussianFactorGraph expected_10;
  expected_10.push_back(JacobianFactor(X(0), I_1x1 * 10, Vector1(-10)));
  expected_10.push_back(JacobianFactor(X(0), -I_1x1, X(1), I_1x1, Vector1(-0)));
  expected_10.push_back(JacobianFactor(X(1), -I_1x1, X(2), I_1x1, Vector1(-1)));
  expected_10.push_back(JacobianFactor(X(1), I_1x1 * 10, Vector1(-10)));
  expected_10.push_back(JacobianFactor(X(2), I_1x1 * 10, Vector1(-10)));

  EXPECT(assert_equal(expected_10, continuous_10));

  DiscreteValues dv11{{M(0), 1}, {M(1), 1}};
  GaussianFactorGraph continuous_11 = graph(dv11);
  GaussianFactorGraph expected_11;
  expected_11.push_back(JacobianFactor(X(0), I_1x1 * 10, Vector1(-10)));
  expected_11.push_back(JacobianFactor(X(0), -I_1x1, X(1), I_1x1, Vector1(-0)));
  expected_11.push_back(JacobianFactor(X(1), -I_1x1, X(2), I_1x1, Vector1(-0)));
  expected_11.push_back(JacobianFactor(X(1), I_1x1 * 10, Vector1(-10)));
  expected_11.push_back(JacobianFactor(X(2), I_1x1 * 10, Vector1(-10)));

  EXPECT(assert_equal(expected_11, continuous_11));
}

/* ************************************************************************* */
TEST(HybridGaussianFactorGraph, optimize) {
  HybridGaussianFactorGraph hfg;

  hfg.add(JacobianFactor(X(0), I_3x3, Z_3x1));
  hfg.add(JacobianFactor(X(0), I_3x3, X(1), -I_3x3, Z_3x1));
  hfg.add(HybridGaussianFactor(m1, two::components(X(1))));

  auto result = hfg.eliminateSequential();

  HybridValues hv = result->optimize();

  EXPECT(assert_equal(hv.atDiscrete(M(1)), int(0)));
}

/* ************************************************************************* */
// Test adding of gaussian conditional and re-elimination.
TEST(HybridGaussianFactorGraph, Conditionals) {
  Switching switching(4);

  HybridGaussianFactorGraph hfg;
  hfg.push_back(switching.linearizedFactorGraph.at(0));  // P(X0)
  Ordering ordering;
  ordering.push_back(X(0));
  HybridBayesNet::shared_ptr bayes_net = hfg.eliminateSequential(ordering);

  HybridGaussianFactorGraph hfg2;
  hfg2.push_back(*bayes_net);                             // P(X0)
  hfg2.push_back(switching.linearizedFactorGraph.at(1));  // P(X0, X1 | M0)
  hfg2.push_back(switching.linearizedFactorGraph.at(2));  // P(X1, X2 | M1)
  hfg2.push_back(switching.linearizedFactorGraph.at(5));  // P(M1)
  ordering += X(1), X(2), M(0), M(1);

  // Created product of first two factors and check eliminate:
  HybridGaussianFactorGraph fragment;
  fragment.push_back(hfg2[0]);
  fragment.push_back(hfg2[1]);

  // Check that product
  HybridGaussianProductFactor product = fragment.collectProductFactor();
  auto leaf = fragment(DiscreteValues{{M(0), 0}});
  EXPECT_LONGS_EQUAL(2, leaf.size());

  // Check product and that pruneEmpty does not touch it
  auto pruned = product.removeEmpty();
  LONGS_EQUAL(2, pruned.nrLeaves());

  // Test eliminate
  auto [hybridConditional, factor] = fragment.eliminate({X(0)});
  EXPECT(hybridConditional->isHybrid());
  EXPECT(hybridConditional->keys() == KeyVector({X(0), X(1), M(0)}));

  EXPECT(dynamic_pointer_cast<HybridGaussianFactor>(factor));
  EXPECT(factor->keys() == KeyVector({X(1), M(0)}));

  bayes_net = hfg2.eliminateSequential(ordering);

  HybridValues result = bayes_net->optimize();

  Values expected_continuous;
  expected_continuous.insert<double>(X(0), 0);
  expected_continuous.insert<double>(X(1), 1);
  expected_continuous.insert<double>(X(2), 2);
  expected_continuous.insert<double>(X(3), 4);
  Values result_continuous = switching.linearizationPoint.retract(result.continuous());
  EXPECT(assert_equal(expected_continuous, result_continuous));

  DiscreteValues expected_discrete;
  expected_discrete[M(0)] = 1;
  expected_discrete[M(1)] = 1;
  EXPECT(assert_equal(expected_discrete, result.discrete()));
}

/* ****************************************************************************/
// Test hybrid gaussian factor graph error and unnormalized probabilities
TEST(HybridGaussianFactorGraph, ErrorAndProbPrime) {
  Switching s(3);

  HybridGaussianFactorGraph graph = s.linearizedFactorGraph;

  HybridBayesNet::shared_ptr hybridBayesNet = graph.eliminateSequential();

  const HybridValues delta = hybridBayesNet->optimize();
  const double error = graph.error(delta);

  // regression
  EXPECT(assert_equal(1.58886, error, 1e-5));

  // Real test:
  EXPECT(assert_equal(graph.probPrime(delta), exp(-error), 1e-7));
}

/* ****************************************************************************/
// Test hybrid gaussian factor graph error and unnormalized probabilities
TEST(HybridGaussianFactorGraph, ErrorAndProbPrimeTree) {
  // Create switching network with three continuous variables and two discrete:
  // ϕ(x0) ϕ(x0,x1,m0) ϕ(x1,x2,m1) ϕ(x0;z0) ϕ(x1;z1) ϕ(x2;z2) ϕ(m0) ϕ(m0,m1)
  Switching s(3);

  const HybridGaussianFactorGraph& graph = s.linearizedFactorGraph;

  const HybridBayesNet::shared_ptr hybridBayesNet = graph.eliminateSequential();

  const HybridValues delta = hybridBayesNet->optimize();

  // regression test for errorTree
  std::vector<double> leaves = {2.7916153, 1.5888555, 1.7233422, 1.6191947};
  AlgebraicDecisionTree<Key> expectedErrors(s.modes, leaves);
  const auto error_tree = graph.errorTree(delta.continuous());
  EXPECT(assert_equal(expectedErrors, error_tree, 1e-7));

  // regression test for discretePosterior
  const AlgebraicDecisionTree<Key> expectedPosterior(
      s.modes, std::vector{0.095516068, 0.31800092, 0.27798511, 0.3084979});
  auto posterior = graph.discretePosterior(delta.continuous());
  EXPECT(assert_equal(expectedPosterior, posterior, 1e-7));
}

/* ****************************************************************************/
// Test hybrid gaussian factor graph errorTree during incremental operation
TEST(HybridGaussianFactorGraph, IncrementalErrorTree) {
  Switching s(4);

  HybridGaussianFactorGraph graph;
  graph.push_back(s.linearizedFactorGraph.at(0));  // f(X0)
  graph.push_back(s.linearizedFactorGraph.at(1));  // f(X0, X1, M0)
  graph.push_back(s.linearizedFactorGraph.at(2));  // f(X1, X2, M1)
  graph.push_back(s.linearizedFactorGraph.at(4));  // f(X1)
  graph.push_back(s.linearizedFactorGraph.at(5));  // f(X2)
  graph.push_back(s.linearizedFactorGraph.at(7));  // f(M0)
  graph.push_back(s.linearizedFactorGraph.at(8));  // f(M0, M1)

  HybridBayesNet::shared_ptr hybridBayesNet = graph.eliminateSequential();
  EXPECT_LONGS_EQUAL(5, hybridBayesNet->size());

  HybridValues delta = hybridBayesNet->optimize();
  auto error_tree = graph.errorTree(delta.continuous());

  std::vector<DiscreteKey> discrete_keys = {m0, m1};
  std::vector<double> leaves = {2.7916153, 1.5888555, 1.7233422, 1.6191947};
  AlgebraicDecisionTree<Key> expected_error(discrete_keys, leaves);

  // regression
  EXPECT(assert_equal(expected_error, error_tree, 1e-7));

  graph = HybridGaussianFactorGraph();
  graph.push_back(*hybridBayesNet);
  graph.push_back(s.linearizedFactorGraph.at(3));  // f(X2, X3, M2)
  graph.push_back(s.linearizedFactorGraph.at(6));  // f(X3)

  hybridBayesNet = graph.eliminateSequential();
  EXPECT_LONGS_EQUAL(7, hybridBayesNet->size());

  delta = hybridBayesNet->optimize();
  auto error_tree2 = graph.errorTree(delta.continuous());

  // regression
  leaves = {
      0.50985198, 0.0097577296, 0.50009425, 0, 0.52922138, 0.029127133, 0.50985105, 0.0097567964};
  AlgebraicDecisionTree<Key> expected_error2(s.modes, leaves);
  EXPECT(assert_equal(expected_error, error_tree, 1e-7));
}

/* ****************************************************************************/
// Check that collectProductFactor works correctly.
TEST(HybridGaussianFactorGraph, collectProductFactor) {
  const int num_measurements = 1;
  VectorValues vv{{Z(0), Vector1(5.0)}};
  auto fg = tiny::createHybridGaussianFactorGraph(num_measurements, vv);
  EXPECT_LONGS_EQUAL(3, fg.size());

  // Assemble graph tree:
  auto actual = fg.collectProductFactor();

  // Create expected decision tree with two factor graphs:

  // Get hybrid factor:
  auto hybrid = fg.at<HybridGaussianFactor>(0);
  CHECK(hybrid);

  // Get prior factor:
  const auto gf = fg.at<HybridConditional>(1);
  CHECK(gf);
  using GF = GaussianFactor::shared_ptr;
  const GF prior = gf->asGaussian();
  CHECK(prior);

  // Create DiscreteValues for both 0 and 1:
  DiscreteValues d0{{M(0), 0}}, d1{{M(0), 1}};

  // Expected decision tree with two factor graphs:
  // f(x0;mode=0)P(x0)
  GaussianFactorGraph expectedFG0{(*hybrid)(d0).first, prior};
  EXPECT(assert_equal(expectedFG0, actual(d0).first, 1e-5));
  EXPECT(assert_equal(0.0, actual(d0).second, 1e-5));

  // f(x0;mode=1)P(x0)
  GaussianFactorGraph expectedFG1{(*hybrid)(d1).first, prior};
  EXPECT(assert_equal(expectedFG1, actual(d1).first, 1e-5));
  EXPECT(assert_equal(1.79176, actual(d1).second, 1e-5));
}

/* ****************************************************************************/
// Check that the factor graph unnormalized probability is proportional to the
// Bayes net probability for the given measurements.
bool ratioTest(const HybridBayesNet& bn,
               const VectorValues& measurements,
               const HybridGaussianFactorGraph& fg,
               size_t num_samples = 100) {
  auto compute_ratio = [&](HybridValues* sample) -> double {
    sample->update(measurements);  // update sample with given measurements:
    return bn.evaluate(*sample) / fg.probPrime(*sample);
  };

  HybridValues sample = bn.sample(&kRng);
  double expected_ratio = compute_ratio(&sample);

  // Test ratios for a number of independent samples:
  for (size_t i = 0; i < num_samples; i++) {
    HybridValues sample = bn.sample(&kRng);
    if (std::abs(expected_ratio - compute_ratio(&sample)) > 1e-6) return false;
  }
  return true;
}

/* ****************************************************************************/
// Check that the bayes net unnormalized probability is proportional to the
// Bayes net probability for the given measurements.
bool ratioTest(const HybridBayesNet& bn,
               const VectorValues& measurements,
               const HybridBayesNet& posterior,
               size_t num_samples = 100) {
  auto compute_ratio = [&](HybridValues* sample) -> double {
    sample->update(measurements);  // update sample with given measurements:
    return bn.evaluate(*sample) / posterior.evaluate(*sample);
  };

  HybridValues sample = bn.sample(&kRng);
  double expected_ratio = compute_ratio(&sample);

  // Test ratios for a number of independent samples:
  for (size_t i = 0; i < num_samples; i++) {
    HybridValues sample = bn.sample(&kRng);
    // std::cout << "ratio: " << compute_ratio(&sample) << std::endl;
    if (std::abs(expected_ratio - compute_ratio(&sample)) > 1e-6) return false;
  }
  return true;
}

/* ****************************************************************************/
// Check that eliminating tiny net with 1 measurement yields correct result.
TEST(HybridGaussianFactorGraph, EliminateTiny1) {
  const int num_measurements = 1;
  const VectorValues measurements{{Z(0), Vector1(5.0)}};
  auto bn = tiny::createHybridBayesNet(num_measurements);
  auto fg = bn.toFactorGraph(measurements);
  EXPECT_LONGS_EQUAL(3, fg.size());

  EXPECT(ratioTest(bn, measurements, fg));

  // Create expected Bayes Net:
  HybridBayesNet expectedBayesNet;

  // Create hybrid Gaussian factor on X(0).
  using tiny::mode;
  // regression, but mean checked to be 5.0 in both cases:
  const auto conditional0 =
                 std::make_shared<GaussianConditional>(X(0), Vector1(14.1421), I_1x1 * 2.82843),
             conditional1 =
                 std::make_shared<GaussianConditional>(X(0), Vector1(10.1379), I_1x1 * 2.02759);
  expectedBayesNet.emplace_shared<HybridGaussianConditional>(
      mode, std::vector{conditional0, conditional1});

  // Add prior on mode.
  expectedBayesNet.emplace_shared<DiscreteConditional>(mode, "74/26");

  // Test elimination
  const auto posterior = fg.eliminateSequential();
  EXPECT(assert_equal(expectedBayesNet, *posterior, 0.01));

  EXPECT(ratioTest(bn, measurements, *posterior));
}

/* ****************************************************************************/
// Check that eliminating tiny net with 1 measurement with mode order swapped
// yields correct result.
TEST(HybridGaussianFactorGraph, EliminateTiny1Swapped) {
  const VectorValues measurements{{Z(0), Vector1(5.0)}};

  HybridBayesNet bn;

  // mode-dependent: 1 is low-noise, 0 is high-noise.
  // Create hybrid Gaussian factor z_0 = x0 + noise for each measurement.
  std::vector<std::pair<Vector, double>> parms{{Z_1x1, 3}, {Z_1x1, 0.5}};
  bn.emplace_shared<HybridGaussianConditional>(m1, Z(0), I_1x1, X(0), parms);

  // Create prior on X(0).
  bn.push_back(GaussianConditional::sharedMeanAndStddev(X(0), Vector1(5.0), 0.5));

  // Add prior on m1.
  bn.emplace_shared<DiscreteConditional>(m1, "1/1");

  // bn.print();
  auto fg = bn.toFactorGraph(measurements);
  EXPECT_LONGS_EQUAL(3, fg.size());

  // fg.print();

  EXPECT(ratioTest(bn, measurements, fg));

  // Create expected Bayes Net:
  HybridBayesNet expectedBayesNet;

  // Create hybrid Gaussian factor on X(0).
  // regression, but mean checked to be 5.0 in both cases:
  const auto conditional0 =
                 std::make_shared<GaussianConditional>(X(0), Vector1(10.1379), I_1x1 * 2.02759),
             conditional1 =
                 std::make_shared<GaussianConditional>(X(0), Vector1(14.1421), I_1x1 * 2.82843);
  expectedBayesNet.emplace_shared<HybridGaussianConditional>(
      m1, std::vector{conditional0, conditional1});

  // Add prior on m1.
  expectedBayesNet.emplace_shared<DiscreteConditional>(m1, "1/1");

  // Test elimination
  const auto posterior = fg.eliminateSequential();
  // EXPECT(assert_equal(expectedBayesNet, *posterior, 0.01));

  EXPECT(ratioTest(bn, measurements, *posterior));

  // posterior->print();
  // posterior->optimize().print();
}

/* ****************************************************************************/
// Check that eliminating tiny net with 2 measurements yields correct result.
TEST(HybridGaussianFactorGraph, EliminateTiny2) {
  // Create factor graph with 2 measurements such that posterior mean = 5.0.
  const int num_measurements = 2;
  const VectorValues measurements{{Z(0), Vector1(4.0)}, {Z(1), Vector1(6.0)}};
  auto bn = tiny::createHybridBayesNet(num_measurements);
  auto fg = bn.toFactorGraph(measurements);
  EXPECT_LONGS_EQUAL(4, fg.size());

  // Create expected Bayes Net:
  HybridBayesNet expectedBayesNet;

  // Create hybrid Gaussian factor on X(0).
  using tiny::mode;
  // regression, but mean checked to be 5.0 in both cases:
  const auto conditional0 =
                 std::make_shared<GaussianConditional>(X(0), Vector1(17.3205), I_1x1 * 3.4641),
             conditional1 =
                 std::make_shared<GaussianConditional>(X(0), Vector1(10.274), I_1x1 * 2.0548);
  expectedBayesNet.emplace_shared<HybridGaussianConditional>(
      mode, std::vector{conditional0, conditional1});

  // Add prior on mode.
  expectedBayesNet.emplace_shared<DiscreteConditional>(mode, "23/77");

  // Test elimination
  const auto posterior = fg.eliminateSequential();
  EXPECT(assert_equal(expectedBayesNet, *posterior, 0.01));

  EXPECT(ratioTest(bn, measurements, *posterior));
}

/* ****************************************************************************/
// Test eliminating tiny net with 1 mode per measurement.
TEST(HybridGaussianFactorGraph, EliminateTiny22) {
  // Create factor graph with 2 measurements such that posterior mean = 5.0.
  const int num_measurements = 2;
  const bool manyModes = true;

  // Create Bayes net and convert to factor graph.
  auto bn = tiny::createHybridBayesNet(num_measurements, manyModes);
  const VectorValues measurements{{Z(0), Vector1(4.0)}, {Z(1), Vector1(6.0)}};
  auto fg = bn.toFactorGraph(measurements);
  EXPECT_LONGS_EQUAL(5, fg.size());

  EXPECT(ratioTest(bn, measurements, fg));

  // Test elimination
  const auto posterior = fg.eliminateSequential();

  EXPECT(ratioTest(bn, measurements, *posterior));
}

/* ****************************************************************************/
// Test elimination of a switching network with one mode per measurement.
TEST(HybridGaussianFactorGraph, EliminateSwitchingNetwork) {
  // Create a switching network with one mode per measurement.
  HybridBayesNet bn;

  // NOTE: we add reverse topological so we can sample from the Bayes net.:

  // Add measurements:
  std::vector<std::pair<Vector, double>> measurementModels{{Z_1x1, 3}, {Z_1x1, 0.5}};
  for (size_t t : {0, 1, 2}) {
    // Create hybrid Gaussian factor on Z(t) conditioned on X(t) and mode N(t):
    const auto noise_mode_t = DiscreteKey{N(t), 2};
    bn.emplace_shared<HybridGaussianConditional>(
        noise_mode_t, Z(t), I_1x1, X(t), measurementModels);

    // Create prior on discrete mode N(t):
    bn.emplace_shared<DiscreteConditional>(noise_mode_t, "20/80");
  }

  // Add motion models. TODO(frank): why are they exactly the same?
  std::vector<std::pair<Vector, double>> motionModels{{Z_1x1, 0.2}, {Z_1x1, 0.2}};
  for (size_t t : {2, 1}) {
    // Create hybrid Gaussian factor on X(t) conditioned on X(t-1)
    // and mode M(t-1):
    const auto motion_model_t = DiscreteKey{M(t), 2};
    bn.emplace_shared<HybridGaussianConditional>(
        motion_model_t, X(t), I_1x1, X(t - 1), motionModels);

    // Create prior on motion model M(t):
    bn.emplace_shared<DiscreteConditional>(motion_model_t, "40/60");
  }

  // Create Gaussian prior on continuous X(0) using sharedMeanAndStddev:
  bn.push_back(GaussianConditional::sharedMeanAndStddev(X(0), Z_1x1, 0.1));

  // Make sure we an sample from the Bayes net:
  EXPECT_LONGS_EQUAL(6, bn.sample().continuous().size());

  // Create measurements consistent with moving right every time:
  const VectorValues measurements{{Z(0), Vector1(0.0)}, {Z(1), Vector1(1.0)}, {Z(2), Vector1(2.0)}};
  const HybridGaussianFactorGraph fg = bn.toFactorGraph(measurements);

  // Factor graph is:
  //      D     D
  //      |     |
  //      m1    m2
  //      |     |
  // C-x0-HC-x1-HC-x2
  //   |     |     |
  //   HF    HF    HF
  //   |     |     |
  //   n0    n1    n2
  //   |     |     |
  //   D     D     D
  EXPECT_LONGS_EQUAL(11, fg.size());
  EXPECT(ratioTest(bn, measurements, fg));

  // Do elimination of X(2) only:
  auto [bn1, fg1] = fg.eliminatePartialSequential(Ordering{X(2)});
  fg1->push_back(*bn1);
  EXPECT(ratioTest(bn, measurements, *fg1));

  // Create ordering that eliminates in time order, then discrete modes:
  Ordering ordering{X(2), X(1), X(0), N(0), N(1), N(2), M(1), M(2)};

  // Do elimination:
  const HybridBayesNet::shared_ptr posterior = fg.eliminateSequential(ordering);

  // Test resulting posterior Bayes net has correct size:
  EXPECT_LONGS_EQUAL(8, posterior->size());

  // Ratio test
  EXPECT(ratioTest(bn, measurements, *posterior));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

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
 */

#include <CppUnitLite/Test.h>
#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/discrete/DecisionTreeFactor.h>
#include <gtsam/discrete/DiscreteKey.h>
#include <gtsam/discrete/DiscreteValues.h>
#include <gtsam/hybrid/GaussianMixture.h>
#include <gtsam/hybrid/GaussianMixtureFactor.h>
#include <gtsam/hybrid/HybridBayesNet.h>
#include <gtsam/hybrid/HybridBayesTree.h>
#include <gtsam/hybrid/HybridConditional.h>
#include <gtsam/hybrid/HybridFactor.h>
#include <gtsam/hybrid/HybridGaussianFactorGraph.h>
#include <gtsam/hybrid/HybridGaussianISAM.h>
#include <gtsam/hybrid/HybridValues.h>
#include <gtsam/inference/BayesNet.h>
#include <gtsam/inference/DotWriter.h>
#include <gtsam/inference/Key.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/JacobianFactor.h>

#include <algorithm>
#include <cstddef>
#include <functional>
#include <iostream>
#include <iterator>
#include <vector>

#include "Switching.h"
#include "TinyHybridExample.h"

using namespace std;
using namespace gtsam;

using gtsam::symbol_shorthand::D;
using gtsam::symbol_shorthand::M;
using gtsam::symbol_shorthand::N;
using gtsam::symbol_shorthand::X;
using gtsam::symbol_shorthand::Y;
using gtsam::symbol_shorthand::Z;

// Set up sampling
std::mt19937_64 kRng(42);

/* ************************************************************************* */
TEST(HybridGaussianFactorGraph, Creation) {
  HybridConditional conditional;

  HybridGaussianFactorGraph hfg;

  hfg.emplace_shared<JacobianFactor>(X(0), I_3x3, Z_3x1);

  // Define a gaussian mixture conditional P(x0|x1, c0) and add it to the factor
  // graph
  GaussianMixture gm({X(0)}, {X(1)}, DiscreteKeys(DiscreteKey{M(0), 2}),
                     GaussianMixture::Conditionals(
                         M(0),
                         boost::make_shared<GaussianConditional>(
                             X(0), Z_3x1, I_3x3, X(1), I_3x3),
                         boost::make_shared<GaussianConditional>(
                             X(0), Vector3::Ones(), I_3x3, X(1), I_3x3)));
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
TEST(HybridGaussianFactorGraph, EliminateMultifrontal) {
  // Test multifrontal elimination
  HybridGaussianFactorGraph hfg;

  DiscreteKey m(M(1), 2);

  // Add priors on x0 and c1
  hfg.add(JacobianFactor(X(0), I_3x3, Z_3x1));
  hfg.add(DecisionTreeFactor(m, {2, 8}));

  Ordering ordering;
  ordering.push_back(X(0));
  auto result = hfg.eliminatePartialMultifrontal(ordering);

  EXPECT_LONGS_EQUAL(result.first->size(), 1);
  EXPECT_LONGS_EQUAL(result.second->size(), 1);
}

/* ************************************************************************* */
TEST(HybridGaussianFactorGraph, eliminateFullSequentialEqualChance) {
  HybridGaussianFactorGraph hfg;

  // Add prior on x0
  hfg.add(JacobianFactor(X(0), I_3x3, Z_3x1));

  // Add factor between x0 and x1
  hfg.add(JacobianFactor(X(0), I_3x3, X(1), -I_3x3, Z_3x1));

  // Add a gaussian mixture factor ϕ(x1, c1)
  DiscreteKey m1(M(1), 2);
  DecisionTree<Key, GaussianFactor::shared_ptr> dt(
      M(1), boost::make_shared<JacobianFactor>(X(1), I_3x3, Z_3x1),
      boost::make_shared<JacobianFactor>(X(1), I_3x3, Vector3::Ones()));
  hfg.add(GaussianMixtureFactor({X(1)}, {m1}, dt));

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

  DiscreteKey m1(M(1), 2);

  // Add prior on x0
  hfg.add(JacobianFactor(X(0), I_3x3, Z_3x1));
  // Add factor between x0 and x1
  hfg.add(JacobianFactor(X(0), I_3x3, X(1), -I_3x3, Z_3x1));

  std::vector<GaussianFactor::shared_ptr> factors = {
      boost::make_shared<JacobianFactor>(X(1), I_3x3, Z_3x1),
      boost::make_shared<JacobianFactor>(X(1), I_3x3, Vector3::Ones())};
  hfg.add(GaussianMixtureFactor({X(1)}, {m1}, factors));

  // Discrete probability table for c1
  hfg.add(DecisionTreeFactor(m1, {2, 8}));
  // Joint discrete probability table for c1, c2
  hfg.add(DecisionTreeFactor({{M(1), 2}, {M(2), 2}}, "1 2 3 4"));

  HybridBayesNet::shared_ptr result = hfg.eliminateSequential();

  // There are 4 variables (2 continuous + 2 discrete) in the bayes net.
  EXPECT_LONGS_EQUAL(4, result->size());
}

/* ************************************************************************* */
TEST(HybridGaussianFactorGraph, eliminateFullMultifrontalSimple) {
  HybridGaussianFactorGraph hfg;

  DiscreteKey m1(M(1), 2);

  hfg.add(JacobianFactor(X(0), I_3x3, Z_3x1));
  hfg.add(JacobianFactor(X(0), I_3x3, X(1), -I_3x3, Z_3x1));

  hfg.add(GaussianMixtureFactor(
      {X(1)}, {{M(1), 2}},
      {boost::make_shared<JacobianFactor>(X(1), I_3x3, Z_3x1),
       boost::make_shared<JacobianFactor>(X(1), I_3x3, Vector3::Ones())}));

  hfg.add(DecisionTreeFactor(m1, {2, 8}));
  // TODO(Varun) Adding extra discrete variable not connected to continuous
  // variable throws segfault
  //  hfg.add(DecisionTreeFactor({{M(1), 2}, {M(2), 2}}, "1 2 3 4"));

  HybridBayesTree::shared_ptr result = hfg.eliminateMultifrontal();

  // The bayes tree should have 3 cliques
  EXPECT_LONGS_EQUAL(3, result->size());
  // GTSAM_PRINT(*result);
  // GTSAM_PRINT(*result->marginalFactor(M(2)));
}

/* ************************************************************************* */
TEST(HybridGaussianFactorGraph, eliminateFullMultifrontalCLG) {
  HybridGaussianFactorGraph hfg;

  DiscreteKey m(M(1), 2);

  // Prior on x0
  hfg.add(JacobianFactor(X(0), I_3x3, Z_3x1));
  // Factor between x0-x1
  hfg.add(JacobianFactor(X(0), I_3x3, X(1), -I_3x3, Z_3x1));

  // Decision tree with different modes on x1
  DecisionTree<Key, GaussianFactor::shared_ptr> dt(
      M(1), boost::make_shared<JacobianFactor>(X(1), I_3x3, Z_3x1),
      boost::make_shared<JacobianFactor>(X(1), I_3x3, Vector3::Ones()));

  // Hybrid factor P(x1|c1)
  hfg.add(GaussianMixtureFactor({X(1)}, {m}, dt));
  // Prior factor on c1
  hfg.add(DecisionTreeFactor(m, {2, 8}));

  // Get a constrained ordering keeping c1 last
  auto ordering_full = HybridOrdering(hfg);

  // Returns a Hybrid Bayes Tree with distribution P(x0|x1)P(x1|c1)P(c1)
  HybridBayesTree::shared_ptr hbt = hfg.eliminateMultifrontal(ordering_full);

  EXPECT_LONGS_EQUAL(3, hbt->size());
}

/* ************************************************************************* */
/*
 * This test is about how to assemble the Bayes Tree roots after we do partial
 * elimination
 */
TEST(HybridGaussianFactorGraph, eliminateFullMultifrontalTwoClique) {
  HybridGaussianFactorGraph hfg;

  hfg.add(JacobianFactor(X(0), I_3x3, X(1), -I_3x3, Z_3x1));
  hfg.add(JacobianFactor(X(1), I_3x3, X(2), -I_3x3, Z_3x1));

  {
    hfg.add(GaussianMixtureFactor(
        {X(0)}, {{M(0), 2}},
        {boost::make_shared<JacobianFactor>(X(0), I_3x3, Z_3x1),
         boost::make_shared<JacobianFactor>(X(0), I_3x3, Vector3::Ones())}));

    DecisionTree<Key, GaussianFactor::shared_ptr> dt1(
        M(1), boost::make_shared<JacobianFactor>(X(2), I_3x3, Z_3x1),
        boost::make_shared<JacobianFactor>(X(2), I_3x3, Vector3::Ones()));

    hfg.add(GaussianMixtureFactor({X(2)}, {{M(1), 2}}, dt1));
  }

  hfg.add(DecisionTreeFactor({{M(1), 2}, {M(2), 2}}, "1 2 3 4"));

  hfg.add(JacobianFactor(X(3), I_3x3, X(4), -I_3x3, Z_3x1));
  hfg.add(JacobianFactor(X(4), I_3x3, X(5), -I_3x3, Z_3x1));

  {
    DecisionTree<Key, GaussianFactor::shared_ptr> dt(
        M(3), boost::make_shared<JacobianFactor>(X(3), I_3x3, Z_3x1),
        boost::make_shared<JacobianFactor>(X(3), I_3x3, Vector3::Ones()));

    hfg.add(GaussianMixtureFactor({X(3)}, {{M(3), 2}}, dt));

    DecisionTree<Key, GaussianFactor::shared_ptr> dt1(
        M(2), boost::make_shared<JacobianFactor>(X(5), I_3x3, Z_3x1),
        boost::make_shared<JacobianFactor>(X(5), I_3x3, Vector3::Ones()));

    hfg.add(GaussianMixtureFactor({X(5)}, {{M(2), 2}}, dt1));
  }

  auto ordering_full =
      Ordering::ColamdConstrainedLast(hfg, {M(0), M(1), M(2), M(3)});

  HybridBayesTree::shared_ptr hbt;
  HybridGaussianFactorGraph::shared_ptr remaining;
  std::tie(hbt, remaining) = hfg.eliminatePartialMultifrontal(ordering_full);

  // 9 cliques in the bayes tree and 0 remaining variables to eliminate.
  EXPECT_LONGS_EQUAL(9, hbt->size());
  EXPECT_LONGS_EQUAL(0, remaining->size());

  /*
  (Fan) Explanation: the Junction tree will need to reeliminate to get to the
  marginal on X(1), which is not possible because it involves eliminating
  discrete before continuous. The solution to this, however, is in Murphy02.
  TLDR is that this is 1. expensive and 2. inexact. nevertheless it is doable.
  And I believe that we should do this.
  */
}

void dotPrint(const HybridGaussianFactorGraph::shared_ptr &hfg,
              const HybridBayesTree::shared_ptr &hbt,
              const Ordering &ordering) {
  DotWriter dw;
  dw.positionHints['c'] = 2;
  dw.positionHints['x'] = 1;
  std::cout << hfg->dot(DefaultKeyFormatter, dw);
  std::cout << "\n";
  hbt->dot(std::cout);

  std::cout << "\n";
  std::cout << hfg->eliminateSequential(ordering)->dot(DefaultKeyFormatter, dw);
}

/* ************************************************************************* */
// TODO(fan): make a graph like Varun's paper one
TEST(HybridGaussianFactorGraph, Switching) {
  auto N = 12;
  auto hfg = makeSwitchingChain(N);

  // X(5) will be the center, X(1-4), X(6-9)
  // X(3), X(7)
  // X(2), X(8)
  // X(1), X(4), X(6), X(9)
  // M(5) will be the center, M(1-4), M(6-8)
  // M(3), M(7)
  // M(1), M(4), M(2), M(6), M(8)
  // auto ordering_full =
  //     Ordering(KeyVector{X(1), X(4), X(2), X(6), X(9), X(8), X(3), X(7),
  //     X(5),
  //                        M(1), M(4), M(2), M(6), M(8), M(3), M(7), M(5)});
  KeyVector ordering;

  {
    std::vector<int> naturalX(N);
    std::iota(naturalX.begin(), naturalX.end(), 1);
    std::vector<Key> ordX;
    std::transform(naturalX.begin(), naturalX.end(), std::back_inserter(ordX),
                   [](int x) { return X(x); });

    KeyVector ndX;
    std::vector<int> lvls;
    std::tie(ndX, lvls) = makeBinaryOrdering(ordX);
    std::copy(ndX.begin(), ndX.end(), std::back_inserter(ordering));
    for (auto &l : lvls) {
      l = -l;
    }
  }
  {
    std::vector<int> naturalC(N - 1);
    std::iota(naturalC.begin(), naturalC.end(), 1);
    std::vector<Key> ordC;
    std::transform(naturalC.begin(), naturalC.end(), std::back_inserter(ordC),
                   [](int x) { return M(x); });
    KeyVector ndC;
    std::vector<int> lvls;

    // std::copy(ordC.begin(), ordC.end(), std::back_inserter(ordering));
    std::tie(ndC, lvls) = makeBinaryOrdering(ordC);
    std::copy(ndC.begin(), ndC.end(), std::back_inserter(ordering));
  }
  auto ordering_full = Ordering(ordering);

  // GTSAM_PRINT(*hfg);
  // GTSAM_PRINT(ordering_full);

  HybridBayesTree::shared_ptr hbt;
  HybridGaussianFactorGraph::shared_ptr remaining;
  std::tie(hbt, remaining) = hfg->eliminatePartialMultifrontal(ordering_full);

  // 12 cliques in the bayes tree and 0 remaining variables to eliminate.
  EXPECT_LONGS_EQUAL(12, hbt->size());
  EXPECT_LONGS_EQUAL(0, remaining->size());
}

/* ************************************************************************* */
// TODO(fan): make a graph like Varun's paper one
TEST(HybridGaussianFactorGraph, SwitchingISAM) {
  auto N = 11;
  auto hfg = makeSwitchingChain(N);

  // X(5) will be the center, X(1-4), X(6-9)
  // X(3), X(7)
  // X(2), X(8)
  // X(1), X(4), X(6), X(9)
  // M(5) will be the center, M(1-4), M(6-8)
  // M(3), M(7)
  // M(1), M(4), M(2), M(6), M(8)
  // auto ordering_full =
  //     Ordering(KeyVector{X(1), X(4), X(2), X(6), X(9), X(8), X(3), X(7),
  //     X(5),
  //                        M(1), M(4), M(2), M(6), M(8), M(3), M(7), M(5)});
  KeyVector ordering;

  {
    std::vector<int> naturalX(N);
    std::iota(naturalX.begin(), naturalX.end(), 1);
    std::vector<Key> ordX;
    std::transform(naturalX.begin(), naturalX.end(), std::back_inserter(ordX),
                   [](int x) { return X(x); });

    KeyVector ndX;
    std::vector<int> lvls;
    std::tie(ndX, lvls) = makeBinaryOrdering(ordX);
    std::copy(ndX.begin(), ndX.end(), std::back_inserter(ordering));
    for (auto &l : lvls) {
      l = -l;
    }
  }
  {
    std::vector<int> naturalC(N - 1);
    std::iota(naturalC.begin(), naturalC.end(), 1);
    std::vector<Key> ordC;
    std::transform(naturalC.begin(), naturalC.end(), std::back_inserter(ordC),
                   [](int x) { return M(x); });
    KeyVector ndC;
    std::vector<int> lvls;

    // std::copy(ordC.begin(), ordC.end(), std::back_inserter(ordering));
    std::tie(ndC, lvls) = makeBinaryOrdering(ordC);
    std::copy(ndC.begin(), ndC.end(), std::back_inserter(ordering));
  }
  auto ordering_full = Ordering(ordering);

  HybridBayesTree::shared_ptr hbt;
  HybridGaussianFactorGraph::shared_ptr remaining;
  std::tie(hbt, remaining) = hfg->eliminatePartialMultifrontal(ordering_full);

  auto new_fg = makeSwitchingChain(12);
  auto isam = HybridGaussianISAM(*hbt);

  // Run an ISAM update.
  HybridGaussianFactorGraph factorGraph;
  factorGraph.push_back(new_fg->at(new_fg->size() - 2));
  factorGraph.push_back(new_fg->at(new_fg->size() - 1));
  isam.update(factorGraph);

  // ISAM should have 12 factors after the last update
  EXPECT_LONGS_EQUAL(12, isam.size());
}

/* ************************************************************************* */
TEST(HybridGaussianFactorGraph, SwitchingTwoVar) {
  const int N = 7;
  auto hfg = makeSwitchingChain(N, X);
  hfg->push_back(*makeSwitchingChain(N, Y, D));

  for (int t = 1; t <= N; t++) {
    hfg->add(JacobianFactor(X(t), I_3x3, Y(t), -I_3x3, Vector3(1.0, 0.0, 0.0)));
  }

  KeyVector ordering;

  KeyVector naturalX(N);
  std::iota(naturalX.begin(), naturalX.end(), 1);
  KeyVector ordX;
  for (size_t i = 1; i <= N; i++) {
    ordX.emplace_back(X(i));
    ordX.emplace_back(Y(i));
  }

  for (size_t i = 1; i <= N - 1; i++) {
    ordX.emplace_back(M(i));
  }
  for (size_t i = 1; i <= N - 1; i++) {
    ordX.emplace_back(D(i));
  }

  {
    DotWriter dw;
    dw.positionHints['x'] = 1;
    dw.positionHints['c'] = 0;
    dw.positionHints['d'] = 3;
    dw.positionHints['y'] = 2;
    // std::cout << hfg->dot(DefaultKeyFormatter, dw);
    // std::cout << "\n";
  }

  {
    DotWriter dw;
    dw.positionHints['y'] = 9;
    // dw.positionHints['c'] = 0;
    // dw.positionHints['d'] = 3;
    dw.positionHints['x'] = 1;
    // std::cout << "\n";
    // std::cout << hfg->eliminateSequential(Ordering(ordX))
    //                  ->dot(DefaultKeyFormatter, dw);
    // hfg->eliminateMultifrontal(Ordering(ordX))->dot(std::cout);
  }

  Ordering ordering_partial;
  for (size_t i = 1; i <= N; i++) {
    ordering_partial.emplace_back(X(i));
    ordering_partial.emplace_back(Y(i));
  }
  HybridBayesNet::shared_ptr hbn;
  HybridGaussianFactorGraph::shared_ptr remaining;
  std::tie(hbn, remaining) = hfg->eliminatePartialSequential(ordering_partial);

  EXPECT_LONGS_EQUAL(14, hbn->size());
  EXPECT_LONGS_EQUAL(11, remaining->size());

  {
    DotWriter dw;
    dw.positionHints['x'] = 1;
    dw.positionHints['c'] = 0;
    dw.positionHints['d'] = 3;
    dw.positionHints['y'] = 2;
    // std::cout << remaining->dot(DefaultKeyFormatter, dw);
    // std::cout << "\n";
  }
}

/* ************************************************************************* */
TEST(HybridGaussianFactorGraph, optimize) {
  HybridGaussianFactorGraph hfg;

  DiscreteKey c1(C(1), 2);

  hfg.add(JacobianFactor(X(0), I_3x3, Z_3x1));
  hfg.add(JacobianFactor(X(0), I_3x3, X(1), -I_3x3, Z_3x1));

  DecisionTree<Key, GaussianFactor::shared_ptr> dt(
      C(1), boost::make_shared<JacobianFactor>(X(1), I_3x3, Z_3x1),
      boost::make_shared<JacobianFactor>(X(1), I_3x3, Vector3::Ones()));

  hfg.add(GaussianMixtureFactor({X(1)}, {c1}, dt));

  auto result = hfg.eliminateSequential();

  HybridValues hv = result->optimize();

  EXPECT(assert_equal(hv.atDiscrete(C(1)), int(0)));
}

/* ************************************************************************* */
// Test adding of gaussian conditional and re-elimination.
TEST(HybridGaussianFactorGraph, Conditionals) {
  Switching switching(4);
  HybridGaussianFactorGraph hfg;

  hfg.push_back(switching.linearizedFactorGraph.at(0));  // P(X1)
  Ordering ordering;
  ordering.push_back(X(0));
  HybridBayesNet::shared_ptr bayes_net = hfg.eliminateSequential(ordering);

  hfg.push_back(switching.linearizedFactorGraph.at(1));  // P(X1, X2 | M1)
  hfg.push_back(*bayes_net);
  hfg.push_back(switching.linearizedFactorGraph.at(2));  // P(X2, X3 | M2)
  hfg.push_back(switching.linearizedFactorGraph.at(5));  // P(M1)
  ordering.push_back(X(1));
  ordering.push_back(X(2));
  ordering.push_back(M(0));
  ordering.push_back(M(1));

  bayes_net = hfg.eliminateSequential(ordering);

  HybridValues result = bayes_net->optimize();

  Values expected_continuous;
  expected_continuous.insert<double>(X(0), 0);
  expected_continuous.insert<double>(X(1), 1);
  expected_continuous.insert<double>(X(2), 2);
  expected_continuous.insert<double>(X(3), 4);
  Values result_continuous =
      switching.linearizationPoint.retract(result.continuous());
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
  Switching s(3);

  HybridGaussianFactorGraph graph = s.linearizedFactorGraph;

  HybridBayesNet::shared_ptr hybridBayesNet = graph.eliminateSequential();

  HybridValues delta = hybridBayesNet->optimize();
  auto error_tree = graph.error(delta.continuous());

  std::vector<DiscreteKey> discrete_keys = {{M(0), 2}, {M(1), 2}};
  std::vector<double> leaves = {0.9998558, 0.4902432, 0.5193694, 0.0097568};
  AlgebraicDecisionTree<Key> expected_error(discrete_keys, leaves);

  // regression
  EXPECT(assert_equal(expected_error, error_tree, 1e-7));

  auto probs = graph.probPrime(delta.continuous());
  std::vector<double> prob_leaves = {0.36793249, 0.61247742, 0.59489556,
                                     0.99029064};
  AlgebraicDecisionTree<Key> expected_probs(discrete_keys, prob_leaves);

  // regression
  EXPECT(assert_equal(expected_probs, probs, 1e-7));
}

/* ****************************************************************************/
// Check that assembleGraphTree assembles Gaussian factor graphs for each
// assignment.
TEST(HybridGaussianFactorGraph, assembleGraphTree) {
  using symbol_shorthand::Z;
  const int num_measurements = 1;
  auto fg = tiny::createHybridGaussianFactorGraph(
      num_measurements, VectorValues{{Z(0), Vector1(5.0)}});
  EXPECT_LONGS_EQUAL(3, fg.size());

  // Assemble graph tree:
  auto actual = fg.assembleGraphTree();

  // Create expected decision tree with two factor graphs:

  // Get mixture factor:
  auto mixture = boost::dynamic_pointer_cast<GaussianMixtureFactor>(fg.at(0));
  CHECK(mixture);

  // Get prior factor:
  const auto gf = boost::dynamic_pointer_cast<HybridConditional>(fg.at(1));
  CHECK(gf);
  using GF = GaussianFactor::shared_ptr;
  const GF prior = gf->asGaussian();
  CHECK(prior);

  // Create DiscreteValues for both 0 and 1:
  DiscreteValues d0{{M(0), 0}}, d1{{M(0), 1}};

  // Expected decision tree with two factor graphs:
  // f(x0;mode=0)P(x0) and f(x0;mode=1)P(x0)
  GaussianFactorGraphTree expected{
      M(0), GaussianFactorGraph(std::vector<GF>{(*mixture)(d0), prior}),
      GaussianFactorGraph(std::vector<GF>{(*mixture)(d1), prior})};

  EXPECT(assert_equal(expected(d0), actual(d0), 1e-5));
  EXPECT(assert_equal(expected(d1), actual(d1), 1e-5));
}

/* ****************************************************************************/
// Check that the factor graph unnormalized probability is proportional to the
// Bayes net probability for the given measurements.
bool ratioTest(const HybridBayesNet &bn, const VectorValues &measurements,
               const HybridGaussianFactorGraph &fg, size_t num_samples = 100) {
  auto compute_ratio = [&](HybridValues *sample) -> double {
    sample->update(measurements);  // update sample with given measurements:
    return bn.evaluate(*sample) / fg.probPrime(*sample);
    // return bn.evaluate(*sample) / posterior->evaluate(*sample);
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
// Check that the factor graph unnormalized probability is proportional to the
// Bayes net probability for the given measurements.
bool ratioTest(const HybridBayesNet &bn, const VectorValues &measurements,
               const HybridBayesNet &posterior, size_t num_samples = 100) {
  auto compute_ratio = [&](HybridValues *sample) -> double {
    sample->update(measurements);  // update sample with given measurements:
    // return bn.evaluate(*sample) / fg.probPrime(*sample);
    return bn.evaluate(*sample) / posterior.evaluate(*sample);
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
// Check that eliminating tiny net with 1 measurement yields correct result.
TEST(HybridGaussianFactorGraph, EliminateTiny1) {
  using symbol_shorthand::Z;
  const int num_measurements = 1;
  const VectorValues measurements{{Z(0), Vector1(5.0)}};
  auto bn = tiny::createHybridBayesNet(num_measurements);
  auto fg = bn.toFactorGraph(measurements);
  EXPECT_LONGS_EQUAL(3, fg.size());

  EXPECT(ratioTest(bn, measurements, fg));

  // Create expected Bayes Net:
  HybridBayesNet expectedBayesNet;

  // Create Gaussian mixture on X(0).
  using tiny::mode;
  // regression, but mean checked to be 5.0 in both cases:
  const auto conditional0 = boost::make_shared<GaussianConditional>(
                 X(0), Vector1(14.1421), I_1x1 * 2.82843),
             conditional1 = boost::make_shared<GaussianConditional>(
                 X(0), Vector1(10.1379), I_1x1 * 2.02759);
  expectedBayesNet.emplace_back(
      new GaussianMixture({X(0)}, {}, {mode}, {conditional0, conditional1}));

  // Add prior on mode.
  expectedBayesNet.emplace_back(new DiscreteConditional(mode, "74/26"));

  // Test elimination
  const auto posterior = fg.eliminateSequential();
  EXPECT(assert_equal(expectedBayesNet, *posterior, 0.01));

  EXPECT(ratioTest(bn, measurements, *posterior));
}

/* ****************************************************************************/
// Check that eliminating tiny net with 2 measurements yields correct result.
TEST(HybridGaussianFactorGraph, EliminateTiny2) {
  // Create factor graph with 2 measurements such that posterior mean = 5.0.
  using symbol_shorthand::Z;
  const int num_measurements = 2;
  const VectorValues measurements{{Z(0), Vector1(4.0)}, {Z(1), Vector1(6.0)}};
  auto bn = tiny::createHybridBayesNet(num_measurements);
  auto fg = bn.toFactorGraph(measurements);
  EXPECT_LONGS_EQUAL(4, fg.size());

  // Create expected Bayes Net:
  HybridBayesNet expectedBayesNet;

  // Create Gaussian mixture on X(0).
  using tiny::mode;
  // regression, but mean checked to be 5.0 in both cases:
  const auto conditional0 = boost::make_shared<GaussianConditional>(
                 X(0), Vector1(17.3205), I_1x1 * 3.4641),
             conditional1 = boost::make_shared<GaussianConditional>(
                 X(0), Vector1(10.274), I_1x1 * 2.0548);
  expectedBayesNet.emplace_back(
      new GaussianMixture({X(0)}, {}, {mode}, {conditional0, conditional1}));

  // Add prior on mode.
  expectedBayesNet.emplace_back(new DiscreteConditional(mode, "23/77"));

  // Test elimination
  const auto posterior = fg.eliminateSequential();
  EXPECT(assert_equal(expectedBayesNet, *posterior, 0.01));

  EXPECT(ratioTest(bn, measurements, *posterior));
}

/* ****************************************************************************/
// Test eliminating tiny net with 1 mode per measurement.
TEST(HybridGaussianFactorGraph, EliminateTiny22) {
  // Create factor graph with 2 measurements such that posterior mean = 5.0.
  using symbol_shorthand::Z;
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
  for (size_t t : {0, 1, 2}) {
    // Create Gaussian mixture on Z(t) conditioned on X(t) and mode N(t):
    const auto noise_mode_t = DiscreteKey{N(t), 2};
    bn.emplace_back(
        new GaussianMixture({Z(t)}, {X(t)}, {noise_mode_t},
                            {GaussianConditional::sharedMeanAndStddev(
                                 Z(t), I_1x1, X(t), Z_1x1, 0.5),
                             GaussianConditional::sharedMeanAndStddev(
                                 Z(t), I_1x1, X(t), Z_1x1, 3.0)}));

    // Create prior on discrete mode M(t):
    bn.emplace_back(new DiscreteConditional(noise_mode_t, "20/80"));
  }

  // Add motion models:
  for (size_t t : {2, 1}) {
    // Create Gaussian mixture on X(t) conditioned on X(t-1) and mode M(t-1):
    const auto motion_model_t = DiscreteKey{M(t), 2};
    bn.emplace_back(
        new GaussianMixture({X(t)}, {X(t - 1)}, {motion_model_t},
                            {GaussianConditional::sharedMeanAndStddev(
                                 X(t), I_1x1, X(t - 1), Z_1x1, 0.2),
                             GaussianConditional::sharedMeanAndStddev(
                                 X(t), I_1x1, X(t - 1), I_1x1, 0.2)}));

    // Create prior on motion model M(t):
    bn.emplace_back(new DiscreteConditional(motion_model_t, "40/60"));
  }

  // Create Gaussian prior on continuous X(0) using sharedMeanAndStddev:
  bn.push_back(GaussianConditional::sharedMeanAndStddev(X(0), Z_1x1, 0.1));

  // Make sure we an sample from the Bayes net:
  EXPECT_LONGS_EQUAL(6, bn.sample().continuous().size());

  // Create measurements consistent with moving right every time:
  const VectorValues measurements{
      {Z(0), Vector1(0.0)}, {Z(1), Vector1(1.0)}, {Z(2), Vector1(2.0)}};
  const HybridGaussianFactorGraph fg = bn.toFactorGraph(measurements);

  // Create ordering that eliminates in time order, then discrete modes:
  Ordering ordering;
  ordering.push_back(X(2));
  ordering.push_back(X(1));
  ordering.push_back(X(0));
  ordering.push_back(N(0));
  ordering.push_back(N(1));
  ordering.push_back(N(2));
  ordering.push_back(M(1));
  ordering.push_back(M(2));

  // Do elimination:
  const HybridBayesNet::shared_ptr posterior = fg.eliminateSequential(ordering);

  // Test resulting posterior Bayes net has correct size:
  EXPECT_LONGS_EQUAL(8, posterior->size());

  EXPECT(ratioTest(bn, measurements, *posterior));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

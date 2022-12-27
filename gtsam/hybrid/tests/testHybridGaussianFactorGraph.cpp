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
#include <gtsam/hybrid/HybridDiscreteFactor.h>
#include <gtsam/hybrid/HybridFactor.h>
#include <gtsam/hybrid/HybridGaussianFactor.h>
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
#include <boost/assign/std/map.hpp>
#include <cstddef>
#include <functional>
#include <iostream>
#include <iterator>
#include <vector>

#include "Switching.h"

using namespace boost::assign;

using namespace std;
using namespace gtsam;

using gtsam::symbol_shorthand::D;
using gtsam::symbol_shorthand::M;
using gtsam::symbol_shorthand::X;
using gtsam::symbol_shorthand::Y;

/* ************************************************************************* */
TEST(HybridGaussianFactorGraph, Creation) {
  HybridConditional conditional;

  HybridGaussianFactorGraph hfg;

  hfg.add(HybridGaussianFactor(JacobianFactor(X(0), I_3x3, Z_3x1)));

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

  hfg.add(HybridGaussianFactor(JacobianFactor(0, I_3x3, Z_3x1)));

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
  hfg.add(HybridDiscreteFactor(DecisionTreeFactor(m, {2, 8})));

  Ordering ordering;
  ordering.push_back(X(0));
  auto result = hfg.eliminatePartialMultifrontal(ordering);

  EXPECT_LONGS_EQUAL(result.first->size(), 1);
  EXPECT_LONGS_EQUAL(result.second->size(), 1);
}

/* ************************************************************************* */
TEST(HybridGaussianFactorGraph, eliminateFullSequentialEqualChance) {
  HybridGaussianFactorGraph hfg;

  DiscreteKey m1(M(1), 2);

  // Add prior on x0
  hfg.add(JacobianFactor(X(0), I_3x3, Z_3x1));
  // Add factor between x0 and x1
  hfg.add(JacobianFactor(X(0), I_3x3, X(1), -I_3x3, Z_3x1));

  // Add a gaussian mixture factor ϕ(x1, c1)
  DecisionTree<Key, GaussianFactor::shared_ptr> dt(
      M(1), boost::make_shared<JacobianFactor>(X(1), I_3x3, Z_3x1),
      boost::make_shared<JacobianFactor>(X(1), I_3x3, Vector3::Ones()));

  hfg.add(GaussianMixtureFactor({X(1)}, {m1}, dt));

  auto result =
      hfg.eliminateSequential(Ordering::ColamdConstrainedLast(hfg, {M(1)}));

  auto dc = result->at(2)->asDiscreteConditional();
  DiscreteValues dv;
  dv[M(1)] = 0;
  EXPECT_DOUBLES_EQUAL(1, dc->operator()(dv), 1e-3);
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

  HybridBayesNet::shared_ptr result = hfg.eliminateSequential(
      Ordering::ColamdConstrainedLast(hfg, {M(1), M(2)}));

  // There are 4 variables (2 continuous + 2 discrete) in the bayes net.
  EXPECT_LONGS_EQUAL(4, result->size());
}

/* ************************************************************************* */
TEST(HybridGaussianFactorGraph, eliminateFullMultifrontalSimple) {
  HybridGaussianFactorGraph hfg;

  DiscreteKey m1(M(1), 2);

  hfg.add(JacobianFactor(X(0), I_3x3, Z_3x1));
  hfg.add(JacobianFactor(X(0), I_3x3, X(1), -I_3x3, Z_3x1));

  hfg.add(GaussianMixtureFactor::FromFactors(
      {X(1)}, {{M(1), 2}},
      {boost::make_shared<JacobianFactor>(X(1), I_3x3, Z_3x1),
       boost::make_shared<JacobianFactor>(X(1), I_3x3, Vector3::Ones())}));

  hfg.add(DecisionTreeFactor(m1, {2, 8}));
  // TODO(Varun) Adding extra discrete variable not connected to continuous
  // variable throws segfault
  //  hfg.add(DecisionTreeFactor({{M(1), 2}, {M(2), 2}}, "1 2 3 4"));

  HybridBayesTree::shared_ptr result =
      hfg.eliminateMultifrontal(hfg.getHybridOrdering());

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
  hfg.add(HybridDiscreteFactor(DecisionTreeFactor(m, {2, 8})));

  // Get a constrained ordering keeping c1 last
  auto ordering_full = hfg.getHybridOrdering();

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
    hfg.add(GaussianMixtureFactor::FromFactors(
        {X(0)}, {{M(0), 2}},
        {boost::make_shared<JacobianFactor>(X(0), I_3x3, Z_3x1),
         boost::make_shared<JacobianFactor>(X(0), I_3x3, Vector3::Ones())}));

    DecisionTree<Key, GaussianFactor::shared_ptr> dt1(
        M(1), boost::make_shared<JacobianFactor>(X(2), I_3x3, Z_3x1),
        boost::make_shared<JacobianFactor>(X(2), I_3x3, Vector3::Ones()));

    hfg.add(GaussianMixtureFactor({X(2)}, {{M(1), 2}}, dt1));
  }

  hfg.add(HybridDiscreteFactor(
      DecisionTreeFactor({{M(1), 2}, {M(2), 2}}, "1 2 3 4")));

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

  auto result =
      hfg.eliminateSequential(Ordering::ColamdConstrainedLast(hfg, {C(1)}));

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

  Ordering hybridOrdering = graph.getHybridOrdering();
  HybridBayesNet::shared_ptr hybridBayesNet =
      graph.eliminateSequential(hybridOrdering);

  HybridValues delta = hybridBayesNet->optimize();
  double error = graph.error(delta.continuous(), delta.discrete());

  double expected_error = 3.20475e-30;
  // regression
  EXPECT(assert_equal(expected_error, error, 1e-9));

  double probs = exp(-error);
  double expected_probs = graph.probPrime(delta.continuous(), delta.discrete());

  // regression
  EXPECT(assert_equal(expected_probs, probs, 1e-7));
}

/* ****************************************************************************/
// Test hybrid gaussian factor graph error and unnormalized probabilities
TEST(HybridGaussianFactorGraph, ErrorAndProbPrimeTree) {
  Switching s(3);

  HybridGaussianFactorGraph graph = s.linearizedFactorGraph;

  Ordering hybridOrdering = graph.getHybridOrdering();
  HybridBayesNet::shared_ptr hybridBayesNet =
      graph.eliminateSequential(hybridOrdering);

  HybridValues delta = hybridBayesNet->optimize();
  auto error_tree = graph.error(delta.continuous());

  std::vector<DiscreteKey> discrete_keys = {{M(0), 2}, {M(1), 2}};
  std::vector<double> leaves = {1.0, 0.5, 0.5, 3.2047474e-30};
  AlgebraicDecisionTree<Key> expected_error(discrete_keys, leaves);

  // regression
  EXPECT(assert_equal(expected_error, error_tree, 1e-7));

  auto probs = graph.probPrime(delta.continuous());
  std::vector<double> prob_leaves = {0.36787944, 0.60653066, 0.60653066, 1.0};
  AlgebraicDecisionTree<Key> expected_probs(discrete_keys, prob_leaves);

  // regression
  EXPECT(assert_equal(expected_probs, probs, 1e-7));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

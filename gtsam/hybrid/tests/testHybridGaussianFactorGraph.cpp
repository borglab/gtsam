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
#include <gtsam/base/Vector.h>
#include <gtsam/discrete/DecisionTreeFactor.h>
#include <gtsam/discrete/DiscreteKey.h>
#include <gtsam/discrete/DiscreteValues.h>
#include <gtsam/hybrid/HybridBayesNet.h>
#include <gtsam/hybrid/HybridBayesTree.h>
#include <gtsam/hybrid/HybridConditional.h>
#include <gtsam/hybrid/HybridFactor.h>
#include <gtsam/hybrid/HybridGaussianConditional.h>
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
#include <cstddef>
#include <functional>
#include <iostream>
#include <iterator>
#include <memory>
#include <numeric>
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

static const DiscreteKey m1(M(1), 2);

/* ************************************************************************* */
TEST(HybridGaussianFactorGraph, Creation) {
  HybridConditional conditional;

  HybridGaussianFactorGraph hfg;

  hfg.emplace_shared<JacobianFactor>(X(0), I_3x3, Z_3x1);

  // Define a hybrid gaussian conditional P(x0|x1, c0)
  // and add it to the factor graph.
  HybridGaussianConditional gm(
      {M(0), 2},
      {std::make_shared<GaussianConditional>(X(0), Z_3x1, I_3x3, X(1), I_3x3),
       std::make_shared<GaussianConditional>(X(0), Vector3::Ones(), I_3x3, X(1),
                                             I_3x3)});
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

  // Add priors on x0 and c1
  hfg.add(JacobianFactor(X(0), I_3x3, Z_3x1));
  hfg.add(DecisionTreeFactor(m1, {2, 8}));

  Ordering ordering;
  ordering.push_back(X(0));
  auto result = hfg.eliminatePartialMultifrontal(ordering);

  EXPECT_LONGS_EQUAL(result.first->size(), 1);
  EXPECT_LONGS_EQUAL(result.second->size(), 1);
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
  EXPECT(assert_equal(DecisionTreeFactor{m1, "1 1"}, *factor));
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
  hfg.add(DecisionTreeFactor({{M(1), 2}, {M(2), 2}}, "1 2 3 4"));

  HybridBayesNet::shared_ptr result = hfg.eliminateSequential();

  // There are 4 variables (2 continuous + 2 discrete) in the bayes net.
  EXPECT_LONGS_EQUAL(4, result->size());
}

/* ************************************************************************* */
TEST(HybridGaussianFactorGraph, eliminateFullMultifrontalSimple) {
  HybridGaussianFactorGraph hfg;

  hfg.add(JacobianFactor(X(0), I_3x3, Z_3x1));
  hfg.add(JacobianFactor(X(0), I_3x3, X(1), -I_3x3, Z_3x1));

  hfg.add(HybridGaussianFactor({M(1), 2}, two::components(X(1))));

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

  // Prior on x0
  hfg.add(JacobianFactor(X(0), I_3x3, Z_3x1));
  // Factor between x0-x1
  hfg.add(JacobianFactor(X(0), I_3x3, X(1), -I_3x3, Z_3x1));

  // Hybrid factor P(x1|c1)
  hfg.add(HybridGaussianFactor(m1, two::components(X(1))));
  // Prior factor on c1
  hfg.add(DecisionTreeFactor(m1, {2, 8}));

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

  hfg.add(HybridGaussianFactor({M(0), 2}, two::components(X(0))));
  hfg.add(HybridGaussianFactor({M(1), 2}, two::components(X(2))));

  hfg.add(DecisionTreeFactor({{M(1), 2}, {M(2), 2}}, "1 2 3 4"));

  hfg.add(JacobianFactor(X(3), I_3x3, X(4), -I_3x3, Z_3x1));
  hfg.add(JacobianFactor(X(4), I_3x3, X(5), -I_3x3, Z_3x1));

  hfg.add(HybridGaussianFactor({M(3), 2}, two::components(X(3))));
  hfg.add(HybridGaussianFactor({M(2), 2}, two::components(X(5))));

  auto ordering_full =
      Ordering::ColamdConstrainedLast(hfg, {M(0), M(1), M(2), M(3)});

  const auto [hbt, remaining] = hfg.eliminatePartialMultifrontal(ordering_full);

  // 9 cliques in the bayes tree and 0 remaining variables to eliminate.
  EXPECT_LONGS_EQUAL(9, hbt->size());
  EXPECT_LONGS_EQUAL(0, remaining->size());

  /*
  (Fan) Explanation: the Junction tree will need to re-eliminate to get to the
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

    auto [ndX, lvls] = makeBinaryOrdering(ordX);
    std::copy(ndX.begin(), ndX.end(), std::back_inserter(ordering));
    // TODO(dellaert): this has no effect!
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

    // std::copy(ordC.begin(), ordC.end(), std::back_inserter(ordering));
    const auto [ndC, lvls] = makeBinaryOrdering(ordC);
    std::copy(ndC.begin(), ndC.end(), std::back_inserter(ordering));
  }
  auto ordering_full = Ordering(ordering);

  // GTSAM_PRINT(*hfg);
  // GTSAM_PRINT(ordering_full);

  const auto [hbt, remaining] =
      hfg->eliminatePartialMultifrontal(ordering_full);

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

    auto [ndX, lvls] = makeBinaryOrdering(ordX);
    std::copy(ndX.begin(), ndX.end(), std::back_inserter(ordering));
    // TODO(dellaert): this has no effect!
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

    // std::copy(ordC.begin(), ordC.end(), std::back_inserter(ordering));
    const auto [ndC, lvls] = makeBinaryOrdering(ordC);
    std::copy(ndC.begin(), ndC.end(), std::back_inserter(ordering));
  }
  auto ordering_full = Ordering(ordering);

  const auto [hbt, remaining] =
      hfg->eliminatePartialMultifrontal(ordering_full);

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
  const auto [hbn, remaining] =
      hfg->eliminatePartialSequential(ordering_partial);

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

/* ****************************************************************************/
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
  // Create switching network with three continuous variables and two discrete:
  // ϕ(x0) ϕ(x0,x1,m0) ϕ(x1,x2,m1) ϕ(x0;z0) ϕ(x1;z1) ϕ(x2;z2) ϕ(m0) ϕ(m0,m1)
  Switching s(3);

  const HybridGaussianFactorGraph &graph = s.linearizedFactorGraph;

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

  std::vector<DiscreteKey> discrete_keys = {{M(0), 2}, {M(1), 2}};
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
  leaves = {0.50985198, 0.0097577296, 0.50009425, 0,
            0.52922138, 0.029127133,  0.50985105, 0.0097567964};
  AlgebraicDecisionTree<Key> expected_error2(s.modes, leaves);
  EXPECT(assert_equal(expected_error, error_tree, 1e-7));
}

/* ****************************************************************************/
// Check that assembleGraphTree assembles Gaussian factor graphs for each
// assignment.
TEST(HybridGaussianFactorGraph, assembleGraphTree) {
  const int num_measurements = 1;
  auto fg = tiny::createHybridGaussianFactorGraph(
      num_measurements, VectorValues{{Z(0), Vector1(5.0)}});
  EXPECT_LONGS_EQUAL(3, fg.size());

  // Assemble graph tree:
  auto actual = fg.assembleGraphTree();

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
  // f(x0;mode=0)P(x0) and f(x0;mode=1)P(x0)
  GaussianFactorGraphTree expected{
      M(0), GaussianFactorGraph(std::vector<GF>{(*hybrid)(d0), prior}),
      GaussianFactorGraph(std::vector<GF>{(*hybrid)(d1), prior})};

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
bool ratioTest(const HybridBayesNet &bn, const VectorValues &measurements,
               const HybridBayesNet &posterior, size_t num_samples = 100) {
  auto compute_ratio = [&](HybridValues *sample) -> double {
    sample->update(measurements);  // update sample with given measurements:
    return bn.evaluate(*sample) / posterior.evaluate(*sample);
  };

  HybridValues sample = bn.sample(&kRng);
  double expected_ratio = compute_ratio(&sample);

  // Test ratios for a number of independent samples:
  for (size_t i = 0; i < num_samples; i++) {
    HybridValues sample = bn.sample(&kRng);
    // GTSAM_PRINT(sample);
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
  const auto conditional0 = std::make_shared<GaussianConditional>(
                 X(0), Vector1(14.1421), I_1x1 * 2.82843),
             conditional1 = std::make_shared<GaussianConditional>(
                 X(0), Vector1(10.1379), I_1x1 * 2.02759);
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
  bn.push_back(
      GaussianConditional::sharedMeanAndStddev(X(0), Vector1(5.0), 0.5));

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
  const auto conditional0 = std::make_shared<GaussianConditional>(
                 X(0), Vector1(10.1379), I_1x1 * 2.02759),
             conditional1 = std::make_shared<GaussianConditional>(
                 X(0), Vector1(14.1421), I_1x1 * 2.82843);
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
  const auto conditional0 = std::make_shared<GaussianConditional>(
                 X(0), Vector1(17.3205), I_1x1 * 3.4641),
             conditional1 = std::make_shared<GaussianConditional>(
                 X(0), Vector1(10.274), I_1x1 * 2.0548);
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
  std::vector<std::pair<Vector, double>> measurementModels{{Z_1x1, 3},
                                                           {Z_1x1, 0.5}};
  for (size_t t : {0, 1, 2}) {
    // Create hybrid Gaussian factor on Z(t) conditioned on X(t) and mode N(t):
    const auto noise_mode_t = DiscreteKey{N(t), 2};
    bn.emplace_shared<HybridGaussianConditional>(noise_mode_t, Z(t), I_1x1,
                                                 X(t), measurementModels);

    // Create prior on discrete mode N(t):
    bn.emplace_shared<DiscreteConditional>(noise_mode_t, "20/80");
  }

  // Add motion models. TODO(frank): why are they exactly the same?
  std::vector<std::pair<Vector, double>> motionModels{{Z_1x1, 0.2},
                                                      {Z_1x1, 0.2}};
  for (size_t t : {2, 1}) {
    // Create hybrid Gaussian factor on X(t) conditioned on X(t-1)
    // and mode M(t-1):
    const auto motion_model_t = DiscreteKey{M(t), 2};
    bn.emplace_shared<HybridGaussianConditional>(motion_model_t, X(t), I_1x1,
                                                 X(t - 1), motionModels);

    // Create prior on motion model M(t):
    bn.emplace_shared<DiscreteConditional>(motion_model_t, "40/60");
  }

  // Create Gaussian prior on continuous X(0) using sharedMeanAndStddev:
  bn.push_back(GaussianConditional::sharedMeanAndStddev(X(0), Z_1x1, 0.1));

  // Make sure we an sample from the Bayes net:
  EXPECT_LONGS_EQUAL(6, bn.sample().continuous().size());

  // Create measurements consistent with moving right every time:
  const VectorValues measurements{
      {Z(0), Vector1(0.0)}, {Z(1), Vector1(1.0)}, {Z(2), Vector1(2.0)}};
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

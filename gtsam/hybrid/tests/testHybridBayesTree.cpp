/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testHybridBayesTree.cpp
 * @brief   Unit tests for HybridBayesTree
 * @author  Varun Agrawal
 * @date    August 2022
 */

#include <gtsam/base/serializationTestHelpers.h>
#include <gtsam/discrete/DiscreteFactorGraph.h>
#include <gtsam/hybrid/HybridBayesTree.h>
#include <gtsam/hybrid/HybridGaussianISAM.h>
#include <gtsam/inference/DotWriter.h>

#include "Switching.h"

// Include for test suite
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;
using symbol_shorthand::D;
using symbol_shorthand::M;
using symbol_shorthand::X;
using symbol_shorthand::Y;

static const DiscreteKey m0(M(0), 2), m1(M(1), 2), m2(M(2), 2), m3(M(3), 2);

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
} // namespace two

/* ************************************************************************* */
TEST(HybridGaussianFactorGraph, eliminateFullMultifrontalSimple) {
  HybridGaussianFactorGraph hfg;

  hfg.add(JacobianFactor(X(0), I_3x3, Z_3x1));
  hfg.add(JacobianFactor(X(0), I_3x3, X(1), -I_3x3, Z_3x1));

  hfg.add(HybridGaussianFactor(m1, two::components(X(1))));

  hfg.add(DecisionTreeFactor(m1, {2, 8}));
  // TODO(Varun) Adding extra discrete variable not connected to continuous
  // variable throws segfault
  //  hfg.add(DecisionTreeFactor({m1, m2, "1 2 3 4"));

  HybridBayesTree::shared_ptr result = hfg.eliminateMultifrontal();

  // The bayes tree should have 3 cliques
  EXPECT_LONGS_EQUAL(3, result->size());
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
// Check assembling the Bayes Tree roots after we do partial elimination
TEST(HybridGaussianFactorGraph, eliminateFullMultifrontalTwoClique) {
  HybridGaussianFactorGraph hfg;

  hfg.add(JacobianFactor(X(0), I_3x3, X(1), -I_3x3, Z_3x1));
  hfg.add(JacobianFactor(X(1), I_3x3, X(2), -I_3x3, Z_3x1));

  hfg.add(HybridGaussianFactor(m0, two::components(X(0))));
  hfg.add(HybridGaussianFactor(m1, two::components(X(2))));

  hfg.add(DecisionTreeFactor({m1, m2}, "1 2 3 4"));

  hfg.add(JacobianFactor(X(3), I_3x3, X(4), -I_3x3, Z_3x1));
  hfg.add(JacobianFactor(X(4), I_3x3, X(5), -I_3x3, Z_3x1));

  hfg.add(HybridGaussianFactor(m3, two::components(X(3))));
  hfg.add(HybridGaussianFactor(m2, two::components(X(5))));

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

/* ************************************************************************* */
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
// Test multifrontal optimize
TEST(HybridBayesTree, OptimizeMultifrontal) {
  Switching s(4);

  HybridBayesTree::shared_ptr hybridBayesTree =
      s.linearizedFactorGraph.eliminateMultifrontal();
  HybridValues delta = hybridBayesTree->optimize();

  VectorValues expectedValues;
  expectedValues.insert(X(0), -0.999904 * Vector1::Ones());
  expectedValues.insert(X(1), -0.99029 * Vector1::Ones());
  expectedValues.insert(X(2), -1.00971 * Vector1::Ones());
  expectedValues.insert(X(3), -1.0001 * Vector1::Ones());

  EXPECT(assert_equal(expectedValues, delta.continuous(), 1e-5));
}

/* ****************************************************************************/
// Test for optimizing a HybridBayesTree with a given assignment.
TEST(HybridBayesTree, OptimizeAssignment) {
  Switching s(4);

  HybridGaussianISAM isam;
  HybridGaussianFactorGraph graph1;

  // Add the 3 hybrid factors, x1-x2, x2-x3, x3-x4
  for (size_t i = 1; i < 4; i++) {
    graph1.push_back(s.linearizedFactorGraph.at(i));
  }

  // Add the Gaussian factors, 1 prior on X(1),
  // 3 measurements on X(2), X(3), X(4)
  graph1.push_back(s.linearizedFactorGraph.at(0));
  for (size_t i = 4; i <= 7; i++) {
    graph1.push_back(s.linearizedFactorGraph.at(i));
  }

  // Add the discrete factors
  for (size_t i = 7; i <= 9; i++) {
    graph1.push_back(s.linearizedFactorGraph.at(i));
  }

  isam.update(graph1);

  DiscreteValues assignment;
  assignment[M(0)] = 1;
  assignment[M(1)] = 1;
  assignment[M(2)] = 1;

  VectorValues delta = isam.optimize(assignment);

  // The linearization point has the same value as the key index,
  // e.g. X(1) = 1, X(2) = 2,
  // but the factors specify X(k) = k-1, so delta should be -1.
  VectorValues expected_delta;
  expected_delta.insert(make_pair(X(0), -Vector1::Ones()));
  expected_delta.insert(make_pair(X(1), -Vector1::Ones()));
  expected_delta.insert(make_pair(X(2), -Vector1::Ones()));
  expected_delta.insert(make_pair(X(3), -Vector1::Ones()));

  EXPECT(assert_equal(expected_delta, delta));

  // Create ordering.
  Ordering ordering;
  for (size_t k = 0; k < s.K; k++)
    ordering.push_back(X(k));

  const auto [hybridBayesNet, remainingFactorGraph] =
      s.linearizedFactorGraph.eliminatePartialSequential(ordering);

  GaussianBayesNet gbn = hybridBayesNet->choose(assignment);
  VectorValues expected = gbn.optimize();

  EXPECT(assert_equal(expected, delta));
}

/* ****************************************************************************/
// Test for optimizing a HybridBayesTree.
TEST(HybridBayesTree, Optimize) {
  Switching s(4);

  HybridGaussianISAM isam;
  HybridGaussianFactorGraph graph1;

  // Add the 3 hybrid factors, x1-x2, x2-x3, x3-x4
  for (size_t i = 1; i < 4; i++) {
    graph1.push_back(s.linearizedFactorGraph.at(i));
  }

  // Add the Gaussian factors, 1 prior on X(0),
  // 3 measurements on X(2), X(3), X(4)
  graph1.push_back(s.linearizedFactorGraph.at(0));
  for (size_t i = 4; i <= 6; i++) {
    graph1.push_back(s.linearizedFactorGraph.at(i));
  }

  // Add the discrete factors
  for (size_t i = 7; i <= 9; i++) {
    graph1.push_back(s.linearizedFactorGraph.at(i));
  }

  isam.update(graph1);

  HybridValues delta = isam.optimize();

  // Create ordering.
  Ordering ordering;
  for (size_t k = 0; k < s.K; k++)
    ordering.push_back(X(k));

  const auto [hybridBayesNet, remainingFactorGraph] =
      s.linearizedFactorGraph.eliminatePartialSequential(ordering);

  DiscreteFactorGraph dfg;
  for (auto &&f : *remainingFactorGraph) {
    auto discreteFactor = dynamic_pointer_cast<DiscreteFactor>(f);
    assert(discreteFactor);
    dfg.push_back(discreteFactor);
  }

  // Add the probabilities for each branch
  DiscreteKeys discrete_keys = {m0, m1, m2};
  vector<double> probs = {0.012519475, 0.041280228, 0.075018647, 0.081663656,
                          0.037152205, 0.12248971,  0.07349729,  0.08};
  dfg.emplace_shared<DecisionTreeFactor>(discrete_keys, probs);

  DiscreteValues expectedMPE = dfg.optimize();
  VectorValues expectedValues = hybridBayesNet->optimize(expectedMPE);

  EXPECT(assert_equal(expectedMPE, delta.discrete()));
  EXPECT(assert_equal(expectedValues, delta.continuous()));
}

/* ****************************************************************************/
// Test for choosing a GaussianBayesTree from a HybridBayesTree.
TEST(HybridBayesTree, Choose) {
  Switching s(4);

  HybridGaussianISAM isam;
  HybridGaussianFactorGraph graph1;

  // Add the 3 hybrid factors, x1-x2, x2-x3, x3-x4
  for (size_t i = 1; i < 4; i++) {
    graph1.push_back(s.linearizedFactorGraph.at(i));
  }

  // Add the Gaussian factors, 1 prior on X(0),
  // 3 measurements on X(2), X(3), X(4)
  graph1.push_back(s.linearizedFactorGraph.at(0));
  for (size_t i = 4; i <= 6; i++) {
    graph1.push_back(s.linearizedFactorGraph.at(i));
  }

  // Add the discrete factors
  for (size_t i = 7; i <= 9; i++) {
    graph1.push_back(s.linearizedFactorGraph.at(i));
  }

  isam.update(graph1);

  DiscreteValues assignment;
  assignment[M(0)] = 1;
  assignment[M(1)] = 1;
  assignment[M(2)] = 1;

  GaussianBayesTree gbt = isam.choose(assignment);

  // Specify ordering so it matches that of HybridGaussianISAM.
  Ordering ordering(KeyVector{X(0), X(1), X(2), X(3), M(0), M(1), M(2)});
  auto bayesTree = s.linearizedFactorGraph.eliminateMultifrontal(ordering);

  auto expected_gbt = bayesTree->choose(assignment);

  EXPECT(assert_equal(expected_gbt, gbt));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

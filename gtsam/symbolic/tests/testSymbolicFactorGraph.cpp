/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   testSymbolicFactorGraph.cpp
 *  @brief  Unit tests for symbolic factor graphs
 *  @author Christian Potthast
 **/

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/symbolic/SymbolicBayesNet.h>
#include <gtsam/symbolic/SymbolicBayesTree.h>
#include <gtsam/symbolic/SymbolicConditional.h>
#include <gtsam/symbolic/SymbolicFactorGraph.h>
#include <gtsam/symbolic/tests/symbolicExampleGraphs.h>

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST(SymbolicFactorGraph, keys1) {
  KeySet expected{0, 1, 2, 3, 4};
  KeySet actual = simpleTestGraph1.keys();
  EXPECT(expected == actual);
}

/* ************************************************************************* */
TEST(SymbolicFactorGraph, keys2) {
  KeySet expected{0, 1, 2, 3, 4, 5};
  KeySet actual = simpleTestGraph2.keys();
  EXPECT(expected == actual);
}

/* ************************************************************************* */
TEST(SymbolicFactorGraph, eliminateFullSequential) {
  // Test with simpleTestGraph1
  Ordering order{0, 1, 2, 3, 4};
  SymbolicBayesNet actual1 = *simpleTestGraph1.eliminateSequential(order);
  EXPECT(assert_equal(simpleTestGraph1BayesNet, actual1));

  // Test with Asia graph
  SymbolicBayesNet actual2 = *asiaGraph.eliminateSequential(asiaOrdering);
  EXPECT(assert_equal(asiaBayesNet, actual2));
}

/* ************************************************************************* */
TEST(SymbolicFactorGraph, eliminatePartialSequential) {
  // Eliminate 0 and 1
  const Ordering order{0, 1};

  const auto expectedBayesNet = SymbolicBayesNet(SymbolicConditional(0, 1, 2))(
      SymbolicConditional(1, 2, 3, 4));

  const auto expectedSfg = SymbolicFactorGraph(SymbolicFactor(2, 3))(
      SymbolicFactor(4, 5))(SymbolicFactor(2, 3, 4));

  const auto [actualBayesNet, actualSfg] =
      simpleTestGraph2.eliminatePartialSequential(Ordering{0, 1});

  EXPECT(assert_equal(expectedSfg, *actualSfg));
  EXPECT(assert_equal(expectedBayesNet, *actualBayesNet));

  const auto [actualBayesNet2, actualSfg2] =
      simpleTestGraph2.eliminatePartialSequential(Ordering{0, 1});

  EXPECT(assert_equal(expectedSfg, *actualSfg2));
  EXPECT(assert_equal(expectedBayesNet, *actualBayesNet2));
}

/* ************************************************************************* */
TEST(SymbolicFactorGraph, eliminateFullMultifrontal) {
  Ordering ordering{0, 1, 2, 3};
  SymbolicBayesTree actual1 = *simpleChain.eliminateMultifrontal(ordering);
  EXPECT(assert_equal(simpleChainBayesTree, actual1));

  SymbolicBayesTree actual2 = *asiaGraph.eliminateMultifrontal(asiaOrdering);
  EXPECT(assert_equal(asiaBayesTree, actual2));
}

/* ************************************************************************* */
TEST(SymbolicFactorGraph, eliminatePartialMultifrontal) {
  SymbolicBayesTree expectedBayesTree;
  SymbolicConditional::shared_ptr root =
      std::make_shared<SymbolicConditional>(
          SymbolicConditional::FromKeys(KeyVector{4, 5, 1}, 2));
  expectedBayesTree.insertRoot(
      std::make_shared<SymbolicBayesTreeClique>(root));

  const auto expectedFactorGraph =
      SymbolicFactorGraph(SymbolicFactor(0, 1))(SymbolicFactor(0, 2))(
          SymbolicFactor(1, 3))(SymbolicFactor(2, 3))(SymbolicFactor(1));

  const auto [actualBayesTree, actualFactorGraph] =
      simpleTestGraph2.eliminatePartialMultifrontal(Ordering{4, 5});

  EXPECT(assert_equal(expectedFactorGraph, *actualFactorGraph));
  EXPECT(assert_equal(expectedBayesTree, *actualBayesTree));

  SymbolicBayesTree expectedBayesTree2;
  SymbolicBayesTreeClique::shared_ptr root2 =
      std::make_shared<SymbolicBayesTreeClique>(
          std::make_shared<SymbolicConditional>(4, 1));
  root2->children.push_back(std::make_shared<SymbolicBayesTreeClique>(
      std::make_shared<SymbolicConditional>(5, 4)));
  expectedBayesTree2.insertRoot(root2);

  const auto [actualBayesTree2, actualFactorGraph2] =
      simpleTestGraph2.eliminatePartialMultifrontal(KeyVector{4, 5});

  EXPECT(assert_equal(expectedFactorGraph, *actualFactorGraph2));
  EXPECT(assert_equal(expectedBayesTree2, *actualBayesTree2));
}

/* ************************************************************************* */
TEST(SymbolicFactorGraph, MarginalMultifrontalBayesNetOrdering) {
  SymbolicBayesNet actual =
      *simpleTestGraph2.marginalMultifrontalBayesNet(Ordering{0, 1, 2, 3});
  auto expectedBayesNet = SymbolicBayesNet({0, 1, 2})({1, 2, 3})({2, 3})({3});
  EXPECT(assert_equal(expectedBayesNet, actual));
}

TEST(SymbolicFactorGraph, MarginalMultifrontalBayesNetKeyVector) {
  SymbolicBayesNet actual =
      *simpleTestGraph2.marginalMultifrontalBayesNet(KeyVector{0, 1, 2, 3});
  // Since we use KeyVector, the variable ordering will be determined by COLAMD:
  auto expectedBayesNet = SymbolicBayesNet({0, 1, 2})({2, 1, 3})({1, 3})({3});
  EXPECT(assert_equal(expectedBayesNet, actual));
}

TEST(SymbolicFactorGraph, MarginalMultifrontalBayesNetOrderingPlus) {
  const Ordering orderedVariables{0, 3},
      marginalizedVariableOrdering{1, 2, 4, 5};
  SymbolicBayesNet actual = *simpleTestGraph2.marginalMultifrontalBayesNet(
      orderedVariables, marginalizedVariableOrdering);
  auto expectedBayesNet = SymbolicBayesNet(SymbolicConditional{0, 3})({3});
  EXPECT(assert_equal(expectedBayesNet, actual));
}

TEST(SymbolicFactorGraph, MarginalMultifrontalBayesNetKeyVectorPlus) {
  const KeyVector variables{0, 1, 3};
  const Ordering marginalizedVariableOrdering{2, 4, 5};
  SymbolicBayesNet actual = *simpleTestGraph2.marginalMultifrontalBayesNet(
      variables, marginalizedVariableOrdering);
  // Since we use KeyVector, the variable ordering will be determined by COLAMD:
  auto expectedBayesNet = SymbolicBayesNet({0, 1, 3})({3, 1})({1});
  EXPECT(assert_equal(expectedBayesNet, actual));
}

/* ************************************************************************* */
TEST(SymbolicFactorGraph, MarginalMultifrontalBayesTreeOrdering) {
  auto expectedBayesTree =
      *simpleTestGraph2.eliminatePartialMultifrontal(Ordering{4, 5})
           .second->eliminateMultifrontal(Ordering{0, 1, 2, 3});

  SymbolicBayesTree actual =
      *simpleTestGraph2.marginalMultifrontalBayesTree(Ordering{0, 1, 2, 3});
  EXPECT(assert_equal(expectedBayesTree, actual));
}

TEST(SymbolicFactorGraph, MarginalMultifrontalBayesTreeKeyVector) {
  // Same: KeyVector variant will use COLAMD:
  auto expectedBayesTree =
      *simpleTestGraph2.eliminatePartialMultifrontal(Ordering{4, 5})
           .second->eliminateMultifrontal(Ordering::OrderingType::COLAMD);

  SymbolicBayesTree actual =
      *simpleTestGraph2.marginalMultifrontalBayesTree(KeyVector{0, 1, 2, 3});
  EXPECT(assert_equal(expectedBayesTree, actual));
}

TEST(SymbolicFactorGraph, MarginalMultifrontalBayesTreeOrderingPlus) {
  const Ordering orderedVariables{0, 3},
      marginalizedVariableOrdering{1, 2, 4, 5};
  auto expectedBayesTree =
      *simpleTestGraph2
           .eliminatePartialMultifrontal(marginalizedVariableOrdering)
           .second->eliminateMultifrontal(orderedVariables);

  SymbolicBayesTree actual = *simpleTestGraph2.marginalMultifrontalBayesTree(
      orderedVariables, marginalizedVariableOrdering);
  EXPECT(assert_equal(expectedBayesTree, actual));
}

TEST(SymbolicFactorGraph, MarginalMultifrontalBayesTreeKeyVectorPlus) {
  // Again: KeyVector variant will use COLAMD:
  const Ordering marginalizedVariableOrdering{2, 4, 5};
  auto expectedBayesTree =
      *simpleTestGraph2
           .eliminatePartialMultifrontal(marginalizedVariableOrdering)
           .second->eliminateMultifrontal(Ordering::OrderingType::COLAMD);

  const KeyVector variables{0, 1, 3};
  SymbolicBayesTree actual = *simpleTestGraph2.marginalMultifrontalBayesTree(
      variables, marginalizedVariableOrdering);
  EXPECT(assert_equal(expectedBayesTree, actual));
}

/* ************************************************************************* */
TEST(SymbolicFactorGraph, eliminate_disconnected_graph) {
  SymbolicFactorGraph fg;
  fg.push_factor(0, 1);
  fg.push_factor(0, 2);
  fg.push_factor(1, 2);
  fg.push_factor(3, 4);

  // create expected Chordal bayes Net
  SymbolicBayesNet expected;
  expected.emplace_shared<SymbolicConditional>(0, 1, 2);
  expected.emplace_shared<SymbolicConditional>(1, 2);
  expected.emplace_shared<SymbolicConditional>(2);
  expected.emplace_shared<SymbolicConditional>(3, 4);
  expected.emplace_shared<SymbolicConditional>(4);

  const Ordering order{0, 1, 2, 3, 4};
  SymbolicBayesNet actual = *fg.eliminateSequential(order);

  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST(SymbolicFactorGraph, marginals) {
  // Create factor graph
  SymbolicFactorGraph fg;
  fg.push_factor(0, 1);
  fg.push_factor(0, 2);
  fg.push_factor(1, 4);
  fg.push_factor(2, 4);
  fg.push_factor(3, 4);

  // eliminate
  Ordering ord{3, 4, 2, 1, 0};
  auto actual = fg.eliminateSequential(ord);
  SymbolicBayesNet expected;
  expected.emplace_shared<SymbolicConditional>(3, 4);
  expected.emplace_shared<SymbolicConditional>(4, 1, 2);
  expected.emplace_shared<SymbolicConditional>(2, 0, 1);
  expected.emplace_shared<SymbolicConditional>(1, 0);
  expected.emplace_shared<SymbolicConditional>(0);
  EXPECT(assert_equal(expected, *actual));

  {
    // jointBayesNet
    Ordering ord{0, 4, 3};
    auto actual = fg.eliminatePartialSequential(ord);
    SymbolicBayesNet expectedBN;
    expectedBN.emplace_shared<SymbolicConditional>(0, 1, 2);
    expectedBN.emplace_shared<SymbolicConditional>(4, 1, 2, 3);
    expectedBN.emplace_shared<SymbolicConditional>(3, 1, 2);
    EXPECT(assert_equal(expectedBN, *(actual.first)));
  }

  {
    // jointBayesNet
    Ordering ord{0, 2, 3};
    auto actual = fg.eliminatePartialSequential(ord);
    SymbolicBayesNet expectedBN;
    expectedBN.emplace_shared<SymbolicConditional>(0, 1, 2);
    expectedBN.emplace_shared<SymbolicConditional>(2, 1, 4);
    expectedBN.emplace_shared<SymbolicConditional>(3, 4);
    EXPECT(assert_equal(expectedBN, *(actual.first)));
  }

  {
    // conditionalBayesNet
    Ordering ord{0, 2};
    auto actual = fg.eliminatePartialSequential(ord);
    SymbolicBayesNet expectedBN;
    expectedBN.emplace_shared<SymbolicConditional>(0, 1, 2);
    expectedBN.emplace_shared<SymbolicConditional>(2, 1, 4);
    EXPECT(assert_equal(expectedBN, *(actual.first)));
  }
}

/* ************************************************************************* */
TEST(SymbolicFactorGraph, constructFromBayesNet) {
  // create expected factor graph
  SymbolicFactorGraph expected;
  expected.push_factor(0, 1, 2);
  expected.push_factor(1, 2);
  expected.push_factor(1);

  // create Bayes Net
  SymbolicBayesNet bayesNet;
  bayesNet.emplace_shared<SymbolicConditional>(0, 1, 2);
  bayesNet.emplace_shared<SymbolicConditional>(1, 2);
  bayesNet.emplace_shared<SymbolicConditional>(1);

  // create actual factor graph from a Bayes Net
  SymbolicFactorGraph actual(bayesNet);

  CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST(SymbolicFactorGraph, constructFromBayesTree) {
  // create expected factor graph
  SymbolicFactorGraph expected;
  expected.push_factor(_E_, _L_, _B_);
  expected.push_factor(_S_, _B_, _L_);
  expected.push_factor(_T_, _E_, _L_);
  expected.push_factor(_X_, _E_);

  // create actual factor graph
  SymbolicFactorGraph actual(asiaBayesTree);

  CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST(SymbolicFactorGraph, push_back) {
  // Create two factor graphs and expected combined graph
  SymbolicFactorGraph fg1, fg2, expected;

  fg1.push_factor(1);
  fg1.push_factor(0, 1);

  fg2.push_factor(1, 2);
  fg2.push_factor(0, 2);

  expected.push_factor(1);
  expected.push_factor(0, 1);
  expected.push_factor(1, 2);
  expected.push_factor(0, 2);

  // combine
  SymbolicFactorGraph actual;
  actual.push_back(fg1);
  actual.push_back(fg2);
  CHECK(assert_equal(expected, actual));

  // combine in second way
  SymbolicFactorGraph actual2 = fg1;
  actual2.push_back(fg2);
  CHECK(assert_equal(expected, actual2));
}

/* ************************************************************************* */
TEST(SymbolicFactorGraph, add_factors) {
  SymbolicFactorGraph fg1;
  fg1.push_factor(10);
  fg1.push_back(SymbolicFactor::shared_ptr());  // empty slot!
  fg1.push_factor(11);

  SymbolicFactorGraph fg2;
  fg2.push_factor(1);
  fg2.push_factor(2);

  SymbolicFactorGraph expected;
  expected.push_factor(10);
  expected.push_factor(1);
  expected.push_factor(11);
  expected.push_factor(2);
  const FactorIndices expectedIndices{1, 3};
  const FactorIndices actualIndices = fg1.add_factors(fg2, true);

  EXPECT(assert_equal(expected, fg1));
  EXPECT(assert_container_equality(expectedIndices, actualIndices));

  expected.push_factor(1);
  expected.push_factor(2);
  const FactorIndices expectedIndices2{4, 5};
  const FactorIndices actualIndices2 = fg1.add_factors(fg2, false);

  EXPECT(assert_equal(expected, fg1));
  EXPECT(assert_container_equality(expectedIndices2, actualIndices2));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

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

#include <gtsam/symbolic/SymbolicFactorGraph.h>

#include <gtsam/base/TestableAssertions.h>
#include <gtsam/symbolic/SymbolicBayesNet.h>
#include <gtsam/symbolic/SymbolicBayesTree.h>
#include <gtsam/symbolic/SymbolicConditional.h>
#include <gtsam/symbolic/tests/symbolicExampleGraphs.h>

#include <CppUnitLite/TestHarness.h>

#include <boost/assign/std/set.hpp>

using namespace std;
using namespace gtsam;
using namespace boost::assign;

/* ************************************************************************* */
TEST(SymbolicFactorGraph, keys1) {
  KeySet expected;
  expected += 0, 1, 2, 3, 4;
  KeySet actual = simpleTestGraph1.keys();
  EXPECT(expected == actual);
}

/* ************************************************************************* */
TEST(SymbolicFactorGraph, keys2) {
  KeySet expected;
  expected += 0, 1, 2, 3, 4, 5;
  KeySet actual = simpleTestGraph2.keys();
  EXPECT(expected == actual);
}

/* ************************************************************************* */
TEST(SymbolicFactorGraph, eliminateFullSequential) {
  // Test with simpleTestGraph1
  Ordering order;
  order += 0, 1, 2, 3, 4;
  SymbolicBayesNet actual1 = *simpleTestGraph1.eliminateSequential(order);
  EXPECT(assert_equal(simpleTestGraph1BayesNet, actual1));

  // Test with Asia graph
  SymbolicBayesNet actual2 = *asiaGraph.eliminateSequential(asiaOrdering);
  EXPECT(assert_equal(asiaBayesNet, actual2));
}

/* ************************************************************************* */
TEST(SymbolicFactorGraph, eliminatePartialSequential) {
  // Eliminate 0 and 1
  const Ordering order = list_of(0)(1);

  const SymbolicBayesNet expectedBayesNet =
      list_of(SymbolicConditional(0, 1, 2))(SymbolicConditional(1, 2, 3, 4));

  const SymbolicFactorGraph expectedSfg = list_of(SymbolicFactor(2, 3))(
      SymbolicFactor(4, 5))(SymbolicFactor(2, 3, 4));

  SymbolicBayesNet::shared_ptr actualBayesNet;
  SymbolicFactorGraph::shared_ptr actualSfg;
  boost::tie(actualBayesNet, actualSfg) =
      simpleTestGraph2.eliminatePartialSequential(Ordering(list_of(0)(1)));

  EXPECT(assert_equal(expectedSfg, *actualSfg));
  EXPECT(assert_equal(expectedBayesNet, *actualBayesNet));

  SymbolicBayesNet::shared_ptr actualBayesNet2;
  SymbolicFactorGraph::shared_ptr actualSfg2;
  boost::tie(actualBayesNet2, actualSfg2) =
      simpleTestGraph2.eliminatePartialSequential(
          list_of(0)(1).convert_to_container<KeyVector>());

  EXPECT(assert_equal(expectedSfg, *actualSfg2));
  EXPECT(assert_equal(expectedBayesNet, *actualBayesNet2));
}

/* ************************************************************************* */
TEST(SymbolicFactorGraph, eliminateFullMultifrontal) {
  Ordering ordering;
  ordering += 0, 1, 2, 3;
  SymbolicBayesTree actual1 = *simpleChain.eliminateMultifrontal(ordering);
  EXPECT(assert_equal(simpleChainBayesTree, actual1));

  SymbolicBayesTree actual2 = *asiaGraph.eliminateMultifrontal(asiaOrdering);
  EXPECT(assert_equal(asiaBayesTree, actual2));
}

/* ************************************************************************* */
TEST(SymbolicFactorGraph, eliminatePartialMultifrontal) {
  SymbolicBayesTree expectedBayesTree;
  SymbolicConditional::shared_ptr root =
      boost::make_shared<SymbolicConditional>(
          SymbolicConditional::FromKeys(list_of(4)(5)(1), 2));
  expectedBayesTree.insertRoot(
      boost::make_shared<SymbolicBayesTreeClique>(root));

  SymbolicFactorGraph expectedFactorGraph =
      list_of(SymbolicFactor(0, 1))(SymbolicFactor(0, 2))(SymbolicFactor(1, 3))(
          SymbolicFactor(2, 3))(SymbolicFactor(1));

  SymbolicBayesTree::shared_ptr actualBayesTree;
  SymbolicFactorGraph::shared_ptr actualFactorGraph;
  boost::tie(actualBayesTree, actualFactorGraph) =
      simpleTestGraph2.eliminatePartialMultifrontal(Ordering(list_of(4)(5)));

  EXPECT(assert_equal(expectedFactorGraph, *actualFactorGraph));
  EXPECT(assert_equal(expectedBayesTree, *actualBayesTree));

  SymbolicBayesTree expectedBayesTree2;
  SymbolicBayesTreeClique::shared_ptr root2 =
      boost::make_shared<SymbolicBayesTreeClique>(
          boost::make_shared<SymbolicConditional>(4, 1));
  root2->children.push_back(boost::make_shared<SymbolicBayesTreeClique>(
      boost::make_shared<SymbolicConditional>(5, 4)));
  expectedBayesTree2.insertRoot(root2);

  SymbolicBayesTree::shared_ptr actualBayesTree2;
  SymbolicFactorGraph::shared_ptr actualFactorGraph2;
  boost::tie(actualBayesTree2, actualFactorGraph2) =
      simpleTestGraph2.eliminatePartialMultifrontal(
          list_of<Key>(4)(5).convert_to_container<KeyVector>());

  EXPECT(assert_equal(expectedFactorGraph, *actualFactorGraph2));
  EXPECT(assert_equal(expectedBayesTree2, *actualBayesTree2));
}

/* ************************************************************************* */
TEST(SymbolicFactorGraph, marginalMultifrontalBayesNet) {
  SymbolicBayesNet expectedBayesNet =
      list_of(SymbolicConditional(0, 1, 2))(SymbolicConditional(1, 2, 3))(
          SymbolicConditional(2, 3))(SymbolicConditional(3));

  SymbolicBayesNet actual1 = *simpleTestGraph2.marginalMultifrontalBayesNet(
      Ordering(list_of(0)(1)(2)(3)));
  EXPECT(assert_equal(expectedBayesNet, actual1));
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
  expected.push_back(boost::make_shared<SymbolicConditional>(0, 1, 2));
  expected.push_back(boost::make_shared<SymbolicConditional>(1, 2));
  expected.push_back(boost::make_shared<SymbolicConditional>(2));
  expected.push_back(boost::make_shared<SymbolicConditional>(3, 4));
  expected.push_back(boost::make_shared<SymbolicConditional>(4));

  Ordering order;
  order += 0, 1, 2, 3, 4;
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
  Ordering ord(list_of(3)(4)(2)(1)(0));
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
    Ordering ord(list_of(0)(4)(3));
    auto actual = fg.eliminatePartialSequential(ord);
    SymbolicBayesNet expectedBN;
    expectedBN.emplace_shared<SymbolicConditional>(0, 1, 2);
    expectedBN.emplace_shared<SymbolicConditional>(4, 1, 2, 3);
    expectedBN.emplace_shared<SymbolicConditional>(3, 1, 2);
    EXPECT(assert_equal(expectedBN, *(actual.first)));
  }

  {
    // jointBayesNet
    Ordering ord(list_of(0)(2)(3));
    auto actual = fg.eliminatePartialSequential(ord);
    SymbolicBayesNet expectedBN;
    expectedBN.emplace_shared<SymbolicConditional>(0, 1, 2);
    expectedBN.emplace_shared<SymbolicConditional>(2, 1, 4);
    expectedBN.emplace_shared<SymbolicConditional>(3, 4);
    EXPECT(assert_equal(expectedBN, *(actual.first)));
  }

  {
    // conditionalBayesNet
    Ordering ord(list_of(0)(2));
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
  bayesNet += SymbolicConditional(0, 1, 2);
  bayesNet += SymbolicConditional(1, 2);
  bayesNet += SymbolicConditional(1);

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
  fg1 += SymbolicFactor::shared_ptr();  // empty slot!
  fg1.push_factor(11);

  SymbolicFactorGraph fg2;
  fg2.push_factor(1);
  fg2.push_factor(2);

  SymbolicFactorGraph expected;
  expected.push_factor(10);
  expected.push_factor(1);
  expected.push_factor(11);
  expected.push_factor(2);
  const FactorIndices expectedIndices = list_of(1)(3);
  const FactorIndices actualIndices = fg1.add_factors(fg2, true);

  EXPECT(assert_equal(expected, fg1));
  EXPECT(assert_container_equality(expectedIndices, actualIndices));

  expected.push_factor(1);
  expected.push_factor(2);
  const FactorIndices expectedIndices2 = list_of(4)(5);
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

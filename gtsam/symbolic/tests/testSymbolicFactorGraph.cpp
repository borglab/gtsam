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

#include <gtsam/symbolic/SymbolicFactorGraphUnordered.h>
#include <gtsam/symbolic/SymbolicBayesNetUnordered.h>
#include <gtsam/symbolic/SymbolicBayesTreeUnordered.h>
#include <gtsam/symbolic/SymbolicConditionalUnordered.h>
#include <gtsam/symbolic/tests/symbolicExampleGraphs.h>
#include <boost/assign/std/vector.hpp>

using namespace std;
using namespace gtsam;
using namespace boost::assign;

/* ************************************************************************* */
TEST(SymbolicFactorGraph, eliminateFullSequential)
{
  // Test with simpleTestGraph1
  OrderingUnordered order;
  order += 0,1,2,3,4;
  SymbolicBayesNetUnordered actual1 = *simpleTestGraph1.eliminateSequential(order);
  EXPECT(assert_equal(simpleTestGraph1BayesNet, actual1));

  // Test with Asia graph
  SymbolicBayesNetUnordered actual2 = *asiaGraph.eliminateSequential(asiaOrdering);
  EXPECT(assert_equal(asiaBayesNet, actual2));
}

/* ************************************************************************* */
TEST(SymbolicFactorGraph, eliminatePartialSequential)
{
  // Eliminate 0 and 1
  const OrderingUnordered order = list_of(0)(1);

  const SymbolicBayesNetUnordered expectedBayesNet = list_of
    (boost::make_shared<SymbolicConditionalUnordered>(0,1,2))
    (boost::make_shared<SymbolicConditionalUnordered>(1,2,3,4));

  const SymbolicFactorGraphUnordered expectedSfg = list_of
    (boost::make_shared<SymbolicFactorUnordered>(2,3))
    (boost::make_shared<SymbolicFactorUnordered>(4,5))
    (boost::make_shared<SymbolicFactorUnordered>(2,3,4));

  SymbolicBayesNetUnordered::shared_ptr actualBayesNet;
  SymbolicFactorGraphUnordered::shared_ptr actualSfg;
  boost::tie(actualBayesNet, actualSfg) =
    simpleTestGraph2.eliminatePartialSequential(OrderingUnordered(list_of(0)(1)));

  EXPECT(assert_equal(expectedSfg, *actualSfg));
  EXPECT(assert_equal(expectedBayesNet, *actualBayesNet));
}

/* ************************************************************************* */
TEST(SymbolicFactorGraph, eliminateFullMultifrontal)
{
  OrderingUnordered ordering; ordering += 0,1,2,3;
  SymbolicBayesTreeUnordered actual1 =
    *simpleChain.eliminateMultifrontal(ordering);
  EXPECT(assert_equal(simpleChainBayesTree, actual1));

  SymbolicBayesTreeUnordered actual2 =
    *asiaGraph.eliminateMultifrontal(asiaOrdering);
  EXPECT(assert_equal(asiaBayesTree, actual2));
}

/* ************************************************************************* */
TEST(SymbolicFactorGraph, eliminatePartialMultifrontal)
{
  SymbolicBayesTreeUnordered expectedBayesTree;
  SymbolicConditionalUnordered::shared_ptr root = boost::make_shared<SymbolicConditionalUnordered>(
    SymbolicConditionalUnordered::FromKeys(list_of(5)(4)(1), 2));
  expectedBayesTree.insertRoot(boost::make_shared<SymbolicBayesTreeCliqueUnordered>(root));

  SymbolicFactorGraphUnordered expectedFactorGraph = list_of
    (boost::make_shared<SymbolicFactorUnordered>(0,1))
    (boost::make_shared<SymbolicFactorUnordered>(0,2))
    (boost::make_shared<SymbolicFactorUnordered>(1,3))
    (boost::make_shared<SymbolicFactorUnordered>(2,3))
    (boost::make_shared<SymbolicFactorUnordered>(1));

  SymbolicBayesTreeUnordered::shared_ptr actualBayesTree;
  SymbolicFactorGraphUnordered::shared_ptr actualFactorGraph;
  boost::tie(actualBayesTree, actualFactorGraph) =
    simpleTestGraph2.eliminatePartialMultifrontal(OrderingUnordered(list_of(4)(5)));

  EXPECT(assert_equal(expectedFactorGraph, *actualFactorGraph));
  EXPECT(assert_equal(expectedBayesTree, *actualBayesTree));
}

/* ************************************************************************* */
TEST(SymbolicFactorGraph, eliminate_disconnected_graph) {
  SymbolicFactorGraphUnordered fg;
  fg.push_factor(0, 1);
  fg.push_factor(0, 2);
  fg.push_factor(1, 2);
  fg.push_factor(3, 4);

  // create expected Chordal bayes Net
  SymbolicBayesNetUnordered expected;
  expected.push_back(boost::make_shared<SymbolicConditionalUnordered>(0,1,2));
  expected.push_back(boost::make_shared<SymbolicConditionalUnordered>(1,2));
  expected.push_back(boost::make_shared<SymbolicConditionalUnordered>(2));
  expected.push_back(boost::make_shared<SymbolicConditionalUnordered>(3,4));
  expected.push_back(boost::make_shared<SymbolicConditionalUnordered>(4));

  OrderingUnordered order;
  order += 0,1,2,3,4;
  SymbolicBayesNetUnordered actual = *fg.eliminateSequential(order);

  EXPECT(assert_equal(expected,actual));
}

/* ************************************************************************* */
//TEST(SymbolicFactorGraph, marginals)
//{
//  // Create factor graph
//  SymbolicFactorGraph fg;
//  fg.push_factor(0, 1);
//  fg.push_factor(0, 2);
//  fg.push_factor(1, 4);
//  fg.push_factor(2, 4);
//  fg.push_factor(3, 4);
//
//  // eliminate
//  SymbolicSequentialSolver solver(fg);
//  SymbolicBayesNet::shared_ptr actual = solver.eliminate();
//  SymbolicBayesNet expected;
//  expected.push_front(boost::make_shared<IndexConditional>(4));
//  expected.push_front(boost::make_shared<IndexConditional>(3, 4));
//  expected.push_front(boost::make_shared<IndexConditional>(2, 4));
//  expected.push_front(boost::make_shared<IndexConditional>(1, 2, 4));
//  expected.push_front(boost::make_shared<IndexConditional>(0, 1, 2));
//  EXPECT(assert_equal(expected,*actual));
//
//  {
//    // jointBayesNet
//    vector<Index> js;
//    js.push_back(0);
//    js.push_back(4);
//    js.push_back(3);
//    SymbolicBayesNet::shared_ptr actualBN = solver.jointBayesNet(js);
//    SymbolicBayesNet expectedBN;
//    expectedBN.push_front(boost::make_shared<IndexConditional>(3));
//    expectedBN.push_front(boost::make_shared<IndexConditional>(4, 3));
//    expectedBN.push_front(boost::make_shared<IndexConditional>(0, 4));
//    EXPECT( assert_equal(expectedBN,*actualBN));
//
//    // jointFactorGraph
//    SymbolicFactorGraph::shared_ptr actualFG = solver.jointFactorGraph(js);
//    SymbolicFactorGraph expectedFG;
//    expectedFG.push_factor(0, 4);
//    expectedFG.push_factor(4, 3);
//    expectedFG.push_factor(3);
//    EXPECT( assert_equal(expectedFG,(SymbolicFactorGraph)(*actualFG)));
//  }
//
//  {
//    // jointBayesNet
//    vector<Index> js;
//    js.push_back(0);
//    js.push_back(2);
//    js.push_back(3);
//    SymbolicBayesNet::shared_ptr actualBN = solver.jointBayesNet(js);
//    SymbolicBayesNet expectedBN;
//    expectedBN.push_front(boost::make_shared<IndexConditional>(2));
//    expectedBN.push_front(boost::make_shared<IndexConditional>(3, 2));
//    expectedBN.push_front(boost::make_shared<IndexConditional>(0, 3, 2));
//    EXPECT( assert_equal(expectedBN,*actualBN));
//
//    // jointFactorGraph
//    SymbolicFactorGraph::shared_ptr actualFG = solver.jointFactorGraph(js);
//    SymbolicFactorGraph expectedFG;
//    expectedFG.push_factor(0, 3, 2);
//    expectedFG.push_factor(3, 2);
//    expectedFG.push_factor(2);
//    EXPECT( assert_equal(expectedFG,(SymbolicFactorGraph)(*actualFG)));
//  }
//
//  {
//    // conditionalBayesNet
//    vector<Index> js;
//    js.push_back(0);
//    js.push_back(2);
//    js.push_back(3);
//    size_t nrFrontals = 2;
//    SymbolicBayesNet::shared_ptr actualBN = //
//      solver.conditionalBayesNet(js, nrFrontals);
//    SymbolicBayesNet expectedBN;
//    expectedBN.push_front(boost::make_shared<IndexConditional>(2, 3));
//    expectedBN.push_front(boost::make_shared<IndexConditional>(0, 2, 3));
//    EXPECT( assert_equal(expectedBN,*actualBN));
//  }
//}

/* ************************************************************************* */
TEST( SymbolicFactorGraph, constructFromBayesNet )
{
  // create expected factor graph
  SymbolicFactorGraphUnordered expected;
  expected.push_factor(0, 1, 2);
  expected.push_factor(1, 2);
  expected.push_factor(1);

  // create Bayes Net
  SymbolicBayesNetUnordered bayesNet;
  bayesNet.add(SymbolicConditionalUnordered(0, 1, 2));
  bayesNet.add(SymbolicConditionalUnordered(1, 2));
  bayesNet.add(SymbolicConditionalUnordered(1));

  // create actual factor graph from a Bayes Net
  SymbolicFactorGraphUnordered actual(bayesNet);

  CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST( SymbolicFactorGraph, constructFromBayesTree )
{
  // create expected factor graph
  SymbolicFactorGraphUnordered expected;
  expected.push_factor(_B_, _L_, _E_, _S_);
  expected.push_factor(_T_, _E_, _L_);
  expected.push_factor(_X_, _E_);

  // create actual factor graph
  SymbolicFactorGraphUnordered actual(asiaBayesTree);

  CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST( SymbolicFactorGraph, push_back )
{
  // Create two factor graphs and expected combined graph
  SymbolicFactorGraphUnordered fg1, fg2, expected;

  fg1.push_factor(1);
  fg1.push_factor(0, 1);

  fg2.push_factor(1, 2);
  fg2.push_factor(0, 2);

  expected.push_factor(1);
  expected.push_factor(0, 1);
  expected.push_factor(1, 2);
  expected.push_factor(0, 2);

  // combine
  SymbolicFactorGraphUnordered actual;
  actual.push_back(fg1);
  actual.push_back(fg2);
  CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */
int main() {  TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */

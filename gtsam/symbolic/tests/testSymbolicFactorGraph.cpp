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
  SymbolicBayesNetUnordered actual1 = *simpleTestGraph1.eliminateSequential(EliminateSymbolicUnordered, order);
  EXPECT(assert_equal(simpleTestGraph1BayesNet, actual1));

  // Test with Asia graph
  SymbolicBayesNetUnordered actual2 = *asiaGraph.eliminateSequential(
    EliminateSymbolicUnordered, asiaOrdering);
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
  boost::tie(actualBayesNet, actualSfg) = simpleTestGraph2.eliminatePartialSequential(
    EliminateSymbolicUnordered, OrderingUnordered(list_of(0)(1)));

  EXPECT(assert_equal(expectedSfg, *actualSfg));
  EXPECT(assert_equal(expectedBayesNet, *actualBayesNet));
}

/* ************************************************************************* */
TEST(SymbolicFactorGraph, eliminateFullMultifrontal)
{
  const Index x2=0, x1=1, x3=2, x4=3;
  SymbolicFactorGraphUnordered fg;
  fg.push_factor(x2,x1);
  fg.push_factor(x2,x3);
  fg.push_factor(x3,x4);

  EXPECT(false);

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
  boost::tie(actualBayesTree, actualFactorGraph) = simpleTestGraph2.eliminatePartialMultifrontal(
    EliminateSymbolicUnordered, OrderingUnordered(list_of(4)(5)));

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
  SymbolicBayesNetUnordered actual = *fg.eliminateSequential(EliminateSymbolicUnordered, order);

  EXPECT(assert_equal(expected,actual));
}

/* ************************************************************************* */
//TEST(SymbolicFactorGraph, eliminateFrontals) {
//
//  SymbolicFactorGraph sfgOrig;
//  sfgOrig.push_factor(0,1);
//  sfgOrig.push_factor(0,2);
//  sfgOrig.push_factor(1,3);
//  sfgOrig.push_factor(1,4);
//  sfgOrig.push_factor(2,3);
//  sfgOrig.push_factor(4,5);
//
//  SymbolicConditionalUnordered::shared_ptr actualCond;
//  SymbolicFactorGraph::shared_ptr actualSfg;
//  boost::tie(actualCond, actualSfg) = sfgOrig.eliminateFrontals(2);
//
//  vector<Index> condIndices;
//  condIndices += 0,1,2,3,4;
//  IndexConditional expectedCond(condIndices, 2);
//
//  SymbolicFactorGraph expectedSfg;
//  expectedSfg.push_factor(2,3);
//  expectedSfg.push_factor(4,5);
//  expectedSfg.push_factor(2,3,4);
//
//  EXPECT(assert_equal(expectedSfg, actualSfg));
//  EXPECT(assert_equal(expectedCond, *actualCond));
//}

///* ************************************************************************* */
//TEST( SymbolicFactorGraph, EliminateOne )
//{
//  // create a test graph
//  SymbolicFactorGraph fg;
//  fg.push_factor(vx2, vx1);
//
//  SymbolicSequentialSolver::EliminateUntil(fg, vx2+1);
//  SymbolicFactorGraph expected;
//  expected.push_back(boost::shared_ptr<IndexFactor>());
//  expected.push_factor(vx1);
//
//  CHECK(assert_equal(expected, fg));
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

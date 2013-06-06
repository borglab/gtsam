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

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST(FactorGraph, eliminateFrontals) {

  SymbolicFactorGraph sfgOrig;
  sfgOrig.push_factor(0,1);
  sfgOrig.push_factor(0,2);
  sfgOrig.push_factor(1,3);
  sfgOrig.push_factor(1,4);
  sfgOrig.push_factor(2,3);
  sfgOrig.push_factor(4,5);

  IndexConditional::shared_ptr actualCond;
  SymbolicFactorGraph actualSfg;
  boost::tie(actualCond, actualSfg) = sfgOrig.eliminateFrontals(2);

  vector<Index> condIndices;
  condIndices += 0,1,2,3,4;
  IndexConditional expectedCond(condIndices, 2);

  SymbolicFactorGraph expectedSfg;
  expectedSfg.push_factor(2,3);
  expectedSfg.push_factor(4,5);
  expectedSfg.push_factor(2,3,4);

  EXPECT(assert_equal(expectedSfg, actualSfg));
  EXPECT(assert_equal(expectedCond, *actualCond));
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

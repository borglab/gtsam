/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testSymbolicFactorGraph.cpp
 * @brief   Unit tests for a symbolic IndexFactor Graph
 * @author  Frank Dellaert
 */

#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>

#include <gtsam/inference/SymbolicFactorGraph.h>
#include <gtsam/inference/BayesNet-inl.h>
#include <gtsam/inference/IndexFactor.h>
#include <gtsam/inference/FactorGraph.h>
#include <gtsam/inference/SymbolicSequentialSolver.h>

using namespace std;
using namespace gtsam;

static const Index vx2 = 0;
static const Index vx1 = 1;
static const Index vl1 = 2;

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
  SymbolicFactorGraph expected;
  expected.push_factor(vx2, vx1, vl1);
  expected.push_factor(vx1, vl1);
  expected.push_factor(vx1);

  // create Bayes Net
  IndexConditional::shared_ptr x2(new IndexConditional(vx2, vx1, vl1));
  IndexConditional::shared_ptr l1(new IndexConditional(vx1, vl1));
  IndexConditional::shared_ptr x1(new IndexConditional(vx1));

  BayesNet<IndexConditional> bayesNet;
  bayesNet.push_back(x2);
  bayesNet.push_back(l1);
  bayesNet.push_back(x1);

  // create actual factor graph from a Bayes Net
  SymbolicFactorGraph actual(bayesNet);

  CHECK(assert_equal((SymbolicFactorGraph)expected,actual));
}

/* ************************************************************************* */
TEST( SymbolicFactorGraph, push_back )
{
  // Create two factor graphs and expected combined graph
  SymbolicFactorGraph fg1, fg2, expected;

  fg1.push_factor(vx1);
  fg1.push_factor(vx2, vx1);

  fg2.push_factor(vx1, vl1);
  fg2.push_factor(vx2, vl1);

  expected.push_factor(vx1);
  expected.push_factor(vx2, vx1);
  expected.push_factor(vx1, vl1);
  expected.push_factor(vx2, vl1);

  // combine
  SymbolicFactorGraph actual = combine(fg1, fg2);
  CHECK(assert_equal(expected, actual));

  // combine using push_back
  fg1.push_back(fg2);
  CHECK(assert_equal(expected, fg1));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

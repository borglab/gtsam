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

#include <gtsam/inference/SymbolicFactorGraphOrdered.h>
#include <gtsam/inference/BayesNetOrdered-inl.h>
#include <gtsam/inference/IndexFactorOrdered.h>
#include <gtsam/inference/FactorGraphOrdered.h>
#include <gtsam/inference/SymbolicSequentialSolverOrdered.h>

using namespace std;
using namespace gtsam;

static const Index vx2 = 0;
static const Index vx1 = 1;
static const Index vl1 = 2;

///* ************************************************************************* */
//TEST( SymbolicFactorGraphOrdered, EliminateOne )
//{
//  // create a test graph
//  SymbolicFactorGraphOrdered fg;
//  fg.push_factor(vx2, vx1);
//
//  SymbolicSequentialSolver::EliminateUntil(fg, vx2+1);
//  SymbolicFactorGraphOrdered expected;
//  expected.push_back(boost::shared_ptr<IndexFactor>());
//  expected.push_factor(vx1);
//
//  CHECK(assert_equal(expected, fg));
//}

/* ************************************************************************* */
TEST( SymbolicFactorGraphOrdered, constructFromBayesNet )
{
  // create expected factor graph
  SymbolicFactorGraphOrdered expected;
  expected.push_factor(vx2, vx1, vl1);
  expected.push_factor(vx1, vl1);
  expected.push_factor(vx1);

  // create Bayes Net
  IndexConditionalOrdered::shared_ptr x2(new IndexConditionalOrdered(vx2, vx1, vl1));
  IndexConditionalOrdered::shared_ptr l1(new IndexConditionalOrdered(vx1, vl1));
  IndexConditionalOrdered::shared_ptr x1(new IndexConditionalOrdered(vx1));

  BayesNetOrdered<IndexConditionalOrdered> bayesNet;
  bayesNet.push_back(x2);
  bayesNet.push_back(l1);
  bayesNet.push_back(x1);

  // create actual factor graph from a Bayes Net
  SymbolicFactorGraphOrdered actual(bayesNet);

  CHECK(assert_equal((SymbolicFactorGraphOrdered)expected,actual));
}

/* ************************************************************************* */
TEST( SymbolicFactorGraphOrdered, push_back )
{
  // Create two factor graphs and expected combined graph
  SymbolicFactorGraphOrdered fg1, fg2, expected;

  fg1.push_factor(vx1);
  fg1.push_factor(vx2, vx1);

  fg2.push_factor(vx1, vl1);
  fg2.push_factor(vx2, vl1);

  expected.push_factor(vx1);
  expected.push_factor(vx2, vx1);
  expected.push_factor(vx1, vl1);
  expected.push_factor(vx2, vl1);

  // combine
  SymbolicFactorGraphOrdered actual = combine(fg1, fg2);
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

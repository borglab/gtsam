/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testJunctionTree.cpp
 * @brief   Unit tests for Junction Tree
 * @author  Kai Ni
 * @author  Frank Dellaert
 */

#include <boost/assign/std/list.hpp> // for operator +=
#include <boost/assign/std/vector.hpp> // for operator +=
#include <boost/assign/std/set.hpp> // for operator +=
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>

#include <gtsam/inference/SymbolicFactorGraphOrdered.h>
#include <gtsam/inference/JunctionTreeOrdered.h>
#include <gtsam/inference/ClusterTreeOrdered.h>
#include <gtsam/inference/JunctionTreeOrdered.h>
#include <gtsam/inference/IndexFactorOrdered.h>
#include <gtsam/inference/SymbolicSequentialSolverOrdered.h>

using namespace gtsam;
using namespace std;

typedef JunctionTreeOrdered<SymbolicFactorGraphOrdered> SymbolicJunctionTree;

/* ************************************************************************* *
 * x1 - x2 - x3 - x4
 * x3 x4
 *    x2 x1 : x3
 ****************************************************************************/
TEST( JunctionTreeOrdered, constructor )
{
  const Index x2=0, x1=1, x3=2, x4=3;
  SymbolicFactorGraphOrdered fg;
  fg.push_factor(x2,x1);
  fg.push_factor(x2,x3);
  fg.push_factor(x3,x4);

  SymbolicJunctionTree actual(fg);

  vector<Index> frontal1; frontal1 += x3, x4;
  vector<Index> frontal2; frontal2 += x2, x1;
  vector<Index> sep1;
  vector<Index> sep2; sep2 += x3;
  CHECK(assert_equal(frontal1, actual.root()->frontal));
  CHECK(assert_equal(sep1,     actual.root()->separator));
  LONGS_EQUAL(1,               actual.root()->size());
  CHECK(assert_equal(frontal2, actual.root()->children().front()->frontal));
  CHECK(assert_equal(sep2,     actual.root()->children().front()->separator));
  LONGS_EQUAL(2,               actual.root()->children().front()->size());
  CHECK(assert_equal(*fg[2], *(*actual.root())[0]));
  CHECK(assert_equal(*fg[0], *(*actual.root()->children().front())[0]));
  CHECK(assert_equal(*fg[1], *(*actual.root()->children().front())[1]));
}

/* ************************************************************************* *
 * x1 - x2 - x3 - x4
 * x3 x4
 *    x2 x1 : x3
 ****************************************************************************/
TEST( JunctionTreeOrdered, eliminate)
{
  const Index x2=0, x1=1, x3=2, x4=3;
  SymbolicFactorGraphOrdered fg;
  fg.push_factor(x2,x1);
  fg.push_factor(x2,x3);
  fg.push_factor(x3,x4);

  SymbolicJunctionTree jt(fg);
  SymbolicBayesTreeOrdered::sharedClique actual = jt.eliminate(&EliminateSymbolic);

  BayesNetOrdered<IndexConditionalOrdered> bn(*SymbolicSequentialSolver(fg).eliminate());
  SymbolicBayesTreeOrdered expected(bn);

//  cout << "BT from JT:\n";
//  actual->printTree("");

  CHECK(assert_equal(*expected.root(), *actual));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file testSymbolicBayesTree.cpp
 * @date sept 15, 2012
 * @author Frank Dellaert
 */

#include <gtsam/inference/SymbolicSequentialSolverOrdered.h>
#include <gtsam/inference/SymbolicFactorGraphOrdered.h>
#include <gtsam/inference/BayesTreeOrdered.h>

#include <boost/assign/list_of.hpp>
#include <boost/assign/std/vector.hpp>
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

static bool debug = false;

/* ************************************************************************* */

TEST_UNSAFE( SymbolicBayesTreeOrdered, thinTree ) {

  // create a thin-tree Bayesnet, a la Jean-Guillaume
  SymbolicBayesNetOrdered bayesNet;
  bayesNet.push_front(boost::make_shared<IndexConditionalOrdered>(14));

  bayesNet.push_front(boost::make_shared<IndexConditionalOrdered>(13, 14));
  bayesNet.push_front(boost::make_shared<IndexConditionalOrdered>(12, 14));

  bayesNet.push_front(boost::make_shared<IndexConditionalOrdered>(11, 13, 14));
  bayesNet.push_front(boost::make_shared<IndexConditionalOrdered>(10, 13, 14));
  bayesNet.push_front(boost::make_shared<IndexConditionalOrdered>(9, 12, 14));
  bayesNet.push_front(boost::make_shared<IndexConditionalOrdered>(8, 12, 14));

  bayesNet.push_front(boost::make_shared<IndexConditionalOrdered>(7, 11, 13));
  bayesNet.push_front(boost::make_shared<IndexConditionalOrdered>(6, 11, 13));
  bayesNet.push_front(boost::make_shared<IndexConditionalOrdered>(5, 10, 13));
  bayesNet.push_front(boost::make_shared<IndexConditionalOrdered>(4, 10, 13));

  bayesNet.push_front(boost::make_shared<IndexConditionalOrdered>(3, 9, 12));
  bayesNet.push_front(boost::make_shared<IndexConditionalOrdered>(2, 9, 12));
  bayesNet.push_front(boost::make_shared<IndexConditionalOrdered>(1, 8, 12));
  bayesNet.push_front(boost::make_shared<IndexConditionalOrdered>(0, 8, 12));

  if (debug) {
    GTSAM_PRINT(bayesNet);
    bayesNet.saveGraph("/tmp/symbolicBayesNet.dot");
  }

  // create a BayesTree out of a Bayes net
  SymbolicBayesTreeOrdered bayesTree(bayesNet);
  if (debug) {
    GTSAM_PRINT(bayesTree);
    bayesTree.saveGraph("/tmp/symbolicBayesTree.dot");
  }

  SymbolicBayesTreeOrdered::Clique::shared_ptr R = bayesTree.root();

  {
    // check shortcut P(S9||R) to root
    SymbolicBayesTreeOrdered::Clique::shared_ptr c = bayesTree[9];
    SymbolicBayesNetOrdered shortcut = c->shortcut(R, EliminateSymbolic);
    SymbolicBayesNetOrdered expected;
    EXPECT(assert_equal(expected, shortcut));
  }

  {
    // check shortcut P(S8||R) to root
    SymbolicBayesTreeOrdered::Clique::shared_ptr c = bayesTree[8];
    SymbolicBayesNetOrdered shortcut = c->shortcut(R, EliminateSymbolic);
    SymbolicBayesNetOrdered expected;
    expected.push_front(boost::make_shared<IndexConditionalOrdered>(12, 14));
    EXPECT(assert_equal(expected, shortcut));
  }

  {
    // check shortcut P(S4||R) to root
    SymbolicBayesTreeOrdered::Clique::shared_ptr c = bayesTree[4];
    SymbolicBayesNetOrdered shortcut = c->shortcut(R, EliminateSymbolic);
    SymbolicBayesNetOrdered expected;
    expected.push_front(boost::make_shared<IndexConditionalOrdered>(10, 13, 14));
    EXPECT(assert_equal(expected, shortcut));
  }

  {
    // check shortcut P(S2||R) to root
    SymbolicBayesTreeOrdered::Clique::shared_ptr c = bayesTree[2];
    SymbolicBayesNetOrdered shortcut = c->shortcut(R, EliminateSymbolic);
    SymbolicBayesNetOrdered expected;
    expected.push_front(boost::make_shared<IndexConditionalOrdered>(12, 14));
    expected.push_front(boost::make_shared<IndexConditionalOrdered>(9, 12, 14));
    EXPECT(assert_equal(expected, shortcut));
  }

  {
    // check shortcut P(S0||R) to root
    SymbolicBayesTreeOrdered::Clique::shared_ptr c = bayesTree[0];
    SymbolicBayesNetOrdered shortcut = c->shortcut(R, EliminateSymbolic);
    SymbolicBayesNetOrdered expected;
    expected.push_front(boost::make_shared<IndexConditionalOrdered>(12, 14));
    expected.push_front(boost::make_shared<IndexConditionalOrdered>(8, 12, 14));
    EXPECT(assert_equal(expected, shortcut));
  }

  SymbolicBayesNetOrdered::shared_ptr actualJoint;

  // Check joint P(8,2)
  if (false) { // TODO, not disjoint
    actualJoint = bayesTree.jointBayesNet(8, 2, EliminateSymbolic);
    SymbolicBayesNetOrdered expected;
    expected.push_front(boost::make_shared<IndexConditionalOrdered>(8));
    expected.push_front(boost::make_shared<IndexConditionalOrdered>(2, 8));
    EXPECT(assert_equal(expected, *actualJoint));
  }

  // Check joint P(1,2)
  if (false) { // TODO, not disjoint
    actualJoint = bayesTree.jointBayesNet(1, 2, EliminateSymbolic);
    SymbolicBayesNetOrdered expected;
    expected.push_front(boost::make_shared<IndexConditionalOrdered>(2));
    expected.push_front(boost::make_shared<IndexConditionalOrdered>(1, 2));
    EXPECT(assert_equal(expected, *actualJoint));
  }

  // Check joint P(2,6)
  if (true) {
    actualJoint = bayesTree.jointBayesNet(2, 6, EliminateSymbolic);
    SymbolicBayesNetOrdered expected;
    expected.push_front(boost::make_shared<IndexConditionalOrdered>(6));
    expected.push_front(boost::make_shared<IndexConditionalOrdered>(2, 6));
    EXPECT(assert_equal(expected, *actualJoint));
  }

  // Check joint P(4,6)
  if (false) { // TODO, not disjoint
    actualJoint = bayesTree.jointBayesNet(4, 6, EliminateSymbolic);
    SymbolicBayesNetOrdered expected;
    expected.push_front(boost::make_shared<IndexConditionalOrdered>(6));
    expected.push_front(boost::make_shared<IndexConditionalOrdered>(4, 6));
    EXPECT(assert_equal(expected, *actualJoint));
  }
}

/* ************************************************************************* *
 Bayes tree for smoother with "natural" ordering:
 C1 5 6
 C2   4 : 5
 C3     3 : 4
 C4       2 : 3
 C5         1 : 2
 C6           0 : 1
 **************************************************************************** */

TEST_UNSAFE( SymbolicBayesTreeOrdered, linear_smoother_shortcuts ) {
  // Create smoother with 7 nodes
  SymbolicFactorGraphOrdered smoother;
  smoother.push_factor(0);
  smoother.push_factor(0, 1);
  smoother.push_factor(1, 2);
  smoother.push_factor(2, 3);
  smoother.push_factor(3, 4);
  smoother.push_factor(4, 5);
  smoother.push_factor(5, 6);

  BayesNetOrdered<IndexConditionalOrdered> bayesNet =
      *SymbolicSequentialSolver(smoother).eliminate();

  if (debug) {
    GTSAM_PRINT(bayesNet);
    bayesNet.saveGraph("/tmp/symbolicBayesNet.dot");
  }

  // create a BayesTree out of a Bayes net
  SymbolicBayesTreeOrdered bayesTree(bayesNet);
  if (debug) {
    GTSAM_PRINT(bayesTree);
    bayesTree.saveGraph("/tmp/symbolicBayesTree.dot");
  }

  SymbolicBayesTreeOrdered::Clique::shared_ptr R = bayesTree.root();

  {
    // check shortcut P(S2||R) to root
    SymbolicBayesTreeOrdered::Clique::shared_ptr c = bayesTree[4]; // 4 is frontal in C2
    SymbolicBayesNetOrdered shortcut = c->shortcut(R, EliminateSymbolic);
    SymbolicBayesNetOrdered expected;
    EXPECT(assert_equal(expected, shortcut));
  }

  {
    // check shortcut P(S3||R) to root
    SymbolicBayesTreeOrdered::Clique::shared_ptr c = bayesTree[3]; // 3 is frontal in C3
    SymbolicBayesNetOrdered shortcut = c->shortcut(R, EliminateSymbolic);
    SymbolicBayesNetOrdered expected;
    expected.push_front(boost::make_shared<IndexConditionalOrdered>(4, 5));
    EXPECT(assert_equal(expected, shortcut));
  }

  {
    // check shortcut P(S4||R) to root
    SymbolicBayesTreeOrdered::Clique::shared_ptr c = bayesTree[2]; // 2 is frontal in C4
    SymbolicBayesNetOrdered shortcut = c->shortcut(R, EliminateSymbolic);
    SymbolicBayesNetOrdered expected;
    expected.push_front(boost::make_shared<IndexConditionalOrdered>(3, 5));
    EXPECT(assert_equal(expected, shortcut));
  }
}

/* ************************************************************************* */
// from testSymbolicJunctionTree, which failed at one point
TEST(SymbolicBayesTreeOrdered, complicatedMarginal) {

  // Create the conditionals to go in the BayesTree
  list<Index> L;
  L = list_of(1)(2)(5);
  IndexConditionalOrdered::shared_ptr R_1_2(new IndexConditionalOrdered(L, 2));
  L = list_of(3)(4)(6);
  IndexConditionalOrdered::shared_ptr R_3_4(new IndexConditionalOrdered(L, 2));
  L = list_of(5)(6)(7)(8);
  IndexConditionalOrdered::shared_ptr R_5_6(new IndexConditionalOrdered(L, 2));
  L = list_of(7)(8)(11);
  IndexConditionalOrdered::shared_ptr R_7_8(new IndexConditionalOrdered(L, 2));
  L = list_of(9)(10)(11)(12);
  IndexConditionalOrdered::shared_ptr R_9_10(new IndexConditionalOrdered(L, 2));
  L = list_of(11)(12);
  IndexConditionalOrdered::shared_ptr R_11_12(new IndexConditionalOrdered(L, 2));

  // Symbolic Bayes Tree
  typedef SymbolicBayesTreeOrdered::Clique Clique;
  typedef SymbolicBayesTreeOrdered::sharedClique sharedClique;

  // Create Bayes Tree
  SymbolicBayesTreeOrdered bt;
  bt.insert(sharedClique(new Clique(R_11_12)));
  bt.insert(sharedClique(new Clique(R_9_10)));
  bt.insert(sharedClique(new Clique(R_7_8)));
  bt.insert(sharedClique(new Clique(R_5_6)));
  bt.insert(sharedClique(new Clique(R_3_4)));
  bt.insert(sharedClique(new Clique(R_1_2)));
  if (debug) {
    GTSAM_PRINT(bt);
    bt.saveGraph("/tmp/symbolicBayesTree.dot");
  }

  SymbolicBayesTreeOrdered::Clique::shared_ptr R = bt.root();
  SymbolicBayesNetOrdered empty;

  // Shortcut on 9
  {
    SymbolicBayesTreeOrdered::Clique::shared_ptr c = bt[9];
    SymbolicBayesNetOrdered shortcut = c->shortcut(R, EliminateSymbolic);
    EXPECT(assert_equal(empty, shortcut));
  }

  // Shortcut on 7
  {
    SymbolicBayesTreeOrdered::Clique::shared_ptr c = bt[7];
    SymbolicBayesNetOrdered shortcut = c->shortcut(R, EliminateSymbolic);
    EXPECT(assert_equal(empty, shortcut));
  }

  // Shortcut on 5
  {
    SymbolicBayesTreeOrdered::Clique::shared_ptr c = bt[5];
    SymbolicBayesNetOrdered shortcut = c->shortcut(R, EliminateSymbolic);
    SymbolicBayesNetOrdered expected;
    expected.push_front(boost::make_shared<IndexConditionalOrdered>(8, 11));
    expected.push_front(boost::make_shared<IndexConditionalOrdered>(7, 8, 11));
    EXPECT(assert_equal(expected, shortcut));
  }

  // Shortcut on 3
  {
    SymbolicBayesTreeOrdered::Clique::shared_ptr c = bt[3];
    SymbolicBayesNetOrdered shortcut = c->shortcut(R, EliminateSymbolic);
    SymbolicBayesNetOrdered expected;
    expected.push_front(boost::make_shared<IndexConditionalOrdered>(6, 11));
    EXPECT(assert_equal(expected, shortcut));
  }

  // Shortcut on 1
  {
    SymbolicBayesTreeOrdered::Clique::shared_ptr c = bt[1];
    SymbolicBayesNetOrdered shortcut = c->shortcut(R, EliminateSymbolic);
    SymbolicBayesNetOrdered expected;
    expected.push_front(boost::make_shared<IndexConditionalOrdered>(5, 11));
    EXPECT(assert_equal(expected, shortcut));
  }

  // Marginal on 5
  {
    IndexFactorOrdered::shared_ptr actual = bt.marginalFactor(5, EliminateSymbolic);
    EXPECT(assert_equal(IndexFactorOrdered(5), *actual, 1e-1));
  }

  // Shortcut on 6
  {
    IndexFactorOrdered::shared_ptr actual = bt.marginalFactor(6, EliminateSymbolic);
    EXPECT(assert_equal(IndexFactorOrdered(6), *actual, 1e-1));
  }

}
/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */


/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file testSymbolicBayesTreeUnordered.cpp
 * @date sept 15, 2012
 * @author  Frank Dellaert
 * @author  Michael Kaess
 * @author  Viorela Ila
 */

#include <gtsam/symbolic/SymbolicBayesTreeUnordered.h>
#include <gtsam/symbolic/SymbolicBayesNetUnordered.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/symbolic/tests/symbolicExampleGraphs.h>

#include <boost/assign/list_of.hpp>
#include <boost/assign/std/vector.hpp>
#include <boost/assign/std/list.hpp>
#include <boost/range/adaptor/indirected.hpp>
using namespace boost::assign;
using boost::adaptors::indirected;

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>

using namespace std;
using namespace gtsam;
using namespace gtsam::symbol_shorthand;

static bool debug = false;

namespace {

  /* ************************************************************************* */
  // Helper functions for below
  template<typename KEYS>
  SymbolicBayesTreeCliqueUnordered::shared_ptr MakeClique(const KEYS& keys, DenseIndex nrFrontals)
  {
    return boost::make_shared<SymbolicBayesTreeCliqueUnordered>(
      boost::make_shared<SymbolicConditionalUnordered>(
      SymbolicConditionalUnordered::FromKeys(keys, nrFrontals)));
  }

  template<typename KEYS, typename CHILDREN>
  SymbolicBayesTreeCliqueUnordered::shared_ptr MakeClique(
    const KEYS& keys, DenseIndex nrFrontals, const CHILDREN& children)
  {
    SymbolicBayesTreeCliqueUnordered::shared_ptr clique =
      boost::make_shared<SymbolicBayesTreeCliqueUnordered>(
      boost::make_shared<SymbolicConditionalUnordered>(
      SymbolicConditionalUnordered::FromKeys(keys, nrFrontals)));
    clique->children = children;
    BOOST_FOREACH(const SymbolicBayesTreeCliqueUnordered::shared_ptr& child, children)
      child->parent_ = clique;
    return clique;
  }

}

/* ************************************************************************* */
TEST(SymbolicBayesTree, clear)
{
  SymbolicBayesTreeUnordered bayesTree = asiaBayesTree;
  bayesTree.clear();

  SymbolicBayesTreeUnordered expected;

  // Check whether cleared BayesTree is equal to a new BayesTree
  CHECK(assert_equal(expected, bayesTree));
}

/* ************************************************************************* *
Bayes Tree for testing conversion to a forest of orphans needed for incremental.
       A,B
   C|A    E|B
   D|C    F|E
   */
/* ************************************************************************* */
TEST( BayesTree, removePath )
{
  const Key _A_=A(0), _B_=B(0), _C_=C(0), _D_=D(0), _E_=E(0), _F_=F(0);

  SymbolicBayesTreeUnordered bayesTreeOrig;
  bayesTreeOrig.insertRoot(
    MakeClique(list_of(_A_)(_B_), 2, list_of
      (MakeClique(list_of(_C_)(_A_), 1, list_of
        (MakeClique(list_of(_D_)(_C_), 1))))
      (MakeClique(list_of(_E_)(_B_), 1, list_of
        (MakeClique(list_of(_F_)(_E_), 1))))));

  SymbolicBayesTreeUnordered bayesTree = bayesTreeOrig;

  // remove C, expected outcome: factor graph with ABC,
  // Bayes Tree now contains two orphan trees: D|C and E|B,F|E
  SymbolicFactorGraphUnordered expected;
  expected += SymbolicFactorUnordered(_A_,_B_);
  expected += SymbolicFactorUnordered(_C_,_A_);
  SymbolicBayesTreeUnordered::Cliques expectedOrphans;
  expectedOrphans += bayesTree[_D_], bayesTree[_E_];

  SymbolicBayesNetUnordered bn;
  SymbolicBayesTreeUnordered::Cliques orphans;
  bayesTree.removePath(bayesTree[_C_], bn, orphans);
  SymbolicFactorGraphUnordered factors(bn);
  CHECK(assert_equal(expected, factors));
  CHECK(assert_container_equal(expectedOrphans|indirected, orphans|indirected));

  bayesTree = bayesTreeOrig;

  // remove E: factor graph with EB; E|B removed from second orphan tree
  SymbolicFactorGraphUnordered expected2;
  expected2 += SymbolicFactorUnordered(_A_,_B_);
  expected2 += SymbolicFactorUnordered(_E_,_B_);
  SymbolicBayesTreeUnordered::Cliques expectedOrphans2;
  expectedOrphans2 += bayesTree[_F_];
  expectedOrphans2 += bayesTree[_C_];

  SymbolicBayesNetUnordered bn2;
  SymbolicBayesTreeUnordered::Cliques orphans2;
  bayesTree.removePath(bayesTree[_E_], bn2, orphans2);
  SymbolicFactorGraphUnordered factors2(bn2);
  CHECK(assert_equal(expected2, factors2));
  CHECK(assert_container_equal(expectedOrphans2|indirected, orphans2|indirected));
}

/* ************************************************************************* */
TEST( BayesTree, removePath2 )
{
  SymbolicBayesTreeUnordered bayesTree = asiaBayesTree;

  // Call remove-path with clique B
  SymbolicBayesNetUnordered bn;
  SymbolicBayesTreeUnordered::Cliques orphans;
  bayesTree.removePath(bayesTree[_B_], bn, orphans);
  SymbolicFactorGraphUnordered factors(bn);

  // Check expected outcome
  SymbolicFactorGraphUnordered expected;
  expected += SymbolicFactorUnordered(_B_,_L_,_E_,_S_);
  CHECK(assert_equal(expected, factors));
  SymbolicBayesTreeUnordered::Cliques expectedOrphans;
  expectedOrphans += bayesTree[_T_], bayesTree[_X_];
  CHECK(assert_container_equal(expectedOrphans|indirected, orphans|indirected));
}

/* ************************************************************************* */
TEST( BayesTree, removePath3 )
{
  SymbolicBayesTreeUnordered bayesTree = asiaBayesTree;

  // Call remove-path with clique S
  SymbolicBayesNetUnordered bn;
  SymbolicBayesTreeUnordered::Cliques orphans;
  bayesTree.removePath(bayesTree[_T_], bn, orphans);
  SymbolicFactorGraphUnordered factors(bn);

  // Check expected outcome
  SymbolicFactorGraphUnordered expected;
  expected += SymbolicFactorUnordered(_B_,_L_,_E_,_S_);
  expected += SymbolicFactorUnordered(_T_,_E_,_L_);
  CHECK(assert_equal(expected, factors));
  SymbolicBayesTreeUnordered::Cliques expectedOrphans;
  expectedOrphans += bayesTree[_X_];
  CHECK(assert_container_equal(expectedOrphans|indirected, orphans|indirected));
}

void getAllCliques(const SymbolicBayesTreeUnordered::sharedClique& subtree, SymbolicBayesTreeUnordered::Cliques& cliques)  {
  // Check if subtree exists
  if (subtree) {
    cliques.push_back(subtree);
    // Recursive call over all child cliques
    BOOST_FOREACH(SymbolicBayesTreeUnordered::sharedClique& childClique, subtree->children) {
      getAllCliques(childClique,cliques);
    }
  }
}

/* ************************************************************************* */
TEST( BayesTree, shortcutCheck )
{
  const Key _A_=6, _B_=5, _C_=4, _D_=3, _E_=2, _F_=1, _G_=0;
  SymbolicFactorGraphUnordered chain = list_of
    (SymbolicFactorUnordered(_A_))
    (SymbolicFactorUnordered(_B_, _A_))
    (SymbolicFactorUnordered(_C_, _A_))
    (SymbolicFactorUnordered(_D_, _C_))
    (SymbolicFactorUnordered(_E_, _B_))
    (SymbolicFactorUnordered(_F_, _E_))
    (SymbolicFactorUnordered(_G_, _F_));
  SymbolicBayesTreeUnordered bayesTree = *chain.eliminateMultifrontal(
    OrderingUnordered(list_of(_G_)(_F_)(_E_)(_D_)(_C_)(_B_)(_A_)));

  //bayesTree.saveGraph("BT1.dot");

  SymbolicBayesTreeUnordered::sharedClique rootClique = bayesTree.roots().front();
  //rootClique->printTree();
  SymbolicBayesTreeUnordered::Cliques allCliques;
  getAllCliques(rootClique,allCliques);

  BOOST_FOREACH(SymbolicBayesTreeUnordered::sharedClique& clique, allCliques) {
    //clique->print("Clique#");
    SymbolicBayesNetUnordered bn = clique->shortcut(rootClique);
    //bn.print("Shortcut:\n");
    //cout << endl;
  }

  // Check if all the cached shortcuts are cleared
  rootClique->deleteCachedShortcuts();
  BOOST_FOREACH(SymbolicBayesTreeUnordered::sharedClique& clique, allCliques) {
    bool notCleared = clique->cachedSeparatorMarginal();
    CHECK( notCleared == false);
  }
  EXPECT_LONGS_EQUAL(0, (long)rootClique->numCachedSeparatorMarginals());

//  BOOST_FOREACH(SymbolicBayesTreeUnordered::sharedClique& clique, allCliques) {
//    clique->print("Clique#");
//    if(clique->cachedShortcut()){
//      bn = clique->cachedShortcut().get();
//      bn.print("Shortcut:\n");
//    }
//    else
//      cout << "Not Initialized" << endl;
//    cout << endl;
//  }
}

/* ************************************************************************* */
TEST( BayesTree, removeTop )
{
  SymbolicBayesTreeUnordered bayesTree = asiaBayesTree;

  // create a new factor to be inserted
  //boost::shared_ptr<IndexFactor> newFactor(new IndexFactor(_S_,_B_));

  // Remove the contaminated part of the Bayes tree
  SymbolicBayesNetUnordered bn;
  SymbolicBayesTreeUnordered::Cliques orphans;
  bayesTree.removeTop(list_of(_B_)(_S_), bn, orphans);

  // Check expected outcome
  SymbolicBayesNetUnordered expected;
  expected += SymbolicConditionalUnordered::FromKeys(list_of(_B_)(_L_)(_E_)(_S_), 4);
  CHECK(assert_equal(expected, bn));

  SymbolicBayesTreeUnordered::Cliques expectedOrphans;
  expectedOrphans += bayesTree[_T_], bayesTree[_X_];
  CHECK(assert_container_equal(expectedOrphans|indirected, orphans|indirected));

  // Try removeTop again with a factor that should not change a thing
  //boost::shared_ptr<IndexFactor> newFactor2(new IndexFactor(_B_));
  SymbolicBayesNetUnordered bn2;
  SymbolicBayesTreeUnordered::Cliques orphans2;
  bayesTree.removeTop(list_of(_B_), bn2, orphans2);
  SymbolicFactorGraphUnordered factors2(bn2);
  SymbolicFactorGraphUnordered expected2;
  CHECK(assert_equal(expected2, factors2));
  SymbolicBayesTreeUnordered::Cliques expectedOrphans2;
  CHECK(assert_container_equal(expectedOrphans2|indirected, orphans2|indirected));
}

/* ************************************************************************* */
TEST( BayesTree, removeTop2 )
{
  SymbolicBayesTreeUnordered bayesTree = asiaBayesTree;

  // create two factors to be inserted
  //SymbolicFactorGraph newFactors;
  //newFactors.push_factor(_B_);
  //newFactors.push_factor(_S_);

  // Remove the contaminated part of the Bayes tree
  SymbolicBayesNetUnordered bn;
  SymbolicBayesTreeUnordered::Cliques orphans;
  bayesTree.removeTop(list_of(_T_), bn, orphans);

  // Check expected outcome
  SymbolicBayesNetUnordered expected = list_of
    (SymbolicConditionalUnordered::FromKeys(list_of(_B_)(_L_)(_E_)(_S_), 4))
    (SymbolicConditionalUnordered::FromKeys(list_of(_T_)(_E_)(_L_), 1));
  CHECK(assert_equal(expected, bn));

  SymbolicBayesTreeUnordered::Cliques expectedOrphans;
  expectedOrphans += bayesTree[_X_];
  CHECK(assert_container_equal(expectedOrphans|indirected, orphans|indirected));
}

/* ************************************************************************* */
TEST( BayesTree, removeTop3 )
{
  SymbolicFactorGraphUnordered graph = list_of
    (SymbolicFactorUnordered(L(5)))
    (SymbolicFactorUnordered(X(4), L(5)))
    (SymbolicFactorUnordered(X(2), X(4)))
    (SymbolicFactorUnordered(X(3), X(2)));
  SymbolicBayesTreeUnordered bayesTree = *graph.eliminateMultifrontal(
    OrderingUnordered(list_of (X(3)) (X(2)) (X(4)) (L(5)) ));

  // remove all
  SymbolicBayesNetUnordered bn;
  SymbolicBayesTreeUnordered::Cliques orphans;
  bayesTree.removeTop(list_of(L(5))(X(4))(X(2))(X(3)), bn, orphans);

  SymbolicBayesNetUnordered expectedBn = list_of
    (SymbolicConditionalUnordered::FromKeys(list_of(L(5))(X(4)), 2))
    (SymbolicConditionalUnordered(X(2), X(4)))
    (SymbolicConditionalUnordered(X(3), X(2)));
  EXPECT(assert_equal(expectedBn, bn));
  EXPECT(orphans.empty());
}

/* ************************************************************************* */
TEST( BayesTree, removeTop4 )
{
  SymbolicFactorGraphUnordered graph = list_of
    (SymbolicFactorUnordered(L(5)))
    (SymbolicFactorUnordered(X(4), L(5)))
    (SymbolicFactorUnordered(X(2), X(4)))
    (SymbolicFactorUnordered(X(3), X(2)));
  SymbolicBayesTreeUnordered bayesTree = *graph.eliminateMultifrontal(
    OrderingUnordered(list_of (X(3)) (X(2)) (X(4)) (L(5)) ));

  // remove all
  SymbolicBayesNetUnordered bn;
  SymbolicBayesTreeUnordered::Cliques orphans;
  bayesTree.removeTop(list_of(X(2))(L(5))(X(4))(X(3)), bn, orphans);

  SymbolicBayesNetUnordered expectedBn = list_of
    (SymbolicConditionalUnordered::FromKeys(list_of(L(5))(X(4)), 2))
    (SymbolicConditionalUnordered(X(2), X(4)))
    (SymbolicConditionalUnordered(X(3), X(2)));
  EXPECT(assert_equal(expectedBn, bn));
  EXPECT(orphans.empty());
}

/* ************************************************************************* */
TEST( SymbolicBayesTreeUnordered, thinTree ) {

  // create a thin-tree Bayesnet, a la Jean-Guillaume
  SymbolicBayesNetUnordered bayesNet;
  bayesNet.push_back(boost::make_shared<SymbolicConditionalUnordered>(14));

  bayesNet.push_back(boost::make_shared<SymbolicConditionalUnordered>(13, 14));
  bayesNet.push_back(boost::make_shared<SymbolicConditionalUnordered>(12, 14));

  bayesNet.push_back(boost::make_shared<SymbolicConditionalUnordered>(11, 13, 14));
  bayesNet.push_back(boost::make_shared<SymbolicConditionalUnordered>(10, 13, 14));
  bayesNet.push_back(boost::make_shared<SymbolicConditionalUnordered>(9, 12, 14));
  bayesNet.push_back(boost::make_shared<SymbolicConditionalUnordered>(8, 12, 14));

  bayesNet.push_back(boost::make_shared<SymbolicConditionalUnordered>(7, 11, 13));
  bayesNet.push_back(boost::make_shared<SymbolicConditionalUnordered>(6, 11, 13));
  bayesNet.push_back(boost::make_shared<SymbolicConditionalUnordered>(5, 10, 13));
  bayesNet.push_back(boost::make_shared<SymbolicConditionalUnordered>(4, 10, 13));

  bayesNet.push_back(boost::make_shared<SymbolicConditionalUnordered>(3, 9, 12));
  bayesNet.push_back(boost::make_shared<SymbolicConditionalUnordered>(2, 9, 12));
  bayesNet.push_back(boost::make_shared<SymbolicConditionalUnordered>(1, 8, 12));
  bayesNet.push_back(boost::make_shared<SymbolicConditionalUnordered>(0, 8, 12));

  if (debug) {
    GTSAM_PRINT(bayesNet);
    bayesNet.saveGraph("/tmp/symbolicBayesNet.dot");
  }

  // create a BayesTree out of a Bayes net
  OrderingUnordered ordering(bayesNet.keys());
  SymbolicBayesTreeUnordered bayesTree = *SymbolicFactorGraphUnordered(bayesNet).eliminateMultifrontal(ordering);
  if (debug) {
    GTSAM_PRINT(bayesTree);
    bayesTree.saveGraph("/tmp/SymbolicBayesTreeUnordered.dot");
  }

  SymbolicBayesTreeUnordered::Clique::shared_ptr R = bayesTree.roots().front();

  {
    // check shortcut P(S9||R) to root
    SymbolicBayesTreeUnordered::Clique::shared_ptr c = bayesTree[9];
    SymbolicBayesNetUnordered shortcut = c->shortcut(R);
    SymbolicBayesNetUnordered expected;
    EXPECT(assert_equal(expected, shortcut));
  }

  {
    // check shortcut P(S8||R) to root
    SymbolicBayesTreeUnordered::Clique::shared_ptr c = bayesTree[8];
    SymbolicBayesNetUnordered shortcut = c->shortcut(R);
    SymbolicBayesNetUnordered expected = list_of(SymbolicConditionalUnordered(12, 14));
    EXPECT(assert_equal(expected, shortcut));
  }

  {
    // check shortcut P(S4||R) to root
    SymbolicBayesTreeUnordered::Clique::shared_ptr c = bayesTree[4];
    SymbolicBayesNetUnordered shortcut = c->shortcut(R);
    SymbolicBayesNetUnordered expected;
    expected.push_back(boost::make_shared<SymbolicConditionalUnordered>(10, 13, 14));
    EXPECT(assert_equal(expected, shortcut));
  }

  {
    // check shortcut P(S2||R) to root
    SymbolicBayesTreeUnordered::Clique::shared_ptr c = bayesTree[2];
    SymbolicBayesNetUnordered shortcut = c->shortcut(R);
    SymbolicBayesNetUnordered expected;
    expected.push_back(boost::make_shared<SymbolicConditionalUnordered>(12, 14));
    expected.push_back(boost::make_shared<SymbolicConditionalUnordered>(9, 12, 14));
    EXPECT(assert_equal(expected, shortcut));
  }

  {
    // check shortcut P(S0||R) to root
    SymbolicBayesTreeUnordered::Clique::shared_ptr c = bayesTree[0];
    SymbolicBayesNetUnordered shortcut = c->shortcut(R);
    SymbolicBayesNetUnordered expected;
    expected.push_back(boost::make_shared<SymbolicConditionalUnordered>(12, 14));
    expected.push_back(boost::make_shared<SymbolicConditionalUnordered>(8, 12, 14));
    EXPECT(assert_equal(expected, shortcut));
  }

  SymbolicBayesNetUnordered::shared_ptr actualJoint;

  // Check joint P(8,2)
  if (false) { // TODO, not disjoint
    actualJoint = bayesTree.jointBayesNet(8, 2);
    SymbolicBayesNetUnordered expected;
    expected.push_back(boost::make_shared<SymbolicConditionalUnordered>(8));
    expected.push_back(boost::make_shared<SymbolicConditionalUnordered>(2, 8));
    EXPECT(assert_equal(expected, *actualJoint));
  }

  // Check joint P(1,2)
  if (false) { // TODO, not disjoint
    actualJoint = bayesTree.jointBayesNet(1, 2);
    SymbolicBayesNetUnordered expected;
    expected.push_back(boost::make_shared<SymbolicConditionalUnordered>(2));
    expected.push_back(boost::make_shared<SymbolicConditionalUnordered>(1, 2));
    EXPECT(assert_equal(expected, *actualJoint));
  }

  // Check joint P(2,6)
  if (true) {
    actualJoint = bayesTree.jointBayesNet(2, 6);
    SymbolicBayesNetUnordered expected;
    expected.push_back(boost::make_shared<SymbolicConditionalUnordered>(2, 6));
    expected.push_back(boost::make_shared<SymbolicConditionalUnordered>(6));
    EXPECT(assert_equal(expected, *actualJoint));
  }

  // Check joint P(4,6)
  if (false) { // TODO, not disjoint
    actualJoint = bayesTree.jointBayesNet(4, 6);
    SymbolicBayesNetUnordered expected;
    expected.push_back(boost::make_shared<SymbolicConditionalUnordered>(6));
    expected.push_back(boost::make_shared<SymbolicConditionalUnordered>(4, 6));
    EXPECT(assert_equal(expected, *actualJoint));
  }
}

/* ************************************************************************* */
TEST(SymbolicBayesTreeUnordered, forest_joint)
{
  // Create forest
  SymbolicBayesTreeCliqueUnordered::shared_ptr root1 = MakeClique(list_of(1), 1);
  SymbolicBayesTreeCliqueUnordered::shared_ptr root2 = MakeClique(list_of(2), 1);
  SymbolicBayesTreeUnordered bayesTree;
  bayesTree.insertRoot(root1);
  bayesTree.insertRoot(root2);

  // Check joint
  SymbolicBayesNetUnordered expected = list_of
    (SymbolicConditionalUnordered(1))
    (SymbolicConditionalUnordered(2));
  SymbolicBayesNetUnordered actual = *bayesTree.jointBayesNet(1, 2);

  EXPECT(assert_equal(expected, actual));
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

TEST( SymbolicBayesTreeUnordered, linear_smoother_shortcuts ) {
  // Create smoother with 7 nodes
  SymbolicFactorGraphUnordered smoother;
  smoother.push_factor(0);
  smoother.push_factor(0, 1);
  smoother.push_factor(1, 2);
  smoother.push_factor(2, 3);
  smoother.push_factor(3, 4);
  smoother.push_factor(4, 5);
  smoother.push_factor(5, 6);

  // Eliminate in numerical order 0..6
  OrderingUnordered ordering(smoother.keys());
  SymbolicBayesNetUnordered bayesNet = *smoother.eliminateSequential(ordering);

  if (debug) {
    GTSAM_PRINT(bayesNet);
    bayesNet.saveGraph("/tmp/symbolicBayesNet.dot");
  }

  // create a BayesTree
  SymbolicBayesTreeUnordered bayesTree = *smoother.eliminateMultifrontal(ordering);
  if (debug) {
    GTSAM_PRINT(bayesTree);
    bayesTree.saveGraph("/tmp/SymbolicBayesTreeUnordered.dot");
  }

  SymbolicBayesTreeUnordered::Clique::shared_ptr R = bayesTree.roots().front();

  {
    // check shortcut P(S2||R) to root
    SymbolicBayesTreeUnordered::Clique::shared_ptr c = bayesTree[4]; // 4 is frontal in C2
    SymbolicBayesNetUnordered shortcut = c->shortcut(R);
    SymbolicBayesNetUnordered expected;
    EXPECT(assert_equal(expected, shortcut));
  }

  {
    // check shortcut P(S3||R) to root
    SymbolicBayesTreeUnordered::Clique::shared_ptr c = bayesTree[3]; // 3 is frontal in C3
    SymbolicBayesNetUnordered shortcut = c->shortcut(R);
    SymbolicBayesNetUnordered expected = list_of(SymbolicConditionalUnordered(4, 5));
    EXPECT(assert_equal(expected, shortcut));
  }

  {
    // check shortcut P(S4||R) to root
    SymbolicBayesTreeUnordered::Clique::shared_ptr c = bayesTree[2]; // 2 is frontal in C4
    SymbolicBayesNetUnordered shortcut = c->shortcut(R);
    SymbolicBayesNetUnordered expected = list_of(SymbolicConditionalUnordered(3, 5));
    EXPECT(assert_equal(expected, shortcut));
  }
}

/* ************************************************************************* */
// from testSymbolicJunctionTree, which failed at one point
TEST(SymbolicBayesTreeUnordered, complicatedMarginal)
{
  // Create the conditionals to go in the BayesTree
  SymbolicBayesTreeCliqueUnordered::shared_ptr cur;
  SymbolicBayesTreeCliqueUnordered::shared_ptr root = MakeClique(list_of(11)(12), 2);
  cur = root;
  
  root->children += MakeClique(list_of(9)(10)(11)(12), 2);
  root->children.back()->parent_ = root;

  root->children += MakeClique(list_of(7)(8)(11), 2);
  root->children.back()->parent_ = root;
  cur = root->children.back();

  cur->children += MakeClique(list_of(5)(6)(7)(8), 2);
  cur->children.back()->parent_ = cur;
  cur = cur->children.back();

  cur->children += MakeClique(list_of(3)(4)(6), 2);
  cur->children.back()->parent_ = cur;

  cur->children += MakeClique(list_of(1)(2)(5), 2);
  cur->children.back()->parent_ = cur;

  // Create Bayes Tree
  SymbolicBayesTreeUnordered bt;
  bt.insertRoot(root);
  if (debug) {
    GTSAM_PRINT(bt);
    bt.saveGraph("/tmp/SymbolicBayesTreeUnordered.dot");
  }

  // Shortcut on 9
  {
    SymbolicBayesTreeUnordered::Clique::shared_ptr c = bt[9];
    SymbolicBayesNetUnordered shortcut = c->shortcut(root);
    EXPECT(assert_equal(SymbolicBayesNetUnordered(), shortcut));
  }

  // Shortcut on 7
  {
    SymbolicBayesTreeUnordered::Clique::shared_ptr c = bt[7];
    SymbolicBayesNetUnordered shortcut = c->shortcut(root);
    EXPECT(assert_equal(SymbolicBayesNetUnordered(), shortcut));
  }

  // Shortcut on 5
  {
    SymbolicBayesTreeUnordered::Clique::shared_ptr c = bt[5];
    SymbolicBayesNetUnordered shortcut = c->shortcut(root);
    SymbolicBayesNetUnordered expected = list_of
      (SymbolicConditionalUnordered(7, 8, 11))
      (SymbolicConditionalUnordered(8, 11));
    EXPECT(assert_equal(expected, shortcut));
  }

  // Shortcut on 3
  {
    SymbolicBayesTreeUnordered::Clique::shared_ptr c = bt[3];
    SymbolicBayesNetUnordered shortcut = c->shortcut(root);
    SymbolicBayesNetUnordered expected = list_of(SymbolicConditionalUnordered(6, 11));
    EXPECT(assert_equal(expected, shortcut));
  }

  // Shortcut on 1
  {
    SymbolicBayesTreeUnordered::Clique::shared_ptr c = bt[1];
    SymbolicBayesNetUnordered shortcut = c->shortcut(root);
    SymbolicBayesNetUnordered expected = list_of(SymbolicConditionalUnordered(5, 11));
    EXPECT(assert_equal(expected, shortcut));
  }

  // Marginal on 5
  {
    SymbolicFactorUnordered::shared_ptr actual = bt.marginalFactor(5);
    EXPECT(assert_equal(SymbolicFactorUnordered(5), *actual, 1e-1));
  }

  // Shortcut on 6
  {
    SymbolicFactorUnordered::shared_ptr actual = bt.marginalFactor(6);
    EXPECT(assert_equal(SymbolicFactorUnordered(6), *actual, 1e-1));
  }

}
/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */


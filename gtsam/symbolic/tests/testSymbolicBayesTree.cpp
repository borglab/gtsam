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
 * @author  Frank Dellaert
 * @author  Michael Kaess
 * @author  Viorela Ila
 */

#include <gtsam/symbolic/SymbolicBayesTree.h>
#include <gtsam/symbolic/SymbolicBayesNet.h>
#include <gtsam/inference/Symbol.h>
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
  SymbolicBayesTreeClique::shared_ptr MakeClique(const KEYS& keys, DenseIndex nrFrontals)
  {
    return boost::make_shared<SymbolicBayesTreeClique>(
      boost::make_shared<SymbolicConditional>(
      SymbolicConditional::FromKeys(keys, nrFrontals)));
  }

  template<typename KEYS, typename CHILDREN>
  SymbolicBayesTreeClique::shared_ptr MakeClique(
    const KEYS& keys, DenseIndex nrFrontals, const CHILDREN& children)
  {
    SymbolicBayesTreeClique::shared_ptr clique =
      boost::make_shared<SymbolicBayesTreeClique>(
      boost::make_shared<SymbolicConditional>(
      SymbolicConditional::FromKeys(keys, nrFrontals)));
    clique->children.assign(children.begin(), children.end());
    for(typename CHILDREN::const_iterator child = children.begin(); child != children.end(); ++child)
      (*child)->parent_ = clique;
    return clique;
  }

}

/* ************************************************************************* */
TEST(SymbolicBayesTree, clear)
{
  SymbolicBayesTree bayesTree = asiaBayesTree;
  bayesTree.clear();

  SymbolicBayesTree expected;

  // Check whether cleared BayesTree is equal to a new BayesTree
  CHECK(assert_equal(expected, bayesTree));
}

/* ************************************************************************* */
TEST(SymbolicBayesTree, clique_structure)
{
  //        l1                  l2
  //    /    |                /  |
  // x1 --- x2 --- x3 --- x4 --- x5
  //                          \  |
  //                            l3
  SymbolicFactorGraph graph;
  graph += SymbolicFactor(X(1), L(1));
  graph += SymbolicFactor(X(1), X(2));
  graph += SymbolicFactor(X(2), L(1));
  graph += SymbolicFactor(X(2), X(3));
  graph += SymbolicFactor(X(3), X(4));
  graph += SymbolicFactor(X(4), L(2));
  graph += SymbolicFactor(X(4), X(5));
  graph += SymbolicFactor(L(2), X(5));
  graph += SymbolicFactor(X(4), L(3));
  graph += SymbolicFactor(X(5), L(3));

  SymbolicBayesTree expected;
  expected.insertRoot(
    MakeClique(list_of(X(2)) (X(3)), 2, list_of
      (MakeClique(list_of(X(4)) (X(3)), 1, list_of
        (MakeClique(list_of(X(5)) (L(2)) (X(4)), 2, list_of
          (MakeClique(list_of(L(3)) (X(4)) (X(5)), 1))))))
      (MakeClique(list_of(X(1)) (L(1)) (X(2)), 2))));

  Ordering order = list_of(X(1)) (L(3)) (L(1)) (X(5)) (X(2)) (L(2)) (X(4)) (X(3));

  SymbolicBayesTree actual = *graph.eliminateMultifrontal(order);

  EXPECT(assert_equal(expected, actual));
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

  SymbolicBayesTree bayesTreeOrig;
  bayesTreeOrig.insertRoot(
    MakeClique(list_of(_A_)(_B_), 2, list_of
      (MakeClique(list_of(_C_)(_A_), 1, list_of
        (MakeClique(list_of(_D_)(_C_), 1))))
      (MakeClique(list_of(_E_)(_B_), 1, list_of
        (MakeClique(list_of(_F_)(_E_), 1))))));

  SymbolicBayesTree bayesTree = bayesTreeOrig;

  // remove C, expected outcome: factor graph with ABC,
  // Bayes Tree now contains two orphan trees: D|C and E|B,F|E
  SymbolicFactorGraph expected;
  expected += SymbolicFactor(_A_,_B_);
  expected += SymbolicFactor(_C_,_A_);
  SymbolicBayesTree::Cliques expectedOrphans;
  expectedOrphans += bayesTree[_D_], bayesTree[_E_];

  SymbolicBayesNet bn;
  SymbolicBayesTree::Cliques orphans;
  bayesTree.removePath(bayesTree[_C_], &bn, &orphans);
  SymbolicFactorGraph factors(bn);
  CHECK(assert_equal(expected, factors));
  CHECK(assert_container_equal(expectedOrphans|indirected, orphans|indirected));

  bayesTree = bayesTreeOrig;

  // remove E: factor graph with EB; E|B removed from second orphan tree
  SymbolicFactorGraph expected2;
  expected2 += SymbolicFactor(_A_,_B_);
  expected2 += SymbolicFactor(_E_,_B_);
  SymbolicBayesTree::Cliques expectedOrphans2;
  expectedOrphans2 += bayesTree[_F_];
  expectedOrphans2 += bayesTree[_C_];

  SymbolicBayesNet bn2;
  SymbolicBayesTree::Cliques orphans2;
  bayesTree.removePath(bayesTree[_E_], &bn2, &orphans2);
  SymbolicFactorGraph factors2(bn2);
  CHECK(assert_equal(expected2, factors2));
  CHECK(assert_container_equal(expectedOrphans2|indirected, orphans2|indirected));
}

/* ************************************************************************* */
TEST( BayesTree, removePath2 )
{
  SymbolicBayesTree bayesTree = asiaBayesTree;

  // Call remove-path with clique B
  SymbolicBayesNet bn;
  SymbolicBayesTree::Cliques orphans;
  bayesTree.removePath(bayesTree[_B_], &bn, &orphans);
  SymbolicFactorGraph factors(bn);

  // Check expected outcome
  SymbolicFactorGraph expected;
  expected += SymbolicFactor(_E_,_L_,_B_);
  CHECK(assert_equal(expected, factors));
  SymbolicBayesTree::Cliques expectedOrphans;
  expectedOrphans += bayesTree[_S_], bayesTree[_T_], bayesTree[_X_];
  CHECK(assert_container_equal(expectedOrphans|indirected, orphans|indirected));
}

/* ************************************************************************* */
TEST(BayesTree, removePath3)
{
  SymbolicBayesTree bayesTree = asiaBayesTree;

  // Call remove-path with clique T
  SymbolicBayesNet bn;
  SymbolicBayesTree::Cliques orphans;
  bayesTree.removePath(bayesTree[_T_], &bn, &orphans);
  SymbolicFactorGraph factors(bn);

  // Check expected outcome
  SymbolicFactorGraph expected;
  expected += SymbolicFactor(_E_, _L_, _B_);
  expected += SymbolicFactor(_T_, _E_, _L_);
  CHECK(assert_equal(expected, factors));
  SymbolicBayesTree::Cliques expectedOrphans;
  expectedOrphans += bayesTree[_S_], bayesTree[_X_];
  CHECK(assert_container_equal(expectedOrphans|indirected, orphans|indirected));
}

void getAllCliques(const SymbolicBayesTree::sharedClique& subtree, SymbolicBayesTree::Cliques& cliques)  {
  // Check if subtree exists
  if (subtree) {
    cliques.push_back(subtree);
    // Recursive call over all child cliques
    for(SymbolicBayesTree::sharedClique& childClique: subtree->children) {
      getAllCliques(childClique,cliques);
    }
  }
}

/* ************************************************************************* */
TEST( BayesTree, shortcutCheck )
{
  const Key _A_=6, _B_=5, _C_=4, _D_=3, _E_=2, _F_=1, _G_=0;
  SymbolicFactorGraph chain = list_of
    (SymbolicFactor(_A_))
    (SymbolicFactor(_B_, _A_))
    (SymbolicFactor(_C_, _A_))
    (SymbolicFactor(_D_, _C_))
    (SymbolicFactor(_E_, _B_))
    (SymbolicFactor(_F_, _E_))
    (SymbolicFactor(_G_, _F_));
  Ordering ordering(list_of(_G_)(_F_)(_E_)(_D_)(_C_)(_B_)(_A_));
  SymbolicBayesTree bayesTree = *chain.eliminateMultifrontal(ordering);

  //bayesTree.saveGraph("BT1.dot");

  SymbolicBayesTree::sharedClique rootClique = bayesTree.roots().front();
  //rootClique->printTree();
  SymbolicBayesTree::Cliques allCliques;
  getAllCliques(rootClique,allCliques);

  for(SymbolicBayesTree::sharedClique& clique: allCliques) {
    //clique->print("Clique#");
    SymbolicBayesNet bn = clique->shortcut(rootClique);
    //bn.print("Shortcut:\n");
    //cout << endl;
  }

  // Check if all the cached shortcuts are cleared
  rootClique->deleteCachedShortcuts();
  for(SymbolicBayesTree::sharedClique& clique: allCliques) {
    bool notCleared = clique->cachedSeparatorMarginal().is_initialized();
    CHECK( notCleared == false);
  }
  EXPECT_LONGS_EQUAL(0, (long)rootClique->numCachedSeparatorMarginals());

//  for(SymbolicBayesTree::sharedClique& clique: allCliques) {
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
  SymbolicBayesTree bayesTree = asiaBayesTree;

  // create a new factor to be inserted
  //boost::shared_ptr<IndexFactor> newFactor(new IndexFactor(_S_,_B_));

  // Remove the contaminated part of the Bayes tree
  SymbolicBayesNet bn;
  SymbolicBayesTree::Cliques orphans;
  bayesTree.removeTop(list_of(_B_)(_S_), &bn, &orphans);

  // Check expected outcome
  SymbolicBayesNet expected;
  expected += SymbolicConditional::FromKeys(list_of(_E_)(_L_)(_B_), 3);
  expected += SymbolicConditional::FromKeys(list_of(_S_)(_B_)(_L_), 1);
  CHECK(assert_equal(expected, bn));

  SymbolicBayesTree::Cliques expectedOrphans;
  expectedOrphans += bayesTree[_T_], bayesTree[_X_];
  CHECK(assert_container_equal(expectedOrphans|indirected, orphans|indirected));

  // Try removeTop again with a factor that should not change a thing
  //boost::shared_ptr<IndexFactor> newFactor2(new IndexFactor(_B_));
  SymbolicBayesNet bn2;
  SymbolicBayesTree::Cliques orphans2;
  bayesTree.removeTop(list_of(_B_), &bn2, &orphans2);
  SymbolicFactorGraph factors2(bn2);
  SymbolicFactorGraph expected2;
  CHECK(assert_equal(expected2, factors2));
  SymbolicBayesTree::Cliques expectedOrphans2;
  CHECK(assert_container_equal(expectedOrphans2|indirected, orphans2|indirected));
}

/* ************************************************************************* */
TEST( BayesTree, removeTop2 )
{
  SymbolicBayesTree bayesTree = asiaBayesTree;

  // create two factors to be inserted
  //SymbolicFactorGraph newFactors;
  //newFactors.push_factor(_B_);
  //newFactors.push_factor(_S_);

  // Remove the contaminated part of the Bayes tree
  SymbolicBayesNet bn;
  SymbolicBayesTree::Cliques orphans;
  bayesTree.removeTop(list_of(_T_), &bn, &orphans);

  // Check expected outcome
  SymbolicBayesNet expected = list_of
    (SymbolicConditional::FromKeys(list_of(_E_)(_L_)(_B_), 3))
    (SymbolicConditional::FromKeys(list_of(_T_)(_E_)(_L_), 1));
  CHECK(assert_equal(expected, bn));

  SymbolicBayesTree::Cliques expectedOrphans;
  expectedOrphans += bayesTree[_S_], bayesTree[_X_];
  CHECK(assert_container_equal(expectedOrphans|indirected, orphans|indirected));
}

/* ************************************************************************* */
TEST( BayesTree, removeTop3 )
{
  SymbolicFactorGraph graph = list_of
    (SymbolicFactor(L(5)))
    (SymbolicFactor(X(4), L(5)))
    (SymbolicFactor(X(2), X(4)))
    (SymbolicFactor(X(3), X(2)));
  Ordering ordering(list_of (X(3)) (X(2)) (X(4)) (L(5)) );
  SymbolicBayesTree bayesTree = *graph.eliminateMultifrontal(ordering);

  // remove all
  SymbolicBayesNet bn;
  SymbolicBayesTree::Cliques orphans;
  bayesTree.removeTop(list_of(L(5))(X(4))(X(2))(X(3)), &bn, &orphans);

  SymbolicBayesNet expectedBn = list_of
    (SymbolicConditional::FromKeys(list_of(X(4))(L(5)), 2))
    (SymbolicConditional(X(2), X(4)))
    (SymbolicConditional(X(3), X(2)));
  EXPECT(assert_equal(expectedBn, bn));
  EXPECT(orphans.empty());
}

/* ************************************************************************* */
TEST( BayesTree, removeTop4 )
{
  SymbolicFactorGraph graph = list_of
    (SymbolicFactor(L(5)))
    (SymbolicFactor(X(4), L(5)))
    (SymbolicFactor(X(2), X(4)))
    (SymbolicFactor(X(3), X(2)));
  Ordering ordering(list_of (X(3)) (X(2)) (X(4)) (L(5)) );
  SymbolicBayesTree bayesTree = *graph.eliminateMultifrontal(ordering);

  // remove all
  SymbolicBayesNet bn;
  SymbolicBayesTree::Cliques orphans;
  bayesTree.removeTop(list_of(X(2))(L(5))(X(4))(X(3)), &bn, &orphans);

  SymbolicBayesNet expectedBn = list_of
    (SymbolicConditional::FromKeys(list_of(X(4))(L(5)), 2))
    (SymbolicConditional(X(2), X(4)))
    (SymbolicConditional(X(3), X(2)));
  EXPECT(assert_equal(expectedBn, bn));
  EXPECT(orphans.empty());
}

/* ************************************************************************* */
TEST( BayesTree, removeTop5 )
{
  // Remove top called with variables that are not in the Bayes tree
  SymbolicFactorGraph graph = list_of
    (SymbolicFactor(L(5)))
    (SymbolicFactor(X(4), L(5)))
    (SymbolicFactor(X(2), X(4)))
    (SymbolicFactor(X(3), X(2)));
  Ordering ordering(list_of (X(3)) (X(2)) (X(4)) (L(5)) );
  SymbolicBayesTree bayesTree = *graph.eliminateMultifrontal(ordering);

  // Remove nonexistant
  SymbolicBayesNet bn;
  SymbolicBayesTree::Cliques orphans;
  bayesTree.removeTop(list_of(X(10)), &bn, &orphans);

  SymbolicBayesNet expectedBn;
  EXPECT(assert_equal(expectedBn, bn));
  EXPECT(orphans.empty());
}

/* ************************************************************************* */
TEST( SymbolicBayesTree, thinTree ) {

  // create a thin-tree Bayesnet, a la Jean-Guillaume
  SymbolicBayesNet bayesNet;
  bayesNet.push_back(boost::make_shared<SymbolicConditional>(14));

  bayesNet.push_back(boost::make_shared<SymbolicConditional>(13, 14));
  bayesNet.push_back(boost::make_shared<SymbolicConditional>(12, 14));

  bayesNet.push_back(boost::make_shared<SymbolicConditional>(11, 13, 14));
  bayesNet.push_back(boost::make_shared<SymbolicConditional>(10, 13, 14));
  bayesNet.push_back(boost::make_shared<SymbolicConditional>(9, 12, 14));
  bayesNet.push_back(boost::make_shared<SymbolicConditional>(8, 12, 14));

  bayesNet.push_back(boost::make_shared<SymbolicConditional>(7, 11, 13));
  bayesNet.push_back(boost::make_shared<SymbolicConditional>(6, 11, 13));
  bayesNet.push_back(boost::make_shared<SymbolicConditional>(5, 10, 13));
  bayesNet.push_back(boost::make_shared<SymbolicConditional>(4, 10, 13));

  bayesNet.push_back(boost::make_shared<SymbolicConditional>(3, 9, 12));
  bayesNet.push_back(boost::make_shared<SymbolicConditional>(2, 9, 12));
  bayesNet.push_back(boost::make_shared<SymbolicConditional>(1, 8, 12));
  bayesNet.push_back(boost::make_shared<SymbolicConditional>(0, 8, 12));

  if (debug) {
    GTSAM_PRINT(bayesNet);
    bayesNet.saveGraph("/tmp/symbolicBayesNet.dot");
  }

  // create a BayesTree out of a Bayes net
  SymbolicBayesTree bayesTree = *SymbolicFactorGraph(bayesNet).eliminateMultifrontal();
  if (debug) {
    GTSAM_PRINT(bayesTree);
    bayesTree.saveGraph("/tmp/SymbolicBayesTree.dot");
  }

  SymbolicBayesTree::Clique::shared_ptr R = bayesTree.roots().front();

  {
    // check shortcut P(S9||R) to root
    SymbolicBayesTree::Clique::shared_ptr c = bayesTree[9];
    SymbolicBayesNet shortcut = c->shortcut(R);
    SymbolicBayesNet expected = list_of(SymbolicConditional(14, 11, 13));
    EXPECT(assert_equal(expected, shortcut));
  }

  {
    // check shortcut P(S8||R) to root
    SymbolicBayesTree::Clique::shared_ptr c = bayesTree[8];
    SymbolicBayesNet shortcut = c->shortcut(R);
    SymbolicBayesNet expected = list_of
      (SymbolicConditional(12, 14))
      (SymbolicConditional(14, 11, 13));
    EXPECT(assert_equal(expected, shortcut));
  }

  {
    // check shortcut P(S4||R) to root
    SymbolicBayesTree::Clique::shared_ptr c = bayesTree[4];
    SymbolicBayesNet shortcut = c->shortcut(R);
    SymbolicBayesNet expected = list_of(SymbolicConditional(10, 11, 13));
    EXPECT(assert_equal(expected, shortcut));
  }

  {
    // check shortcut P(S2||R) to root
    SymbolicBayesTree::Clique::shared_ptr c = bayesTree[2];
    SymbolicBayesNet shortcut = c->shortcut(R);
    SymbolicBayesNet expected = list_of(SymbolicConditional(9, 11, 12, 13))
                                       (SymbolicConditional(12, 11, 13));
    EXPECT(assert_equal(expected, shortcut));
  }

  {
    // check shortcut P(S0||R) to root
    SymbolicBayesTree::Clique::shared_ptr c = bayesTree[0];
    SymbolicBayesNet shortcut = c->shortcut(R);
    SymbolicBayesNet expected = list_of(SymbolicConditional(8, 11, 12, 13))
                                       (SymbolicConditional(12, 11, 13));
    EXPECT(assert_equal(expected, shortcut));
  }

  SymbolicBayesNet::shared_ptr actualJoint;

  // Check joint P(8,2)
  if (false) { // TODO, not disjoint
    actualJoint = bayesTree.jointBayesNet(8, 2);
    SymbolicBayesNet expected;
    expected.push_back(boost::make_shared<SymbolicConditional>(8));
    expected.push_back(boost::make_shared<SymbolicConditional>(2, 8));
    EXPECT(assert_equal(expected, *actualJoint));
  }

  // Check joint P(1,2)
  if (false) { // TODO, not disjoint
    actualJoint = bayesTree.jointBayesNet(1, 2);
    SymbolicBayesNet expected;
    expected.push_back(boost::make_shared<SymbolicConditional>(2));
    expected.push_back(boost::make_shared<SymbolicConditional>(1, 2));
    EXPECT(assert_equal(expected, *actualJoint));
  }

  // Check joint P(2,6)
  if (true) {
    actualJoint = bayesTree.jointBayesNet(2, 6);
    SymbolicBayesNet expected;
    expected.push_back(boost::make_shared<SymbolicConditional>(2, 6));
    expected.push_back(boost::make_shared<SymbolicConditional>(6));
    EXPECT(assert_equal(expected, *actualJoint));
  }

  // Check joint P(4,6)
  if (false) { // TODO, not disjoint
    actualJoint = bayesTree.jointBayesNet(4, 6);
    SymbolicBayesNet expected;
    expected.push_back(boost::make_shared<SymbolicConditional>(6));
    expected.push_back(boost::make_shared<SymbolicConditional>(4, 6));
    EXPECT(assert_equal(expected, *actualJoint));
  }
}

/* ************************************************************************* */
TEST(SymbolicBayesTree, forest_joint)
{
  // Create forest
  SymbolicBayesTreeClique::shared_ptr root1 = MakeClique(list_of(1), 1);
  SymbolicBayesTreeClique::shared_ptr root2 = MakeClique(list_of(2), 1);
  SymbolicBayesTree bayesTree;
  bayesTree.insertRoot(root1);
  bayesTree.insertRoot(root2);

  // Check joint
  SymbolicBayesNet expected = list_of
    (SymbolicConditional(1))
    (SymbolicConditional(2));
  SymbolicBayesNet actual = *bayesTree.jointBayesNet(1, 2);

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

TEST( SymbolicBayesTree, linear_smoother_shortcuts ) {
  // Create smoother with 7 nodes
  SymbolicFactorGraph smoother;
  smoother.push_factor(0);
  smoother.push_factor(0, 1);
  smoother.push_factor(1, 2);
  smoother.push_factor(2, 3);
  smoother.push_factor(3, 4);
  smoother.push_factor(4, 5);
  smoother.push_factor(5, 6);

  // Eliminate in numerical order 0..6
  Ordering ordering(smoother.keys());
  SymbolicBayesNet bayesNet = *smoother.eliminateSequential(ordering);

  if (debug) {
    GTSAM_PRINT(bayesNet);
    bayesNet.saveGraph("/tmp/symbolicBayesNet.dot");
  }

  // create a BayesTree
  SymbolicBayesTree bayesTree = *smoother.eliminateMultifrontal(ordering);
  if (debug) {
    GTSAM_PRINT(bayesTree);
    bayesTree.saveGraph("/tmp/SymbolicBayesTree.dot");
  }

  SymbolicBayesTree::Clique::shared_ptr R = bayesTree.roots().front();

  {
    // check shortcut P(S2||R) to root
    SymbolicBayesTree::Clique::shared_ptr c = bayesTree[4]; // 4 is frontal in C2
    SymbolicBayesNet shortcut = c->shortcut(R);
    SymbolicBayesNet expected;
    EXPECT(assert_equal(expected, shortcut));
  }

  {
    // check shortcut P(S3||R) to root
    SymbolicBayesTree::Clique::shared_ptr c = bayesTree[3]; // 3 is frontal in C3
    SymbolicBayesNet shortcut = c->shortcut(R);
    SymbolicBayesNet expected = list_of(SymbolicConditional(4, 5));
    EXPECT(assert_equal(expected, shortcut));
  }

  {
    // check shortcut P(S4||R) to root
    SymbolicBayesTree::Clique::shared_ptr c = bayesTree[2]; // 2 is frontal in C4
    SymbolicBayesNet shortcut = c->shortcut(R);
    SymbolicBayesNet expected = list_of(SymbolicConditional(3, 5));
    EXPECT(assert_equal(expected, shortcut));
  }
}

/* ************************************************************************* */
// from testSymbolicJunctionTree, which failed at one point
TEST(SymbolicBayesTree, complicatedMarginal)
{
  // Create the conditionals to go in the BayesTree
  SymbolicBayesTreeClique::shared_ptr cur;
  SymbolicBayesTreeClique::shared_ptr root = MakeClique(list_of(11)(12), 2);
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
  SymbolicBayesTree bt;
  bt.insertRoot(root);
  if (debug) {
    GTSAM_PRINT(bt);
    bt.saveGraph("/tmp/SymbolicBayesTree.dot");
  }

  // Shortcut on 9
  {
    SymbolicBayesTree::Clique::shared_ptr c = bt[9];
    SymbolicBayesNet shortcut = c->shortcut(root);
    EXPECT(assert_equal(SymbolicBayesNet(), shortcut));
  }

  // Shortcut on 7
  {
    SymbolicBayesTree::Clique::shared_ptr c = bt[7];
    SymbolicBayesNet shortcut = c->shortcut(root);
    EXPECT(assert_equal(SymbolicBayesNet(), shortcut));
  }

  // Shortcut on 5
  {
    SymbolicBayesTree::Clique::shared_ptr c = bt[5];
    SymbolicBayesNet shortcut = c->shortcut(root);
    SymbolicBayesNet expected = list_of
      (SymbolicConditional(7, 8, 11))
      (SymbolicConditional(8, 11));
    EXPECT(assert_equal(expected, shortcut));
  }

  // Shortcut on 3
  {
    SymbolicBayesTree::Clique::shared_ptr c = bt[3];
    SymbolicBayesNet shortcut = c->shortcut(root);
    SymbolicBayesNet expected = list_of(SymbolicConditional(6, 11));
    EXPECT(assert_equal(expected, shortcut));
  }

  // Shortcut on 1
  {
    SymbolicBayesTree::Clique::shared_ptr c = bt[1];
    SymbolicBayesNet shortcut = c->shortcut(root);
    SymbolicBayesNet expected = list_of(SymbolicConditional(5, 11));
    EXPECT(assert_equal(expected, shortcut));
  }

  // Marginal on 5
  {
    SymbolicFactor::shared_ptr actual = bt.marginalFactor(5);
    EXPECT(assert_equal(SymbolicFactor(5), *actual, 1e-1));
  }

  // Shortcut on 6
  {
    SymbolicFactor::shared_ptr actual = bt.marginalFactor(6);
    EXPECT(assert_equal(SymbolicFactor(6), *actual, 1e-1));
  }

}

/* ************************************************************************* */
TEST(SymbolicBayesTree, COLAMDvsMETIS) {

  // create circular graph
  SymbolicFactorGraph sfg;
  sfg.push_factor(0, 1);
  sfg.push_factor(1, 2);
  sfg.push_factor(2, 3);
  sfg.push_factor(3, 4);
  sfg.push_factor(4, 5);
  sfg.push_factor(0, 5);

  // COLAMD
  {
    Ordering ordering = Ordering::Create(Ordering::COLAMD, sfg);
    EXPECT(assert_equal(Ordering(list_of(0)(5)(1)(4)(2)(3)), ordering));

    //  - P( 4 2 3)
    //  | - P( 1 | 2 4)
    //  | | - P( 5 | 1 4)
    //  | | | - P( 0 | 1 5)
    SymbolicBayesTree expected;
    expected.insertRoot(
        MakeClique(list_of(4)(2)(3), 3,
            list_of(
                MakeClique(list_of(1)(2)(4), 1,
                    list_of(
                        MakeClique(list_of(5)(1)(4), 1,
                            list_of(MakeClique(list_of(0)(1)(5), 1))))))));

    SymbolicBayesTree actual = *sfg.eliminateMultifrontal(ordering);
    EXPECT(assert_equal(expected, actual));
  }

#ifdef GTSAM_SUPPORT_NESTED_DISSECTION
  // METIS
  {
    Ordering ordering = Ordering::Create(Ordering::METIS, sfg);
// Linux and Mac split differently when using mettis
#if !defined(__APPLE__)
    EXPECT(assert_equal(Ordering(list_of(3)(2)(5)(0)(4)(1)), ordering));
#else
    EXPECT(assert_equal(Ordering(list_of(5)(4)(2)(1)(0)(3)), ordering));
#endif

    //  - P( 1 0 3)
    //  | - P( 4 | 0 3)
    //  | | - P( 5 | 0 4)
    //  | - P( 2 | 1 3)
    SymbolicBayesTree expected;
#if !defined(__APPLE__)
    expected.insertRoot(
        MakeClique(list_of(2)(4)(1), 3,
            list_of(
                MakeClique(list_of(0)(1)(4), 1,
                    list_of(MakeClique(list_of(5)(0)(4), 1))))(
                MakeClique(list_of(3)(2)(4), 1))));
#else
    expected.insertRoot(
            MakeClique(list_of(1)(0)(3), 3,
                list_of(
                    MakeClique(list_of(4)(0)(3), 1,
                        list_of(MakeClique(list_of(5)(0)(4), 1))))(
                    MakeClique(list_of(2)(1)(3), 1))));
#endif
    SymbolicBayesTree actual = *sfg.eliminateMultifrontal(ordering);
    EXPECT(assert_equal(expected, actual));
  }
#endif
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */


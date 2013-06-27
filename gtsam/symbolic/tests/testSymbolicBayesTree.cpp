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
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/symbolic/tests/symbolicExampleGraphs.h>

#include <boost/assign/list_of.hpp>
#include <boost/assign/std/vector.hpp>
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;
using namespace gtsam::symbol_shorthand;

static bool debug = false;

/* ************************************************************************* */
// Conditionals for ASIA example from the tutorial with A and D evidence
static SymbolicConditionalUnordered::shared_ptr
  B(new SymbolicConditionalUnordered(_B_)),
  L(new SymbolicConditionalUnordered(_L_, _B_)),
  E(new SymbolicConditionalUnordered(_E_, _L_, _B_)),
  S(new SymbolicConditionalUnordered(_S_, _L_, _B_)),
  T(new SymbolicConditionalUnordered(_T_, _E_, _L_)),
  X(new SymbolicConditionalUnordered(_X_, _E_));

// Cliques
static SymbolicConditionalUnordered::shared_ptr ELB(
  boost::make_shared<SymbolicConditionalUnordered>(
  SymbolicConditionalUnordered::FromKeys(list_of(_E_)(_L_)(_B_), 3)));

/* ************************************************************************* */
TEST_UNSAFE( SymbolicBayesTree, constructor )
{
  // Create using insert
  //SymbolicBayesTreeUnordered bayesTree = createAsiaSymbolicBayesTree();

  //bayesTree.print("bayesTree: ");

  // Check Size
//  LONGS_EQUAL(4, bayesTree.size());
//  EXPECT(!bayesTree.empty());
//
//  // Check root
//  boost::shared_ptr<SymbolicConditionalUnordered> actual_root = bayesTree.root()->conditional();
//  CHECK(assert_equal(*ELB,*actual_root));
//
//  // Create from symbolic Bayes chain in which we want to discover cliques
//  BayesNet<SymbolicConditionalUnordered> ASIA;
//  ASIA.push_back(X);
//  ASIA.push_back(T);
//  ASIA.push_back(S);
//  ASIA.push_back(E);
//  ASIA.push_back(L);
//  ASIA.push_back(B);
//  SymbolicBayesTreeUnordered bayesTree2(ASIA);
//
//  // Check whether the same
//  CHECK(assert_equal(bayesTree,bayesTree2));
//
//  // CHECK findParentClique, should *not depend on order of parents*
////  Ordering ordering; ordering += _X_, _T_, _S_, _E_, _L_, _B_;
////  IndexTable<Symbol> index(ordering);
//
//  list<Index> parents1; parents1 += _E_, _L_;
//  CHECK(assert_equal(_E_, bayesTree.findParentClique(parents1)));
//
//  list<Index> parents2; parents2 += _L_, _E_;
//  CHECK(assert_equal(_E_, bayesTree.findParentClique(parents2)));
//
//  list<Index> parents3; parents3 += _L_, _B_;
//  CHECK(assert_equal(_L_, bayesTree.findParentClique(parents3)));
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

///* ************************************************************************* *
//Bayes Tree for testing conversion to a forest of orphans needed for incremental.
//       A,B
//   C|A    E|B
//   D|C    F|E
//   */
///* ************************************************************************* */
//TEST( BayesTree, removePath )
//{
//  const Index _A_=5, _B_=4, _C_=3, _D_=2, _E_=1, _F_=0;
//  SymbolicConditionalUnordered::shared_ptr
//      A(new SymbolicConditionalUnordered(_A_)),
//      B(new SymbolicConditionalUnordered(_B_, _A_)),
//      C(new SymbolicConditionalUnordered(_C_, _A_)),
//      D(new SymbolicConditionalUnordered(_D_, _C_)),
//      E(new SymbolicConditionalUnordered(_E_, _B_)),
//      F(new SymbolicConditionalUnordered(_F_, _E_));
//  SymbolicBayesTreeUnordered bayesTree;
//  EXPECT(bayesTree.empty());
////  Ordering ord; ord += _A_,_B_,_C_,_D_,_E_,_F_;
//  SymbolicBayesTreeUnordered::insert(bayesTree, A);
//  SymbolicBayesTreeUnordered::insert(bayesTree, B);
//  SymbolicBayesTreeUnordered::insert(bayesTree, C);
//  SymbolicBayesTreeUnordered::insert(bayesTree, D);
//  SymbolicBayesTreeUnordered::insert(bayesTree, E);
//  SymbolicBayesTreeUnordered::insert(bayesTree, F);
//
//  // remove C, expected outcome: factor graph with ABC,
//  // Bayes Tree now contains two orphan trees: D|C and E|B,F|E
//  SymbolicFactorGraph expected;
//  expected.push_factor(_B_,_A_);
////  expected.push_factor(_A_);
//  expected.push_factor(_C_,_A_);
//  SymbolicBayesTreeUnordered::Cliques expectedOrphans;
//  expectedOrphans += bayesTree[_D_], bayesTree[_E_];
//
//  BayesNet<SymbolicConditionalUnordered> bn;
//  SymbolicBayesTreeUnordered::Cliques orphans;
//  bayesTree.removePath(bayesTree[_C_], bn, orphans);
//  SymbolicFactorGraph factors(bn);
//  CHECK(assert_equal((SymbolicFactorGraph)expected, factors));
//  CHECK(assert_equal(expectedOrphans, orphans));
//
//  // remove E: factor graph with EB; E|B removed from second orphan tree
//  SymbolicFactorGraph expected2;
//  expected2.push_factor(_E_,_B_);
//  SymbolicBayesTreeUnordered::Cliques expectedOrphans2;
//  expectedOrphans2 += bayesTree[_F_];
//
//  BayesNet<SymbolicConditionalUnordered> bn2;
//  SymbolicBayesTreeUnordered::Cliques orphans2;
//  bayesTree.removePath(bayesTree[_E_], bn2, orphans2);
//  SymbolicFactorGraph factors2(bn2);
//  CHECK(assert_equal((SymbolicFactorGraph)expected2, factors2));
//  CHECK(assert_equal(expectedOrphans2, orphans2));
//}
//
///* ************************************************************************* */
//TEST( BayesTree, removePath2 )
//{
//  SymbolicBayesTreeUnordered bayesTree = createAsiaSymbolicBayesTree();
//
//  // Call remove-path with clique B
//  BayesNet<SymbolicConditionalUnordered> bn;
//  SymbolicBayesTreeUnordered::Cliques orphans;
//  bayesTree.removePath(bayesTree[_B_], bn, orphans);
//  SymbolicFactorGraph factors(bn);
//
//  // Check expected outcome
//  SymbolicFactorGraph expected;
//  expected.push_factor(_E_,_L_,_B_);
////  expected.push_factor(_L_,_B_);
////  expected.push_factor(_B_);
//  CHECK(assert_equal(expected, factors));
//  SymbolicBayesTreeUnordered::Cliques expectedOrphans;
//  expectedOrphans += bayesTree[_S_], bayesTree[_T_], bayesTree[_X_];
//  CHECK(assert_equal(expectedOrphans, orphans));
//}
//
///* ************************************************************************* */
//TEST( BayesTree, removePath3 )
//{
//  SymbolicBayesTreeUnordered bayesTree = createAsiaSymbolicBayesTree();
//
//  // Call remove-path with clique S
//  BayesNet<SymbolicConditionalUnordered> bn;
//  SymbolicBayesTreeUnordered::Cliques orphans;
//  bayesTree.removePath(bayesTree[_S_], bn, orphans);
//  SymbolicFactorGraph factors(bn);
//
//  // Check expected outcome
//  SymbolicFactorGraph expected;
//  expected.push_factor(_E_,_L_,_B_);
////  expected.push_factor(_L_,_B_);
////  expected.push_factor(_B_);
//  expected.push_factor(_S_,_L_,_B_);
//  CHECK(assert_equal(expected, factors));
//  SymbolicBayesTreeUnordered::Cliques expectedOrphans;
//  expectedOrphans += bayesTree[_T_], bayesTree[_X_];
//  CHECK(assert_equal(expectedOrphans, orphans));
//}
//
//void getAllCliques(const SymbolicBayesTreeUnordered::sharedClique& subtree, SymbolicBayesTreeUnordered::Cliques& cliques)  {
//  // Check if subtree exists
//  if (subtree) {
//    cliques.push_back(subtree);
//    // Recursive call over all child cliques
//    BOOST_FOREACH(SymbolicBayesTreeUnordered::sharedClique& childClique, subtree->children()) {
//      getAllCliques(childClique,cliques);
//    }
//  }
//}
//
///* ************************************************************************* */
//TEST( BayesTree, shortcutCheck )
//{
//  const Index _A_=6, _B_=5, _C_=4, _D_=3, _E_=2, _F_=1, _G_=0;
//  SymbolicConditionalUnordered::shared_ptr
//      A(new SymbolicConditionalUnordered(_A_)),
//      B(new SymbolicConditionalUnordered(_B_, _A_)),
//      C(new SymbolicConditionalUnordered(_C_, _A_)),
//      D(new SymbolicConditionalUnordered(_D_, _C_)),
//      E(new SymbolicConditionalUnordered(_E_, _B_)),
//      F(new SymbolicConditionalUnordered(_F_, _E_)),
//      G(new SymbolicConditionalUnordered(_G_, _F_));
//  SymbolicBayesTreeUnordered bayesTree;
////  Ordering ord; ord += _A_,_B_,_C_,_D_,_E_,_F_;
//  SymbolicBayesTreeUnordered::insert(bayesTree, A);
//  SymbolicBayesTreeUnordered::insert(bayesTree, B);
//  SymbolicBayesTreeUnordered::insert(bayesTree, C);
//  SymbolicBayesTreeUnordered::insert(bayesTree, D);
//  SymbolicBayesTreeUnordered::insert(bayesTree, E);
//  SymbolicBayesTreeUnordered::insert(bayesTree, F);
//  SymbolicBayesTreeUnordered::insert(bayesTree, G);
//
//  //bayesTree.print("BayesTree");
//  //bayesTree.saveGraph("BT1.dot");
//
//  SymbolicBayesTreeUnordered::sharedClique rootClique= bayesTree.root();
//  //rootClique->printTree();
//  SymbolicBayesTreeUnordered::Cliques allCliques;
//  getAllCliques(rootClique,allCliques);
//
//  BayesNet<SymbolicConditionalUnordered> bn;
//  BOOST_FOREACH(SymbolicBayesTreeUnordered::sharedClique& clique, allCliques) {
//    //clique->print("Clique#");
//    bn = clique->shortcut(rootClique, &EliminateSymbolic);
//    //bn.print("Shortcut:\n");
//    //cout << endl;
//  }
//
//  // Check if all the cached shortcuts are cleared
//  rootClique->deleteCachedShortcuts();
//  BOOST_FOREACH(SymbolicBayesTreeUnordered::sharedClique& clique, allCliques) {
//    bool notCleared = clique->cachedSeparatorMarginal();
//    CHECK( notCleared == false);
//  }
//  EXPECT_LONGS_EQUAL(0, rootClique->numCachedSeparatorMarginals());
//
////  BOOST_FOREACH(SymbolicBayesTreeUnordered::sharedClique& clique, allCliques) {
////    clique->print("Clique#");
////    if(clique->cachedShortcut()){
////      bn = clique->cachedShortcut().get();
////      bn.print("Shortcut:\n");
////    }
////    else
////      cout << "Not Initialized" << endl;
////    cout << endl;
////  }
//}
//
///* ************************************************************************* */
//TEST( BayesTree, removeTop )
//{
//  SymbolicBayesTreeUnordered bayesTree = createAsiaSymbolicBayesTree();
//
//  // create a new factor to be inserted
//  boost::shared_ptr<IndexFactor> newFactor(new IndexFactor(_S_,_B_));
//
//  // Remove the contaminated part of the Bayes tree
//  BayesNet<SymbolicConditionalUnordered> bn;
//  SymbolicBayesTreeUnordered::Cliques orphans;
//  list<Index> keys; keys += _B_,_S_;
//  bayesTree.removeTop(keys, bn, orphans);
//  SymbolicFactorGraph factors(bn);
//
//  // Check expected outcome
//  SymbolicFactorGraph expected;
//  expected.push_factor(_E_,_L_,_B_);
////  expected.push_factor(_L_,_B_);
////  expected.push_factor(_B_);
//  expected.push_factor(_S_,_L_,_B_);
//  CHECK(assert_equal(expected, factors));
//  SymbolicBayesTreeUnordered::Cliques expectedOrphans;
//  expectedOrphans += bayesTree[_T_], bayesTree[_X_];
//  CHECK(assert_equal(expectedOrphans, orphans));
//
//  // Try removeTop again with a factor that should not change a thing
//  boost::shared_ptr<IndexFactor> newFactor2(new IndexFactor(_B_));
//  BayesNet<SymbolicConditionalUnordered> bn2;
//  SymbolicBayesTreeUnordered::Cliques orphans2;
//  keys.clear(); keys += _B_;
//  bayesTree.removeTop(keys, bn2, orphans2);
//  SymbolicFactorGraph factors2(bn2);
//  SymbolicFactorGraph expected2;
//  CHECK(assert_equal(expected2, factors2));
//  SymbolicBayesTreeUnordered::Cliques expectedOrphans2;
//  CHECK(assert_equal(expectedOrphans2, orphans2));
//}
//
///* ************************************************************************* */
//TEST( BayesTree, removeTop2 )
//{
//  SymbolicBayesTreeUnordered bayesTree = createAsiaSymbolicBayesTree();
//
//  // create two factors to be inserted
//  SymbolicFactorGraph newFactors;
//  newFactors.push_factor(_B_);
//  newFactors.push_factor(_S_);
//
//  // Remove the contaminated part of the Bayes tree
//  BayesNet<SymbolicConditionalUnordered> bn;
//  SymbolicBayesTreeUnordered::Cliques orphans;
//  list<Index> keys; keys += _B_,_S_;
//  bayesTree.removeTop(keys, bn, orphans);
//  SymbolicFactorGraph factors(bn);
//
//  // Check expected outcome
//  SymbolicFactorGraph expected;
//  expected.push_factor(_E_,_L_,_B_);
////  expected.push_factor(_L_,_B_);
////  expected.push_factor(_B_);
//  expected.push_factor(_S_,_L_,_B_);
//  CHECK(assert_equal(expected, factors));
//  SymbolicBayesTreeUnordered::Cliques expectedOrphans;
//  expectedOrphans += bayesTree[_T_], bayesTree[_X_];
//  CHECK(assert_equal(expectedOrphans, orphans));
//}
//
///* ************************************************************************* */
//TEST( BayesTree, removeTop3 )
//{
//  const Index _x4_=5, _l5_=6;
//  // simple test case that failed after COLAMD was fixed/activated
//  SymbolicConditionalUnordered::shared_ptr
//  X(new SymbolicConditionalUnordered(_l5_)),
//  A(new SymbolicConditionalUnordered(_x4_, _l5_)),
//  B(new SymbolicConditionalUnordered(_x2_, _x4_)),
//  C(new SymbolicConditionalUnordered(_x3_, _x2_));
//
////  Ordering newOrdering;
////  newOrdering += _x3_, _x2_, _x1_, _l2_, _l1_, _x4_, _l5_;
//  SymbolicBayesTreeUnordered bayesTree;
//  SymbolicBayesTreeUnordered::insert(bayesTree, X);
//  SymbolicBayesTreeUnordered::insert(bayesTree, A);
//  SymbolicBayesTreeUnordered::insert(bayesTree, B);
//  SymbolicBayesTreeUnordered::insert(bayesTree, C);
//
//  // remove all
//  list<Index> keys;
//  keys += _l5_, _x2_, _x3_, _x4_;
//  BayesNet<SymbolicConditionalUnordered> bn;
//  SymbolicBayesTreeUnordered::Cliques orphans;
//  bayesTree.removeTop(keys, bn, orphans);
//  SymbolicFactorGraph factors(bn);
//
//  CHECK(orphans.size() == 0);
//}
//
///* ************************************************************************* */
//TEST( BayesTree, permute )
//{
//  // creates a permutation and ensures that the nodes listing is updated
//
//  // initial keys - more than just 6 variables - for a system with 9 variables
//  const Index _A0_=8, _B0_=7, _C0_=6, _D0_=5, _E0_=4, _F0_=0;
//
//  // reduced keys - back to just 6 variables
//  const Index _A_=5, _B_=4, _C_=3, _D_=2, _E_=1, _F_=0;
//
//  // Create and verify the permutation
//  std::set<Index> indices; indices += _A0_, _B0_, _C0_, _D0_, _E0_, _F0_;
//  Permutation actReducingPermutation = gtsam::internal::createReducingPermutation(indices);
//  Permutation expReducingPermutation(6);
//  expReducingPermutation[_A_] = _A0_;
//  expReducingPermutation[_B_] = _B0_;
//  expReducingPermutation[_C_] = _C0_;
//  expReducingPermutation[_D_] = _D0_;
//  expReducingPermutation[_E_] = _E0_;
//  expReducingPermutation[_F_] = _F0_;
//  EXPECT(assert_equal(expReducingPermutation, actReducingPermutation));
//
//  // Invert the permutation
//  gtsam::internal::Reduction inv_reduction = gtsam::internal::Reduction::CreateAsInverse(expReducingPermutation);
//
//  // Build a bayes tree around reduced keys as if just eliminated from subset of factors/variables
//  SymbolicConditionalUnordered::shared_ptr
//      A(new SymbolicConditionalUnordered(_A_)),
//      B(new SymbolicConditionalUnordered(_B_, _A_)),
//      C(new SymbolicConditionalUnordered(_C_, _A_)),
//      D(new SymbolicConditionalUnordered(_D_, _C_)),
//      E(new SymbolicConditionalUnordered(_E_, _B_)),
//      F(new SymbolicConditionalUnordered(_F_, _E_));
//  SymbolicBayesTreeUnordered bayesTreeReduced;
//  SymbolicBayesTreeUnordered::insert(bayesTreeReduced, A);
//  SymbolicBayesTreeUnordered::insert(bayesTreeReduced, B);
//  SymbolicBayesTreeUnordered::insert(bayesTreeReduced, C);
//  SymbolicBayesTreeUnordered::insert(bayesTreeReduced, D);
//  SymbolicBayesTreeUnordered::insert(bayesTreeReduced, E);
//  SymbolicBayesTreeUnordered::insert(bayesTreeReduced, F);
//
////  bayesTreeReduced.print("Reduced bayes tree");
////  P( 4 5)
////    P( 3 | 5)
////      P( 2 | 3)
////    P( 1 | 4)
////      P( 0 | 1)
//
//  // Apply the permutation - should add placeholders for variables not present in nodes
//  SymbolicBayesTreeUnordered actBayesTree = *bayesTreeReduced.clone();
//  actBayesTree.permuteWithInverse(expReducingPermutation);
//
////  actBayesTree.print("Full bayes tree");
////  P( 7 8)
////    P( 6 | 8)
////      P( 5 | 6)
////    P( 4 | 7)
////      P( 0 | 4)
//
//  // check keys in cliques
//  std::vector<Index> expRootIndices; expRootIndices += _B0_, _A0_;
//  SymbolicConditionalUnordered::shared_ptr
//    expRoot(new SymbolicConditionalUnordered(expRootIndices, 2)), // root
//    A0(new SymbolicConditionalUnordered(_A0_)),
//    B0(new SymbolicConditionalUnordered(_B0_, _A0_)),
//    C0(new SymbolicConditionalUnordered(_C0_, _A0_)), // leaf level 1
//    D0(new SymbolicConditionalUnordered(_D0_, _C0_)), // leaf level 2
//    E0(new SymbolicConditionalUnordered(_E0_, _B0_)), // leaf level 2
//    F0(new SymbolicConditionalUnordered(_F0_, _E0_)); // leaf level 3
//
//  CHECK(actBayesTree.root());
//  EXPECT(assert_equal(*expRoot, *actBayesTree.root()->conditional()));
//  EXPECT(assert_equal(*C0, *actBayesTree.root()->children().front()->conditional()));
//  EXPECT(assert_equal(*D0, *actBayesTree.root()->children().front()->children().front()->conditional()));
//  EXPECT(assert_equal(*E0, *actBayesTree.root()->children().back()->conditional()));
//  EXPECT(assert_equal(*F0, *actBayesTree.root()->children().back()->children().front()->conditional()));
//
//  // check nodes structure
//  LONGS_EQUAL(9, actBayesTree.nodes().size());
//
//  SymbolicBayesTreeUnordered expFullTree;
//  SymbolicBayesTreeUnordered::insert(expFullTree, A0);
//  SymbolicBayesTreeUnordered::insert(expFullTree, B0);
//  SymbolicBayesTreeUnordered::insert(expFullTree, C0);
//  SymbolicBayesTreeUnordered::insert(expFullTree, D0);
//  SymbolicBayesTreeUnordered::insert(expFullTree, E0);
//  SymbolicBayesTreeUnordered::insert(expFullTree, F0);
//
//  EXPECT(assert_equal(expFullTree, actBayesTree));
//}
//
/////* ************************************************************************* */
/////**
//// *  x2 - x3 - x4 - x5
//// *   |  /       \   |
//// *  x1 /         \ x6
//// */
////TEST( BayesTree, insert )
////{
////  // construct bayes tree by split the graph along the separator x3 - x4
////  const Index _x1_=0, _x2_=1, _x6_=2, _x5_=3, _x3_=4, _x4_=5;
////  SymbolicFactorGraph fg1, fg2, fg3;
////  fg1.push_factor(_x3_, _x4_);
////  fg2.push_factor(_x1_, _x2_);
////  fg2.push_factor(_x2_, _x3_);
////  fg2.push_factor(_x1_, _x3_);
////  fg3.push_factor(_x5_, _x4_);
////  fg3.push_factor(_x6_, _x5_);
////  fg3.push_factor(_x6_, _x4_);
////
//////  Ordering ordering1; ordering1 += _x3_, _x4_;
//////  Ordering ordering2; ordering2 += _x1_, _x2_;
//////  Ordering ordering3; ordering3 += _x6_, _x5_;
////
////  BayesNet<SymbolicConditionalUnordered> bn1, bn2, bn3;
////  bn1 = *SymbolicSequentialSolver::EliminateUntil(fg1, _x4_+1);
////  bn2 = *SymbolicSequentialSolver::EliminateUntil(fg2, _x2_+1);
////  bn3 = *SymbolicSequentialSolver::EliminateUntil(fg3, _x5_+1);
////
////  // insert child cliques
////  SymbolicBayesTreeUnordered actual;
////  list<SymbolicBayesTreeUnordered::sharedClique> children;
////  SymbolicBayesTreeUnordered::sharedClique r1 = actual.insert(bn2, children);
////  SymbolicBayesTreeUnordered::sharedClique r2 = actual.insert(bn3, children);
////
////  // insert root clique
////  children.push_back(r1);
////  children.push_back(r2);
////  actual.insert(bn1, children, true);
////
////  // traditional way
////  SymbolicFactorGraph fg;
////  fg.push_factor(_x3_, _x4_);
////  fg.push_factor(_x1_, _x2_);
////  fg.push_factor(_x2_, _x3_);
////  fg.push_factor(_x1_, _x3_);
////  fg.push_factor(_x5_, _x4_);
////  fg.push_factor(_x6_, _x5_);
////  fg.push_factor(_x6_, _x4_);
////
//////  Ordering ordering;  ordering += _x1_, _x2_, _x6_, _x5_, _x3_, _x4_;
////  BayesNet<SymbolicConditionalUnordered> bn(*SymbolicSequentialSolver(fg).eliminate());
////  SymbolicBayesTreeUnordered expected(bn);
////  CHECK(assert_equal(expected, actual));
////
////}
//
///* ************************************************************************* */
//
//TEST_UNSAFE( SymbolicBayesTreeUnordered, thinTree ) {
//
//  // create a thin-tree Bayesnet, a la Jean-Guillaume
//  SymbolicBayesNet bayesNet;
//  bayesNet.push_front(boost::make_shared<SymbolicConditionalUnordered>(14));
//
//  bayesNet.push_front(boost::make_shared<SymbolicConditionalUnordered>(13, 14));
//  bayesNet.push_front(boost::make_shared<SymbolicConditionalUnordered>(12, 14));
//
//  bayesNet.push_front(boost::make_shared<SymbolicConditionalUnordered>(11, 13, 14));
//  bayesNet.push_front(boost::make_shared<SymbolicConditionalUnordered>(10, 13, 14));
//  bayesNet.push_front(boost::make_shared<SymbolicConditionalUnordered>(9, 12, 14));
//  bayesNet.push_front(boost::make_shared<SymbolicConditionalUnordered>(8, 12, 14));
//
//  bayesNet.push_front(boost::make_shared<SymbolicConditionalUnordered>(7, 11, 13));
//  bayesNet.push_front(boost::make_shared<SymbolicConditionalUnordered>(6, 11, 13));
//  bayesNet.push_front(boost::make_shared<SymbolicConditionalUnordered>(5, 10, 13));
//  bayesNet.push_front(boost::make_shared<SymbolicConditionalUnordered>(4, 10, 13));
//
//  bayesNet.push_front(boost::make_shared<SymbolicConditionalUnordered>(3, 9, 12));
//  bayesNet.push_front(boost::make_shared<SymbolicConditionalUnordered>(2, 9, 12));
//  bayesNet.push_front(boost::make_shared<SymbolicConditionalUnordered>(1, 8, 12));
//  bayesNet.push_front(boost::make_shared<SymbolicConditionalUnordered>(0, 8, 12));
//
//  if (debug) {
//    GTSAM_PRINT(bayesNet);
//    bayesNet.saveGraph("/tmp/symbolicBayesNet.dot");
//  }
//
//  // create a BayesTree out of a Bayes net
//  SymbolicBayesTreeUnordered bayesTree(bayesNet);
//  if (debug) {
//    GTSAM_PRINT(bayesTree);
//    bayesTree.saveGraph("/tmp/SymbolicBayesTreeUnordered.dot");
//  }
//
//  SymbolicBayesTreeUnordered::Clique::shared_ptr R = bayesTree.root();
//
//  {
//    // check shortcut P(S9||R) to root
//    SymbolicBayesTreeUnordered::Clique::shared_ptr c = bayesTree[9];
//    SymbolicBayesNet shortcut = c->shortcut(R, EliminateSymbolic);
//    SymbolicBayesNet expected;
//    EXPECT(assert_equal(expected, shortcut));
//  }
//
//  {
//    // check shortcut P(S8||R) to root
//    SymbolicBayesTreeUnordered::Clique::shared_ptr c = bayesTree[8];
//    SymbolicBayesNet shortcut = c->shortcut(R, EliminateSymbolic);
//    SymbolicBayesNet expected;
//    expected.push_front(boost::make_shared<SymbolicConditionalUnordered>(12, 14));
//    EXPECT(assert_equal(expected, shortcut));
//  }
//
//  {
//    // check shortcut P(S4||R) to root
//    SymbolicBayesTreeUnordered::Clique::shared_ptr c = bayesTree[4];
//    SymbolicBayesNet shortcut = c->shortcut(R, EliminateSymbolic);
//    SymbolicBayesNet expected;
//    expected.push_front(boost::make_shared<SymbolicConditionalUnordered>(10, 13, 14));
//    EXPECT(assert_equal(expected, shortcut));
//  }
//
//  {
//    // check shortcut P(S2||R) to root
//    SymbolicBayesTreeUnordered::Clique::shared_ptr c = bayesTree[2];
//    SymbolicBayesNet shortcut = c->shortcut(R, EliminateSymbolic);
//    SymbolicBayesNet expected;
//    expected.push_front(boost::make_shared<SymbolicConditionalUnordered>(12, 14));
//    expected.push_front(boost::make_shared<SymbolicConditionalUnordered>(9, 12, 14));
//    EXPECT(assert_equal(expected, shortcut));
//  }
//
//  {
//    // check shortcut P(S0||R) to root
//    SymbolicBayesTreeUnordered::Clique::shared_ptr c = bayesTree[0];
//    SymbolicBayesNet shortcut = c->shortcut(R, EliminateSymbolic);
//    SymbolicBayesNet expected;
//    expected.push_front(boost::make_shared<SymbolicConditionalUnordered>(12, 14));
//    expected.push_front(boost::make_shared<SymbolicConditionalUnordered>(8, 12, 14));
//    EXPECT(assert_equal(expected, shortcut));
//  }
//
//  SymbolicBayesNet::shared_ptr actualJoint;
//
//  // Check joint P(8,2)
//  if (false) { // TODO, not disjoint
//    actualJoint = bayesTree.jointBayesNet(8, 2, EliminateSymbolic);
//    SymbolicBayesNet expected;
//    expected.push_front(boost::make_shared<SymbolicConditionalUnordered>(8));
//    expected.push_front(boost::make_shared<SymbolicConditionalUnordered>(2, 8));
//    EXPECT(assert_equal(expected, *actualJoint));
//  }
//
//  // Check joint P(1,2)
//  if (false) { // TODO, not disjoint
//    actualJoint = bayesTree.jointBayesNet(1, 2, EliminateSymbolic);
//    SymbolicBayesNet expected;
//    expected.push_front(boost::make_shared<SymbolicConditionalUnordered>(2));
//    expected.push_front(boost::make_shared<SymbolicConditionalUnordered>(1, 2));
//    EXPECT(assert_equal(expected, *actualJoint));
//  }
//
//  // Check joint P(2,6)
//  if (true) {
//    actualJoint = bayesTree.jointBayesNet(2, 6, EliminateSymbolic);
//    SymbolicBayesNet expected;
//    expected.push_front(boost::make_shared<SymbolicConditionalUnordered>(6));
//    expected.push_front(boost::make_shared<SymbolicConditionalUnordered>(2, 6));
//    EXPECT(assert_equal(expected, *actualJoint));
//  }
//
//  // Check joint P(4,6)
//  if (false) { // TODO, not disjoint
//    actualJoint = bayesTree.jointBayesNet(4, 6, EliminateSymbolic);
//    SymbolicBayesNet expected;
//    expected.push_front(boost::make_shared<SymbolicConditionalUnordered>(6));
//    expected.push_front(boost::make_shared<SymbolicConditionalUnordered>(4, 6));
//    EXPECT(assert_equal(expected, *actualJoint));
//  }
//}
//
///* ************************************************************************* *
// Bayes tree for smoother with "natural" ordering:
// C1 5 6
// C2   4 : 5
// C3     3 : 4
// C4       2 : 3
// C5         1 : 2
// C6           0 : 1
// **************************************************************************** */
//
//TEST_UNSAFE( SymbolicBayesTreeUnordered, linear_smoother_shortcuts ) {
//  // Create smoother with 7 nodes
//  SymbolicFactorGraph smoother;
//  smoother.push_factor(0);
//  smoother.push_factor(0, 1);
//  smoother.push_factor(1, 2);
//  smoother.push_factor(2, 3);
//  smoother.push_factor(3, 4);
//  smoother.push_factor(4, 5);
//  smoother.push_factor(5, 6);
//
//  BayesNet<SymbolicConditionalUnordered> bayesNet =
//      *SymbolicSequentialSolver(smoother).eliminate();
//
//  if (debug) {
//    GTSAM_PRINT(bayesNet);
//    bayesNet.saveGraph("/tmp/symbolicBayesNet.dot");
//  }
//
//  // create a BayesTree out of a Bayes net
//  SymbolicBayesTreeUnordered bayesTree(bayesNet);
//  if (debug) {
//    GTSAM_PRINT(bayesTree);
//    bayesTree.saveGraph("/tmp/SymbolicBayesTreeUnordered.dot");
//  }
//
//  SymbolicBayesTreeUnordered::Clique::shared_ptr R = bayesTree.root();
//
//  {
//    // check shortcut P(S2||R) to root
//    SymbolicBayesTreeUnordered::Clique::shared_ptr c = bayesTree[4]; // 4 is frontal in C2
//    SymbolicBayesNet shortcut = c->shortcut(R, EliminateSymbolic);
//    SymbolicBayesNet expected;
//    EXPECT(assert_equal(expected, shortcut));
//  }
//
//  {
//    // check shortcut P(S3||R) to root
//    SymbolicBayesTreeUnordered::Clique::shared_ptr c = bayesTree[3]; // 3 is frontal in C3
//    SymbolicBayesNet shortcut = c->shortcut(R, EliminateSymbolic);
//    SymbolicBayesNet expected;
//    expected.push_front(boost::make_shared<SymbolicConditionalUnordered>(4, 5));
//    EXPECT(assert_equal(expected, shortcut));
//  }
//
//  {
//    // check shortcut P(S4||R) to root
//    SymbolicBayesTreeUnordered::Clique::shared_ptr c = bayesTree[2]; // 2 is frontal in C4
//    SymbolicBayesNet shortcut = c->shortcut(R, EliminateSymbolic);
//    SymbolicBayesNet expected;
//    expected.push_front(boost::make_shared<SymbolicConditionalUnordered>(3, 5));
//    EXPECT(assert_equal(expected, shortcut));
//  }
//}
//
///* ************************************************************************* */
//// from testSymbolicJunctionTree, which failed at one point
//TEST(SymbolicBayesTreeUnordered, complicatedMarginal) {
//
//  // Create the conditionals to go in the BayesTree
//  list<Index> L;
//  L = list_of(1)(2)(5);
//  SymbolicConditionalUnordered::shared_ptr R_1_2(new SymbolicConditionalUnordered(L, 2));
//  L = list_of(3)(4)(6);
//  SymbolicConditionalUnordered::shared_ptr R_3_4(new SymbolicConditionalUnordered(L, 2));
//  L = list_of(5)(6)(7)(8);
//  SymbolicConditionalUnordered::shared_ptr R_5_6(new SymbolicConditionalUnordered(L, 2));
//  L = list_of(7)(8)(11);
//  SymbolicConditionalUnordered::shared_ptr R_7_8(new SymbolicConditionalUnordered(L, 2));
//  L = list_of(9)(10)(11)(12);
//  SymbolicConditionalUnordered::shared_ptr R_9_10(new SymbolicConditionalUnordered(L, 2));
//  L = list_of(11)(12);
//  SymbolicConditionalUnordered::shared_ptr R_11_12(new SymbolicConditionalUnordered(L, 2));
//
//  // Symbolic Bayes Tree
//  typedef SymbolicBayesTreeUnordered::Clique Clique;
//  typedef SymbolicBayesTreeUnordered::sharedClique sharedClique;
//
//  // Create Bayes Tree
//  SymbolicBayesTreeUnordered bt;
//  bt.insert(sharedClique(new Clique(R_11_12)));
//  bt.insert(sharedClique(new Clique(R_9_10)));
//  bt.insert(sharedClique(new Clique(R_7_8)));
//  bt.insert(sharedClique(new Clique(R_5_6)));
//  bt.insert(sharedClique(new Clique(R_3_4)));
//  bt.insert(sharedClique(new Clique(R_1_2)));
//  if (debug) {
//    GTSAM_PRINT(bt);
//    bt.saveGraph("/tmp/SymbolicBayesTreeUnordered.dot");
//  }
//
//  SymbolicBayesTreeUnordered::Clique::shared_ptr R = bt.root();
//  SymbolicBayesNet empty;
//
//  // Shortcut on 9
//  {
//    SymbolicBayesTreeUnordered::Clique::shared_ptr c = bt[9];
//    SymbolicBayesNet shortcut = c->shortcut(R, EliminateSymbolic);
//    EXPECT(assert_equal(empty, shortcut));
//  }
//
//  // Shortcut on 7
//  {
//    SymbolicBayesTreeUnordered::Clique::shared_ptr c = bt[7];
//    SymbolicBayesNet shortcut = c->shortcut(R, EliminateSymbolic);
//    EXPECT(assert_equal(empty, shortcut));
//  }
//
//  // Shortcut on 5
//  {
//    SymbolicBayesTreeUnordered::Clique::shared_ptr c = bt[5];
//    SymbolicBayesNet shortcut = c->shortcut(R, EliminateSymbolic);
//    SymbolicBayesNet expected;
//    expected.push_front(boost::make_shared<SymbolicConditionalUnordered>(8, 11));
//    expected.push_front(boost::make_shared<SymbolicConditionalUnordered>(7, 8, 11));
//    EXPECT(assert_equal(expected, shortcut));
//  }
//
//  // Shortcut on 3
//  {
//    SymbolicBayesTreeUnordered::Clique::shared_ptr c = bt[3];
//    SymbolicBayesNet shortcut = c->shortcut(R, EliminateSymbolic);
//    SymbolicBayesNet expected;
//    expected.push_front(boost::make_shared<SymbolicConditionalUnordered>(6, 11));
//    EXPECT(assert_equal(expected, shortcut));
//  }
//
//  // Shortcut on 1
//  {
//    SymbolicBayesTreeUnordered::Clique::shared_ptr c = bt[1];
//    SymbolicBayesNet shortcut = c->shortcut(R, EliminateSymbolic);
//    SymbolicBayesNet expected;
//    expected.push_front(boost::make_shared<SymbolicConditionalUnordered>(5, 11));
//    EXPECT(assert_equal(expected, shortcut));
//  }
//
//  // Marginal on 5
//  {
//    IndexFactor::shared_ptr actual = bt.marginalFactor(5, EliminateSymbolic);
//    EXPECT(assert_equal(IndexFactor(5), *actual, 1e-1));
//  }
//
//  // Shortcut on 6
//  {
//    IndexFactor::shared_ptr actual = bt.marginalFactor(6, EliminateSymbolic);
//    EXPECT(assert_equal(IndexFactor(6), *actual, 1e-1));
//  }
//
//}
/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */


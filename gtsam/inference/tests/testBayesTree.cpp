/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testBayesTree.cpp
 * @brief   Unit tests for Bayes Tree
 * @author  Frank Dellaert
 * @author  Michael Kaess
 * @author  Viorela Ila
 */

#include <boost/assign/std/list.hpp> // for operator +=
#include <boost/assign/std/vector.hpp>
#include <boost/assign/list_of.hpp>
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>

#include <gtsam/inference/SymbolicFactorGraph.h>
#include <gtsam/inference/BayesTree-inl.h>
#include <gtsam/inference/IndexFactor.h>
#include <gtsam/inference/SymbolicSequentialSolver.h>

using namespace std;
using namespace gtsam;

///* ************************************************************************* */
//// SLAM example from RSS sqrtSAM paper
static const Index _x3_=0, _x2_=1;
//static const Index _x1_=2, _l2_=3, _l1_=4; // unused
//IndexConditional::shared_ptr
//		x3(new IndexConditional(_x3_)),
//		x2(new IndexConditional(_x2_,_x3_)),
//		x1(new IndexConditional(_x1_,_x2_,_x3_)),
//		l1(new IndexConditional(_l1_,_x1_,_x2_)),
//		l2(new IndexConditional(_l2_,_x1_,_x3_));
//
//// Bayes Tree for sqrtSAM example
//SymbolicBayesTree createSlamSymbolicBayesTree(){
//	// Create using insert
////	Ordering slamOrdering; slamOrdering += _x3_, _x2_, _x1_, _l2_, _l1_;
//	SymbolicBayesTree bayesTree_slam;
//	bayesTree_slam.insert(x3);
//	bayesTree_slam.insert(x2);
//	bayesTree_slam.insert(x1);
//	bayesTree_slam.insert(l2);
//	bayesTree_slam.insert(l1);
//	return bayesTree_slam;
//}

/* ************************************************************************* */
// Conditionals for ASIA example from the tutorial with A and D evidence
static const Index _X_=0, _T_=1, _S_=2, _E_=3, _L_=4, _B_=5;
static IndexConditional::shared_ptr
	B(new IndexConditional(_B_)),
	L(new IndexConditional(_L_, _B_)),
	E(new IndexConditional(_E_, _L_, _B_)),
	S(new IndexConditional(_S_, _L_, _B_)),
	T(new IndexConditional(_T_, _E_, _L_)),
	X(new IndexConditional(_X_, _E_));

// Cliques
static IndexConditional::shared_ptr
  ELB(IndexConditional::FromKeys(cref_list_of<3>(_E_)(_L_)(_B_), 3));

// Bayes Tree for Asia example
static SymbolicBayesTree createAsiaSymbolicBayesTree() {
	SymbolicBayesTree bayesTree;
//	Ordering asiaOrdering; asiaOrdering += _X_, _T_, _S_, _E_, _L_, _B_;
	SymbolicBayesTree::insert(bayesTree, B);
	SymbolicBayesTree::insert(bayesTree, L);
	SymbolicBayesTree::insert(bayesTree, E);
	SymbolicBayesTree::insert(bayesTree, S);
	SymbolicBayesTree::insert(bayesTree, T);
	SymbolicBayesTree::insert(bayesTree, X);
	return bayesTree;
}

/* ************************************************************************* */
TEST( BayesTree, constructor )
{
	// Create using insert
	SymbolicBayesTree bayesTree = createAsiaSymbolicBayesTree();

	// Check Size
	LONGS_EQUAL(4,bayesTree.size());

	// Check root
	boost::shared_ptr<IndexConditional> actual_root = bayesTree.root()->conditional();
	CHECK(assert_equal(*ELB,*actual_root));

	// Create from symbolic Bayes chain in which we want to discover cliques
	BayesNet<IndexConditional> ASIA;
	ASIA.push_back(X);
	ASIA.push_back(T);
	ASIA.push_back(S);
	ASIA.push_back(E);
	ASIA.push_back(L);
	ASIA.push_back(B);
	SymbolicBayesTree bayesTree2(ASIA);

	// Check whether the same
	CHECK(assert_equal(bayesTree,bayesTree2));

	// CHECK findParentClique, should *not depend on order of parents*
//	Ordering ordering; ordering += _X_, _T_, _S_, _E_, _L_, _B_;
//	IndexTable<Symbol> index(ordering);

	list<Index> parents1; parents1 += _E_, _L_;
	CHECK(assert_equal(_E_, bayesTree.findParentClique(parents1)));

	list<Index> parents2; parents2 += _L_, _E_;
	CHECK(assert_equal(_E_, bayesTree.findParentClique(parents2)));

	list<Index> parents3; parents3 += _L_, _B_;
	CHECK(assert_equal(_L_, bayesTree.findParentClique(parents3)));
}

/* ************************************************************************* */
TEST(BayesTree, clear)
{
//	SymbolicBayesTree bayesTree = createAsiaSymbolicBayesTree();
//	bayesTree.clear();
//
//	SymbolicBayesTree expected;
//
//	// Check whether cleared BayesTree is equal to a new BayesTree
//	CHECK(assert_equal(expected, bayesTree));
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
  const Index _A_=5, _B_=4, _C_=3, _D_=2, _E_=1, _F_=0;
	IndexConditional::shared_ptr
			A(new IndexConditional(_A_)),
			B(new IndexConditional(_B_, _A_)),
			C(new IndexConditional(_C_, _A_)),
			D(new IndexConditional(_D_, _C_)),
			E(new IndexConditional(_E_, _B_)),
			F(new IndexConditional(_F_, _E_));
	SymbolicBayesTree bayesTree;
//	Ordering ord; ord += _A_,_B_,_C_,_D_,_E_,_F_;
	SymbolicBayesTree::insert(bayesTree, A);
	SymbolicBayesTree::insert(bayesTree, B);
	SymbolicBayesTree::insert(bayesTree, C);
	SymbolicBayesTree::insert(bayesTree, D);
	SymbolicBayesTree::insert(bayesTree, E);
	SymbolicBayesTree::insert(bayesTree, F);

	// remove C, expected outcome: factor graph with ABC,
	// Bayes Tree now contains two orphan trees: D|C and E|B,F|E
	SymbolicFactorGraph expected;
	expected.push_factor(_B_,_A_);
//	expected.push_factor(_A_);
	expected.push_factor(_C_,_A_);
	SymbolicBayesTree::Cliques expectedOrphans;
  expectedOrphans += bayesTree[_D_], bayesTree[_E_];

  BayesNet<IndexConditional> bn;
	SymbolicBayesTree::Cliques orphans;
	bayesTree.removePath(bayesTree[_C_], bn, orphans);
	SymbolicFactorGraph factors(bn);
  CHECK(assert_equal((SymbolicFactorGraph)expected, factors));
  CHECK(assert_equal(expectedOrphans, orphans));

  // remove E: factor graph with EB; E|B removed from second orphan tree
	SymbolicFactorGraph expected2;
  expected2.push_factor(_E_,_B_);
  SymbolicBayesTree::Cliques expectedOrphans2;
  expectedOrphans2 += bayesTree[_F_];

  BayesNet<IndexConditional> bn2;
	SymbolicBayesTree::Cliques orphans2;
  bayesTree.removePath(bayesTree[_E_], bn2, orphans2);
  SymbolicFactorGraph factors2(bn2);
  CHECK(assert_equal((SymbolicFactorGraph)expected2, factors2));
  CHECK(assert_equal(expectedOrphans2, orphans2));
}

/* ************************************************************************* */
TEST( BayesTree, removePath2 )
{
	SymbolicBayesTree bayesTree = createAsiaSymbolicBayesTree();

	// Call remove-path with clique B
	BayesNet<IndexConditional> bn;
	SymbolicBayesTree::Cliques orphans;
  bayesTree.removePath(bayesTree[_B_], bn, orphans);
	SymbolicFactorGraph factors(bn);

	// Check expected outcome
	SymbolicFactorGraph expected;
	expected.push_factor(_E_,_L_,_B_);
//	expected.push_factor(_L_,_B_);
//	expected.push_factor(_B_);
  CHECK(assert_equal(expected, factors));
	SymbolicBayesTree::Cliques expectedOrphans;
  expectedOrphans += bayesTree[_S_], bayesTree[_T_], bayesTree[_X_];
  CHECK(assert_equal(expectedOrphans, orphans));
}

/* ************************************************************************* */
TEST( BayesTree, removePath3 )
{
	SymbolicBayesTree bayesTree = createAsiaSymbolicBayesTree();

	// Call remove-path with clique S
	BayesNet<IndexConditional> bn;
	SymbolicBayesTree::Cliques orphans;
  bayesTree.removePath(bayesTree[_S_], bn, orphans);
	SymbolicFactorGraph factors(bn);

	// Check expected outcome
	SymbolicFactorGraph expected;
	expected.push_factor(_E_,_L_,_B_);
//	expected.push_factor(_L_,_B_);
//	expected.push_factor(_B_);
	expected.push_factor(_S_,_L_,_B_);
  CHECK(assert_equal(expected, factors));
	SymbolicBayesTree::Cliques expectedOrphans;
  expectedOrphans += bayesTree[_T_], bayesTree[_X_];
  CHECK(assert_equal(expectedOrphans, orphans));
}

void getAllCliques(const SymbolicBayesTree::sharedClique& subtree, SymbolicBayesTree::Cliques& cliques)	{
	// Check if subtree exists
	if (subtree) {
		cliques.push_back(subtree);
		// Recursive call over all child cliques
		BOOST_FOREACH(SymbolicBayesTree::sharedClique& childClique, subtree->children()) {
			getAllCliques(childClique,cliques);
		}
	}
}

/* ************************************************************************* */
TEST( BayesTree, shortcutCheck )
{
  const Index _A_=6, _B_=5, _C_=4, _D_=3, _E_=2, _F_=1, _G_=0;
	IndexConditional::shared_ptr
			A(new IndexConditional(_A_)),
			B(new IndexConditional(_B_, _A_)),
			C(new IndexConditional(_C_, _A_)),
			D(new IndexConditional(_D_, _C_)),
			E(new IndexConditional(_E_, _B_)),
			F(new IndexConditional(_F_, _E_)),
			G(new IndexConditional(_G_, _F_));
	SymbolicBayesTree bayesTree;
//	Ordering ord; ord += _A_,_B_,_C_,_D_,_E_,_F_;
	SymbolicBayesTree::insert(bayesTree, A);
	SymbolicBayesTree::insert(bayesTree, B);
	SymbolicBayesTree::insert(bayesTree, C);
	SymbolicBayesTree::insert(bayesTree, D);
	SymbolicBayesTree::insert(bayesTree, E);
	SymbolicBayesTree::insert(bayesTree, F);
	SymbolicBayesTree::insert(bayesTree, G);

	//bayesTree.print("BayesTree");
	//bayesTree.saveGraph("BT1.dot");

	SymbolicBayesTree::sharedClique rootClique= bayesTree.root();
	//rootClique->printTree();
	SymbolicBayesTree::Cliques allCliques;
	getAllCliques(rootClique,allCliques);

	BayesNet<IndexConditional> bn;
	BOOST_FOREACH(SymbolicBayesTree::sharedClique& clique, allCliques) {
		//clique->print("Clique#");
		bn = clique->shortcut(rootClique, &EliminateSymbolic);
		//bn.print("Shortcut:\n");
		//cout << endl;
	}

	// Check if all the cached shortcuts are cleared
	rootClique->deleteCachedShorcuts();
	BOOST_FOREACH(SymbolicBayesTree::sharedClique& clique, allCliques) {
		bool notCleared = clique->cachedShortcut();
		CHECK( notCleared == false);
	}

//	BOOST_FOREACH(SymbolicBayesTree::sharedClique& clique, allCliques) {
//		clique->print("Clique#");
//		if(clique->cachedShortcut()){
//			bn = clique->cachedShortcut().get();
//			bn.print("Shortcut:\n");
//		}
//		else
//			cout << "Not Initialized" << endl;
//		cout << endl;
//	}
}



/* ************************************************************************* */
TEST( BayesTree, removeTop )
{
	SymbolicBayesTree bayesTree = createAsiaSymbolicBayesTree();

	// create a new factor to be inserted
	boost::shared_ptr<IndexFactor> newFactor(new IndexFactor(_S_,_B_));

	// Remove the contaminated part of the Bayes tree
	BayesNet<IndexConditional> bn;
	SymbolicBayesTree::Cliques orphans;
	list<Index> keys; keys += _B_,_S_;
	bayesTree.removeTop(keys, bn, orphans);
	SymbolicFactorGraph factors(bn);

	// Check expected outcome
	SymbolicFactorGraph expected;
	expected.push_factor(_E_,_L_,_B_);
//	expected.push_factor(_L_,_B_);
//	expected.push_factor(_B_);
	expected.push_factor(_S_,_L_,_B_);
  CHECK(assert_equal(expected, factors));
	SymbolicBayesTree::Cliques expectedOrphans;
  expectedOrphans += bayesTree[_T_], bayesTree[_X_];
  CHECK(assert_equal(expectedOrphans, orphans));

  // Try removeTop again with a factor that should not change a thing
	boost::shared_ptr<IndexFactor> newFactor2(new IndexFactor(_B_));
	BayesNet<IndexConditional> bn2;
	SymbolicBayesTree::Cliques orphans2;
	keys.clear(); keys += _B_;
	bayesTree.removeTop(keys, bn2, orphans2);
	SymbolicFactorGraph factors2(bn2);
	SymbolicFactorGraph expected2;
  CHECK(assert_equal(expected2, factors2));
	SymbolicBayesTree::Cliques expectedOrphans2;
  CHECK(assert_equal(expectedOrphans2, orphans2));
}

/* ************************************************************************* */
TEST( BayesTree, removeTop2 )
{
	SymbolicBayesTree bayesTree = createAsiaSymbolicBayesTree();

	// create two factors to be inserted
	SymbolicFactorGraph newFactors;
	newFactors.push_factor(_B_);
	newFactors.push_factor(_S_);

	// Remove the contaminated part of the Bayes tree
	BayesNet<IndexConditional> bn;
	SymbolicBayesTree::Cliques orphans;
  list<Index> keys; keys += _B_,_S_;
	bayesTree.removeTop(keys, bn, orphans);
	SymbolicFactorGraph factors(bn);

	// Check expected outcome
	SymbolicFactorGraph expected;
	expected.push_factor(_E_,_L_,_B_);
//	expected.push_factor(_L_,_B_);
//	expected.push_factor(_B_);
	expected.push_factor(_S_,_L_,_B_);
  CHECK(assert_equal(expected, factors));
	SymbolicBayesTree::Cliques expectedOrphans;
  expectedOrphans += bayesTree[_T_], bayesTree[_X_];
	CHECK(assert_equal(expectedOrphans, orphans));
}

/* ************************************************************************* */
TEST( BayesTree, removeTop3 )
{
  const Index _x4_=5, _l5_=6;
	// simple test case that failed after COLAMD was fixed/activated
	IndexConditional::shared_ptr
	X(new IndexConditional(_l5_)),
	A(new IndexConditional(_x4_, _l5_)),
	B(new IndexConditional(_x2_, _x4_)),
	C(new IndexConditional(_x3_, _x2_));

//	Ordering newOrdering;
//	newOrdering += _x3_, _x2_, _x1_, _l2_, _l1_, _x4_, _l5_;
	SymbolicBayesTree bayesTree;
	SymbolicBayesTree::insert(bayesTree, X);
	SymbolicBayesTree::insert(bayesTree, A);
	SymbolicBayesTree::insert(bayesTree, B);
	SymbolicBayesTree::insert(bayesTree, C);

	// remove all
	list<Index> keys;
	keys += _l5_, _x2_, _x3_, _x4_;
	BayesNet<IndexConditional> bn;
	SymbolicBayesTree::Cliques orphans;
	bayesTree.removeTop(keys, bn, orphans);
	SymbolicFactorGraph factors(bn);

	CHECK(orphans.size() == 0);
}
///* ************************************************************************* */
///**
// *  x2 - x3 - x4 - x5
// *   |  /       \   |
// *  x1 /				 \ x6
// */
//TEST( BayesTree, insert )
//{
//	// construct bayes tree by split the graph along the separator x3 - x4
//	const Index _x1_=0, _x2_=1, _x6_=2, _x5_=3, _x3_=4, _x4_=5;
//	SymbolicFactorGraph fg1, fg2, fg3;
//	fg1.push_factor(_x3_, _x4_);
//	fg2.push_factor(_x1_, _x2_);
//	fg2.push_factor(_x2_, _x3_);
//	fg2.push_factor(_x1_, _x3_);
//	fg3.push_factor(_x5_, _x4_);
//	fg3.push_factor(_x6_, _x5_);
//	fg3.push_factor(_x6_, _x4_);
//
////	Ordering ordering1; ordering1 += _x3_, _x4_;
////	Ordering ordering2; ordering2 += _x1_, _x2_;
////	Ordering ordering3; ordering3 += _x6_, _x5_;
//
//	BayesNet<IndexConditional> bn1, bn2, bn3;
//	bn1 = *SymbolicSequentialSolver::EliminateUntil(fg1, _x4_+1);
//	bn2 = *SymbolicSequentialSolver::EliminateUntil(fg2, _x2_+1);
//	bn3 = *SymbolicSequentialSolver::EliminateUntil(fg3, _x5_+1);
//
//	// insert child cliques
//	SymbolicBayesTree actual;
//	list<SymbolicBayesTree::sharedClique> children;
//	SymbolicBayesTree::sharedClique r1 = actual.insert(bn2, children);
//	SymbolicBayesTree::sharedClique r2 = actual.insert(bn3, children);
//
//	// insert root clique
//	children.push_back(r1);
//	children.push_back(r2);
//	actual.insert(bn1, children, true);
//
//	// traditional way
//	SymbolicFactorGraph fg;
//	fg.push_factor(_x3_, _x4_);
//	fg.push_factor(_x1_, _x2_);
//	fg.push_factor(_x2_, _x3_);
//	fg.push_factor(_x1_, _x3_);
//  fg.push_factor(_x5_, _x4_);
//  fg.push_factor(_x6_, _x5_);
//  fg.push_factor(_x6_, _x4_);
//
////	Ordering ordering;  ordering += _x1_, _x2_, _x6_, _x5_, _x3_, _x4_;
//	BayesNet<IndexConditional> bn(*SymbolicSequentialSolver(fg).eliminate());
//	SymbolicBayesTree expected(bn);
//	CHECK(assert_equal(expected, actual));
//
//}
/* ************************************************************************* */

int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

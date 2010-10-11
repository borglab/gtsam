/**
 * @file    testBayesTree.cpp
 * @brief   Unit tests for Bayes Tree
 * @author  Frank Dellaert
 * @author  Michael Kaess
 * @author  Viorela Ila
 */

#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include <gtsam/CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>

#include <gtsam/inference/SymbolicFactorGraph.h>
#include <gtsam/inference/BayesTree-inl.h>
#include <gtsam/inference/Factor-inl.h>
#include <gtsam/inference/inference-inl.h>

using namespace gtsam;

typedef BayesTree<Conditional> SymbolicBayesTree;

///* ************************************************************************* */
//// SLAM example from RSS sqrtSAM paper
static const Index _x3_=0, _x2_=1, _x1_=2, _l2_=3, _l1_=4;
//Conditional::shared_ptr
//		x3(new Conditional(_x3_)),
//		x2(new Conditional(_x2_,_x3_)),
//		x1(new Conditional(_x1_,_x2_,_x3_)),
//		l1(new Conditional(_l1_,_x1_,_x2_)),
//		l2(new Conditional(_l2_,_x1_,_x3_));
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
Conditional::shared_ptr
	B(new Conditional(_B_)),
	L(new Conditional(_L_, _B_)),
	E(new Conditional(_E_, _L_, _B_)),
	S(new Conditional(_S_, _L_, _B_)),
	T(new Conditional(_T_, _E_, _L_)),
	X(new Conditional(_X_, _E_));

// Bayes Tree for Asia example
SymbolicBayesTree createAsiaSymbolicBayesTree() {
	SymbolicBayesTree bayesTree;
//	Ordering asiaOrdering; asiaOrdering += _X_, _T_, _S_, _E_, _L_, _B_;
	bayesTree.insert(B);
	bayesTree.insert(L);
	bayesTree.insert(E);
	bayesTree.insert(S);
	bayesTree.insert(T);
	bayesTree.insert(X);
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
	BayesNet<Conditional> expected_root;
	expected_root.push_back(E);
	expected_root.push_back(L);
	expected_root.push_back(B);
	boost::shared_ptr<BayesNet<Conditional> > actual_root = bayesTree.root();
	CHECK(assert_equal(expected_root,*actual_root));

	// Create from symbolic Bayes chain in which we want to discover cliques
	BayesNet<Conditional> ASIA;
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
	Conditional::shared_ptr
			A(new Conditional(_A_)),
			B(new Conditional(_B_, _A_)),
			C(new Conditional(_C_, _A_)),
			D(new Conditional(_D_, _C_)),
			E(new Conditional(_E_, _B_)),
			F(new Conditional(_F_, _E_));
	SymbolicBayesTree bayesTree;
//	Ordering ord; ord += _A_,_B_,_C_,_D_,_E_,_F_;
	bayesTree.insert(A);
	bayesTree.insert(B);
	bayesTree.insert(C);
	bayesTree.insert(D);
	bayesTree.insert(E);
	bayesTree.insert(F);

	// remove C, expected outcome: factor graph with ABC,
	// Bayes Tree now contains two orphan trees: D|C and E|B,F|E
	SymbolicFactorGraph expected;
	expected.push_factor(_B_,_A_);
	expected.push_factor(_A_);
	expected.push_factor(_C_,_A_);
	SymbolicBayesTree::Cliques expectedOrphans;
  expectedOrphans += bayesTree[_D_], bayesTree[_E_];

  BayesNet<Conditional> bn;
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

  BayesNet<Conditional> bn2;
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
	BayesNet<Conditional> bn;
	SymbolicBayesTree::Cliques orphans;
  bayesTree.removePath(bayesTree[_B_], bn, orphans);
	SymbolicFactorGraph factors(bn);

	// Check expected outcome
	SymbolicFactorGraph expected;
	expected.push_factor(_E_,_L_,_B_);
	expected.push_factor(_L_,_B_);
	expected.push_factor(_B_);
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
	BayesNet<Conditional> bn;
	SymbolicBayesTree::Cliques orphans;
  bayesTree.removePath(bayesTree[_S_], bn, orphans);
	SymbolicFactorGraph factors(bn);

	// Check expected outcome
	SymbolicFactorGraph expected;
	expected.push_factor(_E_,_L_,_B_);
	expected.push_factor(_L_,_B_);
	expected.push_factor(_B_);
	expected.push_factor(_S_,_L_,_B_);
  CHECK(assert_equal(expected, factors));
	SymbolicBayesTree::Cliques expectedOrphans;
  expectedOrphans += bayesTree[_T_], bayesTree[_X_];
  CHECK(assert_equal(expectedOrphans, orphans));
}

/* ************************************************************************* */
TEST( BayesTree, removeTop )
{
	SymbolicBayesTree bayesTree = createAsiaSymbolicBayesTree();

	// create a new factor to be inserted
	boost::shared_ptr<Factor> newFactor(new Factor(_S_,_B_));

	// Remove the contaminated part of the Bayes tree
	BayesNet<Conditional> bn;
	SymbolicBayesTree::Cliques orphans;
	list<Index> keys; keys += _B_,_S_;
	bayesTree.removeTop(keys, bn, orphans);
	SymbolicFactorGraph factors(bn);

	// Check expected outcome
	SymbolicFactorGraph expected;
	expected.push_factor(_E_,_L_,_B_);
	expected.push_factor(_L_,_B_);
	expected.push_factor(_B_);
	expected.push_factor(_S_,_L_,_B_);
  CHECK(assert_equal(expected, factors));
	SymbolicBayesTree::Cliques expectedOrphans;
  expectedOrphans += bayesTree[_T_], bayesTree[_X_];
  CHECK(assert_equal(expectedOrphans, orphans));

  // Try removeTop again with a factor that should not change a thing
	boost::shared_ptr<Factor> newFactor2(new Factor(_B_));
	BayesNet<Conditional> bn2;
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
	BayesNet<Conditional> bn;
	SymbolicBayesTree::Cliques orphans;
  list<Index> keys; keys += _B_,_S_;
	bayesTree.removeTop(keys, bn, orphans);
	SymbolicFactorGraph factors(bn);

	// Check expected outcome
	SymbolicFactorGraph expected;
	expected.push_factor(_E_,_L_,_B_);
	expected.push_factor(_L_,_B_);
	expected.push_factor(_B_);
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
	Conditional::shared_ptr
	X(new Conditional(_l5_)),
	A(new Conditional(_x4_, _l5_)),
	B(new Conditional(_x2_, _x4_)),
	C(new Conditional(_x3_, _x2_));

//	Ordering newOrdering;
//	newOrdering += _x3_, _x2_, _x1_, _l2_, _l1_, _x4_, _l5_;
	SymbolicBayesTree bayesTree;
	bayesTree.insert(X);
	bayesTree.insert(A);
	bayesTree.insert(B);
	bayesTree.insert(C);

	// remove all
	list<Index> keys;
	keys += _l5_, _x2_, _x3_, _x4_;
	BayesNet<Conditional> bn;
	SymbolicBayesTree::Cliques orphans;
	bayesTree.removeTop(keys, bn, orphans);
	SymbolicFactorGraph factors(bn);

	CHECK(orphans.size() == 0);
}
/* ************************************************************************* */
/**
 *  x2 - x3 - x4 - x5
 *   |  /       \   |
 *  x1 /				 \ x6
 */
TEST( BayesTree, insert )
{
	// construct bayes tree by split the graph along the separator x3 - x4
	const Index _x1_=0, _x2_=1, _x6_=2, _x5_=3, _x3_=4, _x4_=5;
	SymbolicFactorGraph fg1, fg2, fg3;
	fg1.push_factor(_x3_, _x4_);
	fg2.push_factor(_x1_, _x2_);
	fg2.push_factor(_x2_, _x3_);
	fg2.push_factor(_x1_, _x3_);
	fg3.push_factor(_x5_, _x4_);
	fg3.push_factor(_x6_, _x5_);
	fg3.push_factor(_x6_, _x4_);

//	Ordering ordering1; ordering1 += _x3_, _x4_;
//	Ordering ordering2; ordering2 += _x1_, _x2_;
//	Ordering ordering3; ordering3 += _x6_, _x5_;

	BayesNet<Conditional> bn1, bn2, bn3;
	bn1 = *Inference::EliminateUntil(fg1, _x4_+1);
	bn2 = *Inference::EliminateUntil(fg2, _x2_+1);
	bn3 = *Inference::EliminateUntil(fg3, _x5_+1);

	// insert child cliques
	SymbolicBayesTree actual;
	list<SymbolicBayesTree::sharedClique> children;
	SymbolicBayesTree::sharedClique r1 = actual.insert(bn2, children);
	SymbolicBayesTree::sharedClique r2 = actual.insert(bn3, children);

	// insert root clique
	children.push_back(r1);
	children.push_back(r2);
	actual.insert(bn1, children, true);

	// traditional way
	SymbolicFactorGraph fg;
	fg.push_factor(_x3_, _x4_);
	fg.push_factor(_x1_, _x2_);
	fg.push_factor(_x2_, _x3_);
	fg.push_factor(_x1_, _x3_);
  fg.push_factor(_x5_, _x4_);
  fg.push_factor(_x6_, _x5_);
  fg.push_factor(_x6_, _x4_);

//	Ordering ordering;  ordering += _x1_, _x2_, _x6_, _x5_, _x3_, _x4_;
	BayesNet<Conditional> bn;
	bn = *Inference::Eliminate(fg);
	SymbolicBayesTree expected(bn);
	CHECK(assert_equal(expected, actual));

}
/* ************************************************************************* */

int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

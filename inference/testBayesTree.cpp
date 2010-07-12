/**
 * @file    testBayesTree.cpp
 * @brief   Unit tests for Bayes Tree
 * @author  Frank Dellaert
 * @author  Michael Kaess
 * @author  Viorela Ila
 */

#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>

#include "SymbolicBayesNet.h"
#include "SymbolicFactorGraph.h"
#include "Ordering.h"
#include "BayesTree-inl.h"
#include "IndexTable.h"

using namespace gtsam;

typedef BayesTree<SymbolicConditional> SymbolicBayesTree;

/* ************************************************************************* */
// SLAM example from RSS sqrtSAM paper
Symbol _x1_('x', 1), _x2_('x', 2), _x3_('x', 3), _l1_('l', 1), _l2_('l', 2);
SymbolicConditional::shared_ptr
		x3(new SymbolicConditional(_x3_)),
		x2(new SymbolicConditional(_x2_,_x3_)),
		x1(new SymbolicConditional(_x1_,_x2_,_x3_)),
		l1(new SymbolicConditional(_l1_,_x1_,_x2_)),
		l2(new SymbolicConditional(_l2_,_x1_,_x3_));

// Bayes Tree for sqrtSAM example
SymbolicBayesTree createSlamSymbolicBayesTree(){
	// Create using insert
	Ordering slamOrdering; slamOrdering += _x3_, _x2_, _x1_, _l2_, _l1_;
	SymbolicBayesTree bayesTree_slam;
	bayesTree_slam.insert(x3,slamOrdering);
	bayesTree_slam.insert(x2,slamOrdering);
	bayesTree_slam.insert(x1,slamOrdering);
	bayesTree_slam.insert(l2,slamOrdering);
	bayesTree_slam.insert(l1,slamOrdering);
	return bayesTree_slam;
}

/* ************************************************************************* */
// Conditionals for ASIA example from the tutorial with A and D evidence
Symbol _B_('B', 0), _L_('L', 0), _E_('E', 0), _S_('S', 0), _T_('T', 0), _X_('X',0);
SymbolicConditional::shared_ptr
	B(new SymbolicConditional(_B_)),
	L(new SymbolicConditional(_L_, _B_)),
	E(new SymbolicConditional(_E_, _B_, _L_)),
	S(new SymbolicConditional(_S_, _L_, _B_)),
	T(new SymbolicConditional(_T_, _E_, _L_)),
	X(new SymbolicConditional(_X_, _E_));

// Bayes Tree for Asia example
SymbolicBayesTree createAsiaSymbolicBayesTree() {
	SymbolicBayesTree bayesTree;
	Ordering asiaOrdering; asiaOrdering += _X_, _T_, _S_, _E_, _L_, _B_;
	bayesTree.insert(B,asiaOrdering);
	bayesTree.insert(L,asiaOrdering);
	bayesTree.insert(E,asiaOrdering);
	bayesTree.insert(S,asiaOrdering);
	bayesTree.insert(T,asiaOrdering);
	bayesTree.insert(X,asiaOrdering);
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
	BayesNet<SymbolicConditional> expected_root;
	expected_root.push_back(E);
	expected_root.push_back(L);
	expected_root.push_back(B);
	boost::shared_ptr<SymbolicBayesNet> actual_root = bayesTree.root();
	CHECK(assert_equal(expected_root,*actual_root));

	// Create from symbolic Bayes chain in which we want to discover cliques
	SymbolicBayesNet ASIA;
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
	Ordering ordering; ordering += _X_, _T_, _S_, _E_, _L_, _B_;
	IndexTable<Symbol> index(ordering);

	list<Symbol> parents1; parents1 += _E_, _L_;
	CHECK(assert_equal(_E_,bayesTree.findParentClique(parents1, index)));

	list<Symbol> parents2; parents2 += _L_, _E_;
	CHECK(assert_equal(_E_,bayesTree.findParentClique(parents2, index)));

	list<Symbol> parents3; parents3 += _L_, _B_;
	CHECK(assert_equal(_L_,bayesTree.findParentClique(parents3, index)));
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
/* ************************************************************************* */
TEST( BayesTree, removePath )
{
	Symbol _A_('A', 0), _B_('B', 0), _C_('C', 0), _D_('D', 0), _E_('E', 0), _F_('F',0);
	SymbolicConditional::shared_ptr
			A(new SymbolicConditional(_A_)),
			B(new SymbolicConditional(_B_, _A_)),
			C(new SymbolicConditional(_C_, _A_)),
			D(new SymbolicConditional(_D_, _C_)),
			E(new SymbolicConditional(_E_, _B_)),
			F(new SymbolicConditional(_F_, _E_));
	SymbolicBayesTree bayesTree;
	Ordering ord; ord += _A_,_B_,_C_,_D_,_E_,_F_;
	bayesTree.insert(A,ord);
	bayesTree.insert(B,ord);
	bayesTree.insert(C,ord);
	bayesTree.insert(D,ord);
	bayesTree.insert(E,ord);
	bayesTree.insert(F,ord);

	// remove C, expected outcome: factor graph with ABC,
	// Bayes Tree now contains two orphan trees: D|C and E|B,F|E
	SymbolicFactorGraph expected;
	expected.push_factor(_A_,_B_);
	expected.push_factor(_A_);
	expected.push_factor(_A_,_C_);
	SymbolicBayesTree::Cliques expectedOrphans;
  expectedOrphans += bayesTree[_D_], bayesTree[_E_];

  BayesNet<SymbolicConditional> bn;
	SymbolicBayesTree::Cliques orphans;
	bayesTree.removePath(bayesTree[_C_], bn, orphans);
	FactorGraph<SymbolicFactor> factors(bn);
  CHECK(assert_equal((FactorGraph<SymbolicFactor>)expected, factors));
  CHECK(assert_equal(expectedOrphans, orphans));

  // remove E: factor graph with EB; E|B removed from second orphan tree
	SymbolicFactorGraph expected2;
  expected2.push_factor(_B_,_E_);
  SymbolicBayesTree::Cliques expectedOrphans2;
  expectedOrphans2 += bayesTree[_F_];

  BayesNet<SymbolicConditional> bn2;
	SymbolicBayesTree::Cliques orphans2;
  bayesTree.removePath(bayesTree[_E_], bn2, orphans2);
  FactorGraph<SymbolicFactor> factors2(bn2);
  CHECK(assert_equal((FactorGraph<SymbolicFactor>)expected2, factors2));
  CHECK(assert_equal(expectedOrphans2, orphans2));
}

/* ************************************************************************* */
TEST( BayesTree, removePath2 )
{
	SymbolicBayesTree bayesTree = createAsiaSymbolicBayesTree();

	// Call remove-path with clique B
	BayesNet<SymbolicConditional> bn;
	SymbolicBayesTree::Cliques orphans;
  bayesTree.removePath(bayesTree[_B_], bn, orphans);
	FactorGraph<SymbolicFactor> factors(bn);

	// Check expected outcome
	SymbolicFactorGraph expected;
	expected.push_factor(_B_,_L_,_E_);
	expected.push_factor(_B_,_L_);
	expected.push_factor(_B_);
  CHECK(assert_equal((FactorGraph<SymbolicFactor>)expected, factors));
	SymbolicBayesTree::Cliques expectedOrphans;
  expectedOrphans += bayesTree[_S_], bayesTree[_T_], bayesTree[_X_];
  CHECK(assert_equal(expectedOrphans, orphans));
}

/* ************************************************************************* */
TEST( BayesTree, removePath3 )
{
	SymbolicBayesTree bayesTree = createAsiaSymbolicBayesTree();

	// Call remove-path with clique S
	BayesNet<SymbolicConditional> bn;
	SymbolicBayesTree::Cliques orphans;
  bayesTree.removePath(bayesTree[_S_], bn, orphans);
	FactorGraph<SymbolicFactor> factors(bn);

	// Check expected outcome
	SymbolicFactorGraph expected;
	expected.push_factor(_B_,_L_,_E_);
	expected.push_factor(_B_,_L_);
	expected.push_factor(_B_);
	expected.push_factor(_L_,_B_,_S_);
  CHECK(assert_equal((FactorGraph<SymbolicFactor>)expected, factors));
	SymbolicBayesTree::Cliques expectedOrphans;
  expectedOrphans += bayesTree[_T_], bayesTree[_X_];
  CHECK(assert_equal(expectedOrphans, orphans));
}

/* ************************************************************************* */
TEST( BayesTree, removeTop )
{
	SymbolicBayesTree bayesTree = createAsiaSymbolicBayesTree();

	// create a new factor to be inserted
	boost::shared_ptr<SymbolicFactor> newFactor(new SymbolicFactor(_B_,_S_));

	// Remove the contaminated part of the Bayes tree
	BayesNet<SymbolicConditional> bn;
	SymbolicBayesTree::Cliques orphans;
	bayesTree.removeTop(newFactor->keys(), bn, orphans);
	FactorGraph<SymbolicFactor> factors(bn);

	// Check expected outcome
	SymbolicFactorGraph expected;
	expected.push_factor(_B_,_L_,_E_);
	expected.push_factor(_B_,_L_);
	expected.push_factor(_B_);
	expected.push_factor(_L_,_B_,_S_);
  CHECK(assert_equal((FactorGraph<SymbolicFactor>)expected, factors));
	SymbolicBayesTree::Cliques expectedOrphans;
  expectedOrphans += bayesTree[_T_], bayesTree[_X_];
  CHECK(assert_equal(expectedOrphans, orphans));

  // Try removeTop again with a factor that should not change a thing
	boost::shared_ptr<SymbolicFactor> newFactor2(new SymbolicFactor(_B_));
	BayesNet<SymbolicConditional> bn2;
	SymbolicBayesTree::Cliques orphans2;
	bayesTree.removeTop(newFactor2->keys(), bn2, orphans2);
	FactorGraph<SymbolicFactor> factors2(bn2);
	SymbolicFactorGraph expected2;
  CHECK(assert_equal((FactorGraph<SymbolicFactor>)expected2, factors2));
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
	BayesNet<SymbolicConditional> bn;
	SymbolicBayesTree::Cliques orphans;
	bayesTree.removeTop(newFactors.keys(), bn, orphans);
	FactorGraph<SymbolicFactor> factors(bn);

	// Check expected outcome
	SymbolicFactorGraph expected;
	expected.push_factor(_B_,_L_,_E_);
	expected.push_factor(_B_,_L_);
	expected.push_factor(_B_);
	expected.push_factor(_L_,_B_,_S_);
  CHECK(assert_equal((FactorGraph<SymbolicFactor>)expected, factors));
	SymbolicBayesTree::Cliques expectedOrphans;
  expectedOrphans += bayesTree[_T_], bayesTree[_X_];
	CHECK(assert_equal(expectedOrphans, orphans));
}

/* ************************************************************************* */
TEST( BayesTree, removeTop3 )
{
	Symbol _l5_('l', 5), _x4_('x', 4);
	// simple test case that failed after COLAMD was fixed/activated
	SymbolicConditional::shared_ptr
	X(new SymbolicConditional(_l5_)),
	A(new SymbolicConditional(_x4_, _l5_)),
	B(new SymbolicConditional(_x3_, _x4_)),
	C(new SymbolicConditional(_x2_, _x3_));

	Ordering newOrdering;
	newOrdering += _x3_, _x2_, _x1_, _l2_, _l1_, _x4_, _l5_;
	SymbolicBayesTree bayesTree;
	bayesTree.insert(X,newOrdering);
	bayesTree.insert(A,newOrdering);
	bayesTree.insert(B,newOrdering);
	bayesTree.insert(C,newOrdering);

	// remove all
	list<Symbol> keys;
	keys += _l5_, _x2_, _x3_, _x4_;
	BayesNet<SymbolicConditional> bn;
	SymbolicBayesTree::Cliques orphans;
	bayesTree.removeTop(keys, bn, orphans);
	FactorGraph<SymbolicFactor> factors(bn);

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
	Symbol _x4_('x', 4), _x5_('x', 5), _x6_('x', 6);
	SymbolicFactorGraph fg1, fg2, fg3;
	fg1.push_factor(_x3_, _x4_);
	fg2.push_factor(_x1_, _x2_);
	fg2.push_factor(_x2_, _x3_);
	fg2.push_factor(_x1_, _x3_);
	fg3.push_factor(_x4_, _x5_);
	fg3.push_factor(_x5_, _x6_);
	fg3.push_factor(_x4_, _x6_);

	Ordering ordering1; ordering1 += _x3_, _x4_;
	Ordering ordering2; ordering2 += _x1_, _x2_;
	Ordering ordering3; ordering3 += _x6_, _x5_;

	BayesNet<SymbolicConditional> bn1, bn2, bn3;
	bn1 = fg1.eliminate(ordering1);
	bn2 = fg2.eliminate(ordering2);
	bn3 = fg3.eliminate(ordering3);

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
	fg.push_factor(_x4_, _x5_);
	fg.push_factor(_x5_, _x6_);
	fg.push_factor(_x4_, _x6_);

	Ordering ordering;  ordering += _x1_, _x2_, _x6_, _x5_, _x3_, _x4_;
	BayesNet<SymbolicConditional> bn;
	bn = fg.eliminate(ordering);
	SymbolicBayesTree expected(bn);
	CHECK(assert_equal(expected, actual));

}
/* ************************************************************************* */

int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

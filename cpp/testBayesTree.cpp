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
#include "smallExample.h"

using namespace gtsam;

typedef BayesTree<SymbolicConditional> SymbolicBayesTree;

/* ************************************************************************* */
// SLAM example from RSS sqrtSAM paper
SymbolicConditional::shared_ptr x3(new SymbolicConditional("x3")),
		x2(new SymbolicConditional("x2","x3")),
		x1(new SymbolicConditional("x1","x2","x3")),
		l1(new SymbolicConditional("l1","x1","x2")),
		l2(new SymbolicConditional("l2","x1","x3"));

// Bayes Tree for sqrtSAM example
SymbolicBayesTree createSlamSymbolicBayesTree(){
	// Create using insert
	SymbolicBayesTree bayesTree_slam;
	bayesTree_slam.insert(x3);
	bayesTree_slam.insert(x2);
	bayesTree_slam.insert(x1);
	bayesTree_slam.insert(l2);
	bayesTree_slam.insert(l1);
	return bayesTree_slam;
}

/* ************************************************************************* */


// Conditionals for ASIA example from the tutorial with A and D evidence
SymbolicConditional::shared_ptr
	B(new SymbolicConditional("B")),
	L(new SymbolicConditional("L", "B")),
	E(new SymbolicConditional("E", "B", "L")),
	S(new SymbolicConditional("S", "L", "B")),
	T(new SymbolicConditional("T", "E", "L")),
	X(new SymbolicConditional("X", "E"));

// Bayes Tree for Asia example
SymbolicBayesTree createAsiaSymbolicBayesTree() {
	SymbolicBayesTree bayesTree;
	bayesTree.insert(B);
	bayesTree.insert(L);
	bayesTree.insert(E);
	bayesTree.insert(S);
	bayesTree.insert(T);
	bayesTree.insert(X);
	return bayesTree;
}

/* ************************************************************************* */
TEST( BayesTree, Front )
{
	SymbolicBayesNet f1;
	f1.push_back(B);
	f1.push_back(L);
	SymbolicBayesNet f2;
	f2.push_back(L);
	f2.push_back(B);
	CHECK(f1.equals(f1));
	CHECK(!f1.equals(f2));
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
}

/* ************************************************************************* *
Bayes Tree for testing conversion to a forest of orphans needed for incremental.
       A,B
   C|A    E|B
   D|C    F|E
/* ************************************************************************* */
TEST( BayesTree, removePath )
{
	SymbolicConditional::shared_ptr
			A(new SymbolicConditional("A")),
			B(new SymbolicConditional("B", "A")),
			C(new SymbolicConditional("C", "A")),
			D(new SymbolicConditional("D", "C")),
			E(new SymbolicConditional("E", "B")),
			F(new SymbolicConditional("F", "E"));
	SymbolicBayesTree bayesTree;
	bayesTree.insert(A);
	bayesTree.insert(B);
	bayesTree.insert(C);
	bayesTree.insert(D);
	bayesTree.insert(E);
	bayesTree.insert(F);

	// remove C, expected outcome: factor graph with ABC,
	// Bayes Tree now contains two orphan trees: D|C and E|B,F|E
	SymbolicFactorGraph expected;
	expected.push_factor("A","B");
	expected.push_factor("A");
	expected.push_factor("A","C");
	SymbolicBayesTree::Cliques expectedOrphans;
  expectedOrphans += bayesTree["D"], bayesTree["E"];

	FactorGraph<SymbolicFactor> factors;
	SymbolicBayesTree::Cliques orphans;
	boost::tie(factors,orphans) = bayesTree.removePath<SymbolicFactor>(bayesTree["C"]);
  CHECK(assert_equal((FactorGraph<SymbolicFactor>)expected, factors));
  CHECK(assert_equal(expectedOrphans, orphans));

  // remove E: factor graph with EB; E|B removed from second orphan tree
	SymbolicFactorGraph expected2;
  expected2.push_factor("B","E");
  SymbolicBayesTree::Cliques expectedOrphans2;
  expectedOrphans2 += bayesTree["F"];

  boost::tie(factors,orphans) = bayesTree.removePath<SymbolicFactor>(bayesTree["E"]);
  CHECK(assert_equal((FactorGraph<SymbolicFactor>)expected2, factors));
  CHECK(assert_equal(expectedOrphans2, orphans));
}

/* ************************************************************************* */
TEST( BayesTree, removePath2 )
{
	SymbolicBayesTree bayesTree = createAsiaSymbolicBayesTree();

	// Call remove-path with clique B
	FactorGraph<SymbolicFactor> factors;
	SymbolicBayesTree::Cliques orphans;
  boost::tie(factors,orphans) = bayesTree.removePath<SymbolicFactor>(bayesTree["B"]);

	// Check expected outcome
	SymbolicFactorGraph expected;
	expected.push_factor("B","L","E");
	expected.push_factor("B","L");
	expected.push_factor("B");
  CHECK(assert_equal((FactorGraph<SymbolicFactor>)expected, factors));
	SymbolicBayesTree::Cliques expectedOrphans;
  expectedOrphans += bayesTree["S"], bayesTree["T"], bayesTree["X"];
  CHECK(assert_equal(expectedOrphans, orphans));
}

/* ************************************************************************* */
TEST( BayesTree, removePath3 )
{
	SymbolicBayesTree bayesTree = createAsiaSymbolicBayesTree();

	// Call remove-path with clique S
	FactorGraph<SymbolicFactor> factors;
	SymbolicBayesTree::Cliques orphans;
  boost::tie(factors,orphans) = bayesTree.removePath<SymbolicFactor>(bayesTree["S"]);

	// Check expected outcome
	SymbolicFactorGraph expected;
	expected.push_factor("B","L","E");
	expected.push_factor("B","L");
	expected.push_factor("B");
	expected.push_factor("L","B","S");
  CHECK(assert_equal((FactorGraph<SymbolicFactor>)expected, factors));
	SymbolicBayesTree::Cliques expectedOrphans;
  expectedOrphans += bayesTree["T"], bayesTree["X"];
  CHECK(assert_equal(expectedOrphans, orphans));
}

/* ************************************************************************* */
TEST( BayesTree, removeTop )
{
	SymbolicBayesTree bayesTree = createAsiaSymbolicBayesTree();

	// create a new factor to be inserted
	boost::shared_ptr<SymbolicFactor> newFactor(new SymbolicFactor("B","S"));

	// Remove the contaminated part of the Bayes tree
	FactorGraph<SymbolicFactor> factors;
	SymbolicBayesTree::Cliques orphans;
	bayesTree.removeTop<SymbolicFactor>(newFactor, factors, orphans);

	// Check expected outcome
	SymbolicFactorGraph expected;
	expected.push_factor("B","L","E");
	expected.push_factor("B","L");
	expected.push_factor("B");
	expected.push_factor("L","B","S");
  CHECK(assert_equal((FactorGraph<SymbolicFactor>)expected, factors));
	SymbolicBayesTree::Cliques expectedOrphans;
  expectedOrphans += bayesTree["T"], bayesTree["X"];
  CHECK(assert_equal(expectedOrphans, orphans));

  // Try removeTop again with a factor that should not change a thing
	boost::shared_ptr<SymbolicFactor> newFactor2(new SymbolicFactor("B"));
	FactorGraph<SymbolicFactor> factors2;
	SymbolicBayesTree::Cliques orphans2;
	bayesTree.removeTop<SymbolicFactor>(newFactor2, factors2, orphans2);
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
	newFactors.push_factor("B");
	newFactors.push_factor("S");

	// Remove the contaminated part of the Bayes tree
	FactorGraph<SymbolicFactor> factors;
	SymbolicBayesTree::Cliques orphans;
	boost::tie(factors,orphans) = bayesTree.removeTop<SymbolicFactor>(newFactors);

	// Check expected outcome
	SymbolicFactorGraph expected;
	expected.push_factor("B","L","E");
	expected.push_factor("B","L");
	expected.push_factor("B");
	expected.push_factor("L","B","S");
  CHECK(assert_equal((FactorGraph<SymbolicFactor>)expected, factors));
	SymbolicBayesTree::Cliques expectedOrphans;
  expectedOrphans += bayesTree["T"], bayesTree["X"];
	CHECK(assert_equal(expectedOrphans, orphans));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

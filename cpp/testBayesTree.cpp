/**
 * @file    testBayesTree.cpp
 * @brief   Unit tests for Bayes Tree
 * @author  Frank Dellaert
 */

#include <boost/assign/list_inserter.hpp> // for 'insert()'
#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>

#include "SymbolicBayesChain.h"
#include "BayesTree-inl.h"

using namespace gtsam;

// Conditionals for ASIA example from the tutorial with A and D evidence
SymbolicConditional::shared_ptr
		B(new SymbolicConditional()),
		L(new SymbolicConditional("B")),
		E(new SymbolicConditional("L","B")),
		S(new SymbolicConditional("L","B")),
		T(new SymbolicConditional("L","E")),
		X(new SymbolicConditional("E"));

/* ************************************************************************* */
TEST( BayesTree, Front )
{
	Front<SymbolicConditional> f1("B",B); f1.add("L",L);
	Front<SymbolicConditional> f2("L",L); f2.add("B",B);
	CHECK(f1.equals(f1));
	CHECK(!f1.equals(f2));
}

/* ************************************************************************* */
TEST( BayesTree, constructor )
{
	// Create using insert
	BayesTree<SymbolicConditional> bayesTree;
	bayesTree.insert("B",B);
	bayesTree.insert("L",L);
	bayesTree.insert("E",E);
	bayesTree.insert("S",S);
	bayesTree.insert("T",T);
	bayesTree.insert("X",X);

	// Check Size
	LONGS_EQUAL(4,bayesTree.size());

	// Check root
	Front<SymbolicConditional> expected_root("B",B);
	expected_root.add("L",L);
	expected_root.add("E",E);
	Front<SymbolicConditional> actual_root = bayesTree.root();
	CHECK(assert_equal(expected_root,actual_root));

	// Create from symbolic Bayes chain in which we want to discover cliques
	map<string, SymbolicConditional::shared_ptr> nodes;
	insert(nodes)("B",B)("L",L)("E",E)("S",S)("T",T)("X",X);
	SymbolicBayesChain ASIA(nodes);
	BayesTree<SymbolicConditional> bayesTree2(ASIA);

	// Check whether the same
	//CHECK(assert_equal(bayesTree,bayesTree2));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

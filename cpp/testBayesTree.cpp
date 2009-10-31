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
#include "SmallExample.h"

using namespace gtsam;

// Conditionals for ASIA example from the tutorial with A and D evidence
SymbolicConditional::shared_ptr B(new SymbolicConditional()), L(
		new SymbolicConditional("B")), E(new SymbolicConditional("L", "B")), S(
		new SymbolicConditional("L", "B")), T(new SymbolicConditional("E", "L")),
		X(new SymbolicConditional("E"));

/* ************************************************************************* */
TEST( BayesTree, Front )
{
	Front<SymbolicConditional> f1("B", B);
	f1.add("L", L);
	Front<SymbolicConditional> f2("L", L);
	f2.add("B", B);
	CHECK(f1.equals(f1));
	CHECK(!f1.equals(f2));
}

/* ************************************************************************* */
TEST( BayesTree, constructor )
{
	// Create using insert
	BayesTree<SymbolicConditional> bayesTree;
	bayesTree.insert("B", B);
	bayesTree.insert("L", L);
	bayesTree.insert("E", E);
	bayesTree.insert("S", S);
	bayesTree.insert("T", T);
	bayesTree.insert("X", X);

	// Check Size
	LONGS_EQUAL(4,bayesTree.size());

	// Check root
	Front<SymbolicConditional> expected_root("B", B);
	expected_root.add("L", L);
	expected_root.add("E", E);
	Front<SymbolicConditional> actual_root = bayesTree.root();
	CHECK(assert_equal(expected_root,actual_root));

	// Create from symbolic Bayes chain in which we want to discover cliques
	SymbolicBayesChain ASIA;
	ASIA.insert("X", X);
	ASIA.insert("T", T);
	ASIA.insert("S", S);
	ASIA.insert("E", E);
	ASIA.insert("L", L);
	ASIA.insert("B", B);
	BayesTree<SymbolicConditional> bayesTree2(ASIA);

	// Check whether the same
	CHECK(assert_equal(bayesTree,bayesTree2));
}

/* ************************************************************************* *
 Bayes tree for smoother with "natural" ordering:
 x6 x7
   x5 : x6
     x4 : x5
       x3 : x4
         x2 : x3
           x1 : x2
/* ************************************************************************* */
TEST( BayesTree, smoother )
{
	// Create smoother with 7 nodes
	LinearFactorGraph smoother = createSmoother(7);
	Ordering ordering;
	for (int t = 1; t <= 7; t++)
		ordering.push_back(symbol('x', t));

	// eliminate using the "natural" ordering
	ChordalBayesNet::shared_ptr chordalBayesNet = smoother.eliminate(ordering);

	// Create the Bayes tree
	BayesTree<ConditionalGaussian> bayesTree(*chordalBayesNet,false);
	LONGS_EQUAL(6,bayesTree.size());
}

/* ************************************************************************* *
 Bayes tree for smoother with "nested dissection" ordering:
 x5 x6 x4
   x3 x2 : x4
     x1 : x2
   x7 : x6
/* ************************************************************************* */
TEST( BayesTree, balanced_smoother )
{
	// Create smoother with 7 nodes
	LinearFactorGraph smoother = createSmoother(7);
	Ordering ordering;
	ordering += "x1","x3","x5","x7","x2","x6","x4";

	// eliminate using a "nested dissection" ordering
	ChordalBayesNet::shared_ptr chordalBayesNet = smoother.eliminate(ordering);

	// Create the Bayes tree
	BayesTree<ConditionalGaussian> bayesTree(*chordalBayesNet,false);
	LONGS_EQUAL(4,bayesTree.size());
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

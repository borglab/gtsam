/**
 * @file    testIncremental.cpp
 * @brief   Unit tests for graph-based iSAM
 * @author  Michael Kaess
 */

#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>

#include "SymbolicBayesNet.h"
#include "SymbolicFactor.h"
#include "GaussianBayesNet.h"
#include "Ordering.h"
#include "BayesTree-inl.h"
#include "smallExample.h"

using namespace gtsam;

typedef BayesTree<SymbolicConditional> SymbolicBayesTree;
typedef BayesTree<GaussianConditional> GaussianBayesTree;


// Conditionals for ASIA example from the tutorial with A and D evidence
SymbolicConditional::shared_ptr B(new SymbolicConditional("B")), L(
		new SymbolicConditional("L", "B")), E(
		new SymbolicConditional("E", "L", "B")), S(new SymbolicConditional("S",
		"L", "B")), T(new SymbolicConditional("T", "E", "L")), X(
		new SymbolicConditional("X", "E"));

/* ************************************************************************* */

SymbolicBayesTree update(const SymbolicBayesTree& initial, const SymbolicFactor& newFactor) {
	return initial;
}

/* ************************************************************************* */
TEST( BayesTree, iSAM )
{
	// Create using insert
	SymbolicBayesTree bayesTree;
	bayesTree.insert(B);
	bayesTree.insert(L);
	bayesTree.insert(E);
	bayesTree.insert(S);
	bayesTree.insert(T);
	bayesTree.insert(X);
	//bayesTree.print("bayesTree");

	// Create expected Bayes tree
	SymbolicBayesTree expected;
	expected.insert(B);
	expected.insert(L);
	expected.insert(S);
	expected.insert(E);
	expected.insert(T);
	expected.insert(X);
	//expected.print("expected");

	// create a new factor to be inserted
	list<string> keys;
	keys += "B","S";
	SymbolicFactor newFactor(keys);

	// do incremental inference
	SymbolicBayesTree actual = update(bayesTree, newFactor);

	// Check whether the same
	//CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

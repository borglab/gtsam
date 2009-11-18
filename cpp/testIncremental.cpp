/**
 * @file    testIncremental.cpp
 * @brief   Unit tests for graph-based iSAM
 * @author  Michael Kaess
 */

#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>

#include "SymbolicBayesNet.h"
#include "SymbolicFactorGraph.h"
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
		new SymbolicConditional("E", "B", "L")), S(new SymbolicConditional("S",
		"L", "B")), T(new SymbolicConditional("T", "E", "L")), X(
		new SymbolicConditional("X", "E"));

/* ************************************************************************* */

SymbolicBayesTree update(const SymbolicBayesTree& initial,
		const boost::shared_ptr<SymbolicFactor>& newFactor) {

	// create an empty factor graph
	SymbolicFactorGraph factorGraph;

	// get the ELB clique
	SymbolicBayesTree::sharedClique ELB = initial["B"];
	FactorGraph<SymbolicFactor> ELB_factors(*ELB);

	// add it to the factor graph
  factorGraph = combine(factorGraph, ELB_factors); // todo: potentially expensive

	// get the SLB clique
	SymbolicBayesTree::sharedClique SLB = initial["S"];
	FactorGraph<SymbolicFactor> SLB_factors(*SLB);

	// add it to the factor graph
  factorGraph = combine(factorGraph, SLB_factors);

	// now add the new factor
	factorGraph.push_back(newFactor);

	// create an ordering ESLB
	Ordering ordering;
	ordering += "E","S","L","B";

	// eliminate into a Bayes net
	SymbolicBayesNet bayesNet = eliminate<SymbolicFactor,SymbolicConditional>(factorGraph,ordering);

	// turn back into a Bayes Tree
	BayesTree<SymbolicConditional> newTree(bayesNet);

	// get the orphan cliques
	SymbolicBayesTree::sharedClique TEL = initial["T"];
	SymbolicBayesTree::sharedClique XE = initial["X"];

  // get clique from new tree to attach to
	SymbolicBayesTree::sharedClique new_ELB = newTree["E"];

	// add orphans to the bottom of the new tree
  new_ELB->children_ += TEL,XE;

	return newTree;
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

	// Create expected Bayes tree
	SymbolicBayesTree expected;
	expected.insert(B);
	expected.insert(L);
	expected.insert(S);
	expected.insert(E);
	expected.insert(T);
	expected.insert(X);

	// create a new factor to be inserted
	list<string> keys;
	keys += "B","S";
	boost::shared_ptr<SymbolicFactor> newFactor(new SymbolicFactor(keys));

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

/**
 * @file    testIncremental.cpp
 * @brief   Unit tests for graph-based iSAM
 * @author  Michael Kaess
 * @author  Viorela Ila
 * @author  Frank Dellaert
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

/* ************************************************************************* */

SymbolicBayesTree update(const SymbolicBayesTree& initial,
		const boost::shared_ptr<SymbolicFactor>& newFactor) {

	// create an empty factor graph
	SymbolicFactorGraph factorGraph;

	// process each key of the new factor
	BOOST_FOREACH(string key, newFactor->keys()) {
		// todo: add path to root

		// only add if key is not yet in the factor graph
		std::list<std::string> keys = factorGraph.keys();
		if (find(keys.begin(), keys.end(), key) == keys.end()) {

			// get the clique
			SymbolicBayesTree::sharedClique clique = initial[key];
			FactorGraph<SymbolicFactor> clique_factors(*clique);

			// add it to the factor graph
			factorGraph.push_back(clique_factors);
		}
	}

	// todo: automatically generate list of orphans, including subtrees!
	std::list<SymbolicBayesTree::sharedClique> orphans;
	SymbolicBayesTree::sharedClique TEL = initial["T"];
	SymbolicBayesTree::sharedClique XE = initial["X"];
	orphans += TEL, XE;

	// now add the new factor
	factorGraph.push_back(newFactor);

	// create an ordering BELS
	Ordering ordering = factorGraph.getOrdering();

	// eliminate into a Bayes net
	SymbolicBayesNet bayesNet = eliminate<SymbolicFactor,SymbolicConditional>(factorGraph,ordering);

	// turn back into a Bayes Tree
	BayesTree<SymbolicConditional> newTree(bayesNet);

	// add orphans to the bottom of the new tree
	BOOST_FOREACH(SymbolicBayesTree::sharedClique orphan, orphans) {
		std::list<std::string>::const_iterator it = orphan->separator_.begin();
		for (; it != orphan->separator_.end(); it++) {

			// get clique from new tree to attach to
			SymbolicBayesTree::sharedClique clique = newTree[*it];

			// check if all conditionals in there, only add once
			bool is_subset = true;
			std::list<std::string>::const_iterator it2 = orphan->separator_.begin();
			for (; it2 != orphan->separator_.end(); it2++) {
				// if any one is not included, then we have to stop and search for another clique
				std::list<std::string> keys = clique->keys();
				if (find(keys.begin(), keys.end(), *it2) == keys.end()) {
					is_subset = false;
					break;
				}
			}

			// this clique contains all the keys of the orphan, so we can add the orphan as a child  todo: what about the tree below the orphan?
			if (is_subset) {
				clique->children_ += orphan;
				break;
			}
		}
	}

	return newTree;
}

/* ************************************************************************* */
TEST( BayesTree, iSAM )
{
	// Conditionals for ASIA example from the tutorial with A and D evidence
	SymbolicConditional::shared_ptr
		B(new SymbolicConditional("B")),
		L(new SymbolicConditional("L", "B")),
		E(new SymbolicConditional("E", "B", "L")),
		S(new SymbolicConditional("S", "L", "B")),
		T(new SymbolicConditional("T", "E", "L")),
		X(new SymbolicConditional("X", "E"));

	// Create using insert
	SymbolicBayesTree bayesTree;
	bayesTree.insert(B);
	bayesTree.insert(L);
	bayesTree.insert(E);
	bayesTree.insert(S);
	bayesTree.insert(T);
	bayesTree.insert(X);

	// Now we modify the Bayes tree by inserting a new factor over B and S

	// New conditionals in modified top of the tree
	SymbolicConditional::shared_ptr
		S_(new SymbolicConditional("S")),
		L_(new SymbolicConditional("L", "S")),
		E_(new SymbolicConditional("E", "L", "S")),
		B_(new SymbolicConditional("B", "E", "L", "S"));

	// Create expected Bayes tree
	SymbolicBayesTree expected;
	expected.insert(S_);
	expected.insert(L_);
	expected.insert(E_);
	expected.insert(B_);
	expected.insert(T);
	expected.insert(X);

	// create a new factor to be inserted
	list<string> keys;
	keys += "B","S";
	boost::shared_ptr<SymbolicFactor> newFactor(new SymbolicFactor(keys));

	// do incremental inference
	SymbolicBayesTree actual = update(bayesTree, newFactor);

	// Check whether the same
  CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

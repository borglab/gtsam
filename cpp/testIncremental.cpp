/**
 * @file    testIncremental.cpp
 * @brief   Unit tests for graph-based iSAM
 * @author  Michael Kaess
 * @author  Viorela Ila
 * @author  Frank Dellaert
 */

#include <boost/tuple/tuple.hpp>
#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>

#include "SymbolicBayesNet.h"
#include "SymbolicFactorGraph.h"
#include "GaussianBayesNet.h"
#include "Ordering.h"
#include "BayesTree-inl.h"
#include "smallExample.h"

using namespace std;
using namespace gtsam;

typedef BayesTree<SymbolicConditional> SymbolicBayesTree;
typedef BayesTree<GaussianConditional> GaussianBayesTree;

/* ************************************************************************* */

void update(SymbolicBayesTree& bayesTree, const FactorGraph<SymbolicFactor> factorGraph) {

	FactorGraph<SymbolicFactor> factors;
	SymbolicBayesTree::Cliques orphans;

	BOOST_FOREACH(FactorGraph<SymbolicFactor>::sharedFactor factor, factorGraph) {
		// Remove the contaminated part of the Bayes tree
		FactorGraph<SymbolicFactor> newFactors;
		SymbolicBayesTree::Cliques newOrphans;
		boost::tie(newFactors, newOrphans) = bayesTree.removeTop<SymbolicFactor>(factor);

		orphans.insert(orphans.begin(), newOrphans.begin(), newOrphans.end());
		const FactorGraph<SymbolicFactor> test = newFactors;
		BOOST_FOREACH(FactorGraph<SymbolicFactor>::sharedFactor newFactor, (const FactorGraph<SymbolicFactor>)newFactors)
			factors.push_back(newFactor);
	}

	// create an ordering for the new and contaminated factors
	Ordering ordering = factors.getOrdering();

	// eliminate into a Bayes net
	SymbolicBayesNet bayesNet = eliminate<SymbolicFactor,SymbolicConditional>(factors,ordering);

	// turn back into a Bayes Tree
	SymbolicBayesNet::const_reverse_iterator rit;
	for ( rit=bayesNet.rbegin(); rit != bayesNet.rend(); ++rit )
		bayesTree.insert(*rit);

	// add orphans to the bottom of the new tree
	BOOST_FOREACH(SymbolicBayesTree::sharedClique orphan, orphans) {
		string key = *(orphan->separator_.begin()); // todo: assumes there is a separator...
		SymbolicBayesTree::sharedClique parent = bayesTree[key];
		parent->children_ += orphan;
	}

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

	// create new factors to be inserted
	SymbolicFactorGraph factorGraph;
	factorGraph.push_factor("B","S");

	// do incremental inference
	update(bayesTree, factorGraph);

	// Check whether the same
  CHECK(assert_equal(expected,bayesTree));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

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

void update(SymbolicBayesTree& bayesTree, const boost::shared_ptr<SymbolicFactor>& newFactor) {

	// Remove the contaminated part of the Bayes tree
	FactorGraph<SymbolicFactor> factors;
	SymbolicBayesTree::Cliques orphans;
	boost::tie(factors,orphans) = bayesTree.removeTop<SymbolicFactor>(newFactor);

	// create an ordering BELS
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

#if 0
		BOOST_FOREACH(string key1, orphan->separator_) {
			// get clique from new tree to attach to
			SymbolicBayesTree::sharedClique candidateParent = bayesTree[key1];

			// check if all conditionals in there, only add once
			bool is_subset = true;
			BOOST_FOREACH(string key2, orphan->separator_) {
				// if any one not included, then we have to stop and search for another clique
				list<string> keys = candidateParent->keys();
				if (find(keys.begin(), keys.end(), key2) == keys.end()) {
					is_subset = false;
					break;
				}
			}

			// this clique contains all the keys of the orphan,
			// so we can add the orphan as a child
			// todo: what about the tree below the orphan?
			if (is_subset) {
				candidateParent->children_ += orphan;
				break;
			}
		}
	}
#endif
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
	update(bayesTree, newFactor);

	// Check whether the same
  CHECK(assert_equal(expected,bayesTree));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

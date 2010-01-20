/**
 * @file    ISAM-inl.h
 * @brief   Incremental update functionality (iSAM) for BayesTree.
 * @author  Michael Kaess
 */

#include <boost/foreach.hpp>
#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include "Conditional.h"
#include "ISAM.h"
#include "BayesTree-inl.h"

namespace gtsam {

	using namespace std;

	/** Create an empty Bayes Tree */
	template<class Conditional>
	ISAM<Conditional>::ISAM() : BayesTree<Conditional>() {}

	/** Create a Bayes Tree from a Bayes Net */
	template<class Conditional>
	ISAM<Conditional>::ISAM(const BayesNet<Conditional>& bayesNet) :
	  BayesTree<Conditional>(bayesNet) {}

	/* ************************************************************************* */
	template<class Conditional>
	template<class Factor>
	void ISAM<Conditional>::update_internal(const FactorGraph<Factor>& newFactors, Cliques& orphans) {

		// Remove the contaminated part of the Bayes tree
		FactorGraph<Factor> factors;
		removeTop(newFactors.keys(), factors, orphans);

		// add the factors themselves
		factors.push_back(newFactors);

		// create an ordering for the new and contaminated factors
		Ordering ordering;
		if (true) {
			ordering = factors.getOrdering();
		} else {
			list<Symbol> keys = factors.keys();
			keys.sort(); // todo: correct sorting order?
			ordering = keys;
		}

		// eliminate into a Bayes net
		BayesNet<Conditional> bayesNet = eliminate<Factor, Conditional>(factors,ordering);

		// insert conditionals back in, straight into the topless bayesTree
		typename BayesNet<Conditional>::const_reverse_iterator rit;
		for ( rit=bayesNet.rbegin(); rit != bayesNet.rend(); ++rit )
			this->insert(*rit, &ordering);

		int count = 0;
		// add orphans to the bottom of the new tree
		BOOST_FOREACH(sharedClique orphan, orphans) {

			Symbol key = findParentClique(orphan->separator_, ordering);
			sharedClique parent = (*this)[key];

			parent->children_ += orphan;
			orphan->parent_ = parent; // set new parent!
		}

	}

	template<class Conditional>
	template<class Factor>
	void ISAM<Conditional>::update(const FactorGraph<Factor>& newFactors) {
		Cliques orphans;
		this->update_internal<Factor>(newFactors, orphans);
	}

}
/// namespace gtsam

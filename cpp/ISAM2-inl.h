/**
 * @file    ISAM2-inl.h
 * @brief   Incremental update functionality (ISAM2) for BayesTree.
 * @author  Michael Kaess
 */

#include <boost/foreach.hpp>
#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include "NonlinearFactorGraph.h"
#include "GaussianFactor.h"
#include "VectorConfig.h"

#include "Conditional.h"
#include "BayesTree-inl.h"
#include "ISAM2.h"

namespace gtsam {

	using namespace std;

	/** Create an empty Bayes Tree */
	template<class Conditional, class Config>
	ISAM2<Conditional, Config>::ISAM2() : BayesTree<Conditional>() {}

	/** Create a Bayes Tree from a Bayes Net */
	template<class Conditional, class Config>
	ISAM2<Conditional, Config>::ISAM2(const BayesNet<Conditional>& bayesNet) : BayesTree<Conditional>(bayesNet) {}

	/* ************************************************************************* */
	template<class Conditional, class Config>
	void ISAM2<Conditional, Config>::update_internal(const NonlinearFactorGraph<Config>& newFactorsXXX, const Config& config, Cliques& orphans) {

		config_ = config; // todo
		FactorGraph<GaussianFactor> newFactors = newFactorsXXX.linearize(config); // todo: just for testing

		// Remove the contaminated part of the Bayes tree
		FactorGraph<GaussianFactor> factors;
		boost::tie(factors, orphans) = this->removeTop(newFactors);

		// add the factors themselves
		factors.push_back(newFactors);

		// create an ordering for the new and contaminated factors
		Ordering ordering;
		if (true) {
			ordering = factors.getOrdering();
		} else {
			list<string> keys = factors.keys();
			keys.sort(); // todo: correct sorting order?
			ordering = keys;
		}

		// eliminate into a Bayes net
		BayesNet<Conditional> bayesNet = eliminate<GaussianFactor, Conditional>(factors,ordering);

		// insert conditionals back in, straight into the topless bayesTree
		typename BayesNet<Conditional>::const_reverse_iterator rit;
		for ( rit=bayesNet.rbegin(); rit != bayesNet.rend(); ++rit )
			this->insert(*rit);

		int count = 0;
		// add orphans to the bottom of the new tree
		BOOST_FOREACH(sharedClique orphan, orphans) {

			string key = orphan->separator_.front();
			sharedClique parent = (*this)[key];

			parent->children_ += orphan;
			orphan->parent_ = parent; // set new parent!
		}

	}

	template<class Conditional, class Config>
	void ISAM2<Conditional, Config>::update(const NonlinearFactorGraph<Config>& newFactors, const Config& config) {
		Cliques orphans;
		this->update_internal(newFactors, config, orphans);
	}

/* ************************************************************************* */

}
/// namespace gtsam

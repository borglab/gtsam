/**
 * @file    ISAM2-inl.h
 * @brief   Incremental update functionality (ISAM2) for BayesTree, with fluid relinearization.
 * @author  Michael Kaess
 */

#include <boost/foreach.hpp>
#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include <set>

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

	/** Create a Bayes Tree from a nonlinear factor graph */
	template<class Conditional, class Config>
	ISAM2<Conditional, Config>::ISAM2(const NonlinearFactorGraph<Config>& nlfg, const Ordering& ordering, const Config& config)
	: BayesTree<Conditional>(nlfg.linearize(config).eliminate(ordering)), nonlinearFactors_(nlfg), config_(config) {}

	/* ************************************************************************* */
	template<class Conditional, class Config>
	void ISAM2<Conditional, Config>::update_internal(const NonlinearFactorGraph<Config>& newFactors,
			const Config& config, Cliques& orphans) {

		// copy variables into config_, but don't overwrite existing entries (current linearization point!)
		for (typename Config::const_iterator it = config.begin(); it!=config.end(); it++) {
			if (!config_.contains(it->first)) {
				config_.insert(it->first, it->second);
			}
		}
		nonlinearFactors_.push_back(newFactors);

		FactorGraph<GaussianFactor> newFactorsLinearized = newFactors.linearize(config_);

		// Remove the contaminated part of the Bayes tree
		FactorGraph<GaussianFactor> affectedFactors;
		boost::tie(affectedFactors, orphans) = this->removeTop(newFactorsLinearized);


#if 1
		// find the corresponding original nonlinear factors, and relinearize them
		NonlinearFactorGraph<Config> nonlinearAffectedFactors;
		set<int> idxs; // avoid duplicates by putting index into set
		BOOST_FOREACH(FactorGraph<GaussianFactor>::sharedFactor fac, affectedFactors) {
			// retrieve correspondent factor from nonlinearFactors_
			Ordering keys = fac->keys();
			BOOST_FOREACH(string key, keys) {
				list<int> indices = nonlinearFactors_.factors(key);
				BOOST_FOREACH(int idx, indices) {
					// todo - only insert index if factor is subset of keys... not needed once we do relinearization - but then how to deal with overlap with orphans?
					bool subset = true;
					BOOST_FOREACH(string k, nonlinearFactors_[idx]->keys()) {
						if (find(keys.begin(), keys.end(), k)==keys.end()) subset = false;
					}
					if (subset) {
						idxs.insert(idx);
					}
				}
			}
		}
		BOOST_FOREACH(int idx, idxs) {
			nonlinearAffectedFactors.push_back(nonlinearFactors_[idx]);
		}
		FactorGraph<GaussianFactor> factors = nonlinearAffectedFactors.linearize(config_);

		// add the new factors themselves
		factors.push_back(newFactorsLinearized);
#endif

		affectedFactors.push_back(newFactorsLinearized);

		// create an ordering for the new and contaminated factors
		Ordering ordering;
		if (true) {
			ordering = /*affectedF*/factors.getOrdering();
		} else {
			list<string> keys = /*affectedF*/factors.keys();
			keys.sort(); // todo: correct sorting order?
			ordering = keys;
		}

		// eliminate into a Bayes net
		BayesNet<Conditional> bayesNet = eliminate<GaussianFactor, Conditional>(affectedFactors,ordering);

#if 1
		BayesNet<Conditional> bayesNetTest = eliminate<GaussianFactor, Conditional>(factors,ordering); // todo - debug only
		if (!bayesNet.equals(bayesNetTest)) {
			printf("differ\n");
			bayesNet.print();
			bayesNetTest.print();
			exit(42);
		}
#endif

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

/**
 * @file    ISAM2-inl.h
 * @brief   Incremental update functionality (ISAM2) for BayesTree, with fluid relinearization.
 * @author  Michael Kaess
 */

#include <boost/foreach.hpp>
#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include <set>

#include "NonlinearFactorGraph-inl.h"
#include "GaussianFactor.h"
#include "VectorConfig.h"

#include "Conditional.h"
#include "BayesTree-inl.h"
#include "ISAM2.h"

namespace gtsam {

	using namespace std;

	// from inference-inl.h - need to additionally return the newly created factor for caching
	boost::shared_ptr<GaussianConditional> _eliminateOne(FactorGraph<GaussianFactor>& graph, cachedFactors& cached, const string& key) {

		// combine the factors of all nodes connected to the variable to be eliminated
		// if no factors are connected to key, returns an empty factor
		boost::shared_ptr<GaussianFactor> joint_factor = removeAndCombineFactors(graph,key);

		// eliminate that joint factor
		boost::shared_ptr<GaussianFactor> factor;
		boost::shared_ptr<GaussianConditional> conditional;
		boost::tie(conditional, factor) = joint_factor->eliminate(key);

		// remember the intermediate result to be able to later restart computation in the middle
		cached[key] = factor;

		// add new factor on separator back into the graph
		if (!factor->empty()) graph.push_back(factor);

		// return the conditional Gaussian
		return conditional;
	}

	// from GaussianFactorGraph.cpp, see _eliminateOne above
	GaussianBayesNet _eliminate(FactorGraph<GaussianFactor>& graph, cachedFactors& cached, const Ordering& ordering) {
		GaussianBayesNet chordalBayesNet; // empty
		BOOST_FOREACH(string key, ordering) {
			GaussianConditional::shared_ptr cg = _eliminateOne(graph, cached, key);
			chordalBayesNet.push_back(cg);
		}
		return chordalBayesNet;
	}

	GaussianBayesNet _eliminate_const(const FactorGraph<GaussianFactor>& graph, cachedFactors& cached, const Ordering& ordering) {
		// make a copy that can be modified locally
		FactorGraph<GaussianFactor> graph_ignored = graph;
		return _eliminate(graph_ignored, cached, ordering);
	}

	/** Create an empty Bayes Tree */
	template<class Conditional, class Config>
	ISAM2<Conditional, Config>::ISAM2() : BayesTree<Conditional>() {}

	/** Create a Bayes Tree from a nonlinear factor graph */
	template<class Conditional, class Config>
	ISAM2<Conditional, Config>::ISAM2(const NonlinearFactorGraph<Config>& nlfg, const Ordering& ordering, const Config& config)
	: BayesTree<Conditional>(nlfg.linearize(config).eliminate(ordering)), nonlinearFactors_(nlfg), config_(config) {
		// todo: repeats calculation above, just to set "cached"
		_eliminate_const(nlfg.linearize(config), cached, ordering);
	}

	/* ************************************************************************* */
	// retrieve all factors that ONLY contain the affected variables
	// (note that the remaining stuff is summarized in the cached factors)
	template<class Conditional, class Config>
	FactorGraph<GaussianFactor> ISAM2<Conditional, Config>::relinearizeAffectedFactors(const list<string>& affectedKeys) {
		NonlinearFactorGraph<Config> nonlinearAffectedFactors;
		typename FactorGraph<NonlinearFactor<Config> >::iterator it;
		for(it = nonlinearFactors_.begin(); it != nonlinearFactors_.end(); it++) {
			bool inside = true;
			BOOST_FOREACH(string key, (*it)->keys()) {
				if (find(affectedKeys.begin(), affectedKeys.end(), key) == affectedKeys.end())
					inside = false;
			}
			if (inside)
				nonlinearAffectedFactors.push_back(*it);
		}
		return nonlinearAffectedFactors.linearize(config_);
	}

	/* ************************************************************************* */
	template<class Conditional, class Config>
	FactorGraph<GaussianFactor> ISAM2<Conditional, Config>::getCachedBoundaryFactors(Cliques& orphans) {
		// add intermediate (linearized) factors from cache that are passed into the affected area
		FactorGraph<GaussianFactor> cachedBoundary;
		BOOST_FOREACH(sharedClique orphan, orphans) {
			// find the last variable that is not part of the separator
			string oneTooFar = orphan->separator_.front();
			list<string> keys = orphan->keys();
			list<string>::iterator it = find(keys.begin(), keys.end(), oneTooFar);
			it--;
			string key = *it;
			// retrieve the cached factor and add to boundary
			cachedBoundary.push_back(cached[key]);
		}
		return cachedBoundary;
	}

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

		FactorGraph<GaussianFactor> newFactorsLinearized = newFactors.linearize(config_);

		// Remove the contaminated part of the Bayes tree
		FactorGraph<GaussianFactor> affectedFactors;
		boost::tie(affectedFactors, orphans) = this->removeTop(newFactorsLinearized);

		// relinearize the affected factors ...
		list<string> affectedKeys = affectedFactors.keys();
		FactorGraph<GaussianFactor> factors = relinearizeAffectedFactors(affectedKeys);

		// ... add the cached intermediate results from the boundary of the orphans ...
		FactorGraph<GaussianFactor> cachedBoundary = getCachedBoundaryFactors(orphans);
		factors.push_back(cachedBoundary);

		// ... and finally add the new linearized factors themselves
		factors.push_back(newFactorsLinearized);

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
		BayesNet<Conditional> bayesNet = _eliminate(factors, cached, ordering);

		// remember the new factors for later relinearization
		nonlinearFactors_.push_back(newFactors);

		// insert conditionals back in, straight into the topless bayesTree
		typename BayesNet<Conditional>::const_reverse_iterator rit;
		for ( rit=bayesNet.rbegin(); rit != bayesNet.rend(); ++rit )
			this->insert(*rit);

		// add orphans to the bottom of the new tree
		BOOST_FOREACH(sharedClique orphan, orphans) {

			string key = orphan->separator_.front();
			sharedClique parent = (*this)[key];

			parent->children_ += orphan;
			orphan->parent_ = parent; // set new parent!
		}

		// update solution
		VectorConfig solution = optimize2(*this);
		solution.print();

	}

	template<class Conditional, class Config>
	void ISAM2<Conditional, Config>::update(const NonlinearFactorGraph<Config>& newFactors, const Config& config) {

		Cliques orphans;
		this->update_internal(newFactors, config, orphans);
	}

/* ************************************************************************* */

}
/// namespace gtsam

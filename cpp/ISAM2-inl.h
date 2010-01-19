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
	boost::shared_ptr<GaussianConditional> _eliminateOne(FactorGraph<GaussianFactor>& graph, CachedFactors& cached, const Symbol& key) {

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
	GaussianBayesNet _eliminate(FactorGraph<GaussianFactor>& graph, CachedFactors& cached, const Ordering& ordering) {
		GaussianBayesNet chordalBayesNet; // empty
		BOOST_FOREACH(const Symbol& key, ordering) {
			GaussianConditional::shared_ptr cg = _eliminateOne(graph, cached, key);
			chordalBayesNet.push_back(cg);
		}
		return chordalBayesNet;
	}

	GaussianBayesNet _eliminate_const(const FactorGraph<GaussianFactor>& graph, CachedFactors& cached, const Ordering& ordering) {
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
	: BayesTree<Conditional>(nlfg.linearize(config).eliminate(ordering)), nonlinearFactors_(nlfg), linPoint_(config) {
		// todo: repeats calculation above, just to set "cached"
		_eliminate_const(nlfg.linearize(config), cached_, ordering);
	}

	/* ************************************************************************* */
	// retrieve all factors that ONLY contain the affected variables
	// (note that the remaining stuff is summarized in the cached factors)
	template<class Conditional, class Config>
	FactorGraph<GaussianFactor> ISAM2<Conditional, Config>::relinearizeAffectedFactors(const list<Symbol>& affectedKeys) {

		NonlinearFactorGraph<Config> nonlinearAffectedFactors;

		typename FactorGraph<NonlinearFactor<Config> >::iterator it;
		for(it = nonlinearFactors_.begin(); it != nonlinearFactors_.end(); it++) {
			bool inside = true;
			BOOST_FOREACH(const Symbol& key, (*it)->keys()) {
				if (find(affectedKeys.begin(), affectedKeys.end(), key) == affectedKeys.end())
					inside = false;
			}
			if (inside)
				nonlinearAffectedFactors.push_back(*it);
		}

		return nonlinearAffectedFactors.linearize(linPoint_);
	}

	/* ************************************************************************* */
	// find intermediate (linearized) factors from cache that are passed into the affected area
	template<class Conditional, class Config>
	FactorGraph<GaussianFactor> ISAM2<Conditional, Config>::getCachedBoundaryFactors(Cliques& orphans) {
		FactorGraph<GaussianFactor> cachedBoundary;

		BOOST_FOREACH(sharedClique orphan, orphans) {
			// find the last variable that is not part of the separator
			Symbol oneTooFar = orphan->separator_.front();
			list<Symbol> keys = orphan->keys();
			list<Symbol>::iterator it = find(keys.begin(), keys.end(), oneTooFar);
			it--;
			const Symbol& key = *it;
			// retrieve the cached factor and add to boundary
			cachedBoundary.push_back(cached_[key]);
		}

		return cachedBoundary;
	}

	/* ************************************************************************* */
	template<class Conditional, class Config>
	void ISAM2<Conditional, Config>::update_internal(const NonlinearFactorGraph<Config>& newFactors,
			const Config& config, Cliques& orphans) {

		// determine which variables to relinearize

		FactorGraph<GaussianFactor> affectedFactors;
		list<Symbol> newFactorsKeys = newFactors.keys();

		// relinearize all keys that are in newFactors, and already exist (not new variables!)
		list<Symbol> keysToRelinearize;
		list<Symbol> oldKeys = nonlinearFactors_.keys();
		BOOST_FOREACH(const Symbol& key, newFactorsKeys) {
			if (find(oldKeys.begin(), oldKeys.end(), key)!=oldKeys.end())
				keysToRelinearize.push_back(key);
		}

#if 1
		// basically calculate all the keys contained in the factors that contain any of the keys...
		// the goal is to relinearize all variables directly affected by new factors
		FactorGraph<NonlinearFactor<Config> > allAffected;
		typename FactorGraph<NonlinearFactor<Config> >::iterator it;
		for(it = nonlinearFactors_.begin(); it != nonlinearFactors_.end(); it++) {
			bool affected = false;
			BOOST_FOREACH(const Symbol& key, (*it)->keys()) {
				if (find(newFactorsKeys.begin(), newFactorsKeys.end(), key) != newFactorsKeys.end())
					affected = true;
			}
			if (affected)
				allAffected.push_back(*it);
		}
		list<Symbol> keysToBeRemoved = allAffected.keys();
#else
		list<Symbol> keysToBeRemoved = nonlinearFactors_.keys(); // todo/debug - relinearize all (a cumbersome way to do batch)
#endif

		this->removeTop(keysToBeRemoved, affectedFactors, orphans);

		// selectively update the linearization point
		VectorConfig selected_delta;
		BOOST_FOREACH(const Symbol& key, keysToRelinearize) {
			if (delta_.contains(key)) // after constructor call, delta is empty...
				selected_delta.insert(key, delta_[key]);
		}
		linPoint_ = expmap(linPoint_, selected_delta); // todo-debug only

		// relinearize the affected factors ...
		list<Symbol> affectedKeys = affectedFactors.keys();
		FactorGraph<GaussianFactor> factors = relinearizeAffectedFactors(affectedKeys); // todo: searches through all factors, potentially expensive

		// ... add the cached intermediate results from the boundary of the orphans ...
		FactorGraph<GaussianFactor> cachedBoundary = getCachedBoundaryFactors(orphans);
		factors.push_back(cachedBoundary);

		// add new variables
		linPoint_.insert(config);
		// ... and finally add the new linearized factors themselves
		FactorGraph<GaussianFactor> newFactorsLinearized = newFactors.linearize(linPoint_);
		factors.push_back(newFactorsLinearized);

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
		BayesNet<Conditional> bayesNet = _eliminate(factors, cached_, ordering);

		// remember the new factors for later relinearization
		nonlinearFactors_.push_back(newFactors);

		// insert conditionals back in, straight into the topless bayesTree
		typename BayesNet<Conditional>::const_reverse_iterator rit;
		for ( rit=bayesNet.rbegin(); rit != bayesNet.rend(); ++rit )
			this->insert(*rit);

		// add orphans to the bottom of the new tree
		BOOST_FOREACH(sharedClique orphan, orphans) {

		  Symbol key = orphan->separator_.front();
			sharedClique parent = (*this)[key];

			parent->children_ += orphan;
			orphan->parent_ = parent; // set new parent!
		}

		// update solution - todo: potentially expensive
		delta_ = optimize2(*this);
	}

	template<class Conditional, class Config>
	void ISAM2<Conditional, Config>::update(const NonlinearFactorGraph<Config>& newFactors, const Config& config) {

		Cliques orphans;
		this->update_internal(newFactors, config, orphans);
	}

/* ************************************************************************* */

}
/// namespace gtsam

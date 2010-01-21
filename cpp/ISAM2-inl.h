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

		// ADDED: remember the intermediate result to be able to later restart computation in the middle
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
	: BayesTree<Conditional>(nlfg.linearize(config).eliminate(ordering)), nonlinearFactors_(nlfg), theta_(config) {
		// todo: repeats calculation above, just to set "cached"
		_eliminate_const(nlfg.linearize(config), cached_, ordering);
	}

	/* ************************************************************************* */
	template<class Conditional, class Config>
	boost::shared_ptr<FactorGraph<NonlinearFactor<Config> > >
	ISAM2<Conditional, Config>::getAffectedFactors(const list<Symbol>& keys) const {
		boost::shared_ptr<FactorGraph<NonlinearFactor<Config> > > allAffected(new FactorGraph<NonlinearFactor<Config> >);
		list<int> indices;
		BOOST_FOREACH(const Symbol& key, keys) {
			const list<int> l = nonlinearFactors_.factors(key);
			indices.insert(indices.begin(), l.begin(), l.end());
		}
		indices.sort();
		indices.unique();
		BOOST_FOREACH(int i, indices) {
			allAffected->push_back(nonlinearFactors_[i]);
		}
		return allAffected;
	}

	/* ************************************************************************* */
	// retrieve all factors that ONLY contain the affected variables
	// (note that the remaining stuff is summarized in the cached factors)
	template<class Conditional, class Config>
	FactorGraph<GaussianFactor> ISAM2<Conditional, Config>::relinearizeAffectedFactors(const list<Symbol>& affectedKeys) const {

		boost::shared_ptr<FactorGraph<NonlinearFactor<Config> > > candidates = getAffectedFactors(affectedKeys);

		NonlinearFactorGraph<Config> nonlinearAffectedFactors;

		typename FactorGraph<NonlinearFactor<Config> >::const_iterator it;
		for(it = candidates->begin(); it != candidates->end(); it++) {
			bool inside = true;
			BOOST_FOREACH(const Symbol& key, (*it)->keys()) {
				if (find(affectedKeys.begin(), affectedKeys.end(), key) == affectedKeys.end()) {
					inside = false;
					break;
				}
			}
			if (inside)
				nonlinearAffectedFactors.push_back(*it);
		}

		return nonlinearAffectedFactors.linearize(theta_);
	}

	/* ************************************************************************* */
	// find intermediate (linearized) factors from cache that are passed into the affected area
	template<class Conditional, class Config>
	FactorGraph<GaussianFactor> ISAM2<Conditional, Config>::getCachedBoundaryFactors(Cliques& orphans) {
		FactorGraph<GaussianFactor> cachedBoundary;

		BOOST_FOREACH(sharedClique orphan, orphans) {
			// find the last variable that was eliminated
			const Symbol& key = orphan->ordering().back();
			// retrieve the cached factor and add to boundary
			cachedBoundary.push_back(cached_[key]);
		}

		return cachedBoundary;
	}

	/* ************************************************************************* */
	template<class Conditional, class Config>
	void ISAM2<Conditional, Config>::update_internal(const NonlinearFactorGraph<Config>& newFactors,
			const Config& theta_new, Cliques& orphans, double wildfire_threshold, double relinearize_threshold) {


		// todo - debug only
		//		marked_ = nonlinearFactors_.keys();

		//// 1 - Remember the new factors for later relinearization
		nonlinearFactors_.push_back(newFactors);

		//// 2 - add in new information
		// add new variables
		theta_.insert(theta_new);

		// todo - not in lyx yet: relin requires more than just removing the cliques corresponding to the variables!!!
		// It's about factors!!!

		// basically calculate all the keys contained in the factors that contain any of the keys...
		// the goal is to relinearize all variables directly affected by new factors
		boost::shared_ptr<FactorGraph<NonlinearFactor<Config> > > allAffected = getAffectedFactors(marked_);
		marked_ = allAffected->keys();

		// merge keys of new factors with mask
		const list<Symbol> newKeys = newFactors.keys();
		marked_.insert(marked_.begin(), newKeys.begin(), newKeys.end());
		// eliminate duplicates
		marked_.sort();
		marked_.unique();

		//// 4 - removeTop invalidate all cliques involving marked variables

		// remove affected factors
		BayesNet<GaussianConditional> affectedBayesNet;
		this->removeTop(marked_, affectedBayesNet, orphans);

		//// 3 - find factors connected to affected variables
		//// 4 - linearize

		// ordering provides all keys in conditionals, there cannot be others because path to root included
		list<Symbol> affectedKeys = affectedBayesNet.ordering();

		// todo - remerge in keys of new factors
		affectedKeys.insert(affectedKeys.begin(), newKeys.begin(), newKeys.end());
		// eliminate duplicates
		affectedKeys.sort();
		affectedKeys.unique();

		FactorGraph<GaussianFactor> factors = relinearizeAffectedFactors(affectedKeys);

		// add the cached intermediate results from the boundary of the orphans ...
		FactorGraph<GaussianFactor> cachedBoundary = getCachedBoundaryFactors(orphans);
		factors.push_back(cachedBoundary);

		//// 5 - eliminate and add orphans back in

		// create an ordering for the new and contaminated factors
		Ordering ordering = factors.getOrdering();

		// eliminate into a Bayes net
		BayesNet<Conditional> bayesNet = _eliminate(factors, cached_, ordering);

		// insert conditionals back in, straight into the topless bayesTree
		typename BayesNet<Conditional>::const_reverse_iterator rit;
		for ( rit=bayesNet.rbegin(); rit != bayesNet.rend(); ++rit ) {
			this->insert(*rit, &ordering);
		}

		// add orphans to the bottom of the new tree
		BOOST_FOREACH(sharedClique orphan, orphans) {
			Symbol key = findParentClique(orphan->separator_, ordering);
			sharedClique parent = (*this)[key];
			parent->children_ += orphan;
			orphan->parent_ = parent; // set new parent!
		}

		//// 6 - update solution

		VectorConfig delta = optimize2(*this, wildfire_threshold);

		//// 7 - mark variables, if significant change

		marked_.clear();
		VectorConfig deltaMarked;
		for (VectorConfig::const_iterator it = delta.begin(); it!=delta.end(); it++) {
			Symbol key = it->first;
			Vector v = it->second;
			if (max(abs(v)) >= relinearize_threshold) {
				marked_.push_back(key);
				deltaMarked.insert(key, v);
			}
		}

		//// 8 - relinearize selected variables

		theta_ = expmap(theta_, deltaMarked);

	}

	template<class Conditional, class Config>
	void ISAM2<Conditional, Config>::update(
			const NonlinearFactorGraph<Config>& newFactors, const Config& config,
			double wildfire_threshold, double relinearize_threshold) {

		Cliques orphans;
		this->update_internal(newFactors, config, orphans, wildfire_threshold, relinearize_threshold);

	}

/* ************************************************************************* */

}
/// namespace gtsam

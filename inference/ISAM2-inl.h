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
	: BayesTree<Conditional>(nlfg.linearize(config)->eliminate(ordering)), theta_(config), thetaFuture_(config), nonlinearFactors_(nlfg) {
		// todo: repeats calculation above, just to set "cached"
		// De-referencing shared pointer can be quite expensive because creates temporary
		_eliminate_const(*nlfg.linearize(config), cached_, ordering);
	}

	/* ************************************************************************* */
	template<class Conditional, class Config>
	list<size_t> ISAM2<Conditional, Config>::getAffectedFactors(const list<Symbol>& keys) const {
	  FactorGraph<NonlinearFactor<Config> > allAffected;
		list<size_t> indices;
		BOOST_FOREACH(const Symbol& key, keys) {
			const list<size_t> l = nonlinearFactors_.factors(key);
			indices.insert(indices.begin(), l.begin(), l.end());
		}
		indices.sort();
		indices.unique();
		return indices;
	}

	/* ************************************************************************* */
	// retrieve all factors that ONLY contain the affected variables
	// (note that the remaining stuff is summarized in the cached factors)
	template<class Conditional, class Config>
	boost::shared_ptr<GaussianFactorGraph> ISAM2<Conditional, Config>::relinearizeAffectedFactors
	(const set<Symbol>& affectedKeys) const {

		list<Symbol> affectedKeysList; // todo: shouldn't have to convert back to list...
		affectedKeysList.insert(affectedKeysList.begin(), affectedKeys.begin(), affectedKeys.end());
		list<size_t> candidates = getAffectedFactors(affectedKeysList);

		NonlinearFactorGraph<Config> nonlinearAffectedFactors;

		BOOST_FOREACH(size_t idx, candidates) {
			bool inside = true;
			BOOST_FOREACH(const Symbol& key, nonlinearFactors_[idx]->keys()) {
				if (affectedKeys.find(key) == affectedKeys.end()) {
					inside = false;
					break;
				}
			}
			if (inside)
				nonlinearAffectedFactors.push_back(nonlinearFactors_[idx]);
		}

		// TODO: temporary might be expensive, return shared pointer ?
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
	// todo: will be obsolete soon
	template<class Conditional, class Config>
	void ISAM2<Conditional, Config>::update_internal(const NonlinearFactorGraph<Config>& newFactors,
			const Config& newTheta, Cliques& orphans, double wildfire_threshold, double relinearize_threshold, bool relinearize) {

		//		marked_ = nonlinearFactors_.keys(); // debug only ////////////

		// only relinearize if requested in previous step AND necessary (ie. at least one variable changes)
		relinearize = true; // todo - switched off
		bool relinFromLast = true; //marked_.size() > 0;

		//// 1 - relinearize selected variables

		if (relinFromLast) {
			theta_ = expmap(theta_, deltaMarked_);
		}

		//// 2 - Add new factors (for later relinearization)

		nonlinearFactors_.push_back(newFactors);

		//// 3 - Initialize new variables

		theta_.insert(newTheta);
		thetaFuture_.insert(newTheta);

		//// 4 - Mark affected variables as invalid

		// todo - not in lyx yet: relin requires more than just removing the cliques corresponding to the variables!!!
		// It's about factors!!!

		if (relinFromLast) {
			// mark variables that have to be removed as invalid (removeFATtop)
			// basically calculate all the keys contained in the factors that contain any of the keys...
			// the goal is to relinearize all variables directly affected by new factors
			list<size_t> allAffected = getAffectedFactors(marked_);
			set<Symbol> accumulate;
			BOOST_FOREACH(int idx, allAffected) {
				list<Symbol> tmp = nonlinearFactors_[idx]->keys();
				accumulate.insert(tmp.begin(), tmp.end());
			}
			marked_.clear();
			marked_.insert(marked_.begin(), accumulate.begin(), accumulate.end());
		} // else: marked_ is empty anyways

		// also mark variables that are affected by new factors as invalid
		const list<Symbol> newKeys = newFactors.keys();
		marked_.insert(marked_.begin(), newKeys.begin(), newKeys.end());
		// eliminate duplicates
		marked_.sort();
		marked_.unique();

		//// 5 - removeTop invalidate all cliques involving marked variables

		// remove affected factors
		BayesNet<GaussianConditional> affectedBayesNet;
		this->removeTop(marked_, affectedBayesNet, orphans);

		//// 6 - find factors connected to affected variables
		//// 7 - linearize

		boost::shared_ptr<GaussianFactorGraph> factors;

		if (relinFromLast) {
			// ordering provides all keys in conditionals, there cannot be others because path to root included
			set<Symbol> affectedKeys;
			list<Symbol> tmp = affectedBayesNet.ordering();
			affectedKeys.insert(tmp.begin(), tmp.end());

			// todo - remerge in keys of new factors
			affectedKeys.insert(newKeys.begin(), newKeys.end());

			// Save number of affected variables
			lastAffectedVariableCount = affectedKeys.size();

			factors = relinearizeAffectedFactors(affectedKeys);

			// Save number of affected factors
			lastAffectedFactorCount = factors->size();

			// add the cached intermediate results from the boundary of the orphans ...
			FactorGraph<GaussianFactor> cachedBoundary = getCachedBoundaryFactors(orphans);
			factors->push_back(cachedBoundary);
		} else {
			// reuse the old factors
			FactorGraph<GaussianFactor> tmp(affectedBayesNet);
			factors.reset(new GaussianFactorGraph);
			factors->push_back(tmp);
			factors->push_back(*newFactors.linearize(theta_)); // avoid temporary ?
		}

		//// 8 - eliminate and add orphans back in

		// create an ordering for the new and contaminated factors
		// newKeys are passed in: those variables will be forced to the end in the ordering
		set<Symbol> newKeysSet;
		newKeysSet.insert(newKeys.begin(), newKeys.end());
		Ordering ordering = factors->getConstrainedOrdering(newKeysSet);

		// eliminate into a Bayes net
		BayesNet<Conditional> bayesNet = _eliminate(*factors, cached_, ordering);

		// Create Index from ordering
		IndexTable<Symbol> index(ordering);

		// insert conditionals back in, straight into the topless bayesTree
		typename BayesNet<Conditional>::const_reverse_iterator rit;
		for ( rit=bayesNet.rbegin(); rit != bayesNet.rend(); ++rit )
			this->insert(*rit, index);

		// Save number of affectedCliques
		lastAffectedCliqueCount = this->size();

		// add orphans to the bottom of the new tree
		BOOST_FOREACH(sharedClique orphan, orphans) {
			Symbol parentRepresentative = findParentClique(orphan->separator_, index);
			sharedClique parent = (*this)[parentRepresentative];
			parent->children_ += orphan;
			orphan->parent_ = parent; // set new parent!
		}

		//// 9 - update solution

		delta_ = optimize2(*this, wildfire_threshold);

		//// 10 - mark variables, if significant change

		marked_.clear();
		deltaMarked_ = VectorConfig(); // clear
		if (relinearize) { // decides about next step!!!

			for (VectorConfig::const_iterator it = delta_.begin(); it!=delta_.end(); it++) {
				Symbol key = it->first;
				Vector v = it->second;
				if (max(abs(v)) >= relinearize_threshold) {
					marked_.push_back(key);
					deltaMarked_.insert(key, v);
				}
			}

			// not part of the formal algorithm, but needed to allow initialization of new variables outside by the user
			thetaFuture_ = expmap(thetaFuture_, deltaMarked_);
		}

	}

	template<class Conditional, class Config>
	void ISAM2<Conditional, Config>::linear_update(const FactorGraph<GaussianFactor>& newFactors) {

		// Input: BayesTree(this), newFactors

		// 1. Remove top of Bayes tree and convert to a factor graph:
		// (a) For each affected variable, remove the corresponding clique and all parents up to the root.
		// (b) Store orphaned sub-trees \BayesTree_{O} of removed cliques.
		const list<Symbol> newKeys = newFactors.keys();
		Cliques orphans;
		BayesNet<GaussianConditional> affectedBayesNet;
		this->removeTop(newKeys, affectedBayesNet, orphans);
		FactorGraph<GaussianFactor> factors(affectedBayesNet);

		// 2. Add the new factors \Factors' into the resulting factor graph
		factors.push_back(newFactors);

		// 3. Re-order and eliminate the factor graph into a Bayes net (Algorithm [alg:eliminate]), and re-assemble into a new Bayes tree (Algorithm [alg:BayesTree])

		// create an ordering for the new and contaminated factors
		// newKeys are passed in: those variables will be forced to the end in the ordering
		set<Symbol> newKeysSet;
		newKeysSet.insert(newKeys.begin(), newKeys.end());
		Ordering ordering = factors.getConstrainedOrdering(newKeysSet);

		// eliminate into a Bayes net
		BayesNet<Conditional> bayesNet = _eliminate(factors, cached_, ordering);

		// Create Index from ordering
		IndexTable<Symbol> index(ordering);

		// insert conditionals back in, straight into the topless bayesTree
		typename BayesNet<Conditional>::const_reverse_iterator rit;
		for ( rit=bayesNet.rbegin(); rit != bayesNet.rend(); ++rit )
			this->insert(*rit, index);

		// Save number of affectedCliques
		lastAffectedCliqueCount = this->size();

		// 4. Insert the orphans back into the new Bayes tree.

		// add orphans to the bottom of the new tree
		BOOST_FOREACH(sharedClique orphan, orphans) {
			Symbol parentRepresentative = findParentClique(orphan->separator_, index);
			sharedClique parent = (*this)[parentRepresentative];
			parent->children_ += orphan;
			orphan->parent_ = parent; // set new parent!
		}

		// Output: BayesTree(this)
	}

	template<class Conditional, class Config>
	void ISAM2<Conditional, Config>::find_all(sharedClique clique, list<Symbol>& keys, const list<Symbol>& marked) {
		// does the separator contain any of the variables?
		bool found = false;
		BOOST_FOREACH(const Symbol& key, clique->separator_) {
			if (find(marked.begin(), marked.end(), key) != marked.end())
				found = true;
		}
		if (found) {
			// then add this clique
			keys.push_back(clique->keys().front());
		}
		BOOST_FOREACH(const sharedClique& child, clique->children_) {
			find_all(child, keys, marked);
		}
	}

	template<class Conditional, class Config>
	void ISAM2<Conditional, Config>::fluid_relinearization(double relinearize_threshold) {

		// Input: nonlinear factors factors_, linearization point theta_, Bayes tree (this), delta_

		// 1. Mark variables in \Delta above threshold \beta: J=\{\Delta_{j}\in\Delta|\Delta_{j}\geq\beta\}.
	  list<Symbol> marked;
		VectorConfig deltaMarked;
		for (VectorConfig::const_iterator it = delta_.begin(); it!=delta_.end(); it++) {
			Symbol key = it->first;
			Vector v = it->second;
			if (max(abs(v)) >= relinearize_threshold) {
				marked.push_back(key);
				deltaMarked.insert(key, v);
			}
		}

		// 2. Update linearization point for marked variables: \Theta_{J}:=\Theta_{J}+\Delta_{J}.
		theta_ = expmap(theta_, deltaMarked);

		// 3. Mark all cliques that involve marked variables \Theta_{J} and all their ancestors.

		// mark all cliques that involve marked variables
		list<Symbol> affectedSymbols(marked); // add all marked
		find_all(this->root(), affectedSymbols, marked); // add other cliques that have the marked ones in the separator

		// 4. From the leaves to the top, if a clique is marked:
		//    re-linearize the original factors in \Factors associated with the clique,
		//    add the cached marginal factors from its children, and re-eliminate.

		// todo: for simplicity, currently simply remove the top and recreate it using the original ordering

		Cliques orphans;
		BayesNet<GaussianConditional> affectedBayesNet;
		this->removeTop(affectedSymbols, affectedBayesNet, orphans);
		// remember original ordering
//		Ordering original_ordering = affectedBayesNet.ordering();

		boost::shared_ptr<GaussianFactorGraph> factors;

		// ordering provides all keys in conditionals, there cannot be others because path to root included
		set<Symbol> affectedKeys;
		list<Symbol> tmp = affectedBayesNet.ordering();
		affectedKeys.insert(tmp.begin(), tmp.end());
		factors = relinearizeAffectedFactors(affectedKeys);

		Ordering original_ordering = factors->getOrdering(); // todo - hack

		// add the cached intermediate results from the boundary of the orphans ...
		FactorGraph<GaussianFactor> cachedBoundary = getCachedBoundaryFactors(orphans);
		factors->push_back(cachedBoundary);

		// eliminate into a Bayes net
		BayesNet<Conditional> bayesNet = _eliminate(*factors, cached_, original_ordering);

		// Create Index from ordering
		IndexTable<Symbol> index(original_ordering);

		// insert conditionals back in, straight into the topless bayesTree
		typename BayesNet<Conditional>::const_reverse_iterator rit;
		for ( rit=bayesNet.rbegin(); rit != bayesNet.rend(); ++rit )
			this->insert(*rit, index);

		// add orphans to the bottom of the new tree
		BOOST_FOREACH(sharedClique orphan, orphans) {
			Symbol parentRepresentative = findParentClique(orphan->separator_, index);
			sharedClique parent = (*this)[parentRepresentative];
			parent->children_ += orphan;
			orphan->parent_ = parent; // set new parent!
		}

		// Output: updated Bayes tree (this), updated linearization point theta_

	}

	template<class Conditional, class Config>
	void ISAM2<Conditional, Config>::update(
			const NonlinearFactorGraph<Config>& newFactors, const Config& newTheta,
			double wildfire_threshold, double relinearize_threshold, bool relinearize) {

#if 1
		// old algorithm:
		Cliques orphans;
		this->update_internal(newFactors, newTheta, orphans, wildfire_threshold, relinearize_threshold, relinearize);

		delta_.print();
		this->print();

#else

		printf("**1\n");fflush(stdout);
		// 1. Add any new factors \Factors:=\Factors\cup\Factors'.
		nonlinearFactors_.push_back(newFactors);
		printf("**2\n");fflush(stdout);
		// 2. Initialize any new variables \Theta_{new} and add \Theta:=\Theta\cup\Theta_{new}.
		theta_.insert(newTheta);
		printf("**3\n");fflush(stdout);

		// 3. Linearize new factor
		boost::shared_ptr<GaussianFactorGraph> linearFactors = newFactors.linearize(theta_);
		printf("**4\n");fflush(stdout);

		// 4. Linear iSAM step (alg 3)
		linear_update(*linearFactors); // in: this

		printf("**5\n");fflush(stdout);
		// 5. Calculate Delta (alg 0)
		delta_ = optimize2(*this, wildfire_threshold);

		printf("**6\n");fflush(stdout);
		// 6. Iterate Algorithm 4 until no more re-linearizations occur
//		if (relinearize)
//			fluid_relinearization(relinearize_threshold); // in: delta_, theta_, nonlinearFactors_, this

		printf("**7\n");fflush(stdout);

		// todo: linearization point and delta_ do not fit... have to update delta again
		delta_ = optimize2(*this, wildfire_threshold);
		printf("**8\n");fflush(stdout);

		delta_.print();
		this->print();

		printf("**9\n");fflush(stdout);


#endif
	}

/* ************************************************************************* */

}
/// namespace gtsam

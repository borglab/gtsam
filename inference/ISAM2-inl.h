/**
 * @file    ISAM2-inl.h
 * @brief   Incremental update functionality (ISAM2) for BayesTree, with fluid relinearization.
 * @author  Michael Kaess
 */

#include <boost/foreach.hpp>
#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include <set>

#include <gtsam/nonlinear/NonlinearFactorGraph-inl.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/linear/VectorConfig.h>

#include <gtsam/inference/Conditional.h>
#include <gtsam/inference/BayesTree-inl.h>
#include <gtsam/inference/ISAM2.h>


#if 1 // timing
#include <sys/time.h>
// simple class for accumulating execution timing information by name
class Timing {
	class Stats {
	public:
		double t0;
		double t;
		double t_max;
		double t_min;
		int n;
	};
	map<string, Stats> stats;
public:
	void add_t0(string id, double t0) {
		stats[id].t0 = t0;
	}
	double get_t0(string id) {
		return stats[id].t0;
	}
	void add_dt(string id, double dt) {
		Stats& s = stats[id];
		s.t += dt;
		s.n++;
		if (s.n==1 || s.t_max < dt) s.t_max = dt;
		if (s.n==1 || s.t_min > dt) s.t_min = dt;
	}
	void print() {
		map<string, Stats>::iterator it;
		for(it = stats.begin(); it!=stats.end(); it++) {
			Stats& s = it->second;
			printf("%s: %g (%i times, min: %g, max: %g)\n",
					it->first.c_str(), s.t, s.n, s.t_min, s.t_max);
		}
	}
	double time(string id) {
		Stats& s = stats[id];
		return s.t;
	}
};
Timing timing;
double tic() {
	struct timeval t;
	gettimeofday(&t, NULL);
	return ((double)t.tv_sec + ((double)t.tv_usec)/1000000.);
}
double toc(double t) {
	double s = tic();
	return (max(0., s-t));
}
double tic(string id) {
	double t0 = tic();
	timing.add_t0(id, t0);
	return t0;
}
double toc(string id) {
	double dt = toc(timing.get_t0(id));
	timing.add_dt(id, dt);
	return dt;
}
void tictoc_print() {
	timing.print();
}
#else
	void tictoc_print() {}
double tic(string id) {return 0.;}
double toc(string id) {return 0.;}
#endif


namespace gtsam {

  using namespace std;

	// from inference-inl.h - need to additionally return the newly created factor for caching
	boost::shared_ptr<GaussianConditional> _eliminateOne(FactorGraph<GaussianFactor>& graph, CachedFactors& cached, const Symbol& key) {

		// combine the factors of all nodes connected to the variable to be eliminated
		// if no factors are connected to key, returns an empty factor
		tic("eliminate_removeandcombinefactors");
		boost::shared_ptr<GaussianFactor> joint_factor = removeAndCombineFactors(graph,key);
		toc("eliminate_removeandcombinefactors");

		// eliminate that joint factor
		boost::shared_ptr<GaussianFactor> factor;
		boost::shared_ptr<GaussianConditional> conditional;
		tic("eliminate_eliminate");
		boost::tie(conditional, factor) = joint_factor->eliminate(key);
		toc("eliminate_eliminate");

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
	: BayesTree<Conditional>(nlfg.linearize(config)->eliminate(ordering)), theta_(config), nonlinearFactors_(nlfg) {
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
	template<class Conditional, class Config>
	void ISAM2<Conditional, Config>::linear_update(const FactorGraph<GaussianFactor>& newFactors) {

		// Input: BayesTree(this), newFactors

//#define PRINT_STATS // todo: figures for paper, disable for timing
#ifdef PRINT_STATS
		static int counter = 0;
		int maxClique = 0;
		double avgClique = 0;
		int numCliques = 0;
		if (counter>0) { // cannot call on empty tree
			GaussianISAM2_P::CliqueData cdata =  this->getCliqueData();
			GaussianISAM2_P::CliqueStats cstats = cdata.getStats();
			maxClique = cstats.maxConditionalSize;
			avgClique = cstats.avgConditionalSize;
			numCliques = cdata.conditionalSizes.size();
		}
		counter++;
#endif

		// 1. Remove top of Bayes tree and convert to a factor graph:
		// (a) For each affected variable, remove the corresponding clique and all parents up to the root.
		// (b) Store orphaned sub-trees \BayesTree_{O} of removed cliques.
		const list<Symbol> newKeys = newFactors.keys();
		Cliques orphans;
		BayesNet<GaussianConditional> affectedBayesNet;
		this->removeTop(newKeys, affectedBayesNet, orphans);

		//		FactorGraph<GaussianFactor> factors(affectedBayesNet);
		// bug was here: we cannot reuse the original factors, because then the cached factors get messed up
		// [all the necessary data is actually contained in the affectedBayesNet, including what was passed in from the boundaries,
		//  so this would be correct; however, in the process we also generate new cached_ entries that will be wrong (ie. they don't
		//  contain what would be passed up at a certain point if batch elimination was done, but that's what we need); we could choose
		//  not to update cached_ from here, but then the new information (and potentially different variable ordering) is not reflected
		//  in the cached_ values which again will be wrong]
		// so instead we have to retrieve the original linearized factors AND add the cached factors from the boundary

		// BEGIN OF COPIED CODE

		tic("linear_lookup1");
		// ordering provides all keys in conditionals, there cannot be others because path to root included
		set<Symbol> affectedKeys;
		list<Symbol> tmp = affectedBayesNet.ordering();
		affectedKeys.insert(tmp.begin(), tmp.end());

		// Save number of affected variables
		lastAffectedVariableCount = affectedKeys.size();

		FactorGraph<GaussianFactor> factors(*relinearizeAffectedFactors(affectedKeys));  // todo: no need to relinearize here, should have cached linearized factors

		// Save number of affected factors
		lastAffectedFactorCount = factors.size();
		// output for generating figures
#ifdef PRINT_STATS
		cout << "linear: #newKeys: " << newKeys.size() << " #affectedVariables: " << affectedKeys.size()
		  << " #affectedFactors: " << factors.size() << " maxCliqueSize: " << maxClique
		  << " avgCliqueSize: " << avgClique << " #Cliques: " << numCliques << endl;
#endif

		toc("linear_lookup1");

		// add the cached intermediate results from the boundary of the orphans ...
		FactorGraph<GaussianFactor> cachedBoundary = getCachedBoundaryFactors(orphans);
		factors.push_back(cachedBoundary);

		// END OF COPIED CODE


		// 2. Add the new factors \Factors' into the resulting factor graph
		factors.push_back(newFactors);

		// 3. Re-order and eliminate the factor graph into a Bayes net (Algorithm [alg:eliminate]), and re-assemble into a new Bayes tree (Algorithm [alg:BayesTree])

		// create an ordering for the new and contaminated factors
		// newKeys are passed in: those variables will be forced to the end in the ordering
		set<Symbol> newKeysSet;
		newKeysSet.insert(newKeys.begin(), newKeys.end());
		Ordering ordering = factors.getConstrainedOrdering(newKeysSet);
//		Ordering ordering = factors.getOrdering();

		// eliminate into a Bayes net
		tic("linear_eliminate");
		BayesNet<Conditional> bayesNet = _eliminate(factors, cached_, ordering);
		toc("linear_eliminate");

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

	/* ************************************************************************* */
	// find all variables that are directly connected by a measurement to one of the marked variables
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

	/* ************************************************************************* */
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

		if (marked.size()>0) {

			// 2. Update linearization point for marked variables: \Theta_{J}:=\Theta_{J}+\Delta_{J}.
			theta_ = theta_.expmap(deltaMarked);

			// 3. Mark all cliques that involve marked variables \Theta_{J} and all their ancestors.

			// mark all cliques that involve marked variables
			list<Symbol> affectedSymbols(marked); // add all marked
			tic("nonlin-find_all");
			find_all(this->root(), affectedSymbols, marked); // add other cliques that have the marked ones in the separator
			affectedSymbols.sort(); // remove duplicates
			affectedSymbols.unique();
			toc("nonlin-find_all");

			// 4. From the leaves to the top, if a clique is marked:
			//    re-linearize the original factors in \Factors associated with the clique,
			//    add the cached marginal factors from its children, and re-eliminate.

			// todo: for simplicity, currently simply remove the top and recreate it using the original ordering

			tic("nonlin-mess");
			Cliques orphans;
			BayesNet<GaussianConditional> affectedBayesNet;
			this->removeTop(affectedSymbols, affectedBayesNet, orphans);
			// remember original ordering
// todo			Ordering original_ordering = affectedBayesNet.ordering(); // does not yield original ordering...
			FactorGraph<GaussianFactor> tmp_factors(affectedBayesNet); // so instead we recalculate an acceptable ordering here
			Ordering original_ordering = tmp_factors.getOrdering(); // todo - remove multiple lines up to here

			boost::shared_ptr<GaussianFactorGraph> factors;

			// ordering provides all keys in conditionals, there cannot be others because path to root included
			set<Symbol> affectedKeys;
			list<Symbol> tmp = affectedBayesNet.ordering();
			affectedKeys.insert(tmp.begin(), tmp.end());
			toc("nonlin-mess");

			tic("nonlin_relin");
			factors = relinearizeAffectedFactors(affectedKeys);
			toc("nonlin_relin");

			cout << "nonlinear: #marked: " << marked.size() << " #affected: " << affectedKeys.size() << " #factors: " << factors->size() << endl;

			// add the cached intermediate results from the boundary of the orphans ...
			FactorGraph<GaussianFactor> cachedBoundary = getCachedBoundaryFactors(orphans);
			factors->push_back(cachedBoundary);

			// eliminate into a Bayes net
			tic("nonlin_eliminate");
			BayesNet<Conditional> bayesNet = _eliminate(*factors, cached_, original_ordering);
			toc("nonlin_eliminate");

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
	}

	/* ************************************************************************* */
	template<class Conditional, class Config>
	void ISAM2<Conditional, Config>::update(
			const NonlinearFactorGraph<Config>& newFactors, const Config& newTheta,
			double wildfire_threshold, double relinearize_threshold, bool relinearize) {

		tic("all");

		tic("step1");
		// 1. Add any new factors \Factors:=\Factors\cup\Factors'.
		nonlinearFactors_.push_back(newFactors);
		toc("step1");

		tic("step2");
		// 2. Initialize any new variables \Theta_{new} and add \Theta:=\Theta\cup\Theta_{new}.
		theta_.insert(newTheta);
		toc("step2");

		tic("step3");
		// 3. Linearize new factor
		boost::shared_ptr<GaussianFactorGraph> linearFactors = newFactors.linearize(theta_);
		toc("step3");

		tic("step4");
		// 4. Linear iSAM step (alg 3)
		linear_update(*linearFactors); // in: this
		toc("step4");

		tic("step5");
		// 5. Calculate Delta (alg 0)
		delta_ = optimize2(*this, wildfire_threshold);
		toc("step5");

		tic("step6");
		// 6. Iterate Algorithm 4 until no more re-linearizations occur
		if (relinearize) {
			fluid_relinearization(relinearize_threshold); // in: delta_, theta_, nonlinearFactors_, this
		}
		toc("step6");

		// todo: not part of algorithm in paper: linearization point and delta_ do not fit... have to update delta again
//todo		delta_ = optimize2(*this, wildfire_threshold);

		toc("all");

		tictoc_print();
	}

/* ************************************************************************* */

}
/// namespace gtsam

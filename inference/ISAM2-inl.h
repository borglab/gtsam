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


#if 1 // timing - note: adds some time when applied in inner loops
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
boost::shared_ptr<GaussianBayesNet> _eliminate(FactorGraph<GaussianFactor>& graph, CachedFactors& cached, const Ordering& ordering) {
	boost::shared_ptr<GaussianBayesNet> chordalBayesNet(new GaussianBayesNet()); // empty
	BOOST_FOREACH(const Symbol& key, ordering) {
		GaussianConditional::shared_ptr cg = _eliminateOne(graph, cached, key);
		chordalBayesNet->push_back(cg);
	}
	return chordalBayesNet;
}

// special const version used in constructor below
boost::shared_ptr<GaussianBayesNet> _eliminate_const(const FactorGraph<GaussianFactor>& graph, CachedFactors& cached, const Ordering& ordering) {
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
: BayesTree<Conditional>(nlfg.linearize(config)->eliminate(ordering)),
  theta_(config), delta_(VectorConfig()), nonlinearFactors_(nlfg) {
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
(const list<Symbol>& affectedKeys) const {

	list<size_t> candidates = getAffectedFactors(affectedKeys);

	NonlinearFactorGraph<Config> nonlinearAffectedFactors;

	// for fast lookup below
	set<Symbol> affectedKeysSet;
	affectedKeysSet.insert(affectedKeys.begin(), affectedKeys.end());

	BOOST_FOREACH(size_t idx, candidates) {
		bool inside = true;
		BOOST_FOREACH(const Symbol& key, nonlinearFactors_[idx]->keys()) {
			if (affectedKeysSet.find(key) == affectedKeysSet.end()) {
				inside = false;
				break;
			}
		}
		if (inside)
			nonlinearAffectedFactors.push_back(nonlinearFactors_[idx]);
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
void ISAM2<Conditional, Config>::recalculate(const list<Symbol>& markedKeys, const FactorGraph<GaussianFactor>* newFactors) {

	// Input: BayesTree(this), newFactors

//#define PRINT_STATS // figures for paper, disable for timing
#ifdef PRINT_STATS
	static int counter = 0;
	int maxClique = 0;
	double avgClique = 0;
	int numCliques = 0;
	int nnzR = 0;
	if (counter>0) { // cannot call on empty tree
		GaussianISAM2_P::CliqueData cdata =  this->getCliqueData();
		GaussianISAM2_P::CliqueStats cstats = cdata.getStats();
		maxClique = cstats.maxConditionalSize;
		avgClique = cstats.avgConditionalSize;
		numCliques = cdata.conditionalSizes.size();
		nnzR = calculate_nnz(this->root());
	}
	counter++;
#endif

	// 1. Remove top of Bayes tree and convert to a factor graph:
	// (a) For each affected variable, remove the corresponding clique and all parents up to the root.
	// (b) Store orphaned sub-trees \BayesTree_{O} of removed cliques.
	tic("re-removetop");
	Cliques orphans;
	BayesNet<GaussianConditional> affectedBayesNet;
	this->removeTop(markedKeys, affectedBayesNet, orphans);
	toc("re-removetop");

	//		FactorGraph<GaussianFactor> factors(affectedBayesNet);
	// bug was here: we cannot reuse the original factors, because then the cached factors get messed up
	// [all the necessary data is actually contained in the affectedBayesNet, including what was passed in from the boundaries,
	//  so this would be correct; however, in the process we also generate new cached_ entries that will be wrong (ie. they don't
	//  contain what would be passed up at a certain point if batch elimination was done, but that's what we need); we could choose
	//  not to update cached_ from here, but then the new information (and potentially different variable ordering) is not reflected
	//  in the cached_ values which again will be wrong]
	// so instead we have to retrieve the original linearized factors AND add the cached factors from the boundary

	// BEGIN OF COPIED CODE

	tic("re-lookup");
	// ordering provides all keys in conditionals, there cannot be others because path to root included
	list<Symbol> affectedKeys = affectedBayesNet.ordering();
	FactorGraph<GaussianFactor> factors(*relinearizeAffectedFactors(affectedKeys));

	lastAffectedMarkedCount = markedKeys.size();
	lastAffectedVariableCount = affectedKeys.size();
	lastAffectedFactorCount = factors.size();

#ifdef PRINT_STATS
	// output for generating figures
	cout << "linear: #markedKeys: " << markedKeys.size() << " #affectedVariables: " << affectedKeys.size()
		  		<< " #affectedFactors: " << factors.size() << " maxCliqueSize: " << maxClique
		  		<< " avgCliqueSize: " << avgClique << " #Cliques: " << numCliques << " nnzR: " << nnzR << endl;
#endif

	toc("re-lookup");

	tic("re-cached");
	// add the cached intermediate results from the boundary of the orphans ...
	FactorGraph<GaussianFactor> cachedBoundary = getCachedBoundaryFactors(orphans);
	factors.push_back(cachedBoundary);
	toc("re-cached");

	// END OF COPIED CODE


	// 2. Add the new factors \Factors' into the resulting factor graph
	tic("re-newfactors");
	if (newFactors) {
		factors.push_back(*newFactors);
	}
	toc("re-newfactors");

	// 3. Re-order and eliminate the factor graph into a Bayes net (Algorithm [alg:eliminate]), and re-assemble into a new Bayes tree (Algorithm [alg:BayesTree])

	tic("re-order");
	// create an ordering for the new and contaminated factors
	// markedKeys are passed in: those variables will be forced to the end in the ordering
	set<Symbol> markedKeysSet;
	markedKeysSet.insert(markedKeys.begin(), markedKeys.end());
	Ordering ordering = factors.getConstrainedOrdering(markedKeysSet); // intelligent ordering
	//		Ordering ordering = factors.getOrdering(); // original ordering, yields bad performance
	toc("re-order");

	// eliminate into a Bayes net
	tic("eliminate");
	boost::shared_ptr<GaussianBayesNet> bayesNet = _eliminate(factors, cached_, ordering);
	toc("eliminate");

	tic("re-assemble");
	// Create Index from ordering
	IndexTable<Symbol> index(ordering);

	// insert conditionals back in, straight into the topless bayesTree
	typename BayesNet<Conditional>::const_reverse_iterator rit;
	for ( rit=bayesNet->rbegin(); rit != bayesNet->rend(); ++rit )
		this->insert(*rit, index);

	// Save number of affectedCliques
	lastAffectedCliqueCount = this->size();
	toc("re-assemble");

	// 4. Insert the orphans back into the new Bayes tree.

	tic("re-orphans");
	// add orphans to the bottom of the new tree
	BOOST_FOREACH(sharedClique orphan, orphans) {
		Symbol parentRepresentative = findParentClique(orphan->separator_, index);
		sharedClique parent = (*this)[parentRepresentative];
		parent->children_ += orphan;
		orphan->parent_ = parent; // set new parent!
	}
	toc("re-orphans");

	// Output: BayesTree(this)
}

/* ************************************************************************* */
template<class Conditional, class Config>
void ISAM2<Conditional, Config>::linear_update(const FactorGraph<GaussianFactor>& newFactors) {
	const list<Symbol> markedKeys = newFactors.keys();
	recalculate(markedKeys, &newFactors);
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
void ISAM2<Conditional, Config>::update(
		const NonlinearFactorGraph<Config>& newFactors, const Config& newTheta,
		double wildfire_threshold, double relinearize_threshold, bool relinearize) {

	static int count = 0;
	count++;

	lastAffectedVariableCount = 0;
	lastAffectedFactorCount = 0;
	lastAffectedCliqueCount = 0;
	lastAffectedMarkedCount = 0;
	lastNonlinearMarkedCount = 0;
	lastNonlinearAffectedVariableCount = 0;
	lastNonlinearAffectedFactorCount = 0;

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
	// 3. Mark linear update
	list<Symbol> markedKeys = newFactors.keys();
	toc("step3");

//#define SEPARATE_STEPS
#ifdef SEPARATE_STEPS // original algorithm from paper: separate relin and optimize
	boost::shared_ptr<GaussianFactorGraph> linearFactors = newFactors.linearize(theta_);
	recalculate(markedKeys, &(*linearFactors));
	markedKeys.clear();
	delta_ = optimize2(*this, wildfire_threshold);
#endif

	VectorConfig deltaMarked;
	if (relinearize && count%10 == 0) { // todo: every n steps
		tic("step4");
		// 4. Mark keys in \Delta above threshold \beta: J=\{\Delta_{j}\in\Delta|\Delta_{j}\geq\beta\}.
		list<Symbol> markedRelin;
		for (VectorConfig::const_iterator it = delta_.begin(); it!=delta_.end(); it++) {
			const Symbol& key = it->first;
			const Vector& v = it->second;
			if (max(abs(v)) >= relinearize_threshold) {
				markedRelin.push_back(key);
				deltaMarked.insert(key, v);
			}
		}
		toc("step4");

		tic("step5");
		// 5. Mark all cliques that involve marked variables \Theta_{J} and all their ancestors.
		list<Symbol> affectedKeys;
		if (markedRelin.size()>0) {
			// mark all cliques that involve marked variables
			affectedKeys = markedRelin; // add all marked
			tic("fluid-find_all");
			find_all(this->root(), affectedKeys, markedRelin); // add other cliques that have the marked ones in the separator
			affectedKeys.sort(); // remove duplicates
			affectedKeys.unique();
			toc("fluid-find_all");
		}
		// merge with markedKeys
		markedKeys.splice(markedKeys.begin(), affectedKeys, affectedKeys.begin(), affectedKeys.end());
		markedKeys.sort(); // remove duplicates
		markedKeys.unique();
		toc("step5");

	}

	tic("step6");
	// 6. Update linearization point for marked variables: \Theta_{J}:=\Theta_{J}+\Delta_{J}.
	if (deltaMarked.size()>0) {
		theta_ = theta_.expmap(deltaMarked);
	}
	toc("step6");

#ifndef SEPARATE_STEPS
	tic("step7");
	// 7. Linearize new factors
	boost::shared_ptr<GaussianFactorGraph> linearFactors = newFactors.linearize(theta_);
	toc("step7");

	tic("step8");
	// 8. Redo top of Bayes tree
	recalculate(markedKeys, &(*linearFactors));
	toc("step8");
#else
	recalculate(markedKeys);
#endif

	tic("step9");
	// 9. Solve
	delta_ = optimize2(*this, wildfire_threshold);
	toc("step9");

	toc("all");
	tictoc_print(); // switch on/off at top of file (#if 1/#if 0)
}

/* ************************************************************************* */

}
/// namespace gtsam

/**
 * @file    ISAM2.h
 * @brief   Incremental update functionality (ISAM2) for BayesTree, with fluid relinearization.
 * @author  Michael Kaess
 */

// \callgraph

#pragma once

#include <map>
#include <list>
#include <vector>
//#include <boost/serialization/map.hpp>
//#include <boost/serialization/list.hpp>
#include <stdexcept>

#include <gtsam/base/types.h>
#include <gtsam/base/Testable.h>
#include <gtsam/inference/FactorGraph.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Ordering.h>
#include <gtsam/inference/BayesNet.h>
#include <gtsam/inference/BayesTree.h>
#include <gtsam/linear/GaussianFactorGraph.h>

namespace gtsam {

//typedef std::vector<GaussianFactor::shared_ptr> CachedFactors;

template<class Conditional, class Values>
class ISAM2: public BayesTree<Conditional> {

protected:

	// current linearization point
	Values theta_;

  // VariableIndex lets us look up factors by involved variable and keeps track of dimensions
  typedef GaussianVariableIndex<VariableIndexStorage_deque> VariableIndexType;
  VariableIndexType variableIndex_;

	// the linear solution, an update to the estimate in theta
	VectorValues deltaUnpermuted_;

  // The residual permutation through which the deltaUnpermuted_ is
  // referenced.  Permuting the VectorValues is slow, so for performance the
  // permutation is applied at access time instead of to the VectorValues
  // itself.
  Permuted<VectorValues> delta_;

	// for keeping all original nonlinear factors
	NonlinearFactorGraph<Values> nonlinearFactors_;

	// The "ordering" allows converting Symbols to varid_t (integer) keys.  We
	// keep it up to date as we add and reorder variables.
	Ordering ordering_;

	// cached intermediate results for restarting computation in the middle
//	CachedFactors cached_;

#ifndef NDEBUG
	std::vector<bool> lastRelinVariables_;
#endif

public:

	/** Create an empty Bayes Tree */
	ISAM2();

//	/** Create a Bayes Tree from a Bayes Net */
//	ISAM2(const NonlinearFactorGraph<Values>& fg, const Ordering& ordering, const Values& config);

	/** Destructor */
	virtual ~ISAM2() {}

	typedef typename BayesTree<Conditional>::sharedClique sharedClique;

	typedef typename BayesTree<Conditional>::Cliques Cliques;

	/**
	 * ISAM2.
	 */
	void update(const NonlinearFactorGraph<Values>& newFactors, const Values& newTheta,
			double wildfire_threshold = 0., double relinearize_threshold = 0., bool relinearize = true);

	// needed to create initial estimates
	const Values& getLinearizationPoint() const {return theta_;}

	// estimate based on incomplete delta (threshold!)
	Values calculateEstimate() const;

	// estimate based on full delta (note that this is based on the current linearization point)
	Values calculateBestEstimate() const;

	const Permuted<VectorValues>& getDelta() const { return delta_; }

	const NonlinearFactorGraph<Values>& getFactorsUnsafe() const { return nonlinearFactors_; }

	const Ordering& getOrdering() const { return ordering_; }

	size_t lastAffectedVariableCount;
	size_t lastAffectedFactorCount;
	size_t lastAffectedCliqueCount;
	size_t lastAffectedMarkedCount;
	size_t lastBacksubVariableCount;
	size_t lastNnzTop;

private:

	std::list<size_t> getAffectedFactors(const std::list<varid_t>& keys) const;
	boost::shared_ptr<GaussianFactorGraph> relinearizeAffectedFactors(const std::list<varid_t>& affectedKeys) const;
	GaussianFactorGraph getCachedBoundaryFactors(Cliques& orphans);

	boost::shared_ptr<std::set<varid_t> > recalculate(const std::set<varid_t>& markedKeys, const std::vector<varid_t>& newKeys, const GaussianFactorGraph* newFactors = NULL);
//	void linear_update(const GaussianFactorGraph& newFactors);
	void find_all(sharedClique clique, std::set<varid_t>& keys, const std::vector<bool>& marked); // helper function

}; // ISAM2

} /// namespace gtsam

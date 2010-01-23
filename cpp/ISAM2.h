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
#include <boost/serialization/map.hpp>
#include <boost/serialization/list.hpp>
#include <stdexcept>

#include "Testable.h"
#include "FactorGraph.h"
#include "NonlinearFactorGraph.h"
#include "BayesNet.h"
#include "BayesTree.h"
#include "Key.h"
#include "SymbolMap.h"

namespace gtsam {

	typedef SymbolMap<GaussianFactor::shared_ptr> CachedFactors;

	template<class Conditional, class Config>
	class ISAM2: public BayesTree<Conditional> {

	protected:

		// current linearization point
		Config theta_;
		Config thetaFuture_; // lin point of next iteration

		// for keeping all original nonlinear factors
		NonlinearFactorGraph<Config> nonlinearFactors_;

		// cached intermediate results for restarting computation in the middle
		CachedFactors cached_;

		// the linear solution, an update to the estimate in theta
		VectorConfig delta_;
		VectorConfig deltaMarked_;

		// variables that have been updated, requiring the corresponding factors to be relinearized
		std::list<Symbol> marked_;

	public:

		/** Create an empty Bayes Tree */
		ISAM2();

		/** Create a Bayes Tree from a Bayes Net */
		ISAM2(const NonlinearFactorGraph<Config>& fg, const Ordering& ordering, const Config& config);

		/** Destructor */
		virtual ~ISAM2() {
		}

		typedef typename BayesTree<Conditional>::sharedClique sharedClique;

		typedef typename BayesTree<Conditional>::Cliques Cliques;

		/**
		 * ISAM2. (update_internal provides access to list of orphans for drawing purposes)
		 */
		void update_internal(const NonlinearFactorGraph<Config>& newFactors,
				const Config& newTheta, Cliques& orphans,
				double wildfire_threshold, double relinearize_threshold, bool relinearize);
		void update(const NonlinearFactorGraph<Config>& newFactors, const Config& newTheta,
				double wildfire_threshold = 0., double relinearize_threshold = 0., bool relinearize = true);

		// needed to create initial estimates (note that this will be the linearization point in the next step!)
		const Config getLinearizationPoint() const {return thetaFuture_;}
		// estimate based on incomplete delta (threshold!)
		const Config calculateEstimate() const {return expmap(theta_, delta_);}
		// estimate based on full delta (note that this is based on the actual current linearization point)
		const Config calculateBestEstimate() const {return expmap(theta_, optimize2(*this, 0.));}

		const std::list<Symbol>& getMarkedUnsafe() const { return marked_; }

		const NonlinearFactorGraph<Config>& getFactorsUnsafe() const { return nonlinearFactors_; }

		const Config& getThetaUnsafe() const { return theta_; }

		const VectorConfig& getDeltaUnsafe() const { return delta_; }

		size_t lastAffectedVariableCount;
		size_t lastAffectedFactorCount;
		size_t lastAffectedCliqueCount;

	private:

		std::list<int> getAffectedFactors(const std::list<Symbol>& keys) const;
		FactorGraph<GaussianFactor> relinearizeAffectedFactors(const std::set<Symbol>& affectedKeys) const;
		FactorGraph<GaussianFactor> getCachedBoundaryFactors(Cliques& orphans);

	}; // ISAM2

} /// namespace gtsam

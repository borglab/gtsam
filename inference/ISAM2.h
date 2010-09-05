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

#include <gtsam/base/Testable.h>
#include <gtsam/inference/FactorGraph.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/BayesNet.h>
#include <gtsam/inference/BayesTree.h>
#include <gtsam/inference/Key.h>
#include <gtsam/inference/SymbolMap.h>

namespace gtsam {

	typedef SymbolMap<GaussianFactor::shared_ptr> CachedFactors;

	template<class Conditional, class Config>
	class ISAM2: public BayesTree<Conditional> {

	protected:

		// current linearization point
		Config theta_;

		// the linear solution, an update to the estimate in theta
		VectorConfig delta_;

		// for keeping all original nonlinear factors
		NonlinearFactorGraph<Config> nonlinearFactors_;

		// cached intermediate results for restarting computation in the middle
		CachedFactors cached_;

	public:

		/** Create an empty Bayes Tree */
		ISAM2();

		/** Create a Bayes Tree from a Bayes Net */
		ISAM2(const NonlinearFactorGraph<Config>& fg, const Ordering& ordering, const Config& config);

		/** Destructor */
		virtual ~ISAM2() {}

		typedef typename BayesTree<Conditional>::sharedClique sharedClique;

		typedef typename BayesTree<Conditional>::Cliques Cliques;

		/**
		 * ISAM2.
		 */
		void update(const NonlinearFactorGraph<Config>& newFactors, const Config& newTheta,
				double wildfire_threshold = 0., double relinearize_threshold = 0., bool relinearize = true);

		// needed to create initial estimates
		const Config getLinearizationPoint() const {return theta_;}

		// estimate based on incomplete delta (threshold!)
		const Config calculateEstimate() const {return theta_.expmap(delta_);}

		// estimate based on full delta (note that this is based on the current linearization point)
		const Config calculateBestEstimate() const {return theta_.expmap(optimize2(*this, 0.));}

		const NonlinearFactorGraph<Config>& getFactorsUnsafe() const { return nonlinearFactors_; }

		size_t lastAffectedVariableCount;
		size_t lastAffectedFactorCount;
		size_t lastAffectedCliqueCount;

	private:

		std::list<size_t> getAffectedFactors(const std::list<Symbol>& keys) const;
		boost::shared_ptr<GaussianFactorGraph> relinearizeAffectedFactors(const std::set<Symbol>& affectedKeys) const;
		FactorGraph<GaussianFactor> getCachedBoundaryFactors(Cliques& orphans);

		void linear_update(const FactorGraph<GaussianFactor>& newFactors);
		void find_all(sharedClique clique, std::list<Symbol>& keys, const std::list<Symbol>& marked); // helper function
		void fluid_relinearization(double relinearize_threshold);

	}; // ISAM2

} /// namespace gtsam

/**
 * @file    ISAM2.h
 * @brief   Incremental update functionality (ISAM2) for BayesTree.
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

namespace gtsam {

	template<class Conditional, class Config>
	class ISAM2: public BayesTree<Conditional> {

		NonlinearFactorGraph<Config> nonlinearFactors_;

	public:

		/** Create an empty Bayes Tree */
		ISAM2();

		/** Create a Bayes Tree from a Bayes Net */
		ISAM2(const BayesNet<Conditional>& bayesNet);

		/** Destructor */
		virtual ~ISAM2() {
		}

		typedef typename BayesTree<Conditional>::sharedClique sharedClique;

		typedef typename BayesTree<Conditional>::Cliques Cliques;

		/**
		 * ISAM2. (update_internal provides access to list of orphans for drawing purposes)
		 */
		void update_internal(const NonlinearFactorGraph<Config>& newFactors, Cliques& orphans);
		void update(const NonlinearFactorGraph<Config>& newFactors);

	}; // ISAM2

} /// namespace gtsam

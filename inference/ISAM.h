/**
 * @file    ISAM.h
 * @brief   Incremental update functionality (iSAM) for BayesTree.
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
#include "BayesNet.h"
#include "BayesTree.h"

namespace gtsam {

	template<class Conditional>
	class ISAM: public BayesTree<Conditional> {

	public:

		/** Create an empty Bayes Tree */
		ISAM();

		/** Create a Bayes Tree from a Bayes Net */
		ISAM(const BayesNet<Conditional>& bayesNet);

		/** Destructor */
		virtual ~ISAM() {
		}

		typedef typename BayesTree<Conditional>::sharedClique sharedClique;

		typedef typename BayesTree<Conditional>::Cliques Cliques;

		/**
		 * iSAM. (update_internal provides access to list of orphans for drawing purposes)
		 */
		template<class Factor>
		void update_internal(const FactorGraph<Factor>& newFactors, Cliques& orphans);
		template<class Factor>
		void update(const FactorGraph<Factor>& newFactors);

	}; // ISAM

} /// namespace gtsam

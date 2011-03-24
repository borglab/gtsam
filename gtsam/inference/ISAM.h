/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

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
#include <deque>
//#include <boost/serialization/map.hpp>
//#include <boost/serialization/list.hpp>
#include <stdexcept>

#include <gtsam/base/Testable.h>
#include <gtsam/inference/FactorGraph.h>
#include <gtsam/inference/BayesNet.h>
#include <gtsam/inference/BayesTree.h>

namespace gtsam {

	template<class CONDITIONAL>
	class ISAM: public BayesTree<CONDITIONAL> {

	public:

		/** Create an empty Bayes Tree */
		ISAM();

		/** Create a Bayes Tree from a Bayes Net */
		ISAM(const BayesNet<CONDITIONAL>& bayesNet);

		typedef typename BayesTree<CONDITIONAL>::sharedClique sharedClique;

		typedef typename BayesTree<CONDITIONAL>::Cliques Cliques;

		/**
		 * iSAM. (update_internal provides access to list of orphans for drawing purposes)
		 */
		template<class FG>
		void update_internal(const FG& newFactors, Cliques& orphans,
				typename FG::Eliminate function);

		template<class FG>
		void update(const FG& newFactors, typename FG::Eliminate function);

	}; // ISAM

} /// namespace gtsam

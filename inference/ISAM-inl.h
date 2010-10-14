/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    ISAM-inl.h
 * @brief   Incremental update functionality (iSAM) for BayesTree.
 * @author  Michael Kaess
 */

#include <boost/foreach.hpp>
#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include <gtsam/inference/Conditional.h>
#include <gtsam/inference/ISAM.h>
#include <gtsam/inference/BayesTree-inl.h>

namespace gtsam {

	using namespace std;

	/** Create an empty Bayes Tree */
	template<class Conditional>
	ISAM<Conditional>::ISAM() : BayesTree<Conditional>() {}

	/** Create a Bayes Tree from a Bayes Net */
	template<class Conditional>
	ISAM<Conditional>::ISAM(const BayesNet<Conditional>& bayesNet) :
	  BayesTree<Conditional>(bayesNet) {}

	/* ************************************************************************* */
	template<class Conditional>
	template<class FactorGraph>
	void ISAM<Conditional>::update_internal(const FactorGraph& newFactors, Cliques& orphans) {

		// Remove the contaminated part of the Bayes tree
		BayesNet<Conditional> bn;
		removeTop(newFactors.keys(), bn, orphans);
		FactorGraph factors(bn);

		// add the factors themselves
		factors.push_back(newFactors);

		// eliminate into a Bayes net
		typename BayesNet<Conditional>::shared_ptr bayesNet = Inference::Eliminate(factors);

		// insert conditionals back in, straight into the topless bayesTree
		typename BayesNet<Conditional>::const_reverse_iterator rit;
		for ( rit=bayesNet->rbegin(); rit != bayesNet->rend(); ++rit )
			this->insert(*rit);

		// add orphans to the bottom of the new tree
		BOOST_FOREACH(sharedClique orphan, orphans) {
		  this->insert(orphan);
		}

	}

	template<class Conditional>
	template<class FactorGraph>
	void ISAM<Conditional>::update(const FactorGraph& newFactors) {
		Cliques orphans;
		this->update_internal(newFactors, orphans);
	}

}
/// namespace gtsam

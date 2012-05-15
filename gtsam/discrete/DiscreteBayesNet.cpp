/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file DiscreteBayesNet.cpp
 * @date Feb 15, 2011
 * @author Duy-Nguyen Ta
 * @author Frank Dellaert
 */

#include <gtsam/discrete/DiscreteBayesNet.h>
#include <gtsam/discrete/DiscreteConditional.h>
#include <gtsam/inference/BayesNet-inl.h>
#include <boost/make_shared.hpp>

namespace gtsam {

	// Explicitly instantiate so we don't have to include everywhere
	template class BayesNet<DiscreteConditional> ;

	/* ************************************************************************* */
	void add_front(DiscreteBayesNet& bayesNet, const Signature& s) {
		bayesNet.push_front(boost::make_shared<DiscreteConditional>(s));
	}

	/* ************************************************************************* */
	void add(DiscreteBayesNet& bayesNet, const Signature& s) {
		bayesNet.push_back(boost::make_shared<DiscreteConditional>(s));
	}

	/* ************************************************************************* */
	DiscreteFactor::sharedValues optimize(const DiscreteBayesNet& bn) {
		// solve each node in turn in topological sort order (parents first)
		DiscreteFactor::sharedValues result(new DiscreteFactor::Values());
		BOOST_REVERSE_FOREACH (DiscreteConditional::shared_ptr conditional, bn)
			conditional->solveInPlace(*result);
		return result;
	}

	/* ************************************************************************* */
	DiscreteFactor::sharedValues sample(const DiscreteBayesNet& bn) {
		// sample each node in turn in topological sort order (parents first)
		DiscreteFactor::sharedValues result(new DiscreteFactor::Values());
		BOOST_REVERSE_FOREACH(DiscreteConditional::shared_ptr conditional, bn)
			conditional->sampleInPlace(*result);
		return result;
	}

/* ************************************************************************* */
} // namespace

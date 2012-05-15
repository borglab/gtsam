/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file DiscreteBayesNet.h
 * @date Feb 15, 2011
 * @author Duy-Nguyen Ta
 */

#pragma once

#include <vector>
#include <map>
#include <boost/shared_ptr.hpp>
#include <gtsam/inference/BayesNet.h>
#include <gtsam/discrete/DiscreteConditional.h>

namespace gtsam {

	typedef BayesNet<DiscreteConditional> DiscreteBayesNet;

	/** Add a DiscreteCondtional */
	void add(DiscreteBayesNet&, const Signature& s);

	/** Add a DiscreteCondtional in front, when listing parents first*/
	void add_front(DiscreteBayesNet&, const Signature& s);

	/** Optimize function for back-substitution. */
	DiscreteFactor::sharedValues optimize(const DiscreteBayesNet& bn);

	/** Do ancestral sampling */
	DiscreteFactor::sharedValues sample(const DiscreteBayesNet& bn);

} // namespace


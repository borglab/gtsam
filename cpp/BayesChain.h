/**
 * @file    BayesChain
 * @brief   Bayes Chain, the result of eliminating a factor graph
 * @author  Frank Dellaert
 */

// \callgraph

#pragma once

#include <list>
#include <boost/serialization/map.hpp>
#include <boost/serialization/list.hpp>

namespace gtsam {

	/**
	 * Bayes Chain, the result of eliminating a factor graph
	 * This is the base class for SymbolicBayesChain, DiscreteBayesChain, and GaussianBayesChain
	 * Corresponding to what is used for the "Conditional" template argument:
	 * a ConditionalProbabilityTable, a ConditionalGaussian, or a SymbolicConditional.
	 */
	template<class Conditional>
	class BayesChain {
	public:
	};

} /// namespace gtsam

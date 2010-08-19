/**
 * @file    SymbolicBayesNet.h
 * @brief   Symbolic Chordal Bayes Net, the result of eliminating a factor graph
 * @author  Frank Dellaert
 */

// \callgraph

#pragma once

#include <list>
#include <boost/serialization/map.hpp>
#include <boost/serialization/list.hpp>

#include <gtsam/base/Testable.h>
#include <gtsam/inference/BayesNet.h>
#include <gtsam/inference/SymbolicConditional.h>
#include <gtsam/inference/Key.h>

namespace gtsam {

	/**
	 *  Symbolic Bayes Chain, the (symbolic) result of eliminating a factor graph
	 */
	typedef BayesNet<SymbolicConditional> SymbolicBayesNet;

	/**
	 * Construct from a Bayes net of any type
	 */
	template<class Conditional>
	SymbolicBayesNet symbolic(const BayesNet<Conditional>& bn) {
		SymbolicBayesNet result;
		typename BayesNet<Conditional>::const_iterator it = bn.begin();
		for (; it != bn.end(); it++) {
			boost::shared_ptr<Conditional> conditional = *it;
			Symbol key = conditional->key();
			std::list<Symbol> parents = conditional->parents();
			SymbolicConditional::shared_ptr c(new SymbolicConditional(key, parents));
			result.push_back(c);
		}
		return result;
	}

} /// namespace gtsam

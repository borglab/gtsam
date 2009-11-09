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

#include "Testable.h"
#include "BayesNet.h"
#include "SymbolicConditional.h"

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
			std::string key = conditional->key();
			std::list<std::string> parents = conditional->parents();
			SymbolicConditional::shared_ptr c(new SymbolicConditional(key, parents));
			result.push_back(c);
		}
		return result;
	}

} /// namespace gtsam

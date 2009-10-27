/**
 * @file    SymbolicBayesChain.h
 * @brief   Symbolic Chordal Bayes Net, the result of eliminating a factor graph
 * @author  Frank Dellaert
 */

// \callgraph

#pragma once

#include <list>
#include <boost/serialization/map.hpp>
#include <boost/serialization/list.hpp>

#include "Testable.h"
#include "BayesChain.h"
#include "FactorGraph.h"

namespace gtsam {

	/**
	 * Conditional node for use in a symbolic Bayes chain
	 */
	class SymbolicConditional {
	};

	/**
	 *  Symbolic Bayes Chain, the (symbolic) result of eliminating a factor graph
	 */
	class SymbolicBayesChain: public BayesChain<SymbolicConditional> ,
			public Testable<SymbolicBayesChain> {
	public:

		/**
		 *  Construct from any factor graph
		 */
		template<class Factor, class Config>
		SymbolicBayesChain(const FactorGraph<Factor, Config>& factorGraph);

		/** Destructor */
		virtual ~SymbolicBayesChain() {
		}

		/** print */
		void print(const std::string& s = "") const;

		/** check equality */
		bool equals(const SymbolicBayesChain& other, double tol = 1e-9) const;
	};

} /// namespace gtsam

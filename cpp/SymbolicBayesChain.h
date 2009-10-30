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
#include "SymbolicConditional.h"

namespace gtsam {

	class Ordering;

	/**
	 *  Symbolic Bayes Chain, the (symbolic) result of eliminating a factor graph
	 */
	class SymbolicBayesChain: public BayesChain<SymbolicConditional> {
	public:

		/** convenience typename for a shared pointer to this class */
		typedef boost::shared_ptr<SymbolicBayesChain> shared_ptr;

		/**
		 * Empty constructor
		 */
		SymbolicBayesChain() {}

		/**
		 *  Construct from a map of nodes
		 */
		SymbolicBayesChain(const std::map<std::string,
				SymbolicConditional::shared_ptr>& nodes);

		/**
		 *  Construct from any factor graph
		 */
		template<class Factor>
		SymbolicBayesChain(const FactorGraph<Factor>& factorGraph,
				const Ordering& ordering);

		/** Destructor */
		virtual ~SymbolicBayesChain() {
		}
	};

} /// namespace gtsam

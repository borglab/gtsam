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
#include "FactorGraph.h"
#include "SymbolicConditional.h"

namespace gtsam {

	class Ordering;

	/**
	 *  Symbolic Bayes Chain, the (symbolic) result of eliminating a factor graph
	 */
	class SymbolicBayesNet: public BayesNet<SymbolicConditional> {
	public:

		/** convenience typename for a shared pointer to this class */
		typedef boost::shared_ptr<SymbolicBayesNet> shared_ptr;

		/**
		 * Empty constructor
		 */
		SymbolicBayesNet() {}

		/** Destructor */
		virtual ~SymbolicBayesNet() {
		}
	};

} /// namespace gtsam

/*
 * SymbolicFactorGraph.h
 *
 *  Created on: Oct 29, 2009
 *      Author: Frank Dellaert
 */

#ifndef SYMBOLICFACTORGRAPH_H_
#define SYMBOLICFACTORGRAPH_H_

#include <string>
#include <list>
#include "FactorGraph.h"
#include "SymbolicFactor.h"

namespace gtsam {

	class SymbolicBayesNet;

	/** Symbolic Factor Graph */
	class SymbolicFactorGraph: public FactorGraph<SymbolicFactor> {
	public:

		/**
		 * Construct empty factor graph
		 */
		SymbolicFactorGraph() {
		}

		/**
		 * Construct from a factor graph of any type
		 */
		template<class Factor>
		SymbolicFactorGraph(const FactorGraph<Factor>& fg) {
			for (size_t i = 0; i < fg.size(); i++) {
				boost::shared_ptr<Factor> f = fg[i];
				std::list<std::string> keys = f->keys();
				SymbolicFactor::shared_ptr factor(new SymbolicFactor(keys));
				push_back(factor);
			}
		}

		/**
		 * eliminate factor graph in place(!) in the given order, yielding
		 * a chordal Bayes net
		 */
		boost::shared_ptr<SymbolicBayesNet> eliminate(const Ordering& ordering);

	};

}

#endif /* SYMBOLICFACTORGRAPH_H_ */

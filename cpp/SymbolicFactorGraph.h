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
#include "SymbolicBayesNet.h"

namespace gtsam {

	class SymbolicConditional;

	/** Symbolic Factor Graph */
	class SymbolicFactorGraph: public FactorGraph<SymbolicFactor> {
	public:

		/** Construct empty factor graph */
		SymbolicFactorGraph() {}

		/** Push back unary factor */
		void push_factor(const std::string& key) {
			boost::shared_ptr<SymbolicFactor> factor(new SymbolicFactor(key));
			push_back(factor);
		}

		/** Push back binary factor */
		void push_factor(const std::string& key1, const std::string& key2) {
			boost::shared_ptr<SymbolicFactor> factor(new SymbolicFactor(key1,key2));
			push_back(factor);
		}

		/** Push back ternary factor */
		void push_factor(const std::string& key1, const std::string& key2, const std::string& key3) {
			boost::shared_ptr<SymbolicFactor> factor(new SymbolicFactor(key1,key2,key3));
			push_back(factor);
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
     * Eliminate a single node yielding a conditional Gaussian
     * Eliminates the factors from the factor graph through findAndRemoveFactors
     * and adds a new factor on the separator to the factor graph
     */
    boost::shared_ptr<SymbolicConditional> eliminateOne(const std::string& key);

		/**
		 * eliminate factor graph in place(!) in the given order, yielding
		 * a chordal Bayes net
		 */
		SymbolicBayesNet eliminate(const Ordering& ordering);

	};

}

#endif /* SYMBOLICFACTORGRAPH_H_ */

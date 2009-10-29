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

namespace gtsam {

	/** Symbolic Factor */
	class SymbolicFactor: public Testable<SymbolicFactor> {
	private:

		std::list<std::string> keys_;

	public:

		typedef boost::shared_ptr<SymbolicFactor> shared_ptr;

		/**
		 * Constructor from a list of keys
		 */
		SymbolicFactor(std::list<std::string> keys) :
			keys_(keys) {
		}

		/**
		 * Constructor that combines a set of factors
		 * @param factors Set of factors to combine
		 */
		SymbolicFactor(const std::vector<shared_ptr> & factors);

		/** print */
		void print(const std::string& s = "SymbolicFactor") const;

		/** check equality */
		bool equals(const SymbolicFactor& other, double tol = 1e-9) const;

		/**
		 * Find all variables
		 * @return The set of all variable keys
		 */
		std::list<std::string> keys() const {
			return keys_;
		}
	};

	/** Symbolic Factor Graph */
	class SymbolicFactorGraph: public FactorGraph<SymbolicFactor> {
	public:

		SymbolicFactorGraph() {
		}

		template<class Factor>
		SymbolicFactorGraph(const FactorGraph<Factor>& fg) {
			for (size_t i = 0; i < fg.size(); i++) {
				boost::shared_ptr<Factor> f = fg[i];
				std::list<std::string> keys = f->keys();
				SymbolicFactor::shared_ptr factor(new SymbolicFactor(keys));
				push_back(factor);
			}
		}

	};

}

#endif /* SYMBOLICFACTORGRAPH_H_ */

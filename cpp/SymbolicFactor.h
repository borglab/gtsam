/*
 * SymbolicFactor.h
 *
 *  Created on: Oct 29, 2009
 *      Author: Frank Dellaert
 */

#ifndef SYMBOLICFACTOR_H_
#define SYMBOLICFACTOR_H_

#include <list>
#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>
#include "Testable.h"
#include "Ordering.h"

namespace gtsam {

	class SymbolicConditional;

	/** Symbolic Factor */
	class SymbolicFactor: public Testable<SymbolicFactor> {
	private:

		std::list<std::string> keys_;

	public:

		typedef boost::shared_ptr<SymbolicFactor> shared_ptr;

		/** Construct from SymbolicConditional */
		SymbolicFactor(const boost::shared_ptr<SymbolicConditional>& c);

		/** Constructor from a list of keys */
		SymbolicFactor(const Ordering& keys) :
			keys_(keys) {
		}

		/** Construct unary factor */
		SymbolicFactor(const std::string& key) {
			keys_.push_back(key);
		}

		/** Construct binary factor */
		SymbolicFactor(const std::string& key1, const std::string& key2) {
			keys_.push_back(key1);
			keys_.push_back(key2);
		}

		/** Construct ternary factor */
		SymbolicFactor(const std::string& key1, const std::string& key2, const std::string& key3) {
			keys_.push_back(key1);
			keys_.push_back(key2);
			keys_.push_back(key3);
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

		/**
		 * eliminate one of the variables connected to this factor
		 * @param key the key of the node to be eliminated
		 * @return a new factor and a symbolic conditional on the eliminated variable
		 */
		std::pair<boost::shared_ptr<SymbolicConditional>, shared_ptr>
		eliminate(const std::string& key) const;

		/**
		 * Check if empty factor
		 */
		inline bool empty() const {
			return keys_.empty();
		}


	};

}

#endif /* SYMBOLICFACTOR_H_ */

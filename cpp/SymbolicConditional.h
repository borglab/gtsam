/**
 * @file    SymbolicConditional.h
 * @brief   Symbolic Conditional node for use in Bayes nets
 * @author  Frank Dellaert
 */

// \callgraph

#pragma once

#include "Testable.h"
#include <boost/foreach.hpp> // TODO: make cpp file

namespace gtsam {

	/**
	 * Conditional node for use in a Bayes net
	 */
	class SymbolicConditional: Testable<SymbolicConditional> {
	private:

		std::list<std::string> parents_;

	public:

		typedef boost::shared_ptr<SymbolicConditional> shared_ptr;

		/**
		 * No parents
		 */
		SymbolicConditional() {
		}

		/**
		 * Single parent
		 */
		SymbolicConditional(const std::string& parent) {
			parents_.push_back(parent);
		}

		/**
		 * Two parents
		 */
		SymbolicConditional(const std::string& parent1, const std::string& parent2) {
			parents_.push_back(parent1);
			parents_.push_back(parent2);
		}

		/**
		 * A list
		 */
		SymbolicConditional(const std::list<std::string>& parents):parents_(parents) {
		}

		/** print */
		void print(const std::string& s = "SymbolicConditional") const {
			std::cout << s << std::endl;
			BOOST_FOREACH(std::string parent, parents_) std::cout << " " << parent;
			std::cout << std::endl;
		}

		/** check equality */
		bool equals(const SymbolicConditional& other, double tol = 1e-9) const {
			return parents_ == other.parents_;
		}

	};

} /// namespace gtsam

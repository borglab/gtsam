/**
 * @file    SymbolicConditional.h
 * @brief   Symbolic Conditional node for use in Bayes nets
 * @author  Frank Dellaert
 */

// \callgraph

#pragma once

#include "Testable.h"

namespace gtsam {

	/**
	 * Conditional node for use in a Bayes nets
	 */
	class SymbolicConditional: Testable<SymbolicConditional> {
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
		SymbolicConditional(const std::string& key) {
		}

		/**
		 * Two parents
		 */
		SymbolicConditional(const std::string& key1, const std::string& key2) {
		}

		/** print */
		void print(const std::string& s = "SymbolicConditional") const {
			std::cout << s << std::endl;
		}

		/** check equality */
		bool equals(const SymbolicConditional& other, double tol = 1e-9) const {
			return false;
		}

	};

} /// namespace gtsam

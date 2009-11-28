/**
 * @file    Ordering.h
 * @brief   Ordering of indices for eliminating a factor graph
 * @author  Frank Dellaert
 */

#pragma once

#include <list>
#include <string>
#include "Testable.h"

namespace gtsam {

	/**
	 * @class Ordering
	 * @brief ordering of indices for eliminating a factor graph
	 */
	class Ordering: public std::list<std::string>, public Testable<Ordering> {
	public:
		/**
		 * Default constructor creates empty ordering
		 */
		Ordering() {
		}

		/**
		 * Create from a single string
		 */
		Ordering(std::string key) {
			push_back(key);
		}

		/**
		 * Copy constructor from string vector
		 */
		Ordering(const std::list<std::string>& strings_in) :
			std::list<std::string>(strings_in) {
		}

		/**
		 * Remove a set of keys from an ordering
		 * @param keys to remove
		 * @return a new ordering without the selected keys
		 */
		Ordering subtract(const Ordering& keys) const;

		void print(const std::string& s = "Ordering") const;

		/**
		 * check if two orderings are the same
		 * @param ordering
		 * @return bool
		 */
		bool equals(const Ordering &ord, double tol=0) const;
	};

}

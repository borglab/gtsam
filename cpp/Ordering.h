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
		 * Copy constructor from string vector
		 */
		Ordering(const std::list<std::string>& strings_in) :
			std::list<std::string>(strings_in) {
		}

		void print(const std::string& s = "Ordering") const;

		/**
		 * check if two orderings are the same
		 * @param ordering
		 * @return bool
		 */
		bool equals(const Ordering &ord, double tol=0) const;
	};

}

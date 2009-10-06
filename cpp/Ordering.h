/**
 * @file    Ordering.h
 * @brief   Ordering of indices for eliminating a factor graph
 * @author  Frank Dellaert
 */

#pragma once

#include <vector>
#include <string>

namespace gtsam {

	/**
	 * @class Ordering
	 * @brief ordering of indices for eliminating a factor graph
	 */
	class Ordering: public std::vector<std::string> {
	public:
		/**
		 * Default constructor creates empty ordering
		 */
		Ordering() {
		}

		/**
		 * Copy constructor from string vector
		 * TODO: should take reference?
		 */
		Ordering(std::vector<std::string> strings_in) :
			std::vector<std::string>(strings_in) {
		}

		void print() const;

		/**
		 * check if two orderings are the same
		 * @param ordering
		 * @return bool
		 */
		bool equals(Ordering &ord);
	};

}

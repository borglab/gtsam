/*
 * @file		Assignment.h
 * @brief		An assignment from labels to a discrete value index (size_t)
 * @author	Frank Dellaert
 * @date	  Feb 5, 2012
 */

#pragma once

#include <boost/foreach.hpp>
#include <iostream>
#include <map>

namespace gtsam {

	/**
	 * An assignment from labels to value index (size_t).
	 * Assigns to each label a value. Implemented as a simple map.
	 * A discrete factor takes an Assignment and returns a value.
	 */
	template <class L>
	class Assignment: public std::map<L, size_t> {
	public:
		void print(const std::string& s = "Assignment: ") const {
			std::cout << s << ": ";
			BOOST_FOREACH(const typename Assignment::value_type& keyValue, *this)
				std::cout << "(" << keyValue.first << ", " << keyValue.second << ")";
			std::cout << std::endl;
		}

		bool equals(const Assignment& other, double tol = 1e-9) const {
			return (*this == other);
		}
	};

} // namespace gtsam

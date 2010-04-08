/**
 * @file    Ordering.h
 * @brief   Ordering of indices for eliminating a factor graph
 * @author  Frank Dellaert
 */

#pragma once

#include <list>
#include <string>
#include "Testable.h"
#include "Key.h"

namespace gtsam {

	/**
	 * @class Ordering
	 * @brief ordering of indices for eliminating a factor graph
	 */
	class Ordering: public std::list<Symbol>, public Testable<Ordering> {
	public:
		/** Default constructor creates empty ordering */
		Ordering() { }

		/** Create from a single symbol */
		Ordering(Symbol key) { push_back(key); }

		/** Copy constructor */
		Ordering(const std::list<Symbol>& keys_in) : std::list<Symbol>(keys_in) {}

		/** whether a key exists */
		bool exists(const Symbol& key) { return std::find(begin(), end(), key) != end(); }

		// Testable
		void print(const std::string& s = "Ordering") const;
		bool equals(const Ordering &ord, double tol=0) const;

		/**
		 * Remove a set of keys from an ordering
		 * @param keys to remove
		 * @return a new ordering without the selected keys
		 */
		Ordering subtract(const Ordering& keys) const;
	};

}

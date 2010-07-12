/**
 * @file    Ordering.h
 * @brief   Ordering of indices for eliminating a factor graph
 * @author  Frank Dellaert
 */

#pragma once

#include <list>
#include <set>
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

	/**
	 * @class Unordered
	 * @brief a set of unordered indice
	 */
	class Unordered: public std::set<Symbol>, public Testable<Unordered> {
	public:
		/** Default constructor creates empty ordering */
		Unordered() { }

		/** Create from a single symbol */
		Unordered(Symbol key) { insert(key); }

		/** Copy constructor */
		Unordered(const std::set<Symbol>& keys_in) : std::set<Symbol>(keys_in) {}

		/** whether a key exists */
		bool exists(const Symbol& key) { return find(key) != end(); }

		// Testable
		void print(const std::string& s = "Unordered") const;
		bool equals(const Unordered &t, double tol=0) const;
	};
}

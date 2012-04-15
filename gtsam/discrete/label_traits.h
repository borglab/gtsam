/*
 * label_traits.h
 * @brief traits class for labels used in Decision Diagram
 * @author Frank Dellaert
 * @date Mar 22, 2011
 */

#pragma once

#include <stdexcept>
#include <boost/functional/hash.hpp>

namespace gtsam {

	/**
	 * Default traits class for label type, http://www.cantrip.org/traits.html
	 * Override to provide non-default behavior, see example in Index
	 */
	template<typename T>
	struct label_traits {
		/** default = binary label */
		static size_t cardinality(const T&) {
			return 2;
		}
		/** default higher(a,b) = a<b */
		static bool higher(const T& a, const T& b) {
			return a < b;
		}
		/** hash function */
		static size_t hash_value(const T& a) {
			boost::hash<T> hasher;
			return hasher(a);
		}
	};

	/* custom hash function for labels, no need to specialize this */
	template<typename T>
	std::size_t hash_value(const T& a) {
		return label_traits<T>::hash_value(a);
	}
}

/**
 * @file TestableAssertions.h
 * @brief Provides additional testing facilities for common data structures
 * @author Alex Cunningham
 */

#pragma once

#include <vector>
#include <boost/foreach.hpp>
#include <gtsam/base/Testable.h>

namespace gtsam {

/**
 * Version of assert_equals to work with vectors
 */
template<class V>
bool assert_equal(const std::vector<V>& expected, const std::vector<V>& actual, double tol = 1e-9) {
	if (expected.size() != actual.size()) {
		printf("Sizes not equal:\n");
		printf("expected size: %lu\n", expected.size());
		printf("actual size: %lu\n", actual.size());
		return false;
	}
	size_t i = 0;
	BOOST_FOREACH(const V& a, expected) {
		if (!assert_equal(a, expected[i++], tol))
			return false;
	}
	return true;
}

} // \namespace gtsam

/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testOrdering
 * @author Alex Cunningham
 */

#include <gtsam/nonlinear/Ordering.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/base/TestableAssertions.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST( testOrdering, simple_modifications ) {
	Ordering ordering;

	// create an ordering
	Symbol x1('x', 1), x2('x', 2), x3('x', 3), x4('x', 4);
	ordering += x1, x2, x3, x4;

	// pop the last two elements
	Ordering::value_type x4p = ordering.pop_back();
	EXPECT_LONGS_EQUAL(3, ordering.size());
	EXPECT(assert_equal(x4, x4p.first));

	Index x3p = ordering.pop_back(x3);
	EXPECT_LONGS_EQUAL(2, ordering.size());
	EXPECT_LONGS_EQUAL(2, (int)x3p);

	// try to pop an element that doesn't exist and isn't last
	CHECK_EXCEPTION(ordering.pop_back(x4), std::invalid_argument);
	CHECK_EXCEPTION(ordering.pop_back(x1), std::invalid_argument);

	// reassemble back make the ordering 1, 2, 4, 3
	EXPECT_LONGS_EQUAL(2, ordering.push_back(x4));
	EXPECT_LONGS_EQUAL(3, ordering.push_back(x3));

	EXPECT_LONGS_EQUAL(2, ordering[x4]);
	EXPECT_LONGS_EQUAL(3, ordering[x3]);

	// verify
	Ordering expectedFinal;
	expectedFinal += x1, x2, x4, x3;
	EXPECT(assert_equal(expectedFinal, ordering));
}

/* ************************************************************************* */
TEST( testOrdering, invert ) {
	// creates a map with the opposite mapping: Index->Key
	Ordering ordering;

	// create an ordering
	Symbol x1('x', 1), x2('x', 2), x3('x', 3), x4('x', 4);
	ordering += x1, x2, x3, x4;

	Ordering::InvertedMap actual = ordering.invert();
	Ordering::InvertedMap expected;
	expected.insert(make_pair(0, x1));
	expected.insert(make_pair(1, x2));
	expected.insert(make_pair(2, x3));
	expected.insert(make_pair(3, x4));

	EXPECT(assert_container_equality(expected, actual));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */

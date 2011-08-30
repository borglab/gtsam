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

#include <CppUnitLite/TestHarness.h>
#include <gtsam/nonlinear/Ordering.h>

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
	ordering.push_back(x4);
	ordering.push_back(x3);

	// verify
	Ordering expectedFinal;
	expectedFinal += x1, x2, x4, x3;
	EXPECT(assert_equal(expectedFinal, ordering));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */

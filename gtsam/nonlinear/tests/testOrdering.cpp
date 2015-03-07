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
TEST( Ordering, simple_modifications ) {
  Ordering ordering;

  // create an ordering
  Symbol x1('x', 1), x2('x', 2), x3('x', 3), x4('x', 4);
  ordering += x1, x2, x3, x4;

  EXPECT_LONGS_EQUAL(0, ordering[x1]);
  EXPECT_LONGS_EQUAL(1, ordering[x2]);
  EXPECT_LONGS_EQUAL(2, ordering[x3]);
  EXPECT_LONGS_EQUAL(3, ordering[x4]);
  EXPECT_LONGS_EQUAL(Key(x1), ordering.key(0));
  EXPECT_LONGS_EQUAL(Key(x2), ordering.key(1));
  EXPECT_LONGS_EQUAL(Key(x3), ordering.key(2));
  EXPECT_LONGS_EQUAL(Key(x4), ordering.key(3));

  // pop the last two elements
  Ordering::value_type x4p = ordering.pop_back();
  EXPECT_LONGS_EQUAL(3, ordering.size());
  EXPECT(assert_equal(x4, x4p.first));

  Index x3p = ordering.pop_back().second;
  EXPECT_LONGS_EQUAL(2, ordering.size());
  EXPECT_LONGS_EQUAL(2, (int)x3p);

  // reassemble back make the ordering 1, 2, 4, 3
  EXPECT_LONGS_EQUAL(2, ordering.push_back(x4));
  EXPECT_LONGS_EQUAL(3, ordering.push_back(x3));

  EXPECT_LONGS_EQUAL(2, ordering[x4]);
  EXPECT_LONGS_EQUAL(3, ordering[x3]);

  EXPECT_LONGS_EQUAL(Key(x4), ordering.key(2));
  EXPECT_LONGS_EQUAL(Key(x3), ordering.key(3));

  // verify
  Ordering expectedFinal;
  expectedFinal += x1, x2, x4, x3;
  EXPECT(assert_equal(expectedFinal, ordering));
}

/* ************************************************************************* */
TEST(Ordering, permute) {
  Ordering ordering;
  ordering += 2, 4, 6, 8;

  Ordering expected;
  expected += 2, 8, 6, 4;

  Permutation permutation(4);
  permutation[0] = 0;
  permutation[1] = 3;
  permutation[2] = 2;
  permutation[3] = 1;

  Ordering actual = ordering;
  actual.permuteInPlace(permutation);

  EXPECT(assert_equal(expected, actual));

  EXPECT_LONGS_EQUAL(0, actual[2]);
  EXPECT_LONGS_EQUAL(1, actual[8]);
  EXPECT_LONGS_EQUAL(2, actual[6]);
  EXPECT_LONGS_EQUAL(3, actual[4]);
  EXPECT_LONGS_EQUAL(2, actual.key(0));
  EXPECT_LONGS_EQUAL(8, actual.key(1));
  EXPECT_LONGS_EQUAL(6, actual.key(2));
  EXPECT_LONGS_EQUAL(4, actual.key(3));
}

/* ************************************************************************* */
TEST( Ordering, invert ) {
  // creates a map with the opposite mapping: Index->Key
  Ordering ordering;

  // create an ordering
  Symbol x1('x', 1), x2('x', 2), x3('x', 3), x4('x', 4);
  ordering += x1, x2, x3, x4;

  EXPECT_LONGS_EQUAL(Key(x1), ordering.key(0));
  EXPECT_LONGS_EQUAL(Key(x2), ordering.key(1));
  EXPECT_LONGS_EQUAL(Key(x3), ordering.key(2));
  EXPECT_LONGS_EQUAL(Key(x4), ordering.key(3));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */

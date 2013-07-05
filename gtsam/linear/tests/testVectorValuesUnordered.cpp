/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testVectorValues.cpp
 * @author  Richard Roberts
 * @date    Sep 16, 2010
 */

#include <boost/assign/std/vector.hpp>

#include <gtsam/base/Testable.h>
#include <gtsam/linear/VectorValuesUnordered.h>

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace boost::assign;
using namespace gtsam;

/* ************************************************************************* */
TEST(VectorValues, insert) {

  // insert
  VectorValuesUnordered actual;
  actual.insert(0, Vector_(1, 1.0));
  actual.insert(1, Vector_(2, 2.0, 3.0));
  actual.insert(5, Vector_(2, 6.0, 7.0));
  actual.insert(2, Vector_(2, 4.0, 5.0));

  // Check dimensions
  LONGS_EQUAL(6, actual.size());
  LONGS_EQUAL(1, actual.dim(0));
  LONGS_EQUAL(2, actual.dim(1));
  LONGS_EQUAL(2, actual.dim(2));
  LONGS_EQUAL(2, actual.dim(5));

  // Logic
  EXPECT(actual.exists(0));
  EXPECT(actual.exists(1));
  EXPECT(actual.exists(2));
  EXPECT(!actual.exists(3));
  EXPECT(!actual.exists(4));
  EXPECT(actual.exists(5));
  EXPECT(!actual.exists(6));

  // Check values
  EXPECT(assert_equal(Vector_(1, 1.0), actual[0]));
  EXPECT(assert_equal(Vector_(2, 2.0, 3.0), actual[1]));
  EXPECT(assert_equal(Vector_(2, 4.0, 5.0), actual[2]));
  EXPECT(assert_equal(Vector_(2, 6.0, 7.0), actual[5]));
  EXPECT(assert_equal(Vector_(7, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0), actual.asVector()));

  // Check exceptions
  CHECK_EXCEPTION(actual.insert(1, Vector()), invalid_argument);
  CHECK_EXCEPTION(actual.dim(3), out_of_range);
}

/* ************************************************************************* */
TEST(VectorValues, combine) {
  VectorValuesUnordered expected;
  expected.insert(0, Vector_(1, 1.0));
  expected.insert(1, Vector_(2, 2.0, 3.0));
  expected.insert(5, Vector_(2, 6.0, 7.0));
  expected.insert(2, Vector_(2, 4.0, 5.0));

  VectorValuesUnordered first;
  first.insert(0, Vector_(1, 1.0));
  first.insert(1, Vector_(2, 2.0, 3.0));

  VectorValuesUnordered second;
  second.insert(5, Vector_(2, 6.0, 7.0));
  second.insert(2, Vector_(2, 4.0, 5.0));

  VectorValuesUnordered actual(first, second);

  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST(VectorValues, subvector) {
  VectorValuesUnordered init;
  init.insert(0, Vector_(1, 1.0));
  init.insert(1, Vector_(2, 2.0, 3.0));
  init.insert(2, Vector_(2, 4.0, 5.0));
  init.insert(3, Vector_(2, 6.0, 7.0));

  std::vector<Key> keys;
  keys += 0, 2, 3;
  Vector expSubVector = Vector_(5, 1.0, 4.0, 5.0, 6.0, 7.0);
  EXPECT(assert_equal(expSubVector, init.vector(keys)));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */

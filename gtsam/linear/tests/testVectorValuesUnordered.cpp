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
#include <boost/assign/list_of.hpp>

#include <gtsam/base/Testable.h>
#include <gtsam/linear/VectorValues.h>

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace boost::assign;
using namespace gtsam;

/* ************************************************************************* */
TEST(VectorValues, basics)
{
  // Tests insert(), size(), dim(), exists(), vector()

  // insert
  VectorValues actual;
  actual.insert(0, Vector_(1, 1.0));
  actual.insert(1, Vector_(2, 2.0, 3.0));
  actual.insert(5, Vector_(2, 6.0, 7.0));
  actual.insert(2, Vector_(2, 4.0, 5.0));

  // Check dimensions
  LONGS_EQUAL(4, actual.size());
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
  FastVector<Key> keys = list_of(0)(1)(2)(5);
  EXPECT(assert_equal(Vector_(7, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0), actual.vector(keys)));

  // Check exceptions
  CHECK_EXCEPTION(actual.insert(1, Vector()), invalid_argument);
  CHECK_EXCEPTION(actual.dim(3), out_of_range);
}

/* ************************************************************************* */
TEST(VectorValues, combine)
{
  VectorValues expected;
  expected.insert(0, Vector_(1, 1.0));
  expected.insert(1, Vector_(2, 2.0, 3.0));
  expected.insert(5, Vector_(2, 6.0, 7.0));
  expected.insert(2, Vector_(2, 4.0, 5.0));

  VectorValues first;
  first.insert(0, Vector_(1, 1.0));
  first.insert(1, Vector_(2, 2.0, 3.0));

  VectorValues second;
  second.insert(5, Vector_(2, 6.0, 7.0));
  second.insert(2, Vector_(2, 4.0, 5.0));

  VectorValues actual(first, second);

  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST(VectorValues, subvector)
{
  VectorValues init;
  init.insert(10, Vector_(1, 1.0));
  init.insert(11, Vector_(2, 2.0, 3.0));
  init.insert(12, Vector_(2, 4.0, 5.0));
  init.insert(13, Vector_(2, 6.0, 7.0));

  std::vector<Key> keys;
  keys += 10, 12, 13;
  Vector expSubVector = Vector_(5, 1.0, 4.0, 5.0, 6.0, 7.0);
  EXPECT(assert_equal(expSubVector, init.vector(keys)));
}

/* ************************************************************************* */
TEST(VectorValues, LinearAlgebra)
{
  VectorValues test1;
  test1.insert(0, Vector_(1, 1.0));
  test1.insert(1, Vector_(2, 2.0, 3.0));
  test1.insert(5, Vector_(2, 6.0, 7.0));
  test1.insert(2, Vector_(2, 4.0, 5.0));

  VectorValues test2;
  test2.insert(0, Vector_(1, 6.0));
  test2.insert(1, Vector_(2, 1.0, 6.0));
  test2.insert(5, Vector_(2, 4.0, 3.0));
  test2.insert(2, Vector_(2, 1.0, 8.0));

  // Dot product
  double dotExpected = test1.vector().dot(test2.vector());
  double dotActual = test1.dot(test2);
  DOUBLES_EQUAL(dotExpected, dotActual, 1e-10);

  // Norm
  double normExpected = test1.vector().norm();
  double normActual = test1.norm();
  DOUBLES_EQUAL(normExpected, normActual, 1e-10);

  // Squared norm
  double sqNormExpected = test1.vector().norm();
  double sqNormActual = test1.norm();
  DOUBLES_EQUAL(sqNormExpected, sqNormActual, 1e-10);

  // Addition
  Vector sumExpected = test1.vector() + test2.vector();
  VectorValues sumActual = test1 + test2;
  EXPECT(sumActual.hasSameStructure(test1));
  EXPECT(assert_equal(sumExpected, sumActual.vector()));
  Vector sum2Expected = sumActual.vector() + test1.vector();
  VectorValues sum2Actual = sumActual;
  sum2Actual += test1;
  EXPECT(assert_equal(sum2Expected, sum2Actual.vector()));

  // Subtraction
  Vector difExpected = test1.vector() - test2.vector();
  VectorValues difActual = test1 - test2;
  EXPECT(difActual.hasSameStructure(test1));
  EXPECT(assert_equal(difExpected, difActual.vector()));

  // Scaling
  Vector scalExpected = test1.vector() * 5.0;
  VectorValues scalActual = test1;
  scalActual *= 5.0;
  EXPECT(assert_equal(scalExpected, scalActual.vector()));
  VectorValues scal2Actual = 5.0 * test1;
  EXPECT(assert_equal(scalExpected, scal2Actual.vector()));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */

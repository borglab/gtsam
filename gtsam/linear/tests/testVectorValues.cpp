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

#include <gtsam/base/Testable.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/inference/Symbol.h>

#include <CppUnitLite/TestHarness.h>

#include <sstream>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST(VectorValues, basics)
{
  // Tests insert(), size(), dim(), exists(), vector()

  // insert
  VectorValues actual;
  actual.insert(0, (Vector(1) << 1).finished());
  actual.insert(1, Vector2(2, 3));
  actual.insert(5, Vector2(6, 7));
  actual.insert(2, Vector2(4, 5));

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
  EXPECT(assert_equal((Vector(1) << 1).finished(), actual[0]));
  EXPECT(assert_equal(Vector2(2, 3), actual[1]));
  EXPECT(assert_equal(Vector2(4, 5), actual[2]));
  EXPECT(assert_equal(Vector2(6, 7), actual[5]));
  KeyVector keys {0, 1, 2, 5};
  EXPECT(assert_equal((Vector(7) << 1, 2, 3, 4, 5, 6, 7).finished(), actual.vector(keys)));

  // Check exceptions
  CHECK_EXCEPTION(actual.insert(1, Vector()), invalid_argument);
  CHECK_EXCEPTION(actual.dim(3), out_of_range);
}

/* ************************************************************************* */

static const VectorValues kExample = {{99, Vector2(2, 3)}};

// Check insert
TEST(VectorValues, Insert) {
  VectorValues actual;
  EXPECT(assert_equal(kExample, actual.insert(kExample)));
}

// Check update.
TEST(VectorValues, Update) {
  VectorValues actual(kExample);
  EXPECT(assert_equal(kExample, actual.update(kExample)));
}

/* ************************************************************************* */
TEST(VectorValues, combine)
{
  VectorValues expected;
  expected.insert(0, (Vector(1) << 1).finished());
  expected.insert(1, Vector2(2, 3));
  expected.insert(5, Vector2(6, 7));
  expected.insert(2, Vector2(4, 5));

  VectorValues first;
  first.insert(0, (Vector(1) << 1).finished());
  first.insert(1, Vector2(2, 3));

  VectorValues second;
  second.insert(5, Vector2(6, 7));
  second.insert(2, Vector2(4, 5));

  VectorValues actual(first, second);

  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST(VectorValues, subvector)
{
  VectorValues init;
  init.insert(10, (Vector(1) << 1).finished());
  init.insert(11, Vector2(2, 3));
  init.insert(12, Vector2(4, 5));
  init.insert(13, Vector2(6, 7));

  KeyVector keys {10, 12, 13};
  Vector expSubVector = (Vector(5) << 1, 4, 5, 6, 7).finished();
  EXPECT(assert_equal(expSubVector, init.vector(keys)));
}

/* ************************************************************************* */
TEST(VectorValues, LinearAlgebra)
{
  VectorValues test1;
  test1.insert(0, (Vector(1) << 1).finished());
  test1.insert(1, Vector2(2, 3));
  test1.insert(5, Vector2(6, 7));
  test1.insert(2, Vector2(4, 5));

  VectorValues test2;
  test2.insert(0, (Vector(1) << 6).finished());
  test2.insert(1, Vector2(1, 6));
  test2.insert(5, Vector2(4, 3));
  test2.insert(2, Vector2(1, 8));

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

  // Add to empty
  VectorValues sumActual3;
  sumActual3.addInPlace_(test1);
  sumActual3.addInPlace_(test2);
  EXPECT(assert_equal(sumExpected, sumActual3.vector()));

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
TEST(VectorValues, convert)
{
  Vector x(7);
  x << 1, 2, 3, 4, 5, 6, 7;

  VectorValues expected;
  expected.insert(0, (Vector(1) << 1).finished());
  expected.insert(1, Vector2(2, 3));
  expected.insert(2, Vector2(4, 5));
  expected.insert(5, Vector2(6, 7));

  std::map<Key,size_t> dims;
  dims.insert(make_pair(0,1));
  dims.insert(make_pair(1,2));
  dims.insert(make_pair(2,2));
  dims.insert(make_pair(5,2));
  VectorValues actual(x,dims);
  EXPECT(assert_equal(expected, actual));

  Scatter scatter;
  scatter.emplace_back(0,1);
  scatter.emplace_back(1,2);
  scatter.emplace_back(2,2);
  scatter.emplace_back(5,2);
  VectorValues actual2(x,scatter);
  EXPECT(assert_equal(expected, actual2));

  // Test other direction, note vector() is not guaranteed to give right result
  KeyVector keys {0, 1, 2, 5};
  EXPECT(assert_equal(x, actual.vector(keys)));

  // Test version with dims argument
  EXPECT(assert_equal(x, actual.vector(dims)));
}

/* ************************************************************************* */
TEST(VectorValues, vector_sub)
{
  VectorValues vv;
  vv.insert(0, (Vector(1) << 1).finished());
  vv.insert(1, Vector2(2, 3));
  vv.insert(2, Vector2(4, 5));
  vv.insert(5, Vector2(6, 7));
  vv.insert(7, Vector2(8, 9));

  std::map<Key,size_t> dims;
  dims.insert(make_pair(0,1));
  dims.insert(make_pair(5,2));

  Vector expected(3);
  expected << 1, 6, 7;

  // Test FastVector version
  KeyVector keys {0, 5};
  EXPECT(assert_equal(expected, vv.vector(keys)));

  // Test version with dims argument
  EXPECT(assert_equal(expected, vv.vector(dims)));
}

/* ************************************************************************* */
TEST(VectorValues, print)
{
  VectorValues vv;
  vv.insert(0, (Vector(1) << 1).finished());
  vv.insert(1, Vector2(2, 3));
  vv.insert(2, Vector2(4, 5));
  vv.insert(5, Vector2(6, 7));
  vv.insert(7, Vector2(8, 9));

  string expected =
      "  0: 1\n  1: 2 3\n  2: 4 5\n  5: 6 7\n  7: 8 9\n";
  stringstream actual;
  actual << vv;
  EXPECT(expected == actual.str());
}

/* ************************************************************************* */
// Check html representation.
TEST(VectorValues, html) {
  VectorValues vv;
  using symbol_shorthand::X;
  vv.insert(X(1), Vector2(2, 3.1));
  vv.insert(X(2), Vector2(4, 5.2));
  vv.insert(X(5), Vector2(6, 7.3));
  vv.insert(X(7), Vector2(8, 9.4));
  string expected =
      "<div>\n"
      "<table class='VectorValues'>\n"
      "  <thead>\n"
      "    <tr><th>Variable</th><th>value</th></tr>\n"
      "  </thead>\n"
      "  <tbody>\n"
      "    <tr><th>x1</th><td>  2 3.1</td></tr>\n"
      "    <tr><th>x2</th><td>  4 5.2</td></tr>\n"
      "    <tr><th>x5</th><td>  6 7.3</td></tr>\n"
      "    <tr><th>x7</th><td>  8 9.4</td></tr>\n"
      "  </tbody>\n"
      "</table>\n"
      "</div>";
  string actual = vv.html();
  EXPECT(actual == expected);
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */

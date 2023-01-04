/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testSignature
 * @brief Tests focusing on the details of Signatures to evaluate boost
 * compliance
 * @author Alex Cunningham
 * @date Sept 19th 2011
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/discrete/Signature.h>

#include <vector>

using namespace std;
using namespace gtsam;

DiscreteKey X(0, 2), Y(1, 3), Z(2, 2);

/* ************************************************************************* */
TEST(testSignature, simple_conditional) {
  Signature sig(X, {Y}, "1/1 2/3 1/4");
  CHECK(sig.table());
  Signature::Table table = *sig.table();
  vector<double> row[3]{{0.5, 0.5}, {0.4, 0.6}, {0.2, 0.8}};
  LONGS_EQUAL(3, table.size());
  CHECK(row[0] == table[0]);
  CHECK(row[1] == table[1]);
  CHECK(row[2] == table[2]);

  CHECK(sig.key() == X);

  DiscreteKeys keys = sig.discreteKeys();
  LONGS_EQUAL(2, keys.size());
  CHECK(keys[0] == X);
  CHECK(keys[1] == Y);

  DiscreteKeys parents = sig.parents();
  LONGS_EQUAL(1, parents.size());
  CHECK(parents[0] == Y);

  EXPECT_LONGS_EQUAL(6, sig.cpt().size());
}

/* ************************************************************************* */
TEST(testSignature, simple_conditional_nonparser) {
  Signature::Row row1{1, 1}, row2{2, 3}, row3{1, 4};
  Signature::Table table{row1, row2, row3};

  Signature sig(X | Y = table);
  CHECK(sig.key() == X);

  DiscreteKeys keys = sig.discreteKeys();
  LONGS_EQUAL(2, keys.size());
  CHECK(keys[0] == X);
  CHECK(keys[1] == Y);

  DiscreteKeys parents = sig.parents();
  LONGS_EQUAL(1, parents.size());
  CHECK(parents[0] == Y);

  EXPECT_LONGS_EQUAL(6, sig.cpt().size());
}

/* ************************************************************************* */
DiscreteKey A(0, 2), S(1, 2), T(2, 2), L(3, 2), B(4, 2), E(5, 2), D(7, 2);

// Make sure we can create all signatures for Asia network with constructor.
TEST(testSignature, all_examples) {
  DiscreteKey X(6, 2);
  Signature a(A, {}, "99/1");
  Signature s(S, {}, "50/50");
  Signature t(T, {A}, "99/1  95/5");
  Signature l(L, {S}, "99/1  90/10");
  Signature b(B, {S}, "70/30  40/60");
  Signature e(E, {T, L}, "F F F 1");
  Signature x(X, {E}, "95/5 2/98");
}

// Make sure we can create all signatures for Asia network with operator magic.
TEST(testSignature, all_examples_magic) {
  DiscreteKey X(6, 2);
  Signature a(A % "99/1");
  Signature s(S % "50/50");
  Signature t(T | A = "99/1  95/5");
  Signature l(L | S = "99/1  90/10");
  Signature b(B | S = "70/30  40/60");
  Signature e((E | T, L) = "F F F 1");
  Signature x(X | E = "95/5 2/98");
}

// Check example from docs.
TEST(testSignature, doxygen_example) {
  Signature::Table table{{0.9, 0.1}, {0.2, 0.8}, {0.3, 0.7}, {0.1, 0.9}};
  Signature d1(D, {E, B}, table);
  Signature d2((D | E, B) = "9/1 2/8 3/7 1/9");
  Signature d3(D, {E, B}, "9/1 2/8 3/7 1/9");
  EXPECT(*(d1.table()) == table);
  EXPECT(*(d2.table()) == table);
  EXPECT(*(d3.table()) == table);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file    testDiscreteConditional.cpp
 * @brief   unit tests for DiscreteConditional
 * @author  Duy-Nguyen Ta
 * @author  Frank dellaert
 * @date    Feb 14, 2011
 */

#include <boost/assign/std/map.hpp>
#include <boost/assign/std/vector.hpp>
#include <boost/make_shared.hpp>
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>
#include <gtsam/discrete/DecisionTreeFactor.h>
#include <gtsam/discrete/DiscreteConditional.h>
#include <gtsam/inference/Symbol.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST(DiscreteConditional, constructors) {
  DiscreteKey X(0, 2), Y(2, 3), Z(1, 2);  // watch ordering !

  DiscreteConditional expected(X | Y = "1/1 2/3 1/4");
  EXPECT_LONGS_EQUAL(0, *(expected.beginFrontals()));
  EXPECT_LONGS_EQUAL(2, *(expected.beginParents()));
  EXPECT(expected.endParents() == expected.end());
  EXPECT(expected.endFrontals() == expected.beginParents());

  DecisionTreeFactor f1(X & Y, "0.5 0.4 0.2 0.5 0.6 0.8");
  DiscreteConditional actual1(1, f1);
  EXPECT(assert_equal(expected, actual1, 1e-9));

  DecisionTreeFactor f2(
      X & Y & Z, "0.2 0.5 0.3 0.6 0.4 0.7 0.25 0.55 0.35 0.65 0.45 0.75");
  DiscreteConditional actual2(1, f2);
  EXPECT(assert_equal(f2 / *f2.sum(1), *actual2.toFactor(), 1e-9));
}

/* ************************************************************************* */
TEST(DiscreteConditional, constructors_alt_interface) {
  DiscreteKey X(0, 2), Y(2, 3), Z(1, 2);  // watch ordering !

  Signature::Table table;
  Signature::Row r1, r2, r3;
  r1 += 1.0, 1.0;
  r2 += 2.0, 3.0;
  r3 += 1.0, 4.0;
  table += r1, r2, r3;
  DiscreteConditional actual1(X, {Y}, table);
  DecisionTreeFactor f1(X & Y, "0.5 0.4 0.2 0.5 0.6 0.8");
  DiscreteConditional expected1(1, f1);
  EXPECT(assert_equal(expected1, actual1, 1e-9));

  DecisionTreeFactor f2(
      X & Y & Z, "0.2 0.5 0.3 0.6 0.4 0.7 0.25 0.55 0.35 0.65 0.45 0.75");
  DiscreteConditional actual2(1, f2);
  EXPECT(assert_equal(f2 / *f2.sum(1), *actual2.toFactor(), 1e-9));
}

/* ************************************************************************* */
TEST(DiscreteConditional, constructors2) {
  // Declare keys and ordering
  DiscreteKey C(0, 2), B(1, 2);
  DecisionTreeFactor actual(C & B, "0.8 0.75 0.2 0.25");
  Signature signature((C | B) = "4/1 3/1");
  DiscreteConditional expected(signature);
  DecisionTreeFactor::shared_ptr expectedFactor = expected.toFactor();
  EXPECT(assert_equal(*expectedFactor, actual));
}

/* ************************************************************************* */
TEST(DiscreteConditional, constructors3) {
  // Declare keys and ordering
  DiscreteKey C(0, 2), B(1, 2), A(2, 2);
  DecisionTreeFactor actual(C & B & A, "0.8 0.5 0.5 0.2 0.2 0.5 0.5 0.8");
  Signature signature((C | B, A) = "4/1 1/1 1/1 1/4");
  DiscreteConditional expected(signature);
  DecisionTreeFactor::shared_ptr expectedFactor = expected.toFactor();
  EXPECT(assert_equal(*expectedFactor, actual));
}

/* ************************************************************************* */
TEST(DiscreteConditional, Combine) {
  DiscreteKey A(0, 2), B(1, 2);
  vector<DiscreteConditional::shared_ptr> c;
  c.push_back(boost::make_shared<DiscreteConditional>(A | B = "1/2 2/1"));
  c.push_back(boost::make_shared<DiscreteConditional>(B % "1/2"));
  DecisionTreeFactor factor(A & B, "0.111111 0.444444 0.222222 0.222222");
  DiscreteConditional expected(2, factor);
  auto actual = DiscreteConditional::Combine(c.begin(), c.end());
  EXPECT(assert_equal(expected, *actual, 1e-5));
}

/* ************************************************************************* */
TEST(DiscreteConditional, likelihood) {
  DiscreteKey X(0, 2), Y(1, 3);
  DiscreteConditional conditional(X | Y = "2/8 4/6 5/5");

  auto actual0 = conditional.likelihood(0);
  DecisionTreeFactor expected0(Y, "0.2 0.4 0.5");
  EXPECT(assert_equal(expected0, *actual0, 1e-9));

  auto actual1 = conditional.likelihood(1);
  DecisionTreeFactor expected1(Y, "0.8 0.6 0.5");
  EXPECT(assert_equal(expected1, *actual1, 1e-9));
}

/* ************************************************************************* */
// Check markdown representation looks as expected, no parents.
TEST(DiscreteConditional, markdown_prior) {
  DiscreteKey A(Symbol('x', 1), 3);
  DiscreteConditional conditional(A % "1/2/2");
  string expected =
      " *P(x1)*:\n\n"
      "|x1|value|\n"
      "|:-:|:-:|\n"
      "|0|0.2|\n"
      "|1|0.4|\n"
      "|2|0.4|\n";
  string actual = conditional.markdown();
  EXPECT(actual == expected);
}

/* ************************************************************************* */
// Check markdown representation looks as expected, multivalued.
TEST(DiscreteConditional, markdown_multivalued) {
  DiscreteKey A(Symbol('a', 1), 3), B(Symbol('b', 1), 5);
  DiscreteConditional conditional(
      A | B = "2/88/10 2/20/78 33/33/34 33/33/34 95/2/3");
  string expected =
      " *P(a1|b1)*:\n\n"
      "|b1|0|1|2|\n"
      "|:-:|:-:|:-:|:-:|\n"
      "|0|0.02|0.88|0.1|\n"
      "|1|0.02|0.2|0.78|\n"
      "|2|0.33|0.33|0.34|\n"
      "|3|0.33|0.33|0.34|\n"
      "|4|0.95|0.02|0.03|\n";
  string actual = conditional.markdown();
  EXPECT(actual == expected);
}

/* ************************************************************************* */
// Check markdown representation looks as expected, two parents.
TEST(DiscreteConditional, markdown) {
  DiscreteKey A(2, 2), B(1, 2), C(0, 3);
  DiscreteConditional conditional(A, {B, C}, "0/1 1/3  1/1 3/1  0/1 1/0");
  string expected =
      " *P(A|B,C)*:\n\n"
      "|B|C|0|1|\n"
      "|:-:|:-:|:-:|:-:|\n"
      "|0|0|0|1|\n"
      "|0|1|0.25|0.75|\n"
      "|0|2|0.5|0.5|\n"
      "|1|0|0.75|0.25|\n"
      "|1|1|0|1|\n"
      "|1|2|1|0|\n";
  vector<string> names{"C", "B", "A"};
  auto formatter = [names](Key key) { return names[key]; };
  string actual = conditional.markdown(formatter);
  EXPECT(actual == expected);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

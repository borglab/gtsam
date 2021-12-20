/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file    testDecisionTreeFactor.cpp
 * @brief   unit tests for DiscreteConditional
 * @author  Duy-Nguyen Ta
 * @date Feb 14, 2011
 */

#include <boost/assign/std/map.hpp>
#include <boost/assign/std/vector.hpp>
#include <boost/make_shared.hpp>
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>
#include <gtsam/discrete/DecisionTreeFactor.h>
#include <gtsam/discrete/DiscreteConditional.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST( DiscreteConditional, constructors)
{
  DiscreteKey X(0, 2), Y(2, 3), Z(1, 2); // watch ordering !

  DiscreteConditional::shared_ptr expected1 = //
      boost::make_shared<DiscreteConditional>(X | Y = "1/1 2/3 1/4");
  EXPECT(expected1);
  EXPECT_LONGS_EQUAL(0, *(expected1->beginFrontals()));
  EXPECT_LONGS_EQUAL(2, *(expected1->beginParents()));
  EXPECT(expected1->endParents() == expected1->end());
  EXPECT(expected1->endFrontals() == expected1->beginParents());
  
  DecisionTreeFactor f1(X & Y, "0.5 0.4 0.2 0.5 0.6 0.8");
  DiscreteConditional actual1(1, f1);
  EXPECT(assert_equal(*expected1, actual1, 1e-9));

  DecisionTreeFactor f2(X & Y & Z,
      "0.2 0.5 0.3 0.6 0.4 0.7 0.25 0.55 0.35 0.65 0.45 0.75");
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
  auto actual1 = boost::make_shared<DiscreteConditional>(X | Y = table);
  EXPECT(actual1);
  DecisionTreeFactor f1(X & Y, "0.5 0.4 0.2 0.5 0.6 0.8");
  DiscreteConditional expected1(1, f1);
  EXPECT(assert_equal(expected1, *actual1, 1e-9));

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
  DiscreteConditional actual(2, factor);
  auto expected = DiscreteConditional::Combine(c.begin(), c.end());
  EXPECT(assert_equal(*expected, actual, 1e-5));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

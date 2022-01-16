/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file    testDiscretePrior.cpp
 * @brief   unit tests for DiscretePrior
 * @author  Frank dellaert
 * @date    December 2021
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/discrete/DiscretePrior.h>
#include <gtsam/discrete/Signature.h>

using namespace std;
using namespace gtsam;

static const DiscreteKey X(0, 2);

/* ************************************************************************* */
TEST(DiscretePrior, constructors) {
  DecisionTreeFactor f(X, "0.4 0.6");
  DiscretePrior expected(f);

  DiscretePrior actual(X % "2/3");
  EXPECT_LONGS_EQUAL(1, actual.nrFrontals());
  EXPECT_LONGS_EQUAL(0, actual.nrParents());
  EXPECT(assert_equal(expected, actual, 1e-9));

  const vector<double> pmf{0.4, 0.6};
  DiscretePrior actual2(X, pmf);
  EXPECT_LONGS_EQUAL(1, actual2.nrFrontals());
  EXPECT_LONGS_EQUAL(0, actual2.nrParents());
  EXPECT(assert_equal(expected, actual2, 1e-9));
}

/* ************************************************************************* */
TEST(DiscretePrior, Multiply) {
  DiscreteKey A(0, 2), B(1, 2);
  DiscreteConditional conditional(A | B = "1/2 2/1");
  DiscretePrior prior(B, "1/2");
  DiscreteConditional actual = prior * conditional;  // P(A|B) * P(B)

  EXPECT_LONGS_EQUAL(2, actual.nrFrontals());  // = P(A,B)
  DecisionTreeFactor factor(A & B, "1 4 2 2");
  DiscreteConditional expected(2, factor);
  EXPECT(assert_equal(expected, actual, 1e-5));
}

/* ************************************************************************* */
TEST(DiscretePrior, operator) {
  DiscretePrior prior(X % "2/3");
  EXPECT_DOUBLES_EQUAL(prior(0), 0.4, 1e-9);
  EXPECT_DOUBLES_EQUAL(prior(1), 0.6, 1e-9);
}

/* ************************************************************************* */
TEST(DiscretePrior, pmf) {
  DiscretePrior prior(X % "2/3");
  vector<double> expected {0.4, 0.6};
  EXPECT(prior.pmf() ==  expected);
}

/* ************************************************************************* */
TEST(DiscretePrior, sample) {
  DiscretePrior prior(X % "2/3");
  prior.sample();
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

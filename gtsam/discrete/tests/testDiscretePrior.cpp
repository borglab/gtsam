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
  DiscretePrior actual(X % "2/3");
  EXPECT_LONGS_EQUAL(1, actual.nrFrontals());
  EXPECT_LONGS_EQUAL(0, actual.nrParents());
  DecisionTreeFactor f(X, "0.4 0.6");
  DiscretePrior expected(f);
  EXPECT(assert_equal(expected, actual, 1e-9));
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

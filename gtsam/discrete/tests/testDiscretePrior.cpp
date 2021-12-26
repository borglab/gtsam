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

/* ************************************************************************* */
TEST(DiscretePrior, constructors) {
  DiscreteKey X(0, 2);
  DiscretePrior actual(X % "2/3");
  DecisionTreeFactor f(X, "0.4 0.6");
  DiscretePrior expected(f);
  EXPECT(assert_equal(expected, actual, 1e-9));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file testGncHelpers.cpp
 * @date July 10, 2023
 * @author Varun Agrawal
 * @brief Tests for Chi-squared distribution.
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/GncHelpers.h>

using namespace gtsam;

/* ************************************************************************* */
TEST(GncHelpers, ChiSqInv) {
  double barcSq = chi_squared_quantile(2, 0.99);
  EXPECT_DOUBLES_EQUAL(9.21034, barcSq, 1e-5);
}

/* ************************************************************************* */
int main() {
  srand(time(nullptr));
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

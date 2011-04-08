/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testNumericalDerivative.cpp
 * @brief   
 * @author  Richard Roberts
 * @created Apr 8, 2011
 */

#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/numericalDerivative.h>

using namespace gtsam;

/* ************************************************************************* */
double f(const LieVector& x) {
  assert(x.size() == 2);
  return sin(x(0)) + cos(x(1));
}

/* ************************************************************************* */
TEST_UNSAFE(testNumericalDerivative, numericalHessian) {
  LieVector center(2, 1.0, 1.0);

  Matrix expected = Matrix_(2,2,
      -sin(center(0)), 0.0,
      0.0, -cos(center(1)));

  Matrix actual = numericalHessian(f, center);

  EXPECT(assert_equal(expected, actual, 1e-5));
}

/* ************************************************************************* */
double f2(const LieVector& x) {
  assert(x.size() == 2);
  return sin(x(0)) * cos(x(1));
}

/* ************************************************************************* */
TEST_UNSAFE(testNumericalDerivative, numericalHessian2) {
  LieVector center(2, 0.5, 1.0);

  Matrix expected = Matrix_(2,2,
      -cos(center(1))*sin(center(0)), -sin(center(1))*cos(center(0)),
      -cos(center(0))*sin(center(1)), -sin(center(0))*cos(center(1)));

  Matrix actual = numericalHessian(f2, center);

  EXPECT(assert_equal(expected, actual, 1e-5));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */

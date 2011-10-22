/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testNumericalDerivative.cpp
 * @author  Richard Roberts
 * @date    Apr 8, 2011
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
  LieVector center = ones(2);

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
double f3(const LieVector& x1, const LieVector& x2) {
  assert(x1.size() == 1 && x2.size() == 1);
  return sin(x1(0)) * cos(x2(0));
}

/* ************************************************************************* */
TEST_UNSAFE(testNumericalDerivative, numericalHessian211) {
  LieVector center1(1, 1.0), center2(1, 5.0);

  Matrix expected11 = Matrix_(1,1,-sin(center1(0))*cos(center2(0)));
  Matrix actual11 = numericalHessian211(f3, center1, center2);
  EXPECT(assert_equal(expected11, actual11, 1e-5));

  Matrix expected12 = Matrix_(1,1,-cos(center1(0))*sin(center2(0)));
  Matrix actual12 = numericalHessian212(f3, center1, center2);
  EXPECT(assert_equal(expected12, actual12, 1e-5));

  Matrix expected22 = Matrix_(1,1,-sin(center1(0))*cos(center2(0)));
  Matrix actual22 = numericalHessian222(f3, center1, center2);
  EXPECT(assert_equal(expected22, actual22, 1e-5));
}

/* ************************************************************************* */
double f4(const LieVector& x, const LieVector& y, const LieVector& z) {
  assert(x.size() == 1 && y.size() == 1 && z.size() == 1);
  return sin(x(0)) * cos(y(0)) * z(0)*z(0);
}

/* ************************************************************************* */
TEST_UNSAFE(testNumericalDerivative, numericalHessian311) {
  LieVector center1(1, 1.0), center2(1, 2.0), center3(1, 3.0);
  double x = center1(0), y = center2(0), z = center3(0);
  Matrix expected11 = Matrix_(1,1,-sin(x)*cos(y)*z*z);
  Matrix actual11 = numericalHessian311(f4, center1, center2, center3);
  EXPECT(assert_equal(expected11, actual11, 1e-5));

  Matrix expected12 = Matrix_(1,1,-cos(x)*sin(y)*z*z);
  Matrix actual12 = numericalHessian312(f4, center1, center2, center3);
  EXPECT(assert_equal(expected12, actual12, 1e-5));

  Matrix expected13 = Matrix_(1,1,cos(x)*cos(y)*2*z);
  Matrix actual13 = numericalHessian313(f4, center1, center2, center3);
  EXPECT(assert_equal(expected13, actual13, 1e-5));

  Matrix expected22 = Matrix_(1,1,-sin(x)*cos(y)*z*z);
  Matrix actual22 = numericalHessian322(f4, center1, center2, center3);
  EXPECT(assert_equal(expected22, actual22, 1e-5));

  Matrix expected23 = Matrix_(1,1,-sin(x)*sin(y)*2*z);
  Matrix actual23 = numericalHessian323(f4, center1, center2, center3);
  EXPECT(assert_equal(expected23, actual23, 1e-5));

  Matrix expected33 = Matrix_(1,1,sin(x)*cos(y)*2);
  Matrix actual33 = numericalHessian333(f4, center1, center2, center3);
  EXPECT(assert_equal(expected33, actual33, 1e-5));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */

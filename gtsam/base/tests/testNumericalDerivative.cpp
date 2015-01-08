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

#include <gtsam/base/numericalDerivative.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

//*****************************************************************************
double f(const Vector2& x) {
  assert(x.size() == 2);
  return sin(x(0)) + cos(x(1));
}

TEST(testNumericalDerivative, NumericalGradient) {
  Vector2 x(1, 1);
  Vector expected(2);
  expected << cos(x(1)), -sin(x(0));
  EXPECT(assert_equal(expected, numericalGradient<Vector2>(f, x), 1e-5));
}

TEST(testNumericalDerivative, NumericalHessian) {
  Vector2 x(1, 1);
  Matrix2 expected;
  expected << -sin(x(0)), 0.0, 0.0, -cos(x(1));
  EXPECT(assert_equal(expected, numericalHessian<Vector2>(f, x), 1e-5));
}

//*****************************************************************************
double f2(const Vector2& x) {
  return sin(x(0)) * cos(x(1));
}

TEST(testNumericalDerivative, NumericalHessian2) {
  double x = 0.5, y = 1;
  Vector2 v(x, y);
  Matrix2 expected;
  expected << -cos(y) * sin(x), -sin(y) * cos(x), //
  -cos(x) * sin(y), -sin(x) * cos(y);
  EXPECT(assert_equal(expected, numericalHessian(f2, v), 1e-5));
}

//*****************************************************************************
// Thomas' unit test :-)
double f3(const Vector2& v) {
  const double& x = v(0), y = v(1);
  return x*x + 2*y*y - 4*x + 4*y;
}

TEST(testNumericalDerivative, NumericalHessian3) {
  Vector2 x(2, -1);
  Matrix2 expected;
  expected << 2, 0, 0, 4;
  EXPECT(assert_equal(expected, numericalHessian(f3, x), 1e-5));
}

//*****************************************************************************
double f4(double x1, double x2) {
  return sin(x1) * cos(x2);
}

TEST(testNumericalDerivative, NumericalHessian211) {
  double x1 = 1, x2 = 5;

  Matrix expected11 = (Matrix(1, 1) << -sin(x1) * cos(x2)).finished();
  Matrix actual11 = numericalHessian211<double, double>(f4, x1, x2);
  EXPECT(assert_equal(expected11, actual11, 1e-5));

  Matrix expected12 = (Matrix(1, 1) << -cos(x1) * sin(x2)).finished();
  Matrix actual12 = numericalHessian212<double, double>(f4, x1, x2);
  EXPECT(assert_equal(expected12, actual12, 1e-5));

  Matrix expected22 = (Matrix(1, 1) << -sin(x1) * cos(x2)).finished();
  Matrix actual22 = numericalHessian222<double, double>(f4, x1, x2);
  EXPECT(assert_equal(expected22, actual22, 1e-5));
}

//*****************************************************************************
TEST(testNumericalDerivative, NumericalHessian212) {
  // TODO should implement test for all the variants of numerical Hessian, for mixed dimension types,
  // like Point3 y = Project(Camera, Point3);
  // I'm not sure how numericalHessian212 is different from 211 or 222 -Mike B.
}

//*****************************************************************************
double f5(double x, double y, double z) {
  return sin(x) * cos(y) * z * z;
}

//*****************************************************************************
TEST(testNumericalDerivative, NumericalHessian311) {
  double x = 1, y = 2, z = 3;
  Matrix expected11 = (Matrix(1, 1) << -sin(x) * cos(y) * z * z).finished();
  Matrix actual11 = numericalHessian311<double, double, double>(f5, x, y, z);
  EXPECT(assert_equal(expected11, actual11, 1e-5));

  Matrix expected12 = (Matrix(1, 1) << -cos(x) * sin(y) * z * z).finished();
  Matrix actual12 = numericalHessian312<double, double, double>(f5, x, y, z);
  EXPECT(assert_equal(expected12, actual12, 1e-5));

  Matrix expected13 = (Matrix(1, 1) << cos(x) * cos(y) * 2 * z).finished();
  Matrix actual13 = numericalHessian313<double, double, double>(f5, x, y, z);
  EXPECT(assert_equal(expected13, actual13, 1e-5));

  Matrix expected22 = (Matrix(1, 1) << -sin(x) * cos(y) * z * z).finished();
  Matrix actual22 = numericalHessian322<double, double, double>(f5, x, y, z);
  EXPECT(assert_equal(expected22, actual22, 1e-5));

  Matrix expected23 = (Matrix(1, 1) << -sin(x) * sin(y) * 2 * z).finished();
  Matrix actual23 = numericalHessian323<double, double, double>(f5, x, y, z);
  EXPECT(assert_equal(expected23, actual23, 1e-5));

  Matrix expected33 = (Matrix(1, 1) << sin(x) * cos(y) * 2).finished();
  Matrix actual33 = numericalHessian333<double, double, double>(f5, x, y, z);
  EXPECT(assert_equal(expected33, actual33, 1e-5));
}

//*****************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//*****************************************************************************

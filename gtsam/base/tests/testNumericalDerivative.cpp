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

/* ************************************************************************* */
double f(const Vector2& x) {
  assert(x.size() == 2);
  return sin(x(0)) + cos(x(1));
}

/* ************************************************************************* */
//
TEST(testNumericalDerivative, numericalGradient) {
  Vector2 x(1, 1.1);

  Vector expected(2);
  expected << cos(x(0)), -sin(x(1));

  Vector actual = numericalGradient<Vector2>(f, x);

  EXPECT(assert_equal(expected, actual, 1e-5));
}

/* ************************************************************************* */
TEST(testNumericalDerivative, numericalHessian) {
  Vector2 x(1, 1.1);

  Matrix expected(2, 2);
  expected << -sin(x(0)), 0.0, 0.0, -cos(x(1));

  Matrix actual = numericalHessian<Vector2>(f, x);

  EXPECT(assert_equal(expected, actual, 1e-5));
}

/* ************************************************************************* */
double f2(const Vector2& x) {
  assert(x.size() == 2);
  return sin(x(0)) * cos(x(1));
}

/* ************************************************************************* */
//
TEST(testNumericalDerivative, numericalHessian2) {
  Vector2 v(0.5, 1.0);
  Vector2 x(v);

  Matrix expected = (Matrix(2, 2) << -cos(x(1)) * sin(x(0)), -sin(x(1))
      * cos(x(0)), -cos(x(0)) * sin(x(1)), -sin(x(0)) * cos(x(1))).finished();

  Matrix actual = numericalHessian(f2, x);

  EXPECT(assert_equal(expected, actual, 1e-5));
}

/* ************************************************************************* */
double f3(double x1, double x2) {
  return sin(x1) * cos(x2);
}

/* ************************************************************************* */
//
TEST(testNumericalDerivative, numericalHessian211) {
  double x1 = 1, x2 = 5;

  Matrix expected11 = (Matrix(1, 1) << -sin(x1) * cos(x2)).finished();
  Matrix actual11 = numericalHessian211<double, double>(f3, x1, x2);
  EXPECT(assert_equal(expected11, actual11, 1e-5));

  Matrix expected12 = (Matrix(1, 1) << -cos(x1) * sin(x2)).finished();
  Matrix actual12 = numericalHessian212<double, double>(f3, x1, x2);
  EXPECT(assert_equal(expected12, actual12, 1e-5));

  Matrix expected22 = (Matrix(1, 1) << -sin(x1) * cos(x2)).finished();
  Matrix actual22 = numericalHessian222<double, double>(f3, x1, x2);
  EXPECT(assert_equal(expected22, actual22, 1e-5));
}

TEST(testNumericalDerivative, numericalHessian212) {
  // TODO should implement test for all the variants of numerical Hessian, for mixed dimension types,
  // like Point3 y = Project(Camera, Point3);
  // I'm not sure how numericalHessian212 is different from 211 or 222 -Mike B.
}

/* ************************************************************************* */
double f4(double x, double y, double z) {
  return sin(x) * cos(y) * z * z;
}

/* ************************************************************************* */
//
TEST(testNumericalDerivative, numericalHessian311) {
  double x = 1, y = 2, z = 3;
  Matrix expected11 = (Matrix(1, 1) << -sin(x) * cos(y) * z * z).finished();
  Matrix actual11 = numericalHessian311<double, double, double>(f4, x, y, z);
  EXPECT(assert_equal(expected11, actual11, 1e-5));

  Matrix expected12 = (Matrix(1, 1) << -cos(x) * sin(y) * z * z).finished();
  Matrix actual12 = numericalHessian312<double, double, double>(f4, x, y, z);
  EXPECT(assert_equal(expected12, actual12, 1e-5));

  Matrix expected13 = (Matrix(1, 1) << cos(x) * cos(y) * 2 * z).finished();
  Matrix actual13 = numericalHessian313<double, double, double>(f4, x, y, z);
  EXPECT(assert_equal(expected13, actual13, 1e-5));

  Matrix expected22 = (Matrix(1, 1) << -sin(x) * cos(y) * z * z).finished();
  Matrix actual22 = numericalHessian322<double, double, double>(f4, x, y, z);
  EXPECT(assert_equal(expected22, actual22, 1e-5));

  Matrix expected23 = (Matrix(1, 1) << -sin(x) * sin(y) * 2 * z).finished();
  Matrix actual23 = numericalHessian323<double, double, double>(f4, x, y, z);
  EXPECT(assert_equal(expected23, actual23, 1e-5));

  Matrix expected33 = (Matrix(1, 1) << sin(x) * cos(y) * 2).finished();
  Matrix actual33 = numericalHessian333<double, double, double>(f4, x, y, z);
  EXPECT(assert_equal(expected33, actual33, 1e-5));
}

/* ************************************************************************* */
Vector6 f6(const double x1, const double x2, const double x3, const double x4,
           const double x5, const double x6) {
  Vector6 result;
  result << sin(x1), cos(x2), x3 * x3, x4 * x4 * x4, sqrt(x5), sin(x6) - cos(x6); 
  return result;
}

Vector g6(const double x1, const double x2, const double x3, const double x4,
          const double x5, const double x6) {
  Vector result(6);
  result << sin(x1), cos(x2), x3 * x3, x4 * x4 * x4, sqrt(x5), sin(x6) - cos(x6);
  return result;
}

/* ************************************************************************* */
//
TEST(testNumericalDerivative, numeriDerivative61) {
  double x1 = 1, x2 = 2, x3 = 3 , x4 = 4, x5 = 5, x6 = 6;
  
  Matrix expected61 = (Matrix(6, 1) << cos(x1), 0, 0, 0, 0, 0).finished();
  Matrix61 actual61 = numericalDerivative61<Vector6, double, double,
      double, double, double, double>(f6, x1, x2, x3, x4, x5, x6);
  
  EXPECT(assert_equal(expected61, actual61, 1e-5));

  Matrix expected61Dynamic = Matrix::Zero(6, 1);
  expected61Dynamic(0, 0) = cos(x1);
  Matrix actual61Dynamic =
      numericalDerivative61<Vector, double, double, double, double, double,
                            double, 1>(g6, x1, x2, x3, x4, x5, x6);

  EXPECT(assert_equal(expected61Dynamic, actual61Dynamic, 1e-5));
}

/* ************************************************************************* */
//
TEST(testNumericalDerivative, numeriDerivative62) {
  double x1 = 1, x2 = 2, x3 = 3 , x4 = 4, x5 = 5, x6 = 6;
  
  Matrix expected62 = (Matrix(6, 1) << 0, -sin(x2), 0, 0, 0, 0).finished();
  Matrix61 actual62 = numericalDerivative62<Vector6, double, double, double,
     double, double, double>(f6, x1, x2, x3, x4, x5, x6);
  
  EXPECT(assert_equal(expected62, actual62, 1e-5));

  Matrix expected62Dynamic = Matrix::Zero(6, 1);
  expected62Dynamic(1, 0) = -sin(x2);
  Matrix61 actual62Dynamic = numericalDerivative62<Vector, double, double,
      double, double, double, double, 1>(f6, x1, x2, x3, x4, x5, x6);

  EXPECT(assert_equal(expected62Dynamic, actual62Dynamic, 1e-5));
}

/* ************************************************************************* */
//
TEST(testNumericalDerivative, numeriDerivative63) {
  double x1 = 1, x2 = 2, x3 = 3 , x4 = 4, x5 = 5, x6 = 6;
  
  Matrix expected63 = (Matrix(6, 1) << 0, 0, 2 * x3, 0, 0, 0).finished();
  Matrix61 actual63 = numericalDerivative63<Vector6, double, double, double,
     double, double, double>(f6, x1, x2, x3, x4, x5, x6);
  
  EXPECT(assert_equal(expected63, actual63, 1e-5));

  Matrix expected63Dynamic = Matrix::Zero(6, 1);
  expected63Dynamic(2, 0) = 2 * x3;
  Matrix61 actual63Dynamic =
      numericalDerivative63<Vector, double, double, double, double, double,
                            double, 1>(f6, x1, x2, x3, x4, x5, x6);

  EXPECT(assert_equal(expected63Dynamic, actual63Dynamic, 1e-5));
}

/* ************************************************************************* */
//
TEST(testNumericalDerivative, numeriDerivative64) {
  double x1 = 1, x2 = 2, x3 = 3 , x4 = 4, x5 = 5, x6 = 6;
  
  Matrix expected64 = (Matrix(6, 1) << 0, 0, 0, 3 * x4 * x4, 0, 0).finished();
  Matrix61 actual64 = numericalDerivative64<Vector6, double, double, double,
     double, double, double>(f6, x1, x2, x3, x4, x5, x6);
  
  EXPECT(assert_equal(expected64, actual64, 1e-5));

  Matrix expected64Dynamic = Matrix::Zero(6, 1);
  expected64Dynamic(3, 0) = 3 * x4 * x4;
  Matrix61 actual64Dynamic =
      numericalDerivative64<Vector, double, double, double, double, double,
                            double, 1>(f6, x1, x2, x3, x4, x5, x6);

  EXPECT(assert_equal(expected64Dynamic, actual64Dynamic, 1e-5));
}

/* ************************************************************************* */
//
TEST(testNumericalDerivative, numeriDerivative65) {
  double x1 = 1, x2 = 2, x3 = 3 , x4 = 4, x5 = 5, x6 = 6;
  
  Matrix expected65 = (Matrix(6, 1) << 0, 0, 0, 0, 0.5 / sqrt(x5), 0).finished();
  Matrix61 actual65 = numericalDerivative65<Vector6, double, double, double,
     double, double, double>(f6, x1, x2, x3, x4, x5, x6);
  
  EXPECT(assert_equal(expected65, actual65, 1e-5));

  Matrix expected65Dynamic = Matrix::Zero(6, 1);
  expected65Dynamic(4, 0) = 0.5 / sqrt(x5);
  Matrix61 actual65Dynamic =
      numericalDerivative65<Vector, double, double, double, double, double,
                            double, 1>(f6, x1, x2, x3, x4, x5, x6);

  EXPECT(assert_equal(expected65Dynamic, actual65Dynamic, 1e-5));
}

/* ************************************************************************* */
//
TEST(testNumericalDerivative, numeriDerivative66) {
  double x1 = 1, x2 = 2, x3 = 3 , x4 = 4, x5 = 5, x6 = 6;
  
  Matrix expected66 = (Matrix(6, 1) << 0, 0, 0, 0, 0, cos(x6) + sin(x6)).finished();
  Matrix61 actual66 = numericalDerivative66<Vector6, double, double, double, 
      double, double, double>(f6, x1, x2, x3, x4, x5, x6);
  
  EXPECT(assert_equal(expected66, actual66, 1e-5));

  Matrix expected66Dynamic = Matrix::Zero(6, 1);
  expected66Dynamic(5, 0) = cos(x6) + sin(x6);
  Matrix61 actual66Dynamic =
      numericalDerivative66<Vector, double, double, double, double, double,
                            double, 1>(f6, x1, x2, x3, x4, x5, x6);

  EXPECT(assert_equal(expected66Dynamic, actual66Dynamic, 1e-5));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

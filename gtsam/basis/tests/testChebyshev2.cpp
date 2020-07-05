/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testChebyshev.cpp
 * @date July 4, 2020
 * @author Varun Agrawal
 * @brief Unit tests for Chebyshev Basis Decompositions
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/factorTesting.h>

#include "../Chebyshev2.h"
#include "../FitBasis.h"

using namespace std;
using namespace gtsam;

noiseModel::Diagonal::shared_ptr model = noiseModel::Unit::Create(1);

//******************************************************************************
TEST(Chebyshev2, Point) {
  static const int N = 5;
  auto points = Chebyshev2::Points(N);
  Vector expected(N);
  expected << -1., -sqrt(2.) / 2., 0., sqrt(2.) / 2., 1.;
  static const double tol = 1e-15;  // changing this reveals errors
  EXPECT_DOUBLES_EQUAL(expected(0), points(0), tol);
  EXPECT_DOUBLES_EQUAL(expected(1), points(1), tol);
  EXPECT_DOUBLES_EQUAL(expected(2), points(2), tol);
  EXPECT_DOUBLES_EQUAL(expected(3), points(3), tol);
  EXPECT_DOUBLES_EQUAL(expected(4), points(4), tol);

  // Check symmetry
  EXPECT_DOUBLES_EQUAL(Chebyshev2::Point(N, 0), -Chebyshev2::Point(N, 4), tol);
  EXPECT_DOUBLES_EQUAL(Chebyshev2::Point(N, 1), -Chebyshev2::Point(N, 3), tol);
}

//******************************************************************************
TEST(Chebyshev2, PointInInterval) {
  static const int N = 5;
  auto points = Chebyshev2::Points(N, 0, 20);
  Vector expected(N);
  expected << 0., 1. - sqrt(2.) / 2., 1., 1. + sqrt(2.) / 2., 2.;
  expected *= 10.0;
  static const double tol = 1e-15;  // changing this reveals errors
  EXPECT_DOUBLES_EQUAL(expected(0), points(0), tol);
  EXPECT_DOUBLES_EQUAL(expected(1), points(1), tol);
  EXPECT_DOUBLES_EQUAL(expected(2), points(2), tol);
  EXPECT_DOUBLES_EQUAL(expected(3), points(3), tol);
  EXPECT_DOUBLES_EQUAL(expected(4), points(4), tol);

  // all at once
  Vector actual = Chebyshev2::Points(N, 0, 20);
  CHECK(assert_equal(expected, actual));
}

//******************************************************************************
// InterpolatingPolynomial[{{-1, 4}, {0, 2}, {1, 6}}, 0.5]
TEST(Chebyshev2, Interpolate2) {
  size_t N = 3;
  Chebyshev2::EvaluationFunctor fx(N, 0.5);
  Vector f(N);
  f << 4, 2, 6;
  EXPECT_DOUBLES_EQUAL(3.25, fx(f), 1e-9);
}

//******************************************************************************
// InterpolatingPolynomial[{{0, 4}, {1, 2}, {2, 6}}, 1.5]
TEST(Chebyshev2, Interpolate2_Interval) {
  Chebyshev2::EvaluationFunctor fx(3, 1.5, 0, 2);
  Vector3 f(4, 2, 6);
  EXPECT_DOUBLES_EQUAL(3.25, fx(f), 1e-9);
}

//******************************************************************************
// InterpolatingPolynomial[{{-1, 4}, {-Sqrt[2]/2, 2}, {0, 6}, {Sqrt[2]/2,3}, {1,
// 3}}, 0.5]
TEST(Chebyshev2, Interpolate5) {
  Chebyshev2::EvaluationFunctor fx(5, 0.5);
  Vector f(5);
  f << 4, 2, 6, 3, 3;
  EXPECT_DOUBLES_EQUAL(4.34283, fx(f), 1e-5);
}

//******************************************************************************
// Interpolating vectors
TEST(Chebyshev2, InterpolateVector) {
  double t = 30, a = 0, b = 100;
  const size_t N = 3;
  // Create 2x3 matrix with Vectors at Chebyshev points
  Matrix X = Matrix::Zero(2, 3);
  X.row(0) = Chebyshev2::Points(N, a, b);  // slope 1 ramp

  // Check value
  Vector expected(2);
  expected << t, 0;
  Eigen::Matrix<double, /*2x2N*/ -1, -1> actualH(2, 2 * N);
  Chebyshev2::VectorEvaluationFunctor<2> fx(N, t, a, b);
  EXPECT(assert_equal(expected, fx(X, actualH), 1e-9));

  // Check derivative
  boost::function<Vector2(Matrix23)> f = boost::bind(
      &Chebyshev2::VectorEvaluationFunctor<2>::operator(), fx, _1, boost::none);
  Matrix numericalH = numericalDerivative11<Vector2, Matrix23>(f, X);
  EXPECT(assert_equal(numericalH, actualH, 1e-9));
}

//******************************************************************************
TEST(Chebyshev2, Decomposition) {
  // Create example sequence
  Sequence sequence;
  for (size_t i = 0; i < 16; i++) {
    double x = (double)i / 16. - 0.99, y = x;
    sequence[x] = y;
  }

  // Do Chebyshev Decomposition
  FitBasis<Chebyshev2> actual(3, sequence, model);

  // Check
  Vector expected(3);
  expected << -1, 0, 1;
  EXPECT(assert_equal(expected, actual.parameters(), 1e-4));
}

//******************************************************************************
TEST(Chebyshev2, DifferentiationMatrix3) {
  // Trefethen00book, p.55
  const size_t N = 3;
  Matrix expected(N, N);
  // Differentiation matrix computed from Chebfun
  expected << 1.5000, -2.0000, 0.5000,  //
      0.5000, -0.0000, -0.5000,         //
      -0.5000, 2.0000, -1.5000;
  // multiply by -1 since the cheb points have a phase shift wrt Trefethen
  // This was verified with chebfun
  expected = -expected;

  Matrix actual = Chebyshev2::DifferentiationMatrix(N);
  EXPECT(assert_equal(expected, actual, 1e-4));
}

//******************************************************************************
TEST(Chebyshev2, DerivativeMatrix6) {
  // Trefethen00book, p.55
  const size_t N = 6;
  Matrix expected(N, N);
  expected << 8.5000, -10.4721, 2.8944, -1.5279, 1.1056, -0.5000,  //
      2.6180, -1.1708, -2.0000, 0.8944, -0.6180, 0.2764,           //
      -0.7236, 2.0000, -0.1708, -1.6180, 0.8944, -0.3820,          //
      0.3820, -0.8944, 1.6180, 0.1708, -2.0000, 0.7236,            //
      -0.2764, 0.6180, -0.8944, 2.0000, 1.1708, -2.6180,           //
      0.5000, -1.1056, 1.5279, -2.8944, 10.4721, -8.5000;
  // multiply by -1 since the cheb points have a phase shift wrt Trefethen
  // This was verified with chebfun
  expected = -expected;

  Matrix actual = Chebyshev2::DifferentiationMatrix(N);
  EXPECT(assert_equal((Matrix)expected, actual, 1e-4));
}

// test function for CalculateWeights and DerivativeWeights
double f(double x) {
  // return 3*(x**3) - 2*(x**2) + 5*x - 11
  return 3.0 * pow(x, 3) - 2.0 * pow(x, 2) + 5.0 * x - 11;
}

// its derivative
double fprime(double x) {
  // return 9*(x**2) - 4*(x) + 5
  return 9.0 * pow(x, 2) - 4.0 * x + 5.0;
}

//******************************************************************************
TEST(Chebyshev2, CalculateWeights) {
  const size_t N = 32;
  Eigen::Matrix<double, -1, 1> fvals(N);
  for (size_t i = 0; i < N; i++) {
    fvals(i) = f(Chebyshev2::Point(N, i));
  }
  double x1 = 0.7, x2 = -0.376;
  Chebyshev2::Weights weights1 = Chebyshev2::CalculateWeights(N, x1);
  Chebyshev2::Weights weights2 = Chebyshev2::CalculateWeights(N, x2);
  EXPECT_DOUBLES_EQUAL(f(x1), weights1 * fvals, 1e-8);
  EXPECT_DOUBLES_EQUAL(f(x2), weights2 * fvals, 1e-8);
}

TEST(Chebyshev2, CalculateWeights2) {
  const size_t N = 32;
  double a = 0, b = 10, x1 = 7, x2 = 4.12;

  Eigen::Matrix<double, -1, 1> fvals(N);
  for (size_t i = 0; i < N; i++) {
    fvals(i) = f(Chebyshev2::Point(N, i, a, b));
  }

  Chebyshev2::Weights weights1 = Chebyshev2::CalculateWeights(N, x1, a, b);
  EXPECT_DOUBLES_EQUAL(f(x1), weights1 * fvals, 1e-8);

  Chebyshev2::Weights weights2 = Chebyshev2::CalculateWeights(N, x2, a, b);
  double expected2 = 185.454784;
  double actual2 = weights2 * fvals;
  EXPECT_DOUBLES_EQUAL(expected2, actual2, 1e-8);
}

TEST(Chebyshev2, DerivativeWeights) {
  const size_t N = 32;

  Eigen::Matrix<double, -1, 1> fvals(N);
  for (size_t i = 0; i < N; i++) {
    fvals(i) = f(Chebyshev2::Point(N, i));
  }
  double x1 = 0.7, x2 = -0.376;
  Chebyshev2::Weights dWeights1 = Chebyshev2::DerivativeWeights(N, x1);
  EXPECT_DOUBLES_EQUAL(fprime(x1), dWeights1 * fvals, 1e-8);

  Chebyshev2::Weights dWeights2 = Chebyshev2::DerivativeWeights(N, x2);
  EXPECT_DOUBLES_EQUAL(fprime(x2), dWeights2 * fvals, 1e-8);

  // test if derivative calculation and cheb point is correct
  double x3 = Chebyshev2::Point(N, 3);
  Chebyshev2::Weights dWeights3 = Chebyshev2::DerivativeWeights(N, x3);
  EXPECT_DOUBLES_EQUAL(fprime(x3), dWeights3 * fvals, 1e-8);
}

TEST(Chebyshev2, DerivativeWeights2) {
  const size_t N = 32;
  double x1 = 5, x2 = 4.12, a = 0, b = 10;

  Eigen::Matrix<double, -1, 1> fvals(N);
  for (size_t i = 0; i < N; i++) {
    fvals(i) = f(Chebyshev2::Point(N, i, a, b));
  }

  Chebyshev2::Weights dWeights1 = Chebyshev2::DerivativeWeights(N, x1, a, b);
  EXPECT_DOUBLES_EQUAL(fprime(x1), dWeights1 * fvals, 1e-8);

  Chebyshev2::Weights dWeights2 = Chebyshev2::DerivativeWeights(N, x2, a, b);
  EXPECT_DOUBLES_EQUAL(fprime(x2), dWeights2 * fvals, 1e-8);

  // test if derivative calculation and cheb point is correct
  double x3 = Chebyshev2::Point(N, 3, a, b);
  Chebyshev2::Weights dWeights3 = Chebyshev2::DerivativeWeights(N, x3, a, b);
  EXPECT_DOUBLES_EQUAL(fprime(x3), dWeights3 * fvals, 1e-8);
}

//******************************************************************************
// Check two different ways to calculate the derivative weights
TEST(Chebyshev2, DerivativeWeightsDifferentiationMatrix) {
  const size_t N6 = 6;
  double x1 = 0.311;
  Matrix D6 = Chebyshev2::DifferentiationMatrix(N6);
  Chebyshev2::Weights expected = Chebyshev2::CalculateWeights(N6, x1) * D6;
  Chebyshev2::Weights actual = Chebyshev2::DerivativeWeights(N6, x1);
  EXPECT(assert_equal(expected, actual, 1e-4));

  double a = -3, b = 8, x2 = 5.05;
  Matrix D6_2 = Chebyshev2::DifferentiationMatrix(N6, a, b);
  Chebyshev2::Weights expected1 =
      Chebyshev2::CalculateWeights(N6, x2, a, b) * D6_2;
  Chebyshev2::Weights actual1 = Chebyshev2::DerivativeWeights(N6, x2, a, b);
  EXPECT(assert_equal(expected1, actual1, 1e-4));
}

//******************************************************************************
// Check two different ways to calculate the derivative weights
TEST(Chebyshev2, DerivativeWeights6) {
  const size_t N6 = 6;
  Matrix D6 = Chebyshev2::DifferentiationMatrix(N6);
  Chebyshev2::Parameters x = Chebyshev2::Points(N6);  // ramp with slope 1
  EXPECT(assert_equal(Vector::Ones(N6), Vector(D6 * x)));
}

//******************************************************************************
// Check two different ways to calculate the derivative weights
TEST(Chebyshev2, DerivativeWeights7) {
  const size_t N7 = 7;
  Matrix D7 = Chebyshev2::DifferentiationMatrix(N7);
  Chebyshev2::Parameters x = Chebyshev2::Points(N7);  // ramp with slope 1
  EXPECT(assert_equal(Vector::Ones(N7), Vector(D7 * x)));
}

//******************************************************************************
// Check derivative in two different ways: numerical and using D on f
Vector6 f3_at_6points = (Vector6() << 4, 2, 6, 2, 4, 3).finished();
double proxy3(double x) {
  return Chebyshev2::EvaluationFunctor(6, x)(f3_at_6points);
}

TEST(Chebyshev2, Derivative6) {
  // Check Derivative evaluation at point x=0.2

  // calculate expected values by numerical derivative of synthesis
  const double x = 0.2;
  Matrix numeric_dTdx = numericalDerivative11<double, double>(proxy3, x);

  // Calculate derivatives at Chebyshev points using D3, interpolate
  Matrix D6 = Chebyshev2::DifferentiationMatrix(6);
  Vector derivative_at_points = D6 * f3_at_6points;
  Chebyshev2::EvaluationFunctor fx(6, x);
  EXPECT_DOUBLES_EQUAL(numeric_dTdx(0, 0), fx(derivative_at_points), 1e-8);

  // Do directly
  Chebyshev2::DerivativeFunctor dfdx(6, x);
  EXPECT_DOUBLES_EQUAL(numeric_dTdx(0, 0), dfdx(f3_at_6points), 1e-8);
}

//******************************************************************************
// Assert that derivative also works in non-standard interval [0,3]
double proxy4(double x) {
  return Chebyshev2::EvaluationFunctor(6, x, 0, 3)(f3_at_6points);
}

TEST(Chebyshev2, Derivative6_03) {
  // Check Derivative evaluation at point x=0.2, in interval [0,3]

  // calculate expected values by numerical derivative of synthesis
  const double x = 0.2;
  Matrix numeric_dTdx = numericalDerivative11<double, double>(proxy4, x);

  // Calculate derivatives at Chebyshev points using D3, interpolate
  Matrix D6 = Chebyshev2::DifferentiationMatrix(6, 0, 3);
  Vector derivative_at_points = D6 * f3_at_6points;
  Chebyshev2::EvaluationFunctor fx(6, x, 0, 3);
  EXPECT_DOUBLES_EQUAL(numeric_dTdx(0, 0), fx(derivative_at_points), 1e-8);

  // Do directly
  Chebyshev2::DerivativeFunctor dfdx(6, x, 0, 3);
  EXPECT_DOUBLES_EQUAL(numeric_dTdx(0, 0), dfdx(f3_at_6points), 1e-8);
}

//******************************************************************************
// Test VectorDerivativeFunctor
TEST(Chebyshev2, VectorDerivativeFunctor) {
  const size_t N = 3, M = 2;
  const double x = 0.2;
  typedef Chebyshev2::VectorDerivativeFunctor<M> VecD;
  VecD fx(N, x, 0, 3);
  Matrix X = Matrix::Zero(M, N);
  Matrix actualH(M, M * N);
  EXPECT(assert_equal(Vector::Zero(M), (Vector)fx(X, actualH), 1e-8));
  Matrix expectedH = numericalDerivative11<Vector2, Matrix23>(
      boost::bind(&VecD::operator(), fx, _1, boost::none), X);
  EXPECT(assert_equal(expectedH, actualH, 1e-7));
}

//******************************************************************************
// Test ComponentDerivativeFunctor
TEST(Chebyshev2, ComponentDerivativeFunctor) {
  const size_t N = 7, M = 3;
  const double x = 0.2;
  typedef Chebyshev2::ComponentDerivativeFunctor<M> CompFunc;
  size_t row = 1;
  CompFunc fx(N, row, x, 0, 3);
  Matrix X = Matrix::Zero(M, 7);
  Matrix actualH(1, M * N);
  EXPECT_DOUBLES_EQUAL(0, fx(X, actualH), 1e-8);
  Matrix expectedH = numericalDerivative11<double, Matrix37>(
      boost::bind(&CompFunc::operator(), fx, _1, boost::none), X);
  EXPECT(assert_equal(expectedH, actualH, 1e-7));
}

//******************************************************************************
TEST(Chebyshev2, IntegralWeights) {
  const size_t N7 = 7;
  Vector actual = Chebyshev2::IntegrationWeights(N7);
  Vector expected = (Vector(N7) << 0.0285714285714286, 0.253968253968254,
                     0.457142857142857, 0.520634920634921, 0.457142857142857,
                     0.253968253968254, 0.0285714285714286)
                        .finished();
  EXPECT(assert_equal(expected, actual));

  const size_t N8 = 8;
  Vector actual2 = Chebyshev2::IntegrationWeights(N8);
  Vector expected2 = (Vector(N8) << 0.0204081632653061, 0.190141007218208,
                      0.352242423718159, 0.437208405798326, 0.437208405798326,
                      0.352242423718159, 0.190141007218208, 0.0204081632653061)
                         .finished();
  EXPECT(assert_equal(expected2, actual2));
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************

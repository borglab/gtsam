/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------1-------------------------------------------
 */

/**
 * @file testFunctorizedFactor.cpp
 * @date May 31, 2020
 * @author Varun Agrawal
 * @brief unit tests for FunctorizedFactor class
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/FunctorizedFactor.h>
#include <gtsam/nonlinear/factorTesting.h>

using namespace std;
using namespace gtsam;

Key key = Symbol('X', 0);
auto model = noiseModel::Isotropic::Sigma(9, 1);

/// Functor that takes a matrix and multiplies every element by m
class MultiplyFunctor {
  double m_;  ///< simple multiplier

 public:
  MultiplyFunctor(double m) : m_(m) {}

  Matrix operator()(const Matrix &X,
                    OptionalJacobian<-1, -1> H = boost::none) const {
    if (H) *H = m_ * Matrix::Identity(X.rows() * X.cols(), X.rows() * X.cols());
    return m_ * X;
  }
};

/* ************************************************************************* */
// Test identity operation for FunctorizedFactor.
TEST(FunctorizedFactor, Identity) {
  Matrix X = Matrix::Identity(3, 3), measurement = Matrix::Identity(3, 3);

  double multiplier = 1.0;
  auto functor = MultiplyFunctor(multiplier);
  auto factor = MakeFunctorizedFactor<Matrix>(key, measurement, model, functor);

  Vector error = factor.evaluateError(X);

  EXPECT(assert_equal(Vector::Zero(9), error, 1e-9));
}

/* ************************************************************************* */
// Test FunctorizedFactor with multiplier value of 2.
TEST(FunctorizedFactor, Multiply2) {
  double multiplier = 2.0;
  Matrix X = Matrix::Identity(3, 3);
  Matrix measurement = multiplier * Matrix::Identity(3, 3);

  auto factor = MakeFunctorizedFactor<Matrix>(key, measurement, model,
                                              MultiplyFunctor(multiplier));

  Vector error = factor.evaluateError(X);

  EXPECT(assert_equal(Vector::Zero(9), error, 1e-9));
}

/* ************************************************************************* */
// Test equality function for FunctorizedFactor.
TEST(FunctorizedFactor, Equality) {
  Matrix measurement = Matrix::Identity(2, 2);

  double multiplier = 2.0;

  auto factor1 = MakeFunctorizedFactor<Matrix>(key, measurement, model,
                                               MultiplyFunctor(multiplier));
  auto factor2 = MakeFunctorizedFactor<Matrix>(key, measurement, model,
                                               MultiplyFunctor(multiplier));

  EXPECT(factor1.equals(factor2));
}

/* *************************************************************************** */
// Test Jacobians of FunctorizedFactor.
TEST(FunctorizedFactor, Jacobians) {
  Matrix X = Matrix::Identity(3, 3);
  Matrix actualH;

  double multiplier = 2.0;

  auto factor =
      MakeFunctorizedFactor<Matrix>(key, X, model, MultiplyFunctor(multiplier));

  Values values;
  values.insert<Matrix>(key, X);

  // Check Jacobians
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

/* ************************************************************************* */
// Test print result of FunctorizedFactor.
TEST(FunctorizedFactor, Print) {
  Matrix X = Matrix::Identity(2, 2);

  double multiplier = 2.0;

  auto factor =
      MakeFunctorizedFactor<Matrix>(key, X, model, MultiplyFunctor(multiplier));

  // redirect output to buffer so we can compare
  stringstream buffer;
  streambuf *old = cout.rdbuf(buffer.rdbuf());

  factor.print();

  // get output string and reset stdout
  string actual = buffer.str();
  cout.rdbuf(old);

  string expected =
      "  keys = { X0 }\n"
      "  noise model: unit (9) \n"
      "FunctorizedFactor(X0)\n"
      "  measurement: [\n"
      "	1, 0;\n"
      " 	0, 1\n"
      "]\n"
      "  noise model sigmas: 1 1 1 1 1 1 1 1 1\n";

  CHECK_EQUAL(expected, actual);
}

/* ************************************************************************* */
// Test FunctorizedFactor using a std::function type.
TEST(FunctorizedFactor, Functional) {
  double multiplier = 2.0;
  Matrix X = Matrix::Identity(3, 3);
  Matrix measurement = multiplier * Matrix::Identity(3, 3);

  std::function<Matrix(Matrix, boost::optional<Matrix &>)> functional =
      MultiplyFunctor(multiplier);
  auto factor =
      MakeFunctorizedFactor<Matrix>(key, measurement, model, functional);

  Vector error = factor.evaluateError(X);

  EXPECT(assert_equal(Vector::Zero(9), error, 1e-9));
}

/* ************************************************************************* */
// Test FunctorizedFactor with a lambda function.
TEST(FunctorizedFactor, Lambda) {
  double multiplier = 2.0;
  Matrix X = Matrix::Identity(3, 3);
  Matrix measurement = multiplier * Matrix::Identity(3, 3);

  auto lambda = [multiplier](const Matrix &X,
                             OptionalJacobian<-1, -1> H = boost::none) {
    if (H)
      *H = multiplier *
           Matrix::Identity(X.rows() * X.cols(), X.rows() * X.cols());
    return multiplier * X;
  };
  // FunctorizedFactor<Matrix> factor(key, measurement, model, lambda);
  auto factor = MakeFunctorizedFactor<Matrix>(key, measurement, model, lambda);

  Vector error = factor.evaluateError(X);

  EXPECT(assert_equal(Vector::Zero(9), error, 1e-9));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

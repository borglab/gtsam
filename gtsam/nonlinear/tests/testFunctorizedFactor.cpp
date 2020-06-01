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

#include <gtsam/base/Testable.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/FunctorizedFactor.h>

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

Key keyX = Symbol('X', 0);
auto model = noiseModel::Isotropic::Sigma(3, 1);

/// Functor that takes a matrix and multiplies every element by m
class MultiplyFunctor {
  double m_; ///< simple multiplier

public:
  using argument_type = Matrix;
  using return_type = Matrix;

  MultiplyFunctor(double m) : m_(m) {}

  Matrix operator()(const Matrix &X,
                    OptionalJacobian<-1, -1> H = boost::none) const {
    return m_ * X;
  }
};

TEST(FunctorizedFactor, Identity) {

  Matrix X = Matrix::Identity(3, 3);

  double multiplier = 1.0;

  FunctorizedFactor<MultiplyFunctor> factor(keyX, X, model, multiplier);

  Values values;
  values.insert<Matrix>(keyX, X);

  Matrix error = factor.evaluateError(X);

  EXPECT(assert_equal(Vector::Zero(9), error, 1e-9));
}

TEST(FunctorizedFactor, Multiply2) {
  Matrix X = Matrix::Identity(3, 3);

  double multiplier = 2.0;

  FunctorizedFactor<MultiplyFunctor> factor(keyX, X, model, multiplier);

  Values values;
  values.insert<Matrix>(keyX, X);

  Matrix error = factor.evaluateError(X);

  Matrix expected = Matrix::Identity(3, 3);
  expected.resize(9, 1);
  EXPECT(assert_equal(expected, error, 1e-9));
}

TEST(FunctorizedFactor, Equality) {
  Matrix X = Matrix::Identity(2, 2);

  double multiplier = 2.0;

  FunctorizedFactor<MultiplyFunctor> factor1(keyX, X, model, multiplier);
  FunctorizedFactor<MultiplyFunctor> factor2(keyX, X, model, multiplier);

  EXPECT(factor1.equals(factor2));
}

TEST(FunctorizedFactor, Print) {
  Matrix X = Matrix::Identity(2, 2);

  double multiplier = 2.0;

  FunctorizedFactor<MultiplyFunctor> factor(keyX, X, model, multiplier);

  // redirect output to buffer so we can compare
  stringstream buffer;
  streambuf *old = cout.rdbuf(buffer.rdbuf());

  factor.print();

  // get output string and reset stdout
  string actual = buffer.str();
  cout.rdbuf(old);

  string expected = "  keys = { X0 }\n"
                    "  noise model: unit (3) \n"
                    "FunctorizedFactor(X0)\n"
                    "  measurement: [\n"
                    "	1, 0;\n"
                    " 	0, 1\n"
                    "]\n"
                    "  noise model sigmas: 1 1 1\n";

  CHECK_EQUAL(expected, actual);
}

/* *************************************************************************
 */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

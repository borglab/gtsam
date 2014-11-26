/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file expressionTesting.h
 * @date September 18, 2014
 * @author Frank Dellaert
 * @author Paul Furgale
 * @brief Test harness methods for expressions.
 */

#pragma once

#include <gtsam_unstable/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/linear/VectorValues.h>

#include <CppUnitLite/TestResult.h>
#include <CppUnitLite/Test.h>
#include <CppUnitLite/Failure.h>

namespace gtsam {

/**
 * Linearize a nonlinear factor using numerical differentiation
 * The benefit of this method is that it does not need to know what types are
 * involved to evaluate the factor. If all the machinery of gtsam is working
 * correctly, we should get the correct numerical derivatives out the other side.
 */
JacobianFactor linearizeNumerically(const NoiseModelFactor& factor,
    const Values& values, double delta = 1e-5) {

  // We will fill a map of Jacobians
  std::map<Key, Matrix> jacobians;

  // Get size
  Eigen::VectorXd e = factor.whitenedError(values);
  const size_t rows = e.size();

  // Loop over all variables
  VectorValues dX = values.zeroVectors();
  BOOST_FOREACH(Key key, factor) {
    // Compute central differences using the values struct.
    const size_t cols = dX.dim(key);
    Matrix J = Matrix::Zero(rows, cols);
    for (size_t col = 0; col < cols; ++col) {
      Eigen::VectorXd dx = Eigen::VectorXd::Zero(cols);
      dx[col] = delta;
      dX[key] = dx;
      Values eval_values = values.retract(dX);
      Eigen::VectorXd left = factor.whitenedError(eval_values);
      dx[col] = -delta;
      dX[key] = dx;
      eval_values = values.retract(dX);
      Eigen::VectorXd right = factor.whitenedError(eval_values);
      J.col(col) = (left - right) * (1.0 / (2.0 * delta));
    }
    jacobians[key] = J;
  }

  // Next step...return JacobianFactor
  return JacobianFactor(jacobians, -e);
}

namespace internal {
// CPPUnitLite-style test for linearization of a factor
void testFactorJacobians(TestResult& result_, const std::string& name_,
    const NoiseModelFactor& factor, const gtsam::Values& values, double delta,
    double tolerance) {

  // Create expected value by numerical differentiation
  JacobianFactor expected = linearizeNumerically(factor, values, delta);

  // Create actual value by linearize
  boost::shared_ptr<JacobianFactor> actual = //
      boost::dynamic_pointer_cast<JacobianFactor>(factor.linearize(values));

  // Check cast result and then equality
  CHECK(actual);
  EXPECT( assert_equal(expected, *actual, tolerance));
}
}

/// \brief Check the Jacobians produced by a factor against finite differences.
/// \param factor The factor to test.
/// \param values Values filled in for testing the Jacobians.
/// \param numerical_derivative_step The step to use when computing the numerical derivative Jacobians
/// \param tolerance The numerical tolerance to use when comparing Jacobians.
#define EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, numerical_derivative_step, tolerance) \
    { gtsam::internal::testFactorJacobians(result_, name_, factor, values, numerical_derivative_step, tolerance); }

namespace internal {
// CPPUnitLite-style test for linearization of an ExpressionFactor
template<typename T>
void testExpressionJacobians(TestResult& result_, const std::string& name_,
    const gtsam::Expression<T>& expression, const gtsam::Values& values,
    double nd_step, double tolerance) {
  // Create factor
  size_t size = traits::dimension<T>::value;
  ExpressionFactor<T> f(noiseModel::Unit::Create(size),
      expression.value(values), expression);
  testFactorJacobians(result_, name_, f, values, nd_step, tolerance);
}
}
} // namespace gtsam

/// \brief Check the Jacobians produced by an expression against finite differences.
/// \param expression The expression to test.
/// \param values Values filled in for testing the Jacobians.
/// \param numerical_derivative_step The step to use when computing the finite difference Jacobians
/// \param tolerance The numerical tolerance to use when comparing Jacobians.
#define EXPECT_CORRECT_EXPRESSION_JACOBIANS(expression, values, numerical_derivative_step, tolerance) \
    { gtsam::internal::testExpressionJacobians(result_, name_, expression, values, numerical_derivative_step, tolerance); }

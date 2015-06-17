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

#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/linear/VectorValues.h>

#include <CppUnitLite/TestResult.h>
#include <CppUnitLite/Test.h>
#include <CppUnitLite/Failure.h>

namespace gtsam {

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
  EXPECT(assert_equal(expected, *actual, tolerance));
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
  size_t size = traits<T>::dimension;
  ExpressionFactor<T> f(noiseModel::Unit::Create(size),
      expression.value(values), expression);
  testFactorJacobians(result_, name_, f, values, nd_step, tolerance);
}
} // namespace internal
} // namespace gtsam

/// \brief Check the Jacobians produced by an expression against finite differences.
/// \param expression The expression to test.
/// \param values Values filled in for testing the Jacobians.
/// \param numerical_derivative_step The step to use when computing the finite difference Jacobians
/// \param tolerance The numerical tolerance to use when comparing Jacobians.
#define EXPECT_CORRECT_EXPRESSION_JACOBIANS(expression, values, numerical_derivative_step, tolerance) \
    { gtsam::internal::testExpressionJacobians(result_, name_, expression, values, numerical_derivative_step, tolerance); }

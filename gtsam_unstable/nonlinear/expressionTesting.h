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

#include "Expression.h"
#include "ExpressionFactor.h"
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <CppUnitLite/TestResult.h>
#include <CppUnitLite/Test.h>
#include <CppUnitLite/Failure.h>
#include <gtsam/base/numericalDerivative.h>

namespace gtsam {

template<typename T>
void testExpressionJacobians(TestResult& result_,
                             const std::string& name_,
                             const gtsam::Expression<T>& expression,
                             const gtsam::Values& values,
                             double nd_step,
                             double tolerance) {
  // Create factor
  size_t size = traits::dimension<T>::value;
  ExpressionFactor<T> f(noiseModel::Unit::Create(size), expression.value(values), expression);
  testFactorJacobians(result_, name_, f, values, nd_step, tolerance);
}
}  // namespace gtsam

/// \brief Check the Jacobians produced by an expression against finite differences.
/// \param expression The expression to test.
/// \param values Values filled in for testing the Jacobians.
/// \param numerical_derivative_step The step to use when computing the finite difference Jacobians
/// \param tolerance The numerical tolerance to use when comparing Jacobians.
#define EXPECT_CORRECT_EXPRESSION_JACOBIANS(expression, values, numerical_derivative_step, tolerance) \
    { gtsam::testExpressionJacobians(result_, name_, expression, values, numerical_derivative_step, tolerance); }

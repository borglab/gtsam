/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Expression.h
 * @date September 18, 2014
 * @author Frank Dellaert
 * @author Paul Furgale
 * @brief Expressions for Block Automatic Differentiation
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

template<typename FactorType>
void testFactorJacobians(TestResult& result_,
                         const std::string& name_,
                         const FactorType& f,
                         const gtsam::Values& values,
                         double fd_step,
                         double tolerance) {
  // Check linearization
  JacobianFactor expected = computeFiniteDifferenceJacobianFactor(f, values, fd_step);
  boost::shared_ptr<GaussianFactor> gf = f.linearize(values);
  boost::shared_ptr<JacobianFactor> jf = //
      boost::dynamic_pointer_cast<JacobianFactor>(gf);

  typedef std::pair<Eigen::MatrixXd, Eigen::VectorXd> Jacobian;
  Jacobian evalJ = jf->jacobianUnweighted();
  Jacobian estJ = expected.jacobianUnweighted();
  EXPECT(assert_equal(evalJ.first, estJ.first, tolerance));
  EXPECT(assert_equal(evalJ.second, Eigen::VectorXd::Zero(evalJ.second.size()), tolerance));
  EXPECT(assert_equal(estJ.second, Eigen::VectorXd::Zero(evalJ.second.size()), tolerance));
}

template<typename T>
void testExpressionJacobians(TestResult& result_,
                             const std::string& name_,
                             const gtsam::Expression<T>& expression,
                             const gtsam::Values& values,
                             double fd_step,
                             double tolerance) {
  // Create factor
  size_t size = traits::dimension<T>::value;
  ExpressionFactor<T> f(noiseModel::Unit::Create(size), expression.value(values), expression);
  testFactorJacobians(result_, name_, f, values, fd_step, tolerance);
}

// Do a full concept check and test the invertibility of
// local() vs. retract().
template<typename T>
void testDefaultChart(TestResult& result_,
                      const std::string& name_,
                      const T& value) {
  T other = value;
  // Check for the existence of a print function.
  gtsam::traits::print<T>()(value, "value");
  gtsam::traits::print<T>()(other, "other");

  // Check for the existence of "equals"
  EXPECT(gtsam::traits::equals<T>()(value, other, 1e-12));

  typedef typename gtsam::DefaultChart<T> Chart;
  typedef typename Chart::vector Vector;

  // Check that the dimension of the local value matches the chart dimension.
  Vector dx = Chart::local(value, other);
  EXPECT_LONGS_EQUAL(Chart::getDimension(value), dx.size());
  // And that the "local" of a value vs. itself is zero.
  EXPECT(assert_equal(Matrix(dx), Matrix(Eigen::VectorXd::Zero(dx.size()))));

  // Test the invertibility of retract/local
  dx.setRandom();
  T updated = Chart::retract(value, dx);
  Vector invdx = Chart::local(value, updated);
  EXPECT(assert_equal(Matrix(dx), Matrix(invdx), 1e-9));

  dx = -dx;
  updated = Chart::retract(value, dx);
  invdx = Chart::local(value, updated);
  EXPECT(assert_equal(Matrix(dx), Matrix(invdx), 1e-9));
}

/// \brief Check the Jacobians produced by a factor against finite differences.
/// \param factor The factor to test.
/// \param values Values filled in for testing the Jacobians.
/// \param finite_difference_step The step to use when computing the finite difference Jacobians
/// \param tolerance The numerical tolerance to use when comparing Jacobians.
#define EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, finite_difference_step, tolerance) \
    { testFactorJacobians(result_, name_, factor, values, finite_difference_step, tolerance); }

/// \brief Check the Jacobians produced by an expression against finite differences.
/// \param expression The expression to test.
/// \param values Values filled in for testing the Jacobians.
/// \param finite_difference_step The step to use when computing the finite difference Jacobians
/// \param tolerance The numerical tolerance to use when comparing Jacobians.
#define EXPECT_CORRECT_EXPRESSION_JACOBIANS(expression, values, finite_difference_step, tolerance) \
    { testExpressionJacobians(result_, name_, expression, values, finite_difference_step, tolerance); }

/// \brief Perform a concept check on the default chart for a type.
/// \param value An instantiation of the type to be tested.
#define CHECK_CHART_CONCEPT(value) \
    { testDefaultChart(result_, name_, value); }
}  // namespace gtsam

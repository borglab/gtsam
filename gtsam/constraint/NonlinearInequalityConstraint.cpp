/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    NonlinearInequalityConstraint.cpp
 * @brief   Nonlinear inequality constraints in constrained optimization.
 * @author  Yetong Zhang, Frank Dellaert
 * @date    Aug 3, 2024
 */

#include <gtsam/constraint/NonlinearInequalityConstraint.h>
#include <gtsam/constraint/RampFunction.h>

namespace gtsam {

/* ************************************************************************* */
Vector NonlinearInequalityConstraint::unwhitenedError(const Values& x,
                                                      OptionalMatrixVecType H) const {
  Vector error = unwhitenedExpr(x, H);
  for (size_t i = 0; i < dim(); i++) {
    if (error(i) < 0) {
      error(i) = 0;
      if (H) {
        for (Matrix& m : *H) {
          m.row(i).setZero();
        }
      }
    }
  }
  return error;
}

/* ************************************************************************* */
bool NonlinearInequalityConstraint::active(const Values& x) const {
  return (unwhitenedExpr(x).array() >= 0).any();
}

/* ************************************************************************* */
NoiseModelFactor::shared_ptr NonlinearInequalityConstraint::penaltyFactorSmooth(
    SmoothRampFunction::shared_ptr func, const double mu) const {
  /// Default behavior, this function should be overriden.
  return penaltyFactor(mu);
}

/* ************************************************************************* */
NonlinearEqualityConstraint::shared_ptr NonlinearInequalityConstraint::createEqualityConstraint()
    const {
  /// Default behavior, this function should be overriden.
  return nullptr;
}

/* ************************************************************************* */
NoiseModelFactor::shared_ptr NonlinearInequalityConstraint::penaltyFactorEquality(
    const double mu) const {
  return createEqualityConstraint()->penaltyFactor(mu);
}

/* ************************************************************************* */
ScalarExpressionInequalityConstraint::ScalarExpressionInequalityConstraint(
    const Double_& expression, const double& sigma)
    : Base(constrainedNoise(Vector1(sigma)), expression.keysAndDims().first),
      expression_(expression),
      dims_(expression.keysAndDims().second) {}

/* ************************************************************************* */
ScalarExpressionInequalityConstraint::shared_ptr ScalarExpressionInequalityConstraint::GeqZero(
    const Double_& expression, const double& sigma) {
  Double_ neg_expr = Double_(0.0) - expression;
  return std::make_shared<ScalarExpressionInequalityConstraint>(neg_expr, sigma);
}

/* ************************************************************************* */
ScalarExpressionInequalityConstraint::shared_ptr ScalarExpressionInequalityConstraint::LeqZero(
    const Double_& expression, const double& sigma) {
  return std::make_shared<ScalarExpressionInequalityConstraint>(expression, sigma);
}

/* ************************************************************************* */
Vector ScalarExpressionInequalityConstraint::unwhitenedExpr(const Values& x,
                                                            OptionalMatrixVecType H) const {
  // Copy-paste from ExpressionFactor.
  if (H) {
    return Vector1(expression_.valueAndDerivatives(x, keys_, dims_, *H));
  } else {
    return Vector1(expression_.value(x));
  }
}

/* ************************************************************************* */
NonlinearEqualityConstraint::shared_ptr
ScalarExpressionInequalityConstraint::createEqualityConstraint() const {
  return std::make_shared<ExpressionEqualityConstraint<double>>(
      expression_, 0.0, noiseModel()->sigmas());
}

/* ************************************************************************* */
NoiseModelFactor::shared_ptr ScalarExpressionInequalityConstraint::penaltyFactor(
    const double mu) const {
  Double_ penalty_expression(RampFunction, expression_);
  return std::make_shared<ExpressionFactor<double>>(penaltyNoise(mu), 0.0, penalty_expression);
}

/* ************************************************************************* */
NoiseModelFactor::shared_ptr ScalarExpressionInequalityConstraint::penaltyFactorSmooth(
    SmoothRampFunction::shared_ptr func, const double mu) const {
  // TODO(yetong): can we pass the functor directly to construct the expression?
  Double_ error(func->function(), expression_);
  return std::make_shared<ExpressionFactor<double>>(penaltyNoise(mu), 0.0, error);
}

/* ************************************************************************* */
NoiseModelFactor::shared_ptr ScalarExpressionInequalityConstraint::penaltyFactorEquality(
    const double mu) const {
  return std::make_shared<ExpressionFactor<double>>(penaltyNoise(mu), 0.0, expression_);
}

/* ************************************************************************* */
size_t NonlinearInequalityConstraints::dim() const {
  size_t dimension = 0;
  for (const auto& constraint : *this) {
    dimension += constraint->dim();
  }
  return dimension;
}

/* ************************************************************************* */
Vector NonlinearInequalityConstraints::violationVector(const Values& values, bool whiten) const {
  Vector violation(dim());
  size_t start_idx = 0;
  for (const auto& constraint : *this) {
    size_t dim = constraint->dim();
    violation.middleCols(start_idx, dim) =
        whiten ? constraint->whitenedError(values) : constraint->unwhitenedError(values);
    start_idx += dim;
  }
  return violation;
}

/* ************************************************************************* */
double NonlinearInequalityConstraints::violationNorm(const Values& values) const {
  return violationVector(values, true).norm();
}

/* ************************************************************************* */
NonlinearFactorGraph NonlinearInequalityConstraints::penaltyGraph(const double mu) const {
  NonlinearFactorGraph graph;
  for (const auto& constraint : *this) {
    graph.add(constraint->penaltyFactor(mu));
  }
  return graph;
}

/* ************************************************************************* */
NonlinearFactorGraph NonlinearInequalityConstraints::penaltyGraphSmooth(
    SmoothRampFunction::shared_ptr func, const double mu) const {
  NonlinearFactorGraph graph;
  for (const auto& constraint : *this) {
    graph.add(constraint->penaltyFactorSmooth(func, mu));
  }
  return graph;
}

}  // namespace gtsam

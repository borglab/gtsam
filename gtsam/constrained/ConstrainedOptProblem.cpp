/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    ConstrainedOptProblem.h
 * @brief   Nonlinear constrained optimization problem.
 * @author  Yetong Zhang, Frank Dellaert
 * @date    Aug 3, 2024
 */

#include <gtsam/constrained/ConstrainedOptProblem.h>

namespace gtsam {

/* ************************************************************************* */
size_t GraphDimension(const NonlinearFactorGraph& graph) {
  size_t total_dim = 0;
  for (const auto& factor : graph) {
    total_dim += factor->dim();
  }
  return total_dim;
}

/* ************************************************************************* */
bool CheckPureCost(const NonlinearFactorGraph& graph) {
  for (const auto& factor : graph) {
    if (NoiseModelFactor::shared_ptr f =
            std::dynamic_pointer_cast<NoiseModelFactor>(factor)) {
      if (f->noiseModel()->isConstrained()) {
        return false;
      }
    }
  }
  return true;
}

/* ************************************************************************* */
ConstrainedOptProblem::ConstrainedOptProblem(
    const NonlinearFactorGraph& costs,
    const NonlinearEqualityConstraints& eqConstraints,
    const NonlinearInequalityConstraints& ineqConstraints)
    : costs_(costs),
      eqConstraints_(eqConstraints),
      ineqConstraints_(ineqConstraints) {
  if (!CheckPureCost(costs)) {
    throw std::runtime_error(
        "Cost contains factors with constrained noise model. They should be "
        "moved to constraints.");
  }
}

ConstrainedOptProblem::ConstrainedOptProblem(
    const NonlinearFactorGraph& graph) {
  for (const auto& factor : graph) {
    if (NonlinearEqualityConstraint::shared_ptr f =
            std::dynamic_pointer_cast<NonlinearEqualityConstraint>(factor)) {
      eqConstraints_.push_back(f);
    } else if (NonlinearInequalityConstraint::shared_ptr f =
                   std::dynamic_pointer_cast<NonlinearInequalityConstraint>(
                       factor)) {
      ineqConstraints_.push_back(f);
    } else {
      costs_.push_back(factor);
    }
  }
}

/* ************************************************************************* */
std::tuple<size_t, size_t, size_t> ConstrainedOptProblem::dim() const {
  return {GraphDimension(costs()), eConstraints().dim(), iConstraints().dim()};
}

/* ************************************************************************* */
std::tuple<double, double, double> ConstrainedOptProblem::evaluate(
    const Values& values) const {
  return {costs().error(values), eConstraints().violationNorm(values),
          iConstraints().violationNorm(values)};
}

/* ************************************************************************* */
std::pair<ConstrainedOptProblem, Values>
ConstrainedOptProblem::auxiliaryProblem(
    const Values& values, const AuxiliaryKeyGenerator& generator) const {
  if (iConstraints().size() == 0) {
    return {*this, values};
  }

  NonlinearEqualityConstraints newEqConstraints = eConstraints();
  Values new_values = values;

  size_t k = 0;
  for (const auto& i_constraint : iConstraints()) {
    if (ScalarExpressionInequalityConstraint::shared_ptr p =
            std::dynamic_pointer_cast<ScalarExpressionInequalityConstraint>(
                i_constraint)) {
      // Generate next available auxiliary key.
      Key aux_key = generator.generate(k, new_values);

      // Construct auxiliary equality constraint.
      Double_ aux_expr(aux_key);
      Double_ equality_expr = p->expression() + aux_expr * aux_expr;
      newEqConstraints.emplace_shared<ExpressionEqualityConstraint<double>>(
          equality_expr, 0.0, p->noiseModel()->sigmas());

      // Compute initial value for auxiliary key.
      if (!i_constraint->feasible(values)) {
        new_values.insert(aux_key, 0.0);
      } else {
        Vector gap = i_constraint->unwhitenedExpr(values);
        new_values.insert(aux_key, sqrt(-gap(0)));
      }
    }
  }
  return {
      ConstrainedOptProblem::EqConstrainedOptProblem(costs_, newEqConstraints),
      new_values};
}

/* ************************************************************************* */

}  // namespace gtsam
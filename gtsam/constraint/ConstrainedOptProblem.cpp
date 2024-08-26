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

#include <gtsam/constraint/ConstrainedOptProblem.h>
#include <memory>
#include <stdexcept>
#include "gtsam/constraint/NonlinearEqualityConstraint.h"
#include "gtsam/nonlinear/NonlinearFactor.h"

namespace gtsam {

/* ********************************************************************************************* */
size_t GraphDimension(const NonlinearFactorGraph& graph) {
  size_t total_dim = 0;
  for (const auto& factor : graph) {
    total_dim += factor->dim();
  }
  return total_dim;
}

/* ********************************************************************************************* */
bool CheckPureCost(const NonlinearFactorGraph& graph) {
  for (const auto& factor : graph) {
    if (NoiseModelFactor::shared_ptr f = std::dynamic_pointer_cast<NoiseModelFactor>(factor)) {
      if (f->noiseModel()->isConstrained()) {
        return false;
      }
    }
  }
  return true;
}

/* ********************************************************************************************* */
ConstrainedOptProblem::ConstrainedOptProblem(const NonlinearFactorGraph& costs,
                                             const NonlinearEqualityConstraints& e_constraints,
                                             const NonlinearInequalityConstraints& i_constraints,
                                             const Values& values)
    : costs_(costs), e_constraints_(e_constraints), i_constraints_(i_constraints), values_(values) {
  if (!CheckPureCost(costs)) {
    throw std::runtime_error(
        "Cost contains factors with constrained noise model. They should be moved to constraints.");
  }
}

/* ********************************************************************************************* */
std::tuple<size_t, size_t, size_t, size_t> ConstrainedOptProblem::dim() const {
  return {
      GraphDimension(costs()), eConstraints().dim(), iConstraints().dim(), initialValues().dim()};
}

/* ********************************************************************************************* */
std::tuple<double, double, double> ConstrainedOptProblem::evaluate(const Values& values) const {
  return {costs().error(values),
          eConstraints().violationNorm(values),
          iConstraints().violationNorm(values)};
}

/* ********************************************************************************************* */
ConstrainedOptProblem ConstrainedOptProblem::auxiliaryProblem(
    const AuxiliaryKeyGenerator& generator) const {
  if (iConstraints().size() == 0) {
    return *this;
  }

  NonlinearEqualityConstraints new_e_constraints = eConstraints();
  Values new_values = initialValues();

  size_t k = 0;
  for (const auto& i_constraint : iConstraints()) {
    if (ScalarExpressionInequalityConstraint::shared_ptr p =
            std::dynamic_pointer_cast<ScalarExpressionInequalityConstraint>(i_constraint)) {
      // Generate next available auxiliary key.
      Key aux_key = generator.generate(k, new_values);

      // Construct auxiliary equality constraint.
      Double_ aux_expr(aux_key);
      Double_ equality_expr = p->expression() + aux_expr * aux_expr;
      new_e_constraints.emplace_shared<ExpressionEqualityConstraint<double>>(
          equality_expr, 0.0, p->noiseModel()->sigmas());

      // Compute initial value for auxiliary key.
      if (!i_constraint->feasible(initialValues())) {
        new_values.insert(aux_key, 0.0);
      } else {
        Vector gap = i_constraint->unwhitenedExpr(initialValues());
        new_values.insert(aux_key, sqrt(-gap(0)));
      }
    }
  }
  return ConstrainedOptProblem::EqConstrainedOptProblem(costs_, new_e_constraints, new_values);
}

/* ********************************************************************************************* */

}  // namespace gtsam
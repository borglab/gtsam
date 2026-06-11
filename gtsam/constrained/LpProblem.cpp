/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    LpProblem.cpp
 * @brief   Linear programming problem over Vector and Matrix values.
 * @author  Frank Dellaert
 */

#include <gtsam/constrained/ActiveSetSolver.h>
#include <gtsam/constrained/LpProblem.h>

namespace gtsam {

/* ************************************************************************* */
void LpProblem::addConstraint(const LinearConstraint& constraint) {
  if (constraint.isEquality()) {
    eqConstraints_.push_back(constraint.createEqualityFactor());
  } else {
    ineqConstraints_.push_back(constraint.createInequalityFactor());
  }
}

/* ************************************************************************* */
double LpProblem::objective(const Values& values) const {
  double total = 0.0;
  for (const LpCost& cost : linearCosts_) {
    total += cost.value(values);
  }
  return total;
}

/* ************************************************************************* */
Values LpProblem::optimize(
    const Values& initialValues,
    std::shared_ptr<ActiveSetSolverParams> params) const {
  ActiveSetSolver solver(*this, params);
  return solver.optimize(initialValues);
}

/* ************************************************************************* */
Values LpProblem::optimize(
    std::shared_ptr<ActiveSetSolverParams> params) const {
  ActiveSetSolver solver(*this, params);
  return solver.optimize();
}

}  // namespace gtsam

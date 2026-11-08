/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    QpProblem.cpp
 * @brief   Quadratic programming problem over Vector and Matrix values.
 * @author  Frank Dellaert
 */

#include <gtsam/base/GenericValue.h>
#include <gtsam/constrained/ActiveSetSolver.h>
#include <gtsam/constrained/AugmentedLagrangianSolver.h>
#include <gtsam/constrained/QpProblem.h>

namespace gtsam {
namespace {

/* ************************************************************************* */
ActiveSetSolverParams::shared_ptr DenseQpParams() {
  auto params = std::make_shared<ActiveSetSolverParams>();
  params->qpSubproblemSolver = ActiveSetSolverParams::QpSubproblemSolver::Dense;
  return params;
}

/** Extract native Gaussian inputs once at the convenience-method boundary. */
Values SolveAlmQp(const QpProblem& problem, const Values* initial,
                  const AugmentedLagrangianParams& params) {
  GaussianFactorGraph costs, equalities, inequalities;
  for (const auto& factor : problem.costs()) {
    const auto* cost = dynamic_cast<const QpCost*>(factor.get());
    if (!cost)
      throw std::invalid_argument("QP ALM requires quadratic cost factors");
    costs.push_back(cost->gaussianFactor().clone());
  }
  auto append = [](const LinearConstraint& constraint,
                   GaussianFactorGraph* graph) {
    auto factor = std::make_shared<JacobianFactor>(constraint.factor());
    if (constraint.sense() == LinearConstraint::Sense::GreaterEqual) {
      for (auto it = factor->begin(); it != factor->end(); ++it)
        factor->getA(it) *= -1;
      factor->getb() *= -1;
    }
    factor->setModel(false, constraint.sigmas());
    graph->push_back(factor);
  };
  for (const auto& factor : problem.eConstraints())
    append(std::dynamic_pointer_cast<LinearEqualityConstraintFactor>(factor)
               ->linearConstraint(),
           &equalities);
  for (const auto& factor : problem.iConstraints())
    append(std::dynamic_pointer_cast<LinearInequalityConstraintFactor>(factor)
               ->linearConstraint(),
           &inequalities);
  AugmentedLagrangianSolver solver(
      costs, equalities, inequalities,
      std::make_shared<AugmentedLagrangianParams>(params));
  VectorValues values;
  if (initial) {
    for (Key key : initial->keys()) {
      const Value& value = initial->at(key);
      if (const auto* vector =
              dynamic_cast<const GenericValue<Vector>*>(&value))
        values.insert(key, vector->value());
      else if (const auto* matrix =
                   dynamic_cast<const GenericValue<Matrix>*>(&value))
        values.insert(key, Eigen::Map<const Vector>(matrix->value().data(),
                                                    matrix->value().size()));
      else
        throw std::invalid_argument(
            "QP ALM supports Vector and Matrix initial values");
    }
  }
  const VectorValues solved =
      initial ? solver.optimize(values) : solver.optimize();
  Values result;
  for (const auto& [key, vector] : solved) {
    if (initial &&
        dynamic_cast<const GenericValue<Matrix>*>(&initial->at(key))) {
      const Matrix& shape = initial->at<Matrix>(key);
      result.insert(key, Matrix(Eigen::Map<const Matrix>(
                             vector.data(), shape.rows(), shape.cols())));
    } else
      result.insert(key, Vector(vector));
  }
  return result;
}

}  // namespace

/* ************************************************************************* */
void QpProblem::addConstraint(const LinearConstraint& constraint) {
  if (constraint.isEquality()) {
    eqConstraints_.push_back(constraint.createEqualityFactor());
  } else {
    ineqConstraints_.push_back(constraint.createInequalityFactor());
  }
}

/* ************************************************************************* */
Values QpProblem::optimize(const Values& initialValues,
                           QpSolverType solverType) const {
  if (solverType == QpSolverType::AugmentedLagrangian)
    return optimize(initialValues, AugmentedLagrangianParams());
  if (solverType == QpSolverType::Dense) {
    ActiveSetSolver solver(*this, DenseQpParams());
    return solver.optimize(initialValues);
  }
  ActiveSetSolver solver(*this);
  return solver.optimize(initialValues);
}

/* ************************************************************************* */
Values QpProblem::optimize(QpSolverType solverType) const {
  if (solverType == QpSolverType::AugmentedLagrangian)
    return optimize(AugmentedLagrangianParams());
  if (solverType == QpSolverType::Dense) {
    ActiveSetSolver solver(*this, DenseQpParams());
    return solver.optimize();
  }
  ActiveSetSolver solver(*this);
  return solver.optimize();
}

/* ************************************************************************* */
Values QpProblem::optimize(const Values& initialValues,
                           const AugmentedLagrangianParams& params) const {
  return SolveAlmQp(*this, &initialValues, params);
}

/* ************************************************************************* */
Values QpProblem::optimize(const AugmentedLagrangianParams& params) const {
  return SolveAlmQp(*this, nullptr, params);
}

}  // namespace gtsam

/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    QcqpProblem.cpp
 * @brief   QCQP problem implementations.
 * @author  Frank Dellaert
 */

#include <gtsam/constrained/QcqpProblem.h>

namespace gtsam {

/* ************************************************************************* */
QcqpProblem::QcqpProblem(const NonlinearFactorGraph& graph,
                         size_t columnDimension,
                         bool useRedundantConstraints) {
  if (columnDimension == 0) {
    throw std::invalid_argument(
        "QcqpProblem: columnDimension must be positive.");
  }

  QcqpVariableConstraints variableConstraints;
  QcqpVariableConstraints redundantConstraints;
  for (const auto& factor : graph) {
    if (!factor) continue;

    factor->qcqpFactors(costs_, eqConstraints_, variableConstraints,
                        columnDimension);
    if (useRedundantConstraints) {
      const auto* provider =
          dynamic_cast<const RedundantQcqpProvider*>(factor.get());
      if (provider) {
        provider->appendRedundantConstraints(columnDimension,
                                              redundantConstraints);
      }
    }
  }
  eqConstraints_.add(variableConstraints.constraints());
  eqConstraints_.add(redundantConstraints.constraints());
}

/* ************************************************************************* */
void QcqpProblem::addConstraint(const LinearConstraint& constraint) {
  if (constraint.isEquality()) {
    eqConstraints_.push_back(constraint.createEqualityFactor());
  } else {
    ineqConstraints_.push_back(constraint.createInequalityFactor());
  }
}

/* ************************************************************************* */
void QcqpProblem::addConstraint(const QuadraticConstraint& constraint) {
  if (constraint.isEquality()) {
    eqConstraints_.push_back(constraint.createEqualityFactor());
  } else {
    ineqConstraints_.push_back(constraint.createInequalityFactor());
  }
}

}  // namespace gtsam

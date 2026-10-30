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

  QcqpConstraintMap variableConstraints;
  QcqpConstraintMap redundantByKey;
  for (const auto& factor : graph) {
    if (!factor) continue;

    NonlinearFactorGraph factorCosts;
    NonlinearEqualityConstraints factorConstraints;
    factor->qcqpFactors(&factorCosts, &factorConstraints, &variableConstraints,
                        columnDimension);
    if (useRedundantConstraints) {
      const auto* provider =
          dynamic_cast<const RedundantQcqpProvider*>(factor.get());
      if (provider) {
        provider->appendRedundantConstraints(columnDimension, &redundantByKey);
      }
    }
    costs_.add(factorCosts);
    eqConstraints_.add(factorConstraints);
  }
  for (const auto& entry : variableConstraints) {
    eqConstraints_.add(entry.second);
  }
  for (const auto& entry : redundantByKey) {
    eqConstraints_.add(entry.second);
  }
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

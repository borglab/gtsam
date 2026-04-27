/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    QcqpProblem.h
 * @brief   QCQP represented as a constrained optimization problem.
 * @author  Frank Dellaert
 */

#pragma once

#include <gtsam/base/Manifold.h>
#include <gtsam/constrained/ConstrainedOptProblem.h>
#include <gtsam/constrained/LinearConstraint.h>
#include <gtsam/constrained/QpCost.h>
#include <gtsam/constrained/QuadraticConstraint.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <stdexcept>

namespace gtsam {

/**
 * Insert one lifted matrix value using traits<T>::QcqpValue<D>.
 *
 * D is the number of columns in the matrix-valued QCQP variable. D=1 covers
 * vector QCQPs as r-by-1 matrices; larger D values cover matrix-valued lifts
 * without changing the row-space cost matrices.
 */
template <typename T, int D = 1>
void InsertLiftedValue(Key key, const T& value, Values* liftedValues) {
  if (!liftedValues) {
    throw std::invalid_argument("InsertLiftedValue: liftedValues is null.");
  }

  const Matrix lifted = traits<T>::template QcqpValue<D>(value);
  if (liftedValues->exists(key)) {
    const Matrix existing = liftedValues->at<Matrix>(key);
    if (existing.rows() != lifted.rows() || existing.cols() != lifted.cols()) {
      throw std::invalid_argument(
          "InsertLiftedValue: inconsistent lifted matrix dimensions.");
    }
    return;
  }
  liftedValues->insert(key, lifted);
}

/**
 * Insert QCQP equality constraints for one variable, skipping duplicate keys.
 */
template <typename T, int D = 1>
void InsertQcqpConstraints(Key key, NonlinearEqualityConstraints* constraints) {
  if (!constraints) {
    throw std::invalid_argument("InsertQcqpConstraints: constraints is null.");
  }

  for (const auto& factor : *constraints) {
    if (factor && factor->size() == 1 && factor->keys().front() == key) {
      return;
    }
  }

  for (const auto& [A, b] : traits<T>::template QcqpConstraints<D>()) {
    constraints->push_back(
        QuadraticConstraint::Equal(key, A, b).createEqualityFactor());
  }
}

/**
 * Thin constrained optimization problem for QCQPs over Vector or Matrix values.
 */
class GTSAM_EXPORT QcqpProblem : public ConstrainedOptProblem {
 public:
  using Base = ConstrainedOptProblem;
  using This = QcqpProblem;
  using shared_ptr = std::shared_ptr<This>;

  /** Default constructor creates an empty QCQP problem. */
  QcqpProblem() = default;

  /** Construct from QCQP cost factors and equality constraints. */
  QcqpProblem(const NonlinearFactorGraph& costs,
              const NonlinearEqualityConstraints& eqConstraints)
      : Base(costs, eqConstraints, NonlinearInequalityConstraints()) {}

  /** Construct from QCQP cost factors and equality/inequality constraints. */
  QcqpProblem(const NonlinearFactorGraph& costs,
              const NonlinearEqualityConstraints& eqConstraints,
              const NonlinearInequalityConstraints& ineqConstraints)
      : Base(costs, eqConstraints, ineqConstraints) {}

  /** Convert a supported nonlinear factor graph into QCQP costs/constraints. */
  explicit QcqpProblem(const NonlinearFactorGraph& graph,
                       size_t columnDimension = 1) {
    for (const auto& factor : graph) {
      if (factor) {
        factor->qcqpFactors(&costs_, &eqConstraints_, columnDimension);
      }
    }
  }

  /** Add a quadratic cost. */
  void addCost(const QpCost& cost) { costs_.emplace_shared<QpCost>(cost); }

  /** Add a linear constraint. */
  void addConstraint(const LinearConstraint& constraint);

  /** Add a quadratic constraint. */
  void addConstraint(const QuadraticConstraint& constraint);
};

}  // namespace gtsam

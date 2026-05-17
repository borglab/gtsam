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

#include <gtsam/constrained/ConstrainedOptProblem.h>
#include <gtsam/constrained/LinearConstraint.h>
#include <gtsam/constrained/QpCost.h>
#include <gtsam/constrained/QuadraticConstraint.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace gtsam {

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

  /** Add a quadratic cost. */
  void addCost(const QpCost& cost) { costs_.emplace_shared<QpCost>(cost); }

  /** Add a linear constraint. */
  void addConstraint(const LinearConstraint& constraint);

  /** Add a quadratic constraint. */
  void addConstraint(const QuadraticConstraint& constraint);
};

}  // namespace gtsam

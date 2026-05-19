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
#include <type_traits>
#include <utility>

namespace gtsam {

namespace internal {

template <typename T, int D, typename = void>
struct HasQcqpVariableTraits : std::false_type {};

template <typename T, int D>
struct HasQcqpVariableTraits<
    T, D,
    std::void_t<
        decltype(traits<T>::template QcqpValue<D>(std::declval<T>())),
        decltype(traits<T>::template QcqpConstraints<D>())>>
    : std::true_type {};

}  // namespace internal

/**
 * Insert one matrix-valued QCQP variable using traits<T>::QcqpValue<D>.
 *
 * D is the column dimension of the matrix-form QCQP variable (the row count
 * is type-intrinsic, e.g. 2 for Rot2 matrix form, 3 for Rot3 matrix form).
 * D=1 is the vec(R) form; D >= traits<T>::QcqpIntrinsicDim is the matrix
 * form. The Riemannian Staircase climbs D starting at QcqpIntrinsicDim and
 * the lift fills in the extra columns at each level.
 */
template <typename T, int D = 1>
void InsertQcqpValue(Key key, const T& value, Values* qcqpValues) {
  static_assert(internal::HasQcqpVariableTraits<T, D>::value,
                "InsertQcqpValue requires traits<T>::QcqpValue<D> and "
                "traits<T>::QcqpConstraints<D>.");
  if (!qcqpValues) {
    throw std::invalid_argument("InsertQcqpValue: qcqpValues is null.");
  }

  const Matrix qcqpValue = traits<T>::template QcqpValue<D>(value);
  if (qcqpValues->exists(key)) {
    const Matrix existing = qcqpValues->at<Matrix>(key);
    if (existing.rows() != qcqpValue.rows() ||
        existing.cols() != qcqpValue.cols()) {
      throw std::invalid_argument(
          "InsertQcqpValue: inconsistent QCQP variable dimensions.");
    }
    return;
  }
  qcqpValues->insert(key, qcqpValue);
}

/**
 * Insert QCQP equality constraints for one variable, skipping duplicate keys.
 * Constraints come from traits<T>::QcqpConstraints<D>(). For matrix form
 * (D >= QcqpIntrinsicDim) the constraints are D-independent in structure
 * (only QpCost's Kronecker expansion depends on D); for D=1 they are the
 * polynomial vec-form constraints.
 */
template <typename T, int D = 1>
void InsertQcqpConstraints(Key key, NonlinearEqualityConstraints* constraints) {
  static_assert(internal::HasQcqpVariableTraits<T, D>::value,
                "InsertQcqpConstraints requires traits<T>::QcqpValue<D> and "
                "traits<T>::QcqpConstraints<D>.");
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

  /**
   * Convert a supported nonlinear factor graph into QCQP costs/constraints
   * at column dimension K (the matrix-form column count).
   *
   * @param K column dimension of every QCQP variable. Each variable has row
   *   count `traits<T>::QcqpIntrinsicDim` (a per-type constant); K must be
   *   >= that intrinsic row dim for every variable type in the graph (the
   *   per-factor `qcqpFactors` enforces this). The Riemannian Staircase
   *   climbs K starting at the intrinsic row dim.
   */
  explicit QcqpProblem(const NonlinearFactorGraph& graph, size_t K) {
    if (K == 0) {
      throw std::invalid_argument("QcqpProblem: K must be >= 1.");
    }
    for (const auto& factor : graph) {
      if (factor) {
        factor->qcqpFactors(&costs_, &eqConstraints_, K);
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

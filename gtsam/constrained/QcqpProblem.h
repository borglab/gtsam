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
 *
 * Originally implemented for fixed-dimension QCQPs; extended here to a
 * column-dimension–parametric form. The same QCQP can now be re-emitted at
 * any column count K via `InsertQcqpValue<T, D>`,
 * `InsertQcqpConstraints<T, D>`, and the graph-driven constructor /
 * `rebuildAt(K)`. K = d (the variable's intrinsic dim) is the natural
 * problem; K > d is the low-rank factorization used by the Riemannian
 * Staircase, which calls `rebuildAt(K+1)` between levels to climb the
 * ladder without re-specifying the original factor graph.
 *
 * @author  Frank Dellaert
 * @author  Zhexin Xu
 * @author  David M. Rosen
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

/// SFINAE probe for traits<T>::QcqpIntrinsicDim.
template <typename T, typename = void>
struct HasQcqpIntrinsicDim : std::false_type {};

template <typename T>
struct HasQcqpIntrinsicDim<T, std::void_t<decltype(traits<T>::QcqpIntrinsicDim)>>
    : std::true_type {};

}  // namespace internal

/**
 * Insert one matrix-valued QCQP variable using `traits<T>::QcqpValue<D>`.
 *
 * D is the column dimension of the matrix-form QCQP variable; the row count
 * is type-intrinsic (e.g. 2 for Rot2, 3 for Rot3). Three regimes:
 *  - D = 1: the vec(R) form (legacy polynomial relaxation path).
 *  - D = `traits<T>::QcqpIntrinsicDim`: the natural matrix form of the
 *    QCQP.
 *  - D > `traits<T>::QcqpIntrinsicDim`: the low-rank factorization of the
 *    matrix form, used by the Riemannian Staircase to lift the SDP
 *    relaxation rung by rung — the extra columns are what the lift fills
 *    in at each level.
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
 * Insert QCQP equality constraints for one variable. Constraints come from
 * `traits<T>::QcqpConstraints<D>()`.
 *
 * For matrix form (D ≥ QcqpIntrinsicDim, including the D > d low-rank
 * factorization case) the constraint *structure* is D-independent — only
 * QpCost's Kronecker expansion depends on D — so climbing the Riemannian
 * Staircase doesn't change the constraint set, only the cost. For D = 1
 * these are the polynomial vec-form constraints (the original fixed-dim
 * path).
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

  // Halve b: trait emits (A, b) for `trace(X^T A X) = b`, QuadraticConstraint
  // is `0.5 * trace(X^T A X) - b = 0` — same zero-set with b/2.
  for (const auto& [A, b] : traits<T>::template QcqpConstraints<D>()) {
    constraints->push_back(
        QuadraticConstraint::Equal(key, A, 0.5 * b).createEqualityFactor());
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
   * Convert a supported `NonlinearFactorGraph` into QCQP costs/constraints
   * at column dimension K. K must be ≥ `traits<T>::QcqpIntrinsicDim` for
   * every variable type; K = d gives the natural QCQP and K > d gives the
   * low-rank-factorized form used by the Riemannian Staircase.
   *
   * The QCQP cost matches the paper form `‖residual‖²` — each factor's
   * `qcqpFactors` absorbs GTSAM's 0.5, so
   * `problem.costs().error(X) == 2 * graph.error(X)` on feasible X.
   */
  explicit QcqpProblem(const NonlinearFactorGraph& graph, size_t K)
      : originalGraph_(graph), K_(K) {
    if (K == 0) {
      throw std::invalid_argument("QcqpProblem: K must be >= 1.");
    }
    rebuildFromOriginalGraph();
  }

  /** Add a quadratic cost. */
  void addCost(const QpCost& cost) { costs_.emplace_shared<QpCost>(cost); }

  /** Add a linear constraint. */
  void addConstraint(const LinearConstraint& constraint);

  /** Add a quadratic constraint. */
  void addConstraint(const QuadraticConstraint& constraint);

  /// Re-derive costs/constraints from the original graph at a new K. Used
  /// by the Riemannian Staircase between levels to lift the QCQP into its
  /// next low-rank-factorized form (rank K+1) without re-specifying the
  /// source graph. Throws if this QcqpProblem was not constructed from a
  /// NonlinearFactorGraph.
  void rebuildAt(size_t K) {
    if (K == 0) {
      throw std::invalid_argument("QcqpProblem::rebuildAt: K must be >= 1.");
    }
    if (originalGraph_.empty()) {
      throw std::runtime_error(
          "QcqpProblem::rebuildAt: no original NonlinearFactorGraph is "
          "associated with this QcqpProblem. Construct it via "
          "QcqpProblem(graph, K) to enable staircase rebuilds.");
    }
    K_ = K;
    rebuildFromOriginalGraph();
  }

  /// Column dimension K the QcqpProblem was last built at; 0 if unknown.
  size_t K() const { return K_; }

  /// Source graph; empty if assembled via the cost/constraint constructors.
  const NonlinearFactorGraph& originalGraph() const { return originalGraph_; }

 private:
  void rebuildFromOriginalGraph() {
    costs_.resize(0);
    eqConstraints_.resize(0);
    for (const auto& factor : originalGraph_) {
      if (factor) {
        factor->qcqpFactors(&costs_, &eqConstraints_, K_);
      }
    }
  }

  NonlinearFactorGraph originalGraph_;  ///< source graph; empty if not built from one
  size_t K_ = 0;                        ///< current column dim
};

}  // namespace gtsam

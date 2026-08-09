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
#include <vector>

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

template <typename T, int D, typename = void>
struct HasQcqpExtractionTraits : std::false_type {};

template <typename T, int D>
struct HasQcqpExtractionTraits<
    T, D,
    std::void_t<decltype(traits<T>::template FromQcqpValue<D>(
        std::declval<const Matrix&>()))>> : HasQcqpVariableTraits<T, D> {};

template <typename T, typename = void>
struct HasQcqpVectorDim : std::false_type {};

template <typename T>
struct HasQcqpVectorDim<T, std::void_t<decltype(traits<T>::QcqpVectorDim)>>
    : std::true_type {};

}  // namespace internal

/**
 * Insert one matrix-valued QCQP variable using traits<T>::QcqpValue<D>.
 *
 * D is the number of columns in the matrix-valued QCQP variable. D=1 stores
 * vector QCQPs as r-by-1 matrices; larger D values store matrix-valued QCQP
 * variables without changing the row-space cost matrices.
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
 * Insert the QCQP equality constraints for one variable.
 *
 * Only an already-present quadratic equality with the same key, matrix, and
 * right-hand side is considered a duplicate. Other unary constraints on the
 * key are preserved and do not suppress the manifold constraints.
 */
template <typename T, int D = 1>
void InsertQcqpConstraints(Key key, NonlinearEqualityConstraints* constraints) {
  static_assert(internal::HasQcqpVariableTraits<T, D>::value,
                "InsertQcqpConstraints requires traits<T>::QcqpValue<D> and "
                "traits<T>::QcqpConstraints<D>.");
  if (!constraints) {
    throw std::invalid_argument("InsertQcqpConstraints: constraints is null.");
  }

  for (const auto& [A, b] : traits<T>::template QcqpConstraints<D>()) {
    bool alreadyPresent = false;
    for (const auto& factor : *constraints) {
      const auto* quadratic =
          dynamic_cast<const QuadraticEqualityConstraintFactor*>(factor.get());
      if (!quadratic) continue;
      const QuadraticConstraint& existing = quadratic->quadraticConstraint();
      if (existing.key() == key && existing.A().isApprox(A, 0.0) &&
          existing.b() == b && existing.sigma() == 1.0) {
        alreadyPresent = true;
        break;
      }
    }
    if (!alreadyPresent) {
      constraints->push_back(
          QuadraticConstraint::Equal(key, A, b).createEqualityFactor());
    }
  }
}

/**
 * Project QCQP variables back into typed values.
 *
 * D=1 accepts the exact homogenized vector dimension defined by T. D>1 accepts
 * exact N-by-D matrix slices, where N is the intrinsic matrix row dimension
 * of T. Canonical lifts and solutions whose connected component has been
 * explicitly gauge-aligned have meaningful absolute rotations. An unaligned
 * D>1 component has a common right-O(D) gauge, so its independently extracted
 * absolute rotations are intentionally best-effort and gauge-dependent.
 */
template <typename T, int D>
std::vector<std::pair<Key, T>> ExtractQcqpValues(const Values& qcqpValues) {
  static_assert(internal::HasQcqpExtractionTraits<T, D>::value,
                "ExtractQcqpValues requires traits<T>::QcqpValue<D>, "
                "QcqpConstraints<D>, and FromQcqpValue<D>.");
  static_assert(D != 1 || internal::HasQcqpVectorDim<T>::value,
                "ExtractQcqpValues<T, 1> requires "
                "traits<T>::QcqpVectorDim.");
  constexpr int expectedRows = [] {
    if constexpr (D == 1) {
      if constexpr (internal::HasQcqpVectorDim<T>::value) {
        return traits<T>::QcqpVectorDim;
      } else {
        return Eigen::Dynamic;
      }
    } else {
      return T::LieAlgebra::RowsAtCompileTime;
    }
  }();
  std::vector<std::pair<Key, T>> out;
  for (const auto& [key, M] : qcqpValues.extract<Matrix>()) {
    if (M.rows() == expectedRows && M.cols() == D) {
      out.emplace_back(key, traits<T>::template FromQcqpValue<D>(M));
    }
  }
  return out;
}

/** Return the exact D=1 QCQP vector for a typed value. */
template <typename T>
Matrix qcqpValue(const T& typedValue) {
  static_assert(internal::HasQcqpVariableTraits<T, 1>::value,
                "qcqpValue requires traits<T>::QcqpValue<1> and "
                "traits<T>::QcqpConstraints<1>.");
  return traits<T>::template QcqpValue<1>(typedValue);
}

/** Insert a typed value as an exact D=1 QCQP matrix. */
template <typename T>
void insertQcqpValue(Key key, const T& typedValue, Values& qcqpValues) {
  static_assert(internal::HasQcqpVariableTraits<T, 1>::value,
                "insertQcqpValue requires traits<T>::QcqpValue<1> and "
                "traits<T>::QcqpConstraints<1>.");
  InsertQcqpValue<T, 1>(key, typedValue, &qcqpValues);
}

/** Recover a typed value from an exact D=1 QCQP vector. */
template <typename T>
T fromQcqpValue(const Matrix& qcqpValue) {
  static_assert(internal::HasQcqpExtractionTraits<T, 1>::value,
                "fromQcqpValue requires traits<T>::QcqpValue<1>, "
                "QcqpConstraints<1>, and FromQcqpValue<1>.");
  static_assert(internal::HasQcqpVectorDim<T>::value,
                "fromQcqpValue requires traits<T>::QcqpVectorDim.");
  return traits<T>::template FromQcqpValue<1>(qcqpValue);
}

/** Extract all matching exact D=1 QCQP vectors as typed Values. */
template <typename T>
Values extractQcqpValues(const Values& qcqpValues) {
  static_assert(internal::HasQcqpExtractionTraits<T, 1>::value,
                "extractQcqpValues requires traits<T>::QcqpValue<1>, "
                "QcqpConstraints<1>, and FromQcqpValue<1>.");
  static_assert(internal::HasQcqpVectorDim<T>::value,
                "extractQcqpValues requires traits<T>::QcqpVectorDim.");
  Values typedValues;
  for (const auto& [key, typedValue] : ExtractQcqpValues<T, 1>(qcqpValues)) {
    typedValues.insert(key, typedValue);
  }
  return typedValues;
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
    if (columnDimension == 0) {
      throw std::invalid_argument(
          "QcqpProblem: columnDimension must be positive.");
    }
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

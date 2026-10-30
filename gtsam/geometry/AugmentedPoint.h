/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file AugmentedPoint.h
 * @brief N-Dimensional Euclidean point with a lifted QCQP representation.
 * @author Oluwatodimu Afolayan
 * @author Frederike Dümbgen
 * @author Frank Dellaert
 */
#pragma once

#include <gtsam/base/VectorSpace.h>
#include <gtsam/constrained/QcqpProblem.h>

#include <Eigen/Core>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

namespace gtsam {

/**
 * An N-dimensional Euclidean point supporting a quadratic QCQP lift.
 *
 * AugmentedPoint<N> represents the same Euclidean coordinates as a standard
 * fixed-size Eigen vector (e.g., Point3 for N=3). Both can be used with
 * conventional nonlinear optimization in GTSAM.
 *
 * Unlike Point3, AugmentedPoint<N> also provides QCQP traits that allow
 * compatible factors to be converted into a quadratically constrained
 * quadratic program (QCQP), which can subsequently be relaxed to an SDP.
 * Use Point3 for ordinary nonlinear optimization and AugmentedPoint<3>
 * when QCQP conversion is required.
 *
 * Its tangent space has dimension N. For D=1, its lifted representation is
 * X = [h, x.transpose(), u].transpose(), where the canonical lift has h = 1
 * and u = x.dot(x). The N+2 QCQP coordinates are not tangent coordinates.
 *
 * @tparam N Number of Euclidean coordinates (strictly positive).
 * @ingroup geometry
 */
template <int N>
class AugmentedPoint {
 public:
  static_assert(N > 0, "AugmentedPoint requires N > 0");

  using VectorType = Eigen::Matrix<double, N, 1>;
  static constexpr int dimension = N;  ///< Euclidean tangent dimension.

  /// Construct the origin.
  AugmentedPoint() : point_(VectorType::Zero()) {}

  /// Construct from a fixed-size Euclidean coordinate vector.
  explicit AugmentedPoint(const VectorType& point) : point_(point) {}

  /// Construct from an Eigen vector expression, checking its dimensions.
  template <typename Derived>
  explicit AugmentedPoint(const Eigen::MatrixBase<Derived>& point) {
    if (point.rows() != N || point.cols() != 1) {
      throw std::invalid_argument("AugmentedPoint: invalid point dimension.");
    }
    point_ = point;
  }

  /// Return the additive identity.
  static AugmentedPoint Identity() { return AugmentedPoint(); }

  /// Add two points in the underlying vector space.
  AugmentedPoint operator+(const AugmentedPoint& other) const {
    return AugmentedPoint(point_ + other.point_);
  }

  /// Add an Eigen tangent vector or vector expression of dimension N.
  template <typename Derived>
  AugmentedPoint operator+(const Eigen::MatrixBase<Derived>& delta) const {
    if (delta.rows() != N || delta.cols() != 1) {
      throw std::invalid_argument(
          "AugmentedPoint: tangent dimension mismatch.");
    }
    return AugmentedPoint(point_ + delta);
  }

  /// Subtract two points in the underlying vector space.
  AugmentedPoint operator-(const AugmentedPoint& other) const {
    return AugmentedPoint(point_ - other.point_);
  }

  /// Return the additive inverse.
  AugmentedPoint operator-() const { return AugmentedPoint(-point_); }

  /// Multiply by a scalar on the right.
  AugmentedPoint operator*(double scalar) const {
    return AugmentedPoint(point_ * scalar);
  }

  /// Multiply by a scalar on the left.
  friend AugmentedPoint operator*(double scalar, const AugmentedPoint& point) {
    return AugmentedPoint(scalar * point.point_);
  }

  /// Return the Euclidean dimension.
  int dim() const { return N; }

  /// Return the Euclidean norm.
  double norm() const { return point_.norm(); }

  /// Return the Euclidean inner product.
  double dot(const AugmentedPoint& other) const {
    return point_.dot(other.point_);
  }

  /// Return the underlying Euclidean coordinates.
  VectorType vector() const { return point_; }

  /// Print the coordinates, optionally preceded by a label.
  void print(const std::string& s = "") const {
    if (!s.empty()) {
      std::cout << s << " ";
    }
    std::cout << point_.transpose() << '\n';
  }

  /// Compare coordinates using a component-wise absolute tolerance.
  bool equals(const AugmentedPoint& other, double tol = 1e-9) const {
    return (point_ - other.point_).cwiseAbs().maxCoeff() <= tol;
  }

 private:
  VectorType point_;  ///< Euclidean point coordinates.
};

/**
 * GTSAM vector-space traits and D=1 QCQP lift for AugmentedPoint<N>.
 *
 * The lifted variable X = [h, x.transpose(), u].transpose() satisfies
 * x.dot(x) - h*u = 0 and h*h = 1. Its canonical form has h = 1; the quadratic
 * normalization permits h = -1 as well, which is handled during extraction.
 *
 * @tparam N Euclidean point dimension.
 */
template <int N>
struct traits<AugmentedPoint<N>>
    : public internal::VectorSpace<AugmentedPoint<N>> {
  using Point = AugmentedPoint<N>;
  using VectorType = typename Point::VectorType;

  /// Number of rows in the D=1 lifted QCQP variable.
  static constexpr int QcqpVectorDim = N + 2;

  /**
   * Lift a point to [1, x.transpose(), x.dot(x)].transpose().
   *
   * @tparam D Number of QCQP columns; only D=1 is supported.
   * @throws std::invalid_argument for D other than 1.
   */
  template <int D = 1>
  static Matrix QcqpValue(const Point& value) {
    if constexpr (D == 1) {
      Eigen::Matrix<double, QcqpVectorDim, 1> X;
      X(0) = 1.0;
      X.template segment<N>(1) = value.vector();
      X(N + 1) = value.dot(value);
      return X;
    } else {
      throw std::invalid_argument(
          "traits<AugmentedPoint>::QcqpValue only supports D=1.");
    }
  }

  /**
   * Return the two quadratic equalities as (A, a, b) triples.
   *
   * Each tuple represents X.transpose()*A*X + 2*a.dot(X) = b. The first
   * equality enforces x.dot(x) = h*u; the second enforces h*h = 1.
   * Both use an empty linear term a.
   *
   * @tparam D Number of QCQP columns; only D=1 is supported.
   * @throws std::invalid_argument for D other than 1.
   */
  template <int D = 1>
  static std::vector<std::tuple<Matrix, Vector, double>> QcqpConstraints() {
    if constexpr (D == 1) {
      std::vector<std::tuple<Matrix, Vector, double>> constraints;
      constraints.reserve(2);

      Matrix liftConstraint = Matrix::Zero(QcqpVectorDim, QcqpVectorDim);
      liftConstraint.block(1, 1, N, N).setIdentity();
      liftConstraint(0, N + 1) = -0.5;
      liftConstraint(N + 1, 0) = -0.5;
      constraints.emplace_back(liftConstraint, Vector(), 0.0);

      Matrix normalization = Matrix::Zero(QcqpVectorDim, QcqpVectorDim);
      normalization(0, 0) = 1.0;
      constraints.emplace_back(normalization, Vector(), 1.0);
      return constraints;
    } else {
      throw std::invalid_argument(
          "traits<AugmentedPoint>::QcqpConstraints only supports D=1.");
    }
  }

  /**
   * Recover an AugmentedPoint from a D=1 QCQP column.
   *
   * The recovered Euclidean coordinates are x/h. This removes the global
   * sign ambiguity in the homogeneous representation, provided h is nonzero.
   *
   * @param X Lifted (N+2)-by-1 column.
   * @throws std::invalid_argument for unsupported D, invalid dimensions,
   *         or a non-finite or near-zero homogenization coordinate.
   */
  template <int D = 1>
  static Point FromQcqpValue(const Matrix& X) {
    if constexpr (D == 1) {
      if (X.rows() != QcqpVectorDim || X.cols() != 1) {
        throw std::invalid_argument(
            "traits<AugmentedPoint>::FromQcqpValue: " + std::to_string(N) +
            "-dimensional AugmentedPoint requires a " +
            std::to_string(QcqpVectorDim) + "-by-1 vector.");
      }

      const double h = X(0, 0);
      if (!std::isfinite(h) || std::abs(h) < 1e-9) {
        throw std::invalid_argument(
            "traits<AugmentedPoint>::FromQcqpValue requires a finite, "
            "nonzero homogenization coordinate.");
      }
      const VectorType coordinates = X.col(0).segment<N>(1) / h;
      return Point(coordinates);
    } else {
      throw std::invalid_argument(
          "traits<AugmentedPoint>::FromQcqpValue only supports D=1.");
    }
  }
};

/// Preserve vector-space traits for const-qualified points.
template <int N>
struct traits<const AugmentedPoint<N>> : public traits<AugmentedPoint<N>> {};

/// Return the exact QCQP column for an N-dimensional augmented point.
template <int N>
Matrix qcqpValue(const AugmentedPoint<N>& typedValue) {
  return qcqpValue<AugmentedPoint<N>>(typedValue);
}

/// Insert an N-dimensional augmented point as an exact QCQP column.
template <int N>
void insertQcqpValue(Key key, const AugmentedPoint<N>& typedValue,
                     Values& qcqpValues) {
  insertQcqpValue<AugmentedPoint<N>>(key, typedValue, qcqpValues);
}

/// Recover an N-dimensional augmented point from an exact QCQP column.
template <int N>
AugmentedPoint<N> fromQcqpValue(const Matrix& qcqpValue) {
  return fromQcqpValue<AugmentedPoint<N>>(qcqpValue);
}

/// Recover N-dimensional augmented points from matching exact QCQP columns.
template <int N>
Values extractQcqpValues(const Values& qcqpValues) {
  return extractQcqpValues<AugmentedPoint<N>>(qcqpValues);
}

}  // namespace gtsam

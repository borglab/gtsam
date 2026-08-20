/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Unit2.h
 * @brief   Direction in 2D, a point on the unit circle S^1.
 */

#pragma once

#include <gtsam/base/Manifold.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point2.h>

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace gtsam {

/**
 * Direction in 2D: a unit vector on S^1, the planar sibling of `Unit3`.
 *
 * Not completely functional like Unit3, mainly for supporting the certifiable
 * range-aided SLAM.
 */
class Unit2 {
 private:
  Point2 p_;  ///< unit-norm direction

 public:
  inline constexpr static auto dimension = 1;

  /// Default is the +x direction.
  Unit2() : p_(1.0, 0.0) {}

  /// Construct from a nonzero vector, normalizing it.
  explicit Unit2(const Point2& direction) {
    const double norm = direction.norm();
    if (norm == 0.0) {
      throw std::invalid_argument("Unit2: direction must be nonzero.");
    }
    p_ = direction / norm;
  }

  /// Construct from an angle in radians.
  static Unit2 FromAngle(double theta) {
    return Unit2(Point2(std::cos(theta), std::sin(theta)));
  }

  const Point2& unitVector() const { return p_; }
  double angle() const { return std::atan2(p_.y(), p_.x()); }

  void print(const std::string& s = std::string()) const {
    std::cout << s << " : " << p_.transpose() << std::endl;
  }

  bool equals(const Unit2& other, double tol = 1e-9) const {
    return (p_ - other.p_).cwiseAbs().maxCoeff() < tol;
  }

  /// Rotate the direction by `v(0)` radians.
  Unit2 retract(const Vector1& v) const { return FromAngle(angle() + v(0)); }

  /// Signed angle from this direction to `other`.
  Vector1 localCoordinates(const Unit2& other) const {
    double delta = other.angle() - angle();
    while (delta > M_PI) delta -= 2.0 * M_PI;
    while (delta < -M_PI) delta += 2.0 * M_PI;
    return Vector1(delta);
  }
};

/**
 * QCQP traits for the unit circle S^1.
 *
 * A direction lifts to a single row of length D, and the only constraint is
 * `trace(X' A X) = 1` with `A = I_1`, i.e. `||X||^2 = 1`. Both the row count
 * and the constraint are identical to `Unit3`'s: the lifted sphere does not
 * depend on the ambient dimension, only recovery does.
 */
template <>
struct traits<Unit2> : public internal::Manifold<Unit2> {
  /// Dimension of the D=1 homogenized QCQP vector: [1; p].
  inline constexpr static int QcqpVectorDim = 3;

  /// Lift a direction: a 3-by-1 homogenized column at D=1, otherwise a 1-by-D
  /// row zero-padded above the ambient dimension.
  template <int D = 1>
  static Matrix QcqpValue(const Unit2& value) {
    if constexpr (D == 1) {
      Eigen::Matrix<double, 3, 1> X;
      X(0, 0) = 1.0;
      X.segment<2>(1) = value.unitVector();
      return X;
    } else if constexpr (D >= 2) {
      Matrix X = Matrix::Zero(1, D);
      X.block(0, 0, 1, 2) = value.unitVector().transpose();
      return X;
    } else {
      throw std::invalid_argument(
          "traits<Unit2>::QcqpValue supports D=1 and D>=2.");
    }
  }

  /// The single unit-norm constraint, `||X||^2 = 1`.
  template <int D = 1>
  static std::vector<std::pair<Matrix, double>> QcqpConstraints() {
    if constexpr (D == 1) {
      std::vector<std::pair<Matrix, double>> constraints;
      Matrix A = Matrix::Zero(3, 3);
      A(0, 0) = 1.0;
      constraints.emplace_back(A, 1.0);
      A.setZero();
      A(1, 1) = A(2, 2) = 1.0;
      constraints.emplace_back(A, 1.0);
      return constraints;
    } else if constexpr (D >= 2) {
      return {{Matrix::Identity(1, 1), 1.0}};
    } else {
      throw std::invalid_argument(
          "traits<Unit2>::QcqpConstraints supports D=1 and D>=2.");
    }
  }

  /// Recover a direction by normalizing the leading two entries.
  template <int D = 1>
  static Unit2 FromQcqpValue(const Matrix& X) {
    if constexpr (D == 1) {
      if (X.rows() != QcqpVectorDim || X.cols() != 1) {
        throw std::invalid_argument(
            "traits<Unit2>::FromQcqpValue requires a 3-by-1 matrix.");
      }
      return Unit2(Point2(X(1, 0), X(2, 0)));
    } else if constexpr (D >= 2) {
      if (X.rows() != 1 || X.cols() != D) {
        throw std::invalid_argument(
            "traits<Unit2>::FromQcqpValue requires a 1-by-D matrix.");
      }
      return Unit2(Point2(X(0, 0), X(0, 1)));
    } else {
      throw std::invalid_argument(
          "traits<Unit2>::FromQcqpValue supports D=1 and D>=2.");
    }
  }
};

template <>
struct traits<const Unit2> : public internal::Manifold<Unit2> {};

}  // namespace gtsam

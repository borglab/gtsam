/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    QuadraticRangeFactor.h
 * @brief   Quadratic range factor. Ternary factor over the two positions t_i
 *          and target, plus one auxiliary direction u per measurement. The raw
 *          residual (||target - t_i|| - range)^2 is not polynomial and cannot
 *          enter a QCQP; the auxiliary direction makes the term quadratic:
 *
 *            min          nu ||target - t_i - range * u||^2
 *            over         t_i, target in R^d,  u in R^d
 *            subject to   ||u||^2 = 1
 *
 *          Its minimum over the unit sphere is the raw residual, so the
 *          reformulation is exact (see [1]).
 *
 *          The auxiliary uses Rot2 in 2D and Unit3 in 3D; in 2D the unit
 *          vector is the rotation's first column, u = R * e_1.
 * @author  Zhexin Xu
 *
 * REFERENCES:
 * [1] T. Halsted, M. Schwager, "The Riemannian Elevator for Certifiable
 *     Distance-Based Localization", 2022.
 *     Available: https://msl.stanford.edu/papers/halsted_riemannian_2022.pdf
 * [2] A. Papalia, A. Fishberg, B. W. O'Neill, J. P. How, D. M. Rosen,
 *     J. J. Leonard, "Certifiably Correct Range-Aided SLAM", IEEE Transactions
 *     on Robotics, 40:4265-4283, 2024.
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/constrained/QcqpProblem.h>
#include <gtsam/constrained/QpCost.h>
#include <gtsam/constrained/QuadraticConstraint.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <cmath>
#include <stdexcept>
#include <string>
#include <type_traits>

namespace gtsam {

template <int d>
class QuadraticRangeFactor
    : public NoiseModelFactorN<
          Eigen::Matrix<double, d, 1>, Eigen::Matrix<double, d, 1>,
          typename std::conditional<d == 2, Rot2, Unit3>::type> {
  static_assert(d == 2 || d == 3,
                "QuadraticRangeFactor supports d = 2 or 3.");

  using Point = Eigen::Matrix<double, d, 1>;
  /// Lifted constraints come from the type's own QCQP traits.
  using Direction = typename std::conditional<d == 2, Rot2, Unit3>::type;
  using Base = NoiseModelFactorN<Point, Point, Direction>;

  double range_;   ///< measured distance
  double weight_;  ///< measurement precision, nu

 public:
  /// Rows the lifted direction occupies: a Rot2 frame has two, only the first
  /// of which carries the direction; a Unit3 has one.
  static constexpr int kDirectionRows = (d == 2) ? 2 : 1;

  using Base::evaluateError;

  /**
   * @param translationKey Translation of the pose the range is measured from.
   * @param targetKey Landmark or position the range is measured to.
   * @param unitVectorKey Auxiliary direction for this measurement.
   * @param range Measured distance.
   * @param weight Measurement precision, nu.
   */
  QuadraticRangeFactor(Key translationKey, Key targetKey, Key unitVectorKey,
                       double range, double weight)
      : Base(noiseModel::Unit::Create(d), translationKey, targetKey,
             unitVectorKey),
        range_(range),
        weight_(weight) {
    if (weight <= 0.0) {
      throw std::invalid_argument(
          "QuadraticRangeFactor: weight must be positive.");
    }
    if (range < 0.0) {
      throw std::invalid_argument(
          "QuadraticRangeFactor: range must be non-negative.");
    }
  }

  double range() const { return range_; }
  double weight() const { return weight_; }

  void print(const std::string& s = "",
             const KeyFormatter& keyFormatter =
                 DefaultKeyFormatter) const override {
    std::cout << s << "QuadraticRangeFactor(" << keyFormatter(this->key1())
              << "," << keyFormatter(this->key2()) << ","
              << keyFormatter(this->key3()) << ") range=" << range_
              << " weight=" << weight_ << "\n";
  }

  bool equals(const NonlinearFactor& other, double tol = 1e-9) const override {
    const auto* e = dynamic_cast<const QuadraticRangeFactor*>(&other);
    return e != nullptr && Base::equals(other, tol) &&
           std::abs(range_ - e->range_) < tol &&
           std::abs(weight_ - e->weight_) < tol;
  }

  /// Weighted residual `sqrt(weight) * (target - t_i - range * u)`.
  /// In "d == 2", we use Rot2 to replace Unit2
  Vector evaluateError(const Point& translation, const Point& target,
                       const Direction& direction, OptionalMatrixType H1,
                       OptionalMatrixType H2,
                       OptionalMatrixType H3) const override {
    const double sw = std::sqrt(weight_);
    const Point u = [&direction] {
      if constexpr (d == 2) return Point(direction.c(), direction.s());
      else return direction.unitVector();
    }();
    if (H1) *H1 = -sw * Matrix::Identity(d, d);
    if (H2) *H2 = sw * Matrix::Identity(d, d);
    if (H3) {
      Matrix J = Matrix::Zero(d, Direction::dimension);
      if constexpr (d == 2) {
        J(0, 0) = -u(1);
        J(1, 0) = u(0);
      } else {
        J = direction.basis();
      }
      *H3 = -sw * range_ * J;
    }
    return sw * (target - translation - range_ * u);
  }

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(
            new QuadraticRangeFactor(*this)));
  }

  /// Add this range factor as a QCQP cost when traits exist.
  void qcqpFactors(NonlinearFactorGraph* costs,
                   NonlinearEqualityConstraints* constraints,
                   size_t columnDimension = 1) const override {
    if (columnDimension < static_cast<size_t>(d)) {
      throw std::invalid_argument(
          "QuadraticRangeFactor::qcqpFactors requires columnDimension "
          ">= d.");
    }
    if (!costs) {
      throw std::invalid_argument(
          "QuadraticRangeFactor::qcqpFactors: costs is null.");
    }

    InsertQcqpConstraints<Direction, d>(this->key3(), constraints);

    // The direction is a Rot2 in 2D, which takes 2 rows, and a Unit3 in 3D,
    // which takes 1. Only the first row is used; any others stay zero.
    const double sw = std::sqrt(weight_);
    Matrix B = Matrix::Zero(1, 2 + kDirectionRows);
    B(0, 0) = -sw;            // pose translation
    B(0, 1) = sw;             // target
    B(0, 2) = -sw * range_;   // leading row of the auxiliary direction

    const Matrix Q = B.transpose() * B;
    const SymmetricBlockMatrix blockQ(
        std::vector<DenseIndex>{1, 1, kDirectionRows}, Q);
    costs->push_back(std::make_shared<QpCost>(
        KeyVector{this->key1(), this->key2(), this->key3()}, blockQ,
        columnDimension));
  }
};

using QuadraticRangeFactor2 = QuadraticRangeFactor<2>;
using QuadraticRangeFactor3 = QuadraticRangeFactor<3>;

}  // namespace gtsam

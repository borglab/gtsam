/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file PointCorrespondenceFactor.h
 * @brief Factor for a known point correspondence under an unknown transform.
 */

#pragma once

#include <gtsam/constrained/QcqpProblem.h>
#include <gtsam/constrained/QpCost.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>

#include <memory>
#include <stdexcept>
#include <vector>

namespace gtsam {

namespace internal {

/** Dispatches the transformation of a point by a supported group type. */
template <class T>
struct TransformPoint;

/** Transforms a Point2 by a Rot2. */
template <>
struct TransformPoint<Rot2> {
  using Point = Point2;

  /// Apply the rotation and optionally compute its Jacobian.
  static Point apply(const Rot2& transform, const Point& point,
                     OptionalMatrixType H) {
    return transform.rotate(point, H);
  }
};

/** Transforms a Point3 by a Rot3. */
template <>
struct TransformPoint<Rot3> {
  using Point = Point3;

  /// Apply the rotation and optionally compute its Jacobian.
  static Point apply(const Rot3& transform, const Point& point,
                     OptionalMatrixType H) {
    return transform.rotate(point, H);
  }
};

/** Transforms a Point2 by a Pose2. */
template <>
struct TransformPoint<Pose2> {
  using Point = Point2;

  /// Apply the pose and optionally compute its Jacobian.
  static Point apply(const Pose2& transform, const Point& point,
                     OptionalMatrixType H) {
    return transform.transformFrom(point, H);
  }
};

/** Transforms a Point3 by a Pose3. */
template <>
struct TransformPoint<Pose3> {
  using Point = Point3;

  /// Apply the pose and optionally compute its Jacobian.
  static Point apply(const Pose3& transform, const Point& point,
                     OptionalMatrixType H) {
    return transform.transformFrom(point, H);
  }
};

}  // namespace internal

/**
 * A unary factor for a known point correspondence under an unknown transform.
 * The unwhitened residual is
 *
 *   e(T) = T * sourcePoint - measuredPoint.
 *
 * For rotation groups, this is a vector correspondence term in the standard
 * or matrix-weighted Wahba problem.
 *
 * The source point is fixed data rather than a variable. Full anisotropic
 * measurement weights are represented by the supplied noise model.
 */
template <class T>
class PointCorrespondenceFactor : public NoiseModelFactorN<T> {
 public:
  using Base = NoiseModelFactorN<T>;
  using Point = typename internal::TransformPoint<T>::Point;

 private:
  Point sourcePoint_;    ///< Known point before transformation.
  Point measuredPoint_;  ///< Corresponding point after transformation.

 public:
  // Provide access to the Matrix& version of evaluateError.
  using Base::evaluateError;

  /// Construct from a key, source point, measured point, and noise model.
  PointCorrespondenceFactor(Key key, const Point& sourcePoint,
                            const Point& measuredPoint,
                            const SharedNoiseModel& model)
      : Base(model, key),
        sourcePoint_(sourcePoint),
        measuredPoint_(measuredPoint) {}

  /// Evaluate T * sourcePoint - measuredPoint and its optional Jacobian.
  Vector evaluateError(const T& transform,
                       OptionalMatrixType H) const override {
    const Point predictedPoint =
        internal::TransformPoint<T>::apply(transform, sourcePoint_, H);
    return predictedPoint - measuredPoint_;
  }

  /** Add the exact D=1 homogeneous point-correspondence cost to a QCQP. */
  void qcqpFactors(NonlinearFactorGraph* costs,
                   NonlinearEqualityConstraints* constraints,
                   size_t columnDimension = 1) const override {
    if (columnDimension != 1) {
      throw std::invalid_argument(
          "PointCorrespondenceFactor::qcqpFactors only supports column "
          "dimension 1");
    }
    if (!costs) {
      throw std::invalid_argument(
          "PointCorrespondenceFactor::qcqpFactors costs is null");
    }
    if (!this->noiseModel_ ||
        std::dynamic_pointer_cast<noiseModel::Robust>(this->noiseModel_) ||
        this->noiseModel_->isConstrained()) {
      throw std::runtime_error(
          "PointCorrespondenceFactor::qcqpFactors requires a non-null, "
          "non-robust/non-hard quadratic noise model");
    }

    constexpr int PointDim = Point::RowsAtCompileTime;
    constexpr int N = T::LieAlgebra::RowsAtCompileTime;
    constexpr int ActiveDim = PointDim * N;
    constexpr int LiftedDim = ActiveDim + 1;
    static_assert(N == PointDim || N == PointDim + 1,
                  "Unsupported transform and point dimensions");

    Matrix B = Matrix::Zero(PointDim, LiftedDim);
    B.col(0) = -measuredPoint_;
    for (int column = 0; column < N; ++column) {
      const double coefficient =
          column < PointDim ? sourcePoint_(column) : 1.0;
      B.block(0, 1 + column * PointDim, PointDim, PointDim)
          .diagonal()
          .setConstant(coefficient);
    }

    const Matrix whitenedB = this->noiseModel_->Whiten(B);
    const Matrix Q = whitenedB.transpose() * whitenedB;

    InsertQcqpConstraints<T, 1>(this->key(), constraints);
    const SymmetricBlockMatrix blockQ(std::vector<DenseIndex>{LiftedDim}, Q);
    costs->push_back(
        std::make_shared<QpCost>(KeyVector{this->key()}, blockQ));
  }
};

// TODO: Add clone, print, equals, accessors, and serialization if future
// examples require the fuller factor interface.

}  // namespace gtsam

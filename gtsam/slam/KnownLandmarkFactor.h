/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file KnownLandmarkFactor.h
 * @brief Factors for observing a known landmark under an unknown transform.
 * @author Avinash Subramanian
 * @author Frederike Dümbgen
 * @author Frank Dellaert
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

/** Predict a frame-k point from a conventional world-from-k state. */
template <class T>
struct KnownLandmarkPrediction;

template <>
struct KnownLandmarkPrediction<Rot2> {
  using Point = Point2;

  static Point apply(const Rot2& wRk, const Point& wP, OptionalMatrixType H) {
    return wRk.unrotate(wP, H);
  }
};

template <>
struct KnownLandmarkPrediction<Rot3> {
  using Point = Point3;

  static Point apply(const Rot3& wRk, const Point& wP, OptionalMatrixType H) {
    return wRk.unrotate(wP, H);
  }
};

template <>
struct KnownLandmarkPrediction<Pose2> {
  using Point = Point2;

  static Point apply(const Pose2& wTk, const Point& wL, OptionalMatrixType H) {
    return wTk.transformTo(wL, H);
  }
};

template <>
struct KnownLandmarkPrediction<Pose3> {
  using Point = Point3;

  static Point apply(const Pose3& wTk, const Point& wL, OptionalMatrixType H) {
    return wTk.transformTo(wL, H);
  }
};

}  // namespace internal

/**
 * A unary known-landmark factor using the conventional GTSAM state direction.
 *
 * For a pose wTk that maps frame k into the world frame, the residual is
 *
 *   e(wTk) = wTk.transformTo(wL) - measured_kP.
 *
 * The rotation-only equivalent uses wRk.unrotate(wP). The known world point is
 * fixed data, and the supplied noise model represents the measurement weight.
 * This conventional formulation does not provide an exact QCQP conversion.
 */
template <class T>
class KnownLandmarkFactor : public NoiseModelFactorN<T> {
 public:
  using Base = NoiseModelFactorN<T>;
  using Point = typename internal::KnownLandmarkPrediction<T>::Point;

 private:
  Point wP_;           ///< Known point expressed in the world frame.
  Point measured_kP_;  ///< Corresponding point measured in frame k.

 public:
  using Base::evaluateError;

  /// Construct from a key, world point, frame-k measurement, and noise model.
  KnownLandmarkFactor(Key key, const Point& wP, const Point& measured_kP,
                      const SharedNoiseModel& model)
      : Base(model, key), wP_(wP), measured_kP_(measured_kP) {}

  /** Evaluate the conventional world-from-k prediction minus measured_kP. */
  Vector evaluateError(const T& wTk, OptionalMatrixType H) const override {
    const Point predicted_kP =
        internal::KnownLandmarkPrediction<T>::apply(wTk, wP_, H);
    return predicted_kP - measured_kP_;
  }
};

namespace internal {

/** Predict a frame-k point from a k-from-world state. */
template <class T>
struct KnownLandmarkPrediction2;

template <>
struct KnownLandmarkPrediction2<Rot2> {
  using Point = Point2;

  static Point apply(const Rot2& kRw, const Point& wP, OptionalMatrixType H) {
    return kRw.rotate(wP, H);
  }
};

template <>
struct KnownLandmarkPrediction2<Rot3> {
  using Point = Point3;

  static Point apply(const Rot3& kRw, const Point& wP, OptionalMatrixType H) {
    return kRw.rotate(wP, H);
  }
};

template <>
struct KnownLandmarkPrediction2<Pose2> {
  using Point = Point2;

  static Point apply(const Pose2& kTw, const Point& wL, OptionalMatrixType H) {
    return kTw.transformFrom(wL, H);
  }
};

template <>
struct KnownLandmarkPrediction2<Pose3> {
  using Point = Point3;

  static Point apply(const Pose3& kTw, const Point& wL, OptionalMatrixType H) {
    return kTw.transformFrom(wL, H);
  }
};

}  // namespace internal

/**
 * A unary known-landmark factor using a k-from-world state direction.
 *
 * For a pose kTw that maps world coordinates into frame k, the residual is
 *
 *   e(kTw) = kTw * wL - measured_kP.
 *
 * This direction makes the residual affine in homogeneous matrix entries and
 * supports the exact D=1 QCQP conversion used by certifiable localization.
 */
template <class T>
class KnownLandmarkFactor2 : public NoiseModelFactorN<T> {
 public:
  using Base = NoiseModelFactorN<T>;
  using Point = typename internal::KnownLandmarkPrediction2<T>::Point;

 private:
  Point wP_;           ///< Known point expressed in the world frame.
  Point measured_kP_;  ///< Corresponding point measured in frame k.

 public:
  using Base::evaluateError;

  /// Construct from a key, world point, frame-k measurement, and noise model.
  KnownLandmarkFactor2(Key key, const Point& wP, const Point& measured_kP,
                       const SharedNoiseModel& model)
      : Base(model, key), wP_(wP), measured_kP_(measured_kP) {}

  /** Evaluate the k-from-world prediction minus measured_kP. */
  Vector evaluateError(const T& kTw, OptionalMatrixType H) const override {
    const Point predicted_kP =
        internal::KnownLandmarkPrediction2<T>::apply(kTw, wP_, H);
    return predicted_kP - measured_kP_;
  }

  /** Add the exact D=1 homogeneous known-landmark cost to a QCQP. */
  void qcqpFactors(NonlinearFactorGraph* costs,
                   NonlinearEqualityConstraints* constraints,
                   size_t columnDimension = 1) const override {
    if (columnDimension != 1) {
      throw std::invalid_argument(
          "KnownLandmarkFactor2::qcqpFactors only supports column dimension "
          "1");
    }
    if (!costs) {
      throw std::invalid_argument(
          "KnownLandmarkFactor2::qcqpFactors costs is null");
    }
    if (!this->noiseModel_ ||
        std::dynamic_pointer_cast<noiseModel::Robust>(this->noiseModel_) ||
        this->noiseModel_->isConstrained()) {
      throw std::runtime_error(
          "KnownLandmarkFactor2::qcqpFactors requires a non-null, "
          "non-robust/non-hard quadratic noise model");
    }

    constexpr int PointDim = Point::RowsAtCompileTime;
    constexpr int N = T::LieAlgebra::RowsAtCompileTime;
    constexpr int LiftedDim = 1 + PointDim * N;
    static_assert(N == PointDim || N == PointDim + 1,
                  "Unsupported transform and point dimensions");

    Matrix B = Matrix::Zero(PointDim, LiftedDim);
    B.col(0) = -measured_kP_;
    for (int column = 0; column < N; ++column) {
      const double coefficient = column < PointDim ? wP_(column) : 1.0;
      B.block(0, 1 + column * PointDim, PointDim, PointDim)
          .diagonal()
          .setConstant(coefficient);
    }

    const Matrix whitenedB = this->noiseModel_->Whiten(B);
    const Matrix Q = whitenedB.transpose() * whitenedB;

    InsertQcqpConstraints<T, 1>(this->key(), constraints);
    const SymmetricBlockMatrix blockQ(std::vector<DenseIndex>{LiftedDim}, Q);
    costs->push_back(std::make_shared<QpCost>(KeyVector{this->key()}, blockQ));
  }
};

// TODO: Add clone, print, equals, accessors, and serialization if future
// examples require the fuller factor interface.

}  // namespace gtsam

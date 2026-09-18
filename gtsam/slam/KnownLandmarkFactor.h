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
#include <gtsam/nonlinear/NoiseModelFactorN.h>

#include <memory>
#include <stdexcept>
#include <type_traits>
#include <vector>

namespace gtsam {

/**
 * A unary known-landmark factor using the conventional GTSAM state direction.
 *
 * For a pose wTk that maps frame k into the world frame, the residual is
 *
 *   e(wTk) = wTk.transformTo(wL) - measured_kP.
 *
 * The known world landmark is fixed data, and the supplied noise model
 * represents the measurement weight. This conventional formulation does not
 * provide an exact QCQP conversion.
 *
 * See `gtsam/slam/doc/KnownLandmarkFactor.ipynb` for frame conventions,
 * wrapper names, and comparison with KnownLandmarkFactor2.
 */
template <class T>
class KnownLandmarkFactor : public NoiseModelFactorN<T> {
 public:
  static_assert(std::is_same_v<T, Pose2> || std::is_same_v<T, Pose3>,
                "KnownLandmarkFactor supports only Pose2 and Pose3");

  using Base = NoiseModelFactorN<T>;
  using This = KnownLandmarkFactor<T>;
  using Point = typename T::Translation;

 private:
  Point wL_;           ///< Known landmark expressed in the world frame.
  Point measured_kP_;  ///< Corresponding point measured in frame k.

 public:
  using Base::evaluateError;

  /// Construct from a key, world landmark, frame-k measurement, and noise model.
  KnownLandmarkFactor(Key key, const Point& wL, const Point& measured_kP,
                      const SharedNoiseModel& model)
      : Base(model, key), wL_(wL), measured_kP_(measured_kP) {}

  NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<This>(*this);
  }

  bool equals(const NonlinearFactor& expected,
              double tol = 1e-9) const override {
    const auto* e = dynamic_cast<const This*>(&expected);
    return e != nullptr && Base::equals(*e, tol) &&
           traits<Point>::Equals(wL_, e->wL_, tol) &&
           traits<Point>::Equals(measured_kP_, e->measured_kP_, tol);
  }

  /** Evaluate the conventional world-from-k prediction minus measured_kP. */
  Vector evaluateError(const T& wTk, OptionalMatrixType H) const override {
    const Point predicted_kP = wTk.transformTo(wL_, H);
    return predicted_kP - measured_kP_;
  }
};

/**
 * A unary known-landmark factor using a k-from-world state direction.
 *
 * For a pose kTw that maps world coordinates into frame k, the residual is
 *
 *   e(kTw) = kTw * wL - measured_kP.
 *
 * This direction makes the residual affine in homogeneous matrix entries and
 * supports the exact D=1 QCQP conversion used by certifiable localization.
 *
 * See `gtsam/slam/doc/KnownLandmarkFactor.ipynb` for the exact QCQP mapping and
 * its relationship to the conventional factor.
 */
template <class T>
class KnownLandmarkFactor2 : public NoiseModelFactorN<T> {
 public:
  static_assert(std::is_same_v<T, Pose2> || std::is_same_v<T, Pose3>,
                "KnownLandmarkFactor2 supports only Pose2 and Pose3");

  using Base = NoiseModelFactorN<T>;
  using This = KnownLandmarkFactor2<T>;
  using Point = typename T::Translation;

 private:
  Point wL_;           ///< Known landmark expressed in the world frame.
  Point measured_kP_;  ///< Corresponding point measured in frame k.

 public:
  using Base::evaluateError;

  /// Construct from a key, world landmark, frame-k measurement, and noise model.
  KnownLandmarkFactor2(Key key, const Point& wL, const Point& measured_kP,
                       const SharedNoiseModel& model)
      : Base(model, key), wL_(wL), measured_kP_(measured_kP) {}

  NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<This>(*this);
  }

  bool equals(const NonlinearFactor& expected,
              double tol = 1e-9) const override {
    const auto* e = dynamic_cast<const This*>(&expected);
    return e != nullptr && Base::equals(*e, tol) &&
           traits<Point>::Equals(wL_, e->wL_, tol) &&
           traits<Point>::Equals(measured_kP_, e->measured_kP_, tol);
  }

  /** Evaluate the k-from-world prediction minus measured_kP. */
  Vector evaluateError(const T& kTw, OptionalMatrixType H) const override {
    const Point predicted_kP = kTw.transformFrom(wL_, H);
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
      const double coefficient = column < PointDim ? wL_(column) : 1.0;
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

}  // namespace gtsam

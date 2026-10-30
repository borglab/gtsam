/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file KnownLandmarkSquaredRangeFactor.h
 * @brief Squared-range observation factor with an optional lifted QCQP state.
 * @author Oluwatodimu Afolayan
 * @author Frederike Dümbgen
 * @author Frank Dellaert
 */
#pragma once

#include <gtsam/constrained/QpCost.h>
#include <gtsam/geometry/AugmentedPoint.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>

#include <Eigen/Core>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

namespace gtsam {

/**
 * Unary squared-range factor for a known Euclidean landmark.
 *
 * Given an unknown state x, known world landmark L, and measured range d,
 * the scalar residual is ||x-L||^2 - d^2. The supplied noise model weights
 * this squared-range residual (not the unsquared range).
 *
 * Supports fixed-size Eigen vectors (e.g., Point3) and AugmentedPoint<N>
 * as position types. Both support nonlinear optimization, but AugmentedPoint<N>
 * additionally enables exact D=1 QCQP conversion using N+2 lifted coordinates.
 * AugmentedPoint<N> is defined in gtsam/geometry/AugmentedPoint.h.
 *
 * @tparam Position Euclidean point representing robot location.
 * @ingroup slam
 *
 * See `gtsam/slam/doc/KnownLandmarkSquaredRangeFactor.ipynb` for usage.
 */
template <typename Position>
class KnownLandmarkSquaredRangeFactor : public NoiseModelFactorN<Position> {
 public:
  static constexpr int N = traits<Position>::dimension;
  static_assert(N > 0,
                "KnownLandmarkSquaredRangeFactor requires a fixed positive "
                "position dimension.");
  using VectorType = Eigen::Matrix<double, N, 1>;
  using Point = Position;
  using Base = NoiseModelFactorN<Position>;
  using This = KnownLandmarkSquaredRangeFactor<Position>;
  static_assert(std::is_same_v<Position, VectorType> ||
                    std::is_same_v<Position, AugmentedPoint<N>>,
                "KnownLandmarkSquaredRangeFactor requires an Eigen vector "
                "or AugmentedPoint<N> position.");

 private:
  VectorType wL_;                ///< Known landmark in world coordinates.
  double measuredRangeSquared_;  ///< Squared measured range (d^2).

  /// Validate landmark dimensions before conversion to fixed-size storage.
  template <typename Derived>
  static VectorType checkedLandmark(
      const Eigen::MatrixBase<Derived>& landmark) {
    if (landmark.rows() != N || landmark.cols() != 1) {
      throw std::invalid_argument(
          "KnownLandmarkSquaredRangeFactor: invalid landmark dimension.");
    }
    return VectorType(landmark);
  }

  /// Return underlying state coordinates for either supported state type.
  static VectorType coordinates(const Point& point) {
    if constexpr (std::is_same_v<Point, AugmentedPoint<N>>) {
      return point.vector();
    } else {
      return point;
    }
  }

 public:
  using Base::evaluateError;

  /**
   * Construct from a fixed-size known landmark and scalar range measurement.
   *
   * @param key Key associated with the unknown point.
   * @param wL Known landmark expressed in the world frame.
   * @param measuredRange Nonnegative measured range.
   * @param model Noise model for the scalar squared-range residual.
   * @throws std::invalid_argument if measuredRange is negative or non-finite.
   */
  KnownLandmarkSquaredRangeFactor(Key key, const VectorType& wL,
                                  double measuredRange,
                                  const SharedNoiseModel& model)
      : Base(model, key),
        wL_(wL),
        measuredRangeSquared_(measuredRange * measuredRange) {
    if (!std::isfinite(measuredRange) || measuredRange < 0.0) {
      throw std::invalid_argument(
          "KnownLandmarkSquaredRangeFactor: range must be finite and "
          "non-negative.");
    }
  }

  /**
   * Construct from an Eigen vector expression, including dynamic vectors.
   *
   * @throws std::invalid_argument if its dimension is not N or the range is
   *         negative or non-finite.
   */
  template <typename Derived>
  KnownLandmarkSquaredRangeFactor(Key key, const Eigen::MatrixBase<Derived>& wL,
                                  double measuredRange,
                                  const SharedNoiseModel& model)
      : KnownLandmarkSquaredRangeFactor(key, checkedLandmark(wL), measuredRange,
                                        model) {}

  /**
   * Construct from an AugmentedPoint landmark, regardless of the state type.
   *
   * The known landmark is stored as N Euclidean coordinates; no QCQP lift is
   * needed for fixed measurement data.
   *
   * @throws std::invalid_argument if measuredRange is negative or non-finite.
   */
  KnownLandmarkSquaredRangeFactor(Key key, const AugmentedPoint<N>& wL,
                                  double measuredRange,
                                  const SharedNoiseModel& model)
      : KnownLandmarkSquaredRangeFactor(key, wL.vector(), measuredRange,
                                        model) {}

  /// Return a deep copy of this factor.
  NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<This>(*this);
  }

  /// Print the key, known landmark, measured range, and noise model.
  void print(
      const std::string& s = "",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    std::cout << s << "KnownLandmarkSquaredRangeFactor on "
              << keyFormatter(this->key1()) << "\n";
    traits<VectorType>::Print(wL_, "  landmark: ");
    std::cout << "  measured range squared: " << measuredRangeSquared_ << "\n";
    if (this->noiseModel_) {
      this->noiseModel_->print("  noise model: ");
    } else {
      std::cout << "  no noise model\n";
    }
  }

  /// Compare keys, noise models, landmarks, and squared ranges.
  bool equals(const NonlinearFactor& expected,
              double tol = 1e-9) const override {
    const auto* other = dynamic_cast<const This*>(&expected);
    return other != nullptr && Base::equals(*other, tol) &&
           traits<VectorType>::Equals(wL_, other->wL_, tol) &&
           traits<double>::Equals(measuredRangeSquared_,
                                  other->measuredRangeSquared_, tol);
  }

  /**
   * Evaluate ||x-L||^2-d^2 and optionally its 1-by-N tangent Jacobian.
   *
   * The Jacobian is 2(x-L).transpose(), valid for either Euclidean state type.
   *
   * @param wX Unknown point in world coordinates.
   * @param H Optional 1-by-N Jacobian with respect to the state tangent vector.
   * @return A one-element vector containing the squared-range residual.
   */
  Vector evaluateError(const Point& wX, OptionalMatrixType H) const override {
    const VectorType difference = coordinates(wX) - wL_;
    const double error = difference.squaredNorm() - measuredRangeSquared_;
    if (H) {
      *H = 2.0 * difference.transpose();
    }
    return Vector1(error);
  }

  /**
   * Append this factor's exact D=1 QCQP quadratic cost and lift constraints.
   *
   * For X=[h,x,u], the scalar residual is B*X with
   * B=[||L||^2-d^2, -2*L.transpose(), 1]. Its whitened squared residual
   * contributes 0.5*X.transpose()*Q*X, with Q=B.transpose()*Sigma^-1*B.
   * GTSAM's HessianFactor adds a final constant slot, whose coefficient is
   * zero because h is already part of the lifted state.
   *
   * The method is supported only when Position = AugmentedPoint<N>.
   * The optional redundant-constraint output is forwarded to the point traits;
   * AugmentedPoint<N> does not currently define additional redundant
   * constraints.
   *
   * @throws std::invalid_argument for null costs or constraints,
   * columnDimension != 1, or a noise model whose dimension is not 1.
   * @throws std::runtime_error for a non-augmented state or an unsupported
   *         noise model (null, robust, or constrained).
   */
  void qcqpFactors(NonlinearFactorGraph* costs,
                   NonlinearEqualityConstraints* constraints,
                   NonlinearEqualityConstraints* redundantConstraints,
                   size_t columnDimension = 1) const override {
    if constexpr (!std::is_same_v<Point, AugmentedPoint<N>>) {
      throw std::runtime_error(
          "KnownLandmarkSquaredRangeFactor::qcqpFactors requires an "
          "AugmentedPoint<N> state.");
    } else {
      if (columnDimension != 1) {
        throw std::invalid_argument(
            "KnownLandmarkSquaredRangeFactor::qcqpFactors only supports D=1.");
      }
      if (!costs || !constraints) {
        throw std::invalid_argument(
            "KnownLandmarkSquaredRangeFactor::qcqpFactors: null output.");
      }
      if (!this->noiseModel_ ||
          std::dynamic_pointer_cast<noiseModel::Robust>(this->noiseModel_) ||
          this->noiseModel_->isConstrained()) {
        throw std::runtime_error(
            "KnownLandmarkSquaredRangeFactor::qcqpFactors requires a "
            "non-null, non-robust, unconstrained noise model.");
      }
      if (this->noiseModel_->dim() != 1) {
        throw std::invalid_argument(
            "KnownLandmarkSquaredRangeFactor::qcqpFactors requires a "
            "one-dimensional noise model.");
      }

      constexpr int LiftedDim = traits<Point>::QcqpVectorDim;
      // The last column is GTSAM's HessianFactor constant slot (coefficient 0).
      Matrix B = Matrix::Zero(1, LiftedDim + 1);
      B(0, 0) = wL_.squaredNorm() - measuredRangeSquared_;
      B.block(0, 1, 1, N) = -2.0 * wL_.transpose();
      B(0, N + 1) = 1.0;

      const Matrix whitenedB = this->noiseModel_->Whiten(B);
      const Matrix Q = whitenedB.transpose() * whitenedB;
      InsertQcqpConstraints<Point, 1>(this->key(), constraints,
                                      redundantConstraints);
      const SymmetricBlockMatrix blockQ(std::vector<DenseIndex>{LiftedDim, 1},
                                        Q);
      costs->push_back(std::make_shared<QpCost>(
          HessianFactor(KeyVector{this->key()}, blockQ)));
    }
  }
};

}  // namespace gtsam

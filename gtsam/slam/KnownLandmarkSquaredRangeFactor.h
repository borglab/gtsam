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

#include <gtsam/base/VectorSpace.h>
#include <gtsam/constrained/QcqpProblem.h>
#include <gtsam/constrained/QpCost.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>

#include <Eigen/Core>

#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <tuple>
#include <type_traits>
#include <vector>

namespace gtsam {

/**
 * An N-dimensional Euclidean point supporting a quadratic QCQP lift.
 *
 * Its tangent space has dimension N. For D=1, its lifted representation is
 * X = [h, x.transpose(), u].transpose(), where the canonical lift has h = 1
 * and u = x.dot(x). The N+2 QCQP coordinates are not tangent coordinates.
 *
 * @tparam N Number of Euclidean coordinates (strictly positive).
 * @ingroup slam
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

  /** Add an Eigen tangent vector or vector expression of dimension N. */
  template <typename Derived>
  AugmentedPoint operator+(const Eigen::MatrixBase<Derived>& delta) const {
    if (delta.rows() != N || delta.cols() != 1) {
      throw std::invalid_argument("AugmentedPoint: tangent dimension mismatch.");
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

/**
 * Unary squared-range factor for a known Euclidean landmark.
 *
 * Given an unknown state x, known world landmark L, and measured range d,
 * the scalar residual is ||x-L||^2 - d^2. The supplied noise model weights
 * this squared-range residual (not the unsquared range).
 *
 * Supports fixed-size Eigen vectors and AugmentedPoint<N> as position types.
 * The latter enables exact D=1 QCQP conversion while retaining compatibility
 * with nonlinear optimization. The QCQP lift has N+2 coordinates.
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

  /** Validate the dimension of a dynamically sized landmark. */
  static VectorType checkedLandmark(const Vector& landmark) {
    if (landmark.size() != N) {
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
   * Construct from a dynamically sized landmark vector.
   *
   * @throws std::invalid_argument if its dimension is not N or the range is
   *         negative or non-finite.
   */
  KnownLandmarkSquaredRangeFactor(Key key, const Vector& wL,
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

  /** Print the key, known landmark, measured range, and noise model. */
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

  /** Compare keys, noise models, landmarks, and squared ranges. */
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
   *
   * @throws std::invalid_argument for null outputs or columnDimension != 1.
   * @throws std::runtime_error for a non-augmented state or an unsupported
   *         noise model (null, robust, or constrained).
   */
  void qcqpFactors(NonlinearFactorGraph* costs,
                   NonlinearEqualityConstraints* constraints,
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

      constexpr int LiftedDim = traits<Point>::QcqpVectorDim;
      // The last column is GTSAM's HessianFactor constant slot (coefficient 0).
      Matrix B = Matrix::Zero(1, LiftedDim + 1);
      B(0, 0) = wL_.squaredNorm() - measuredRangeSquared_;
      B.block(0, 1, 1, N) = -2.0 * wL_.transpose();
      B(0, N + 1) = 1.0;

      const Matrix whitenedB = this->noiseModel_->Whiten(B);
      const Matrix Q = whitenedB.transpose() * whitenedB;
      InsertQcqpConstraints<Point, 1>(this->key(), constraints);
      const SymmetricBlockMatrix blockQ(std::vector<DenseIndex>{LiftedDim, 1},
                                        Q);
      costs->push_back(std::make_shared<QpCost>(
          HessianFactor(KeyVector{this->key()}, blockQ)));
    }
  }
};

}  // namespace gtsam

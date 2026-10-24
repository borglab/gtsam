/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */
/**
 * @file KnownLandmarkSquaredRangeFactor.h
 * @brief Squared-range observation factor and Euclidean point QCQP lift.
 * @author Oluwatodimu Afolayan
 * @author Frederike Dümbgen
 * @author Frank Dellaert
 */
#pragma once

#include <gtsam/base/VectorSpace.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>

#include <Eigen/Core>

#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

namespace gtsam {

/**
 * An N-dimensional Euclidean point with a quadratic lift for squared-range
 * constraints.
 *
 * The underlying vector space has dimension N. Its D=1 QCQP representation is
 * X = [h, x.transpose(), u].transpose(), with h = 1 and u = x.dot(x).
 * The representation has N+2 entries and is not the manifold tangent vector.
 */
template <int N>
class AugmentedPoint {
 public:
  static_assert(N > 0, "AugmentedPoint requires N > 0");

  using VectorType = Eigen::Matrix<double, N, 1>;

  /// Dimension of the underlying Euclidean point.
  static constexpr int dimension = N;

  /// Construct the origin.
  AugmentedPoint() : point_(VectorType::Zero()) {}

  /// Construct a point from its Euclidean coordinates.
  explicit AugmentedPoint(const VectorType& point) : point_(point) {}

  /// Return the additive identity.
  static AugmentedPoint Identity() { return AugmentedPoint(); }

  /// Add two Euclidean points.
  AugmentedPoint operator+(const AugmentedPoint& other) const {
    return AugmentedPoint(point_ + other.point_);
  }

  /** Add a fixed-size or dynamic Eigen tangent vector. */
  template <typename Derived>
  AugmentedPoint operator+(const Eigen::MatrixBase<Derived>& delta) const {
    if (delta.rows() != N || delta.cols() != 1) {
      throw std::invalid_argument("AugmentedPoint: tangent dimension mismatch.");
    }
    return AugmentedPoint(point_ + delta);
  }

  /// Subtract two Euclidean points.
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

  /// Return the underlying vector-space dimension.
  int dim() const { return N; }

  /// Return the Euclidean norm.
  double norm() const { return point_.norm(); }

  /// Return the Euclidean inner product with another point.
  double dot(const AugmentedPoint& other) const {
    return point_.dot(other.point_);
  }

  /// Return the Euclidean coordinates.
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
  VectorType point_;
};

/**
 * GTSAM vector-space traits with an N+2 dimensional D=1 QCQP lift.
 *
 * The QCQP variable is X = [h, x.transpose(), u].transpose(). The two
 * quadratic equality constraints enforce x.dot(x) = h*u and h*h = 1.
 */
template <int N>
struct traits<AugmentedPoint<N>>
    : public internal::VectorSpace<AugmentedPoint<N>> {
  using Point = AugmentedPoint<N>;
  using VectorType = typename Point::VectorType;

  /// Number of entries in the D=1 lifted QCQP vector.
  static constexpr int QcqpVectorDim = N + 2;

  /**
   * Embed a Euclidean point as X = [1, x.transpose(), x.dot(x)].transpose().
   * Only the single-column QCQP representation (D=1) is supported.
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
   * Return (A, a, b) triples for X.transpose()*A*X + 2*a.dot(X) = b.
   *
   * The first equality enforces x.dot(x) - h*u = 0, and the second enforces
   * h*h = 1. Both constraints have no linear term.
   */
  template <int D = 1>
  static std::vector<std::tuple<Matrix, Vector, double>> QcqpConstraints() {
    if constexpr (D == 1) {
      std::vector<std::tuple<Matrix, Vector, double>> constraints;
      constraints.reserve(2);

      Matrix A = Matrix::Zero(QcqpVectorDim, QcqpVectorDim);
      A.block(1, 1, N, N).setIdentity();
      A(0, N + 1) = -0.5;
      A(N + 1, 0) = -0.5;
      constraints.emplace_back(A, Vector(), 0.0);

      Matrix H = Matrix::Zero(QcqpVectorDim, QcqpVectorDim);
      H(0, 0) = 1.0;
      constraints.emplace_back(H, Vector(), 1.0);

      return constraints;
    } else {
      throw std::invalid_argument(
          "traits<AugmentedPoint>::QcqpConstraints only supports D=1.");
    }
  }

  /**
   * Recover Euclidean coordinates from a D=1 QCQP vector.
   *
   * Divide by the homogenization coordinate to remove the global sign
   * ambiguity (h = +/-1 in an exact feasible vector). Reject invalid shape
   * and a zero or non-finite homogenization coordinate.
   */
  template <int D>
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
            "traits<AugmentedPoint>::FromQcqpValue requires a nonzero, "
            "finite homogenization coordinate.");
      }

      const VectorType x = X.col(0).segment<N>(1) / h;
      return Point(x);
    } else {
      throw std::invalid_argument(
          "traits<AugmentedPoint>::FromQcqpValue only supports D=1.");
    }
  }
};

/// Preserve vector-space traits for const-qualified points.
template <int N>
struct traits<const AugmentedPoint<N>> : public traits<AugmentedPoint<N>> {};

/**
 * Unary factor on an N-dimensional Euclidean point using squared range to a
 * fixed known landmark.
 *
 * For a point wX, known world landmark wL, and measured range d,
 * the residual is
 *
 *   e(wX) = ||wX - wL||^2 - d^2.
 *
 * Squaring the range eliminates the square root present in an ordinary range
 * residual and gives a polynomial form suitable for future lifted QCQP and
 * semidefinite programming formulations.
 *
 * Unlike KnownLandmarkFactor, which constrains a pose using a relative
 * landmark-position measurement, this factor constrains an N-dimensional
 * Euclidean point using only a scalar range measurement.
 *
 * The known landmark is fixed data, and the supplied noise model weights the
 * scalar squared-range residual.
 *
 * @note The factor still uses an Eigen vector as its state type. The
 * AugmentedPoint QCQP traits above are not used by this factor until its
 * variable type and QCQP conversion are integrated separately.
 *
 * @tparam N Dimension of the Euclidean point state and known landmark.
 * @ingroup slam
 *
 * See `gtsam/slam/doc/KnownLandmarkSquaredRangeFactor.ipynb` for more details
 * regarding usage.
 */
template <int N>
class KnownLandmarkSquaredRangeFactor
    : public NoiseModelFactorN<Eigen::Matrix<double, N, 1>> {
 public:
  static_assert(N > 0, "KnownLandmarkSquaredRangeFactor requires N > 0");
  using Point = Eigen::Matrix<double, N, 1>;
  using Base = NoiseModelFactorN<Point>;
  using This = KnownLandmarkSquaredRangeFactor<N>;

 private:
  Point wL_;  ///< Known landmark expressed in the world frame.
  double measuredRangeSquared_;  ///< Squared measured range to wL_ (d^2).

  /**
   * Validate the landmark dimension before converting to a fixed-size Point.
   *
   * @throws std::invalid_argument if the landmark dimension is incorrect.
   */
  static Point checkedLandmark(const Vector& wL) {
    if (wL.size() != N) {
      throw std::invalid_argument(
          "KnownLandmarkSquaredRangeFactor: invalid landmark dimension.");
    }
    return Point(wL);
  }

 public:
  /// Expose the base-class evaluateError overloads.
  using Base::evaluateError;

  /**
   * Construct from a key, world landmark, range measurement, and noise model.
   *
   * @param key Key associated with the unknown position.
   * @param wL Known landmark expressed in the world frame.
   * @param measuredRange Measured range to the landmark.
   * @param model Noise model for the scalar squared-range residual.
   * @throws std::invalid_argument if measuredRange is negative.
   */
  KnownLandmarkSquaredRangeFactor(Key key, const Point& wL,
                                  double measuredRange,
                                  const SharedNoiseModel& model)
      : Base(model, key),
        wL_(wL),
        measuredRangeSquared_(measuredRange * measuredRange) {
    if (measuredRange < 0.0) {
      throw std::invalid_argument(
          "KnownLandmarkSquaredRangeFactor: range must be non-negative.");
    }
  }

  /**
   * Construct from a dynamically sized landmark vector.
   *
   * Validates the landmark dimension before converting it to a fixed-size
   * Point.
   *
   * @param key Key associated with the unknown position.
   * @param wL Known landmark expressed in the world frame.
   * @param measuredRange Measured range to the landmark.
   * @param model Noise model for the scalar squared-range residual.
   * @throws std::invalid_argument if the landmark dimension is incorrect
   *         or measuredRange is negative.
   */
  KnownLandmarkSquaredRangeFactor(Key key, const Vector& wL,
                                  double measuredRange,
                                  const SharedNoiseModel& model)
      : KnownLandmarkSquaredRangeFactor(key, checkedLandmark(wL), measuredRange,
                                        model) {}

  /** Return a deep copy of this factor. */
  NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<This>(*this);
  }

  /**
   * Print the factor.
   *
   * @param s Optional string printed before the factor.
   * @param keyFormatter Formatter used to display the variable key.
   */
  void print(
      const std::string& s = "",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    std::cout << s << "KnownLandmarkSquaredRangeFactor<" << N << "> on "
              << keyFormatter(this->key1()) << "\n";
    traits<Point>::Print(wL_, "  landmark: ");
    std::cout << "  measured range squared: " << measuredRangeSquared_ << "\n";
    if (this->noiseModel_) {
      this->noiseModel_->print("  noise model: ");
    } else {
      std::cout << "  no noise model\n";
    }
  }

  /**
   * Check whether this factor equals another factor within a tolerance.
   *
   * @param expected Factor to compare against.
   * @param tol Numerical comparison tolerance.
   * @return True if the factors are equal within tol.
   */
  bool equals(const NonlinearFactor& expected,
              double tol = 1e-9) const override {
    const auto* e = dynamic_cast<const This*>(&expected);
    return e != nullptr && Base::equals(*e, tol) &&
           traits<Point>::Equals(wL_, e->wL_, tol) &&
           traits<double>::Equals(measuredRangeSquared_,
                                  e->measuredRangeSquared_, tol);
  }

  /**
   * Evaluate the squared-range residual and optionally its Jacobian.
   *
   * @param wX Current value of the unknown point in the world frame.
   * @param H Optional 1-by-N Jacobian of the residual with respect to wX.
   * @return One-dimensional residual vector containing
   *         ||wX - wL||^2 - d^2.
   */
  Vector evaluateError(const Point& wX, OptionalMatrixType H) const override {
    const Point difference = wX - wL_;
    const double predictedRangeSquared = difference.squaredNorm();
    const double error = predictedRangeSquared - measuredRangeSquared_;
    if (H) {
      *H = 2.0 * difference.transpose();
    }
    return Vector1(error);
  }
};

}  // namespace gtsam

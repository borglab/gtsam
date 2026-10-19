/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file KnownLandmarkSquaredRangeFactor.h
 * @brief Factor for range-only observation of a known landmark from an unknown
 * position.
 * @author Oluwatodimu Afolayan
 * @author Frederike Dümbgen
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/nonlinear/NoiseModelFactorN.h>

#include <Eigen/Core>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

namespace gtsam {

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

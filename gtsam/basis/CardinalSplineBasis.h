/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file CardinalSplineBasis.h
 * @brief Cubic cardinal splines integrated with the GTSAM Basis framework.
 * @author This file was written by AI
 */

#pragma once

#include <gtsam/basis/Basis.h>
#include <gtsam/basis/IrwinHall.h>

#include <stdexcept>

namespace gtsam {

/**
 * Cubic cardinal-spline basis for scalar, vector, and manifold Basis functors.
 *
 * This class expresses the same rear-padded cubic interpolation used by
 * `CumulativeSplineTrajectory` as a dense weight vector. It therefore works
 * with `EvaluationFactor`, `VectorEvaluationFactor`, and the other reusable
 * factor types in BasisFactors.h when the sample coordinate is fixed.
 */
class CardinalSplineBasis : public Basis<CardinalSplineBasis> {
 public:
  /// Scalar control-point values consumed by the inherited basis functors.
  using Parameters = Vector;

  /** Calculate weights for unit-spaced control points. */
  static Weights CalculateWeights(size_t N, double x) {
    return calculateWeights(N, x, 0);
  }

  /** Calculate weights after mapping `[a,b]` onto the full spline support. */
  static Weights CalculateWeights(size_t N, double x, double a, double b) {
    const double scale = coordinateScale(N, a, b);
    return calculateWeights(N, (x - a) * scale, 0);
  }

  /** Calculate first-derivative weights for unit-spaced control points. */
  static Weights DerivativeWeights(size_t N, double x) {
    return calculateWeights(N, x, 1);
  }

  /**
   * Calculate first-derivative weights after mapping `[a,b]` onto the full
   * spline support.
   */
  static Weights DerivativeWeights(size_t N, double x, double a, double b) {
    const double scale = coordinateScale(N, a, b);
    return scale * calculateWeights(N, (x - a) * scale, 1);
  }

 private:
  /// Map a bounded domain onto the complete padded cardinal-spline support.
  static double coordinateScale(size_t N, double a, double b) {
    if (a == b) {
      throw std::invalid_argument(
          "CardinalSplineBasis interval must have nonzero length");
    }
    return (static_cast<double>(N > 0 ? N - 1 : 0) +
            kernels::IrwinHallCDF2.getLength()) /
           (b - a);
  }

  /// Convert cumulative kernel values into per-control-point basis weights.
  static Weights calculateWeights(size_t N, double x, size_t derivative) {
    if (N == 0) return Weights(0);
    if (N == 1) {
      Weights weights(1);
      weights(0) = derivative == 0 ? 1.0 : 0.0;
      return weights;
    }

    const double kernelTime = x + kernels::IrwinHallCDF2.getBeginning();
    Weights cumulative(N);
    for (size_t index = 0; index < N; ++index) {
      cumulative(static_cast<Eigen::Index>(index)) =
          kernels::IrwinHallCDF2.evaluateDerivative(
              derivative, kernelTime - static_cast<double>(index));
    }

    Weights weights(N);
    weights(0) = derivative == 0 ? 1.0 - cumulative(1) : -cumulative(1);
    for (size_t index = 1; index + 1 < N; ++index) {
      weights(static_cast<Eigen::Index>(index)) =
          cumulative(static_cast<Eigen::Index>(index)) -
          cumulative(static_cast<Eigen::Index>(index + 1));
    }
    weights(static_cast<Eigen::Index>(N - 1)) =
        cumulative(static_cast<Eigen::Index>(N - 1));
    return weights;
  }
};

}  // namespace gtsam

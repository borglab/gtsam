/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file PiecewisePolynomial.h
 * @brief Fixed-order piecewise polynomial kernels with analytic derivatives.
 * @author Brett Downing
 */

#pragma once

#include <gtsam/basis/Kernel.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <initializer_list>
#include <stdexcept>

namespace gtsam {

/**
 * A one-dimensional piecewise polynomial kernel.
 *
 * Coefficients for each piece are stored in ascending power order. Values
 * outside the declared interval range are evaluated at the nearest endpoint.
 *
 * @tparam Order Polynomial order of every piece.
 * @tparam Pieces Number of polynomial pieces.
 */
template <size_t Order, size_t Pieces>
class PiecewisePolynomial : public KernelBase {
 public:
  static constexpr size_t order = Order;
  static constexpr size_t pieces = Pieces;

  /** Coefficients, interval boundaries, and center defining the kernel. */
  struct Parameters {
    std::array<std::array<double, order + 1>, pieces> coefficients{};
    std::array<double, pieces + 1> intervals{};
    double center = 0.0;

    /** Construct from row-major coefficients and ascending boundaries. */
    Parameters(std::initializer_list<double> coefficientValues,
               std::initializer_list<double> intervalValues, double centerValue)
        : center(centerValue) {
      if (coefficientValues.size() != pieces * (order + 1) ||
          intervalValues.size() != pieces + 1) {
        throw std::invalid_argument(
            "PiecewisePolynomial parameter dimensions do not match template "
            "arguments");
      }
      auto coefficient = coefficientValues.begin();
      for (auto& piece : coefficients) {
        for (double& value : piece) value = *coefficient++;
      }
      std::copy(intervalValues.begin(), intervalValues.end(),
                intervals.begin());
    }
  };

  /** Construct a kernel from fixed-size coefficient data. */
  explicit PiecewisePolynomial(const Parameters& parameters)
      : parameters_(parameters) {}

  double getCenter() const override { return parameters_.center; }

  double getBeginning() const override { return parameters_.intervals.front(); }

  double getEnd() const override { return parameters_.intervals.back(); }

  size_t getValidDerivatives() const override { return order; }

  /// Return interval boundaries, primarily for validation and visualization.
  const std::array<double, pieces + 1>& getIntervals() const {
    return parameters_.intervals;
  }

  double evaluate(double t, OptionalJacobian<1, 1> H = {}) const override {
    return evaluateDerivative(0, t, H);
  }

  double evaluateDerivative(size_t derivative, double t,
                            OptionalJacobian<1, 1> H = {}) const override {
    if (derivative > order) {
      if (H) (*H)(0, 0) = 0.0;
      return 0.0;
    }

    t = std::max(parameters_.intervals.front(),
                 std::min(t, parameters_.intervals.back()));

    for (size_t piece = 0; piece < pieces; ++piece) {
      if (t <= parameters_.intervals[piece + 1]) {
        double powerOfT = 1.0;
        double result = 0.0;
        for (size_t exponent = derivative; exponent <= order; ++exponent) {
          size_t powerRule = 1;
          for (size_t count = 0; count < derivative; ++count) {
            powerRule *= exponent - count;
          }
          result +=
              powerRule * powerOfT * parameters_.coefficients[piece][exponent];
          powerOfT *= t;
        }
        if (H) (*H)(0, 0) = evaluateDerivative(derivative + 1, t);
        return result;
      }
    }

    if (H) (*H)(0, 0) = 0.0;
    return 0.0;
  }

 private:
  const Parameters parameters_;
};

}  // namespace gtsam

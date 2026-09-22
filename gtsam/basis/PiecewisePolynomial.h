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

#include <gtsam/basis/KernelBase.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <initializer_list>
#include <stdexcept>

namespace gtsam {

/**
 * A one-dimensional piecewise polynomial kernel.
 *
 * On interval @f$[b_j,b_{j+1}]@f$, the value is
 * @f$p_j(t)=\sum_{k=0}^{Order} a_{j,k}t^k@f$. Coefficients for each piece are
 * therefore stored in ascending power order. The @f$r@f$-th derivative uses
 * the exact power rule,
 * @f$p_j^{(r)}(t)=\sum_{k=r}^{Order}\frac{k!}{(k-r)!}a_{j,k}t^{k-r}@f$.
 * Values outside the declared interval range are evaluated at the nearest
 * endpoint, which lets cumulative kernels remain constant outside support.
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
    /// Polynomial coefficients indexed by piece, then ascending power.
    std::array<std::array<double, order + 1>, pieces> coefficients{};
    /// Monotonically increasing boundaries for the polynomial pieces.
    std::array<double, pieces + 1> intervals{};
    /// Representative center of the complete kernel support.
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

  /// Return the representative center of the kernel support.
  double getCenter() const override { return parameters_.center; }

  /// Return the first boundary of the kernel support.
  double getBeginning() const override { return parameters_.intervals.front(); }

  /// Return the last boundary of the kernel support.
  double getEnd() const override { return parameters_.intervals.back(); }

  /// Return the largest derivative order available analytically.
  size_t getValidDerivatives() const override { return order; }

  /// Return interval boundaries, primarily for validation and visualization.
  const std::array<double, pieces + 1>& getIntervals() const {
    return parameters_.intervals;
  }

  /// Evaluate the selected polynomial piece and optionally its slope.
  double evaluate(double t, OptionalJacobian<1, 1> H = {}) const override {
    return evaluateDerivative(0, t, H);
  }

  /// Evaluate an analytic derivative and optionally the next derivative.
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

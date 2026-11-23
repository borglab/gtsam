// Copyright (c) Georgia Tech Research Corporation. See LICENSE for details.

/**
 * @file SamplingAlgorithms.h
 * @author Frank Dellaert
 * @author Akash Ratheesh
 */

#pragma once

#include <gtsam/planning/ControlProposals.h>

#include <cmath>
#include <limits>
#include <stdexcept>

namespace gtsam {

/// MPPI's cost-weighted update of the nominal control sequence.
class MPPI {
 public:
  /// MPPI draws every candidate from the proposal.
  static constexpr bool includeIncumbent = false;

  /// Construct with inverse temperature 1/lambda. Zero gives uniform weights.
  explicit MPPI(double inverseTemperature = 1.0)
      : inverseTemperature_(inverseTemperature) {
    if (!std::isfinite(inverseTemperature_) || inverseTemperature_ < 0.0)
      throw std::invalid_argument(
          "Expected a finite nonnegative inverse temperature.");
  }

  /**
   * Normalize exp(-inverseTemperature * cost) after subtracting the lowest
   * finite cost. A positive infinite cost rejects a candidate. At least one
   * candidate must have finite cost.
   */
  Vector weights(const Vector& costs) const {
    if (!costs.size())
      throw std::invalid_argument("MPPI requires at least one candidate.");
    const double minimum = costs.minCoeff();
    if (costs.hasNaN() || !std::isfinite(minimum))
      throw std::domain_error(
          "MPPI needs finite costs or +infinity and a finite minimum.");
    Vector result;
    if (inverseTemperature_ == 0.) {
      result = costs.array().isFinite().cast<double>();
    } else {
      result = (-inverseTemperature_ * (costs.array() - minimum)).exp();
    }
    result /= result.sum();
    // Subnormal weights cannot affect a practical control update.
    return (result.array() < std::numeric_limits<double>::min())
        .select(0., result);
  }

  /// Convert factor-graph costs to weights and update the proposal's search
  /// state.
  template <class Proposal, class Search, class Batch>
  void apply(const Proposal& proposal, Search& search, const Batch& batch,
             const Vector& costs) const {
    proposal.update(search, batch, weights(costs));
  }

 private:
  double inverseTemperature_;
};

}  // namespace gtsam

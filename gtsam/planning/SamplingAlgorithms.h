// Copyright (c) Georgia Tech Research Corporation. See LICENSE for details.

/**
 * @file    SamplingAlgorithms.h
 * @author  Frank Dellaert
 * @author  Akash Ratheesh
 */

#pragma once

#include <gtsam/planning/ControlProposals.h>

#include <optional>

namespace gtsam {
/// Empirical quantile of finite scenario costs, interpolating at alpha*(N-1).
GTSAM_EXPORT double valueAtRisk(const Vector& costs, double alpha);

/**
 * MPPI-Generic conditional value at risk. Average the interpolated quantile
 * and all observations starting at index ceil(alpha*(N-1)) in sorted order.
 * When the quantile is an observation, upstream counts that observation twice.
 */
GTSAM_EXPORT double conditionalValueAtRisk(const Vector& costs, double alpha);

/// Exponentially weight candidate sequences using graph costs and optional log
/// offsets.
class GTSAM_EXPORT MPPI {
 public:
  static constexpr bool includeIncumbent = false;
  /// Construct an update rule for an independent proposal.
  explicit MPPI(double inverseTemperature = 1.0);
  /// Compute normalized weights proportional to exp(-beta * cost + log offset).
  Vector weights(const Vector& costs,
                 const Vector& logWeightOffsets = {}) const;

  /// Give the proposal the weights to update its sampling coordinates.
  template <class Proposal, class Search, class Batch>
  void apply(const Proposal& proposal, Search& state, const Batch& batch,
             const Vector& costs) const {
    proposal.update(state, batch, weights(costs, batch.logWeightOffsets));
  }

 private:
  double inverseTemperature_;
};

/// Select the lowest-cost sequence, reserving one slot for the incumbent.
class GTSAM_EXPORT PredictiveSampling {
 public:
  static constexpr bool includeIncumbent = true;
  PredictiveSampling() = default;
  /// Return a one-hot vector selecting the first minimum-cost sequence.
  Vector weights(const Vector& costs) const;
  /// Update the proposal with the selected candidate.
  template <class Proposal, class Search, class Batch>
  void apply(const Proposal& proposal, Search& state, const Batch& batch,
             const Vector& costs) const {
    proposal.update(state, batch, weights(costs));
  }
};

/// Cross-entropy search with an explicit proposal-specific fit to the elites.
class GTSAM_EXPORT CEM {
 public:
  static constexpr bool includeIncumbent = false;
  /// Set the elite count and positive floor on fitted standard deviations.
  explicit CEM(size_t elites, double minStddev = 1e-3);
  /// Return uniform weights on the lowest-cost finite elites.
  Vector weights(const Vector& costs) const;
  /// Fit the proposal family to the elite candidates.
  template <class Proposal, class Search, class Batch>
  void apply(const Proposal& proposal, Search& state, const Batch& batch,
             const Vector& costs) const {
    proposal.fit(state, batch, weights(costs), minStddev_);
  }

 private:
  size_t elites_;
  double minStddev_;
};

}  // namespace gtsam

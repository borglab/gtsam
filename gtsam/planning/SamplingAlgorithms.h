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

/// Base class for sampling-based trajectory optimization update rules.
class GTSAM_EXPORT SamplingAlgorithm {
 public:
  virtual ~SamplingAlgorithm() = default;

  /// Whether the proposal should reserve the first candidate for the incumbent.
  virtual bool includesIncumbent() const { return false; }

  /// Whether this rule fits the proposal spread (e.g., CEM, EliteMPPI) rather
  /// than only updating the nominal controls.
  virtual bool fitsSpread() const { return false; }

  /// Minimum standard deviation floor used when fitsSpread() is true.
  virtual double minStddev() const { return 0.0; }

  /// Compute normalized candidate weights from trajectory costs and optional
  /// proposal log-weight offsets.
  virtual Vector computeWeights(const Vector& costs,
                                const Vector& logWeightOffsets = {}) const = 0;

  /// Apply this rule to a proposal in its native sampling coordinates.
  template <class Proposal, class Search, class Batch>
  void apply(const Proposal& proposal, Search& state, const Batch& batch,
             const Vector& costs) const {
    const Vector w = computeWeights(costs, batch.logWeightOffsets);
    if (fitsSpread()) {
      proposal.fit(state, batch, w, minStddev());
    } else {
      proposal.update(state, batch, w);
    }
  }
};

/// Exponentially weight candidate sequences using graph costs and optional log
/// offsets.
class GTSAM_EXPORT MPPI : public SamplingAlgorithm {
 public:
  static constexpr bool includeIncumbent = false;
  /// Construct an update rule for an independent proposal.
  explicit MPPI(double inverseTemperature = 1.0);
  /// Compute normalized weights proportional to exp(-beta * cost + log offset).
  Vector weights(const Vector& costs,
                 const Vector& logWeightOffsets = {}) const;
  Vector computeWeights(const Vector& costs,
                        const Vector& logWeightOffsets = {}) const override {
    return weights(costs, logWeightOffsets);
  }

 private:
  double inverseTemperature_;
};

/// Select the lowest-cost sequence, reserving one slot for the incumbent.
class GTSAM_EXPORT PredictiveSampling : public SamplingAlgorithm {
 public:
  static constexpr bool includeIncumbent = true;
  PredictiveSampling() = default;
  bool includesIncumbent() const override { return true; }
  /// Return a one-hot vector selecting the first minimum-cost sequence.
  Vector weights(const Vector& costs) const;
  Vector computeWeights(const Vector& costs,
                        const Vector& = {}) const override {
    return weights(costs);
  }
};

/// Cross-entropy search with an explicit proposal-specific fit to the elites.
class GTSAM_EXPORT CEM : public SamplingAlgorithm {
 public:
  static constexpr bool includeIncumbent = false;
  /// Set the elite count and positive floor on fitted standard deviations.
  explicit CEM(size_t elites, double minStddev = 1e-3);
  bool fitsSpread() const override { return true; }
  double minStddev() const override { return minStddev_; }
  /// Return uniform weights on the lowest-cost finite elites.
  Vector weights(const Vector& costs) const;
  Vector computeWeights(const Vector& costs,
                        const Vector& = {}) const override {
    return weights(costs);
  }

 private:
  size_t elites_;
  double minStddev_;
};

}  // namespace gtsam

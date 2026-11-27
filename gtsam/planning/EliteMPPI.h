// Copyright (c) Georgia Tech Research Corporation. See LICENSE for details.
/**
 * @file EliteMPPI.h
 * @author Frank Dellaert
 * @author Akash Ratheesh
 */
#pragma once

#include <gtsam/planning/SamplingAlgorithms.h>

namespace gtsam {

/// Fit a proposal to the lowest-cost finite elites with exponential cost
/// weights.
class GTSAM_EXPORT EliteMPPI : public SamplingAlgorithm {
 public:
  static constexpr bool includeIncumbent = false;

  /// Set the number of elites, inverse temperature, and minimum fitted spread.
  explicit EliteMPPI(size_t elites, double inverseTemperature = 1.0,
                     double minStddev = 1e-3);

  bool fitsSpread() const override { return true; }
  double minStddev() const override { return minStddev_; }

  /// Normalized Boltzmann weights restricted to the elites ranked by trajectory
  /// cost.
  Vector weights(const Vector& costs) const;
  Vector computeWeights(const Vector& costs,
                        const Vector& = {}) const override {
    return weights(costs);
  }

  /// Fit the proposal in its native sampling coordinates and return the
  /// weights.
  template <class Proposal, class Search, class Batch>
  Vector apply(const Proposal& proposal, Search& state, const Batch& batch,
               const Vector& costs) const {
    Vector normalized = weights(costs);
    proposal.fit(state, batch, normalized, minStddev_);
    return normalized;
  }

 private:
  CEM selection_;
  MPPI weighting_;
  double minStddev_;
};

}  // namespace gtsam

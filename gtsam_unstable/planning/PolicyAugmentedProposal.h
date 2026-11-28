// Copyright (c) Georgia Tech Research Corporation. See LICENSE for details.
/**
 * @file PolicyAugmentedProposal.h
 * @author Frank Dellaert
 * @author Akash Ratheesh
 */
#pragma once
#include <gtsam/planning/ControlProposals.h>
#include <gtsam_unstable/dllexport.h>

namespace gtsam {
/**
 * Augment local Gaussian exploration with sequences drawn by a user policy.
 * The generator receives the current solve's initial Values and nominal plan,
 * allowing state or observation conditioning and warm starts. The two sources
 * occupy disjoint slots in one batch. MPPI and predictive sampling update
 * the nominal controls, while CEM fits the Gaussian to elites from both
 * sources.
 */
class GTSAM_UNSTABLE_EXPORT PolicyAugmentedProposal : public GaussianProposal {
 public:
  using Generator = std::function<Matrix(const Values&, const Matrix&)>;
  using SeededGenerator =
      std::function<Matrix(const Values&, const Matrix&, std::uint64_t)>;

  /// Allocate a positive number of candidates to each source.
  PolicyAugmentedProposal(const Matrix& covariance, size_t gaussianSamples,
                          size_t policySamples, Generator generator,
                          size_t seed = 42);
  /** Provide a seed per policy candidate. A deterministic function of these
   * arguments reuses identical random innovations for coupled initial states.
   * With the two-argument overload the caller controls any policy randomness.
   */
  PolicyAugmentedProposal(const Matrix& covariance, size_t gaussianSamples,
                          size_t policySamples, SeededGenerator generator,
                          size_t seed = 42);
  /// Preserve both random streams for replay in tracking.
  std::unique_ptr<GaussianProposal> clone() const override {
    return std::make_unique<PolicyAugmentedProposal>(*this);
  }

 protected:
  /// Draw the Gaussian block first, then one sequence per policy draw.
  ControlBatch sampleImpl(const State& state, const Values& initial, double dt,
                          bool includeIncumbent) const override;

 private:
  size_t policySamples_;
  SeededGenerator generator_;
  mutable std::mt19937_64 policySeeds_;
};

}  // namespace gtsam

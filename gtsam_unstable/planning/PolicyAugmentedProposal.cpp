// Copyright (c) Georgia Tech Research Corporation. See LICENSE for details.
// Authors: Akash Ratheesh and Frank Dellaert.
#include <gtsam_unstable/planning/PolicyAugmentedProposal.h>

namespace gtsam {
namespace {
PolicyAugmentedProposal::SeededGenerator ignoreSeed(
    PolicyAugmentedProposal::Generator generator) {
  if (!generator) throw std::invalid_argument("Missing policy generator.");
  return [generator = std::move(generator)](const Values& initial,
                                            const Matrix& mean, std::uint64_t) {
    return generator(initial, mean);
  };
}

}  // namespace

PolicyAugmentedProposal::PolicyAugmentedProposal(const Matrix& covariance,
                                                 size_t gaussianSamples,
                                                 size_t policySamples,
                                                 Generator generator,
                                                 size_t seed)
    : PolicyAugmentedProposal(covariance, gaussianSamples, policySamples,
                              ignoreSeed(std::move(generator)), seed) {}

PolicyAugmentedProposal::PolicyAugmentedProposal(const Matrix& covariance,
                                                 size_t gaussianSamples,
                                                 size_t policySamples,
                                                 SeededGenerator generator,
                                                 size_t seed)
    : GaussianProposal(covariance, gaussianSamples, seed),
      policySamples_(policySamples),
      generator_(std::move(generator)),
      policySeeds_(~seed) {
  if (!policySamples || !generator_ ||
      policySamples >
          static_cast<size_t>(std::numeric_limits<Eigen::Index>::max()) ||
      gaussianSamples >
          static_cast<size_t>(std::numeric_limits<Eigen::Index>::max()) -
              policySamples)
    throw std::invalid_argument(
        "Invalid counts or missing generator for a policy-augmented proposal.");
}

ControlBatch PolicyAugmentedProposal::sampleImpl(const State& state,
                                                 const Values& initial,
                                                 double dt,
                                                 bool includeIncumbent) const {
  const Eigen::Index horizon = state.mean.cols();
  const size_t total = sampler().numSamples() + policySamples_;
  if (!horizon ||
      total > static_cast<size_t>(std::numeric_limits<Eigen::Index>::max() /
                                  horizon))
    throw std::invalid_argument(
        "Policy-augmented batch exceeds representable dimensions.");
  ControlBatch batch =
      GaussianProposal::sampleImpl(state, initial, dt, includeIncumbent);
  batch.controls.conservativeResize(Eigen::NoChange,
                                    static_cast<Eigen::Index>(total) * horizon);
  for (Eigen::Index j = static_cast<Eigen::Index>(sampler().numSamples());
       j < static_cast<Eigen::Index>(total); ++j) {
    const Matrix draw = generator_(initial, state.mean, policySeeds_());
    if (draw.rows() != state.mean.rows() || draw.cols() != horizon ||
        !draw.allFinite())
      throw std::invalid_argument(
          "Policy generator must return a finite plan of the nominal shape.");
    controlTrajectory(batch.controls, j, horizon) = draw;
  }
  return batch;
}

}  // namespace gtsam

// Copyright (c) Georgia Tech Research Corporation. See LICENSE for details.

/**
 * @file ControlProposals.h
 * @author Frank Dellaert
 * @author Akash Ratheesh
 */

#pragma once

#include <gtsam/nonlinear/Values.h>
#include <gtsam/planning/GaussianNoiseSampler.h>

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <utility>

namespace gtsam {

/// Physical controls of a batch, grouped by candidate, then time.
struct ControlBatch {
  Matrix controls;
};

/// Search state of a proposal with fixed dispersion, one column per time step.
struct ControlDistribution {
  Matrix mean;
};

/// Independent Gaussian perturbations around a nominal control sequence.
class GaussianProposal {
 public:
  using State = ControlDistribution;

  /// Seed the per-candidate random streams for this proposal.
  GaussianProposal(const Matrix& covariance, size_t samples,
                   std::uint64_t seed = 42)
      : sampler_(covariance, samples, seed) {}

  /// Initialize the proposal around an M-by-H nominal control sequence.
  State initialize(Matrix nominal) const {
    if (!nominal.size())
      throw std::invalid_argument(
          "Expected a nonempty nominal control sequence.");
    return {std::move(nominal)};
  }

  /**
   * Sample complete sequences around the mean. When requested, reserve the
   * first sequence for the incumbent. The context and dt are part of the
   * proposal interface for draws that depend on the initial state or time.
   */
  ControlBatch sample(const State& state, const Values& /*initial*/,
                      double /*dt*/, bool includeIncumbent = false) const {
    ControlBatch batch{sampler_.sampleControls(state.mean)};
    const Eigen::Index horizon = state.mean.cols();
    if (!horizon || batch.controls.rows() != state.mean.rows() ||
        !batch.controls.cols() || batch.controls.cols() % horizon)
      throw std::invalid_argument(
          "Expected a batch of complete control sequences.");
    if (includeIncumbent) batch.controls.leftCols(horizon) = state.mean;
    return batch;
  }

  /// Replace the mean with the weighted average of the sampled trajectories.
  void update(State& state, const ControlBatch& batch,
              const Vector& weights) const {
    if (!state.mean.size() || !weights.size() ||
        batch.controls.rows() != state.mean.rows() ||
        batch.controls.cols() != state.mean.cols() * weights.size())
      throw std::invalid_argument("Expected one complete plan per weight.");
    Matrix mean = Matrix::Zero(state.mean.rows(), state.mean.cols());
    for (Eigen::Index i = 0; i < weights.size(); ++i) {
      // A rejected trajectory may contain nonfinite controls.
      if (weights[i] != 0.)
        mean += weights[i] *
                batch.controls.middleCols(i * mean.cols(), mean.cols());
    }
    state.mean.swap(mean);
  }

 private:
  GaussianNoiseSampler sampler_;
};

}  // namespace gtsam

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

/// Search state of a proposal with per-knot marginal standard deviations, one
/// column per time step.
struct ControlDistribution {
  Matrix mean;
  Matrix stddev{};
  size_t iteration = 0;
};

/// Reserve the first candidate for the nominal plan in every search rule.
struct GaussianProposalOptions {
  bool includeNominal = false;
};

/// Independent Gaussian perturbations around a nominal control sequence.
class GaussianProposal {
 public:
  using State = ControlDistribution;

  /// Seed the per-candidate random streams for this proposal.
  GaussianProposal(const Matrix& covariance, size_t samples,
                   std::uint64_t seed = 42,
                   GaussianProposalOptions options = {})
      : sampler_(covariance, samples, seed),
        initialStddev_(covariance.diagonal().array().sqrt()),
        options_(options) {
    if (!samples)
      throw std::invalid_argument("Expected at least one candidate.");
  }

  /// Initialize the proposal around an M-by-H nominal control sequence.
  State initialize(Matrix nominal) const {
    if (!nominal.cols() || nominal.rows() != initialStddev_.size() ||
        !nominal.allFinite())
      throw std::invalid_argument(
          "Expected finite nominal controls matching the covariance.");
    Matrix spread = initialStddev_.replicate(1, nominal.cols());
    return {std::move(nominal), std::move(spread)};
  }

  /**
   * Sample complete sequences around the mean. When requested, reserve the
   * first sequence for the incumbent. The context and dt are part of the
   * proposal interface for draws that depend on the initial state or time.
   */
  ControlBatch sample(const State& state, const Values& /*initial*/,
                      double /*dt*/, bool includeIncumbent = false) const {
    const Eigen::Index horizon = state.mean.cols();
    if (!horizon || state.mean.rows() != initialStddev_.size())
      throw std::invalid_argument(
          "Nominal controls must match the covariance.");
    ControlBatch batch;
    if (!state.stddev.size()) {
      batch.controls = sampler_.sampleControls(state.mean);
    } else {
      if (state.stddev.rows() != state.mean.rows() ||
          state.stddev.cols() != horizon || !state.stddev.allFinite() ||
          (state.stddev.array() < 0.).any())
        throw std::invalid_argument("Invalid marginal standard deviations.");
      if ((state.stddev.colwise() - initialStddev_).isZero(0.)) {
        batch.controls = sampler_.sampleControls(state.mean);
      } else {
        batch.controls = sampler_.sampleNoise(horizon).array().colwise() /
                         initialStddev_.array();
        for (Eigen::Index k = 0; k < batch.controls.cols(); k += horizon) {
          auto trajectory = batch.controls.middleCols(k, horizon);
          trajectory.array() *= state.stddev.array();
          trajectory += state.mean;
        }
      }
    }
    if (includeIncumbent || options_.includeNominal)
      batch.controls.leftCols(horizon) = state.mean;
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
    ++state.iteration;
  }

  /// Refit marginal standard deviations while retaining cross-control
  /// correlations.
  void fit(State& state, const ControlBatch& batch, const Vector& weights,
           double minStddev) const {
    if (!(minStddev > 0.) || !std::isfinite(minStddev))
      throw std::invalid_argument("Expected a positive finite spread floor.");
    update(state, batch, weights);
    Matrix variance = Matrix::Zero(state.mean.rows(), state.mean.cols());
    for (Eigen::Index i = 0; i < weights.size(); ++i) {
      if (weights[i] > 0.)
        variance.array() +=
            weights[i] *
            (batch.controls.middleCols(i * variance.cols(), variance.cols()) -
             state.mean)
                .array()
                .square();
    }
    state.stddev = variance.array().sqrt().max(minStddev);
  }

 private:
  GaussianNoiseSampler sampler_;
  Vector initialStddev_;
  GaussianProposalOptions options_;
};

}  // namespace gtsam

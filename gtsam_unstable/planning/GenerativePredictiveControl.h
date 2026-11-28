// Copyright (c) Georgia Tech Research Corporation. See LICENSE for details.

/**
 * @file GenerativePredictiveControl.h
 * @author Frank Dellaert
 * @author Akash Ratheesh
 */

#pragma once

#include <gtsam_unstable/dllexport.h>
#include <gtsam_unstable/planning/PolicyAugmentedProposal.h>

#include <functional>
#include <random>

namespace gtsam {

/// Parameters for one collection and fitting cycle of GPC Algorithm 1.
struct GPCTrainingParams {
  size_t episodeLength = 32;
  size_t gaussianSamples = 32;
  size_t policySamples = 8;
  double explorationStddev = 1.0;
  double warmStart = 0.0;
  size_t flowSteps = 8;
  size_t gradientSteps = 100;
  size_t batchSize = 32;
  double learningRate = 0.01;
  double cosineWeight = 2.0;
  /// Use AdamW for flow fitting and retain optimizer state across cycles.
  bool useAdamW = false;
};

/**
 * Conditional flow-matching controller from Kurtz and Burdick (2025).
 * A tanh network predicts control-sequence velocity from the sequence,
 * observation, and flow time. Controls use M x H column-major matrices.
 * Supply observations and controls scaled to comparable numerical ranges.
 * Sampling advances this instance's RNG. Use separate instances for
 * concurrent sampling or training.
 */
class GTSAM_UNSTABLE_EXPORT GenerativePredictiveControl {
 public:
  /// Construct a one-hidden-layer velocity model with reproducible parameters.
  GenerativePredictiveControl(size_t observationDim, size_t controlDim,
                              size_t horizon, size_t hiddenDim, size_t seed);

  /// Flow a Gaussian draw blended with the previous plan from time zero to one.
  Matrix generate(const Vector& observation, const Matrix& previous,
                  double warmStart, size_t flowSteps) const;

  /// Concatenate local Gaussian and conditional flow samples for GPC+.
  Matrix candidates(const Vector& observation, const Matrix& previous,
                    size_t gaussianSamples, size_t policySamples,
                    double explorationStddev, double warmStart,
                    size_t flowSteps) const;

  /**
   * Fit Equation 27 by minibatch SGD with the cosine weight of Equation 34.
   * Each column is one example. previousPlans and targets contain flattened
   * control sequences (all control coordinates at step zero, then step one).
   * Return the average minibatch loss from the final gradient step.
   */
  double fit(const Matrix& observations, const Matrix& previousPlans,
             const Matrix& targets, size_t gradientSteps, size_t batchSize,
             double learningRate, double cosineWeight);

  /**
   * Fit the same flow objective with AdamW (betas 0.9 and 0.999,
   * epsilon 1e-8, decoupled decay 1e-4). Moments persist across calls.
   * Each column is one example, as for fit().
   */
  double fitAdamW(const Matrix& observations, const Matrix& previousPlans,
                  const Matrix& targets, size_t gradientSteps, size_t batchSize,
                  double learningRate, double cosineWeight);

  /**
   * Collect closed-loop examples and fit the flow once (Algorithm 1).
   * Each column of initialStates starts a separate episode. scoreBatch(x,U)
   * returns one cost per sequence in U, for example from a factor graph.
   * weights(costs) supplies the search rule's normalized weights. advance
   * applies the updated plan's first action. Repeated calls alternate data
   * collection using the current model and supervised flow matching.
   */
  double trainCycle(
      const Matrix& initialStates, const GPCTrainingParams& params,
      const std::function<Vector(const Vector&)>& observe,
      const std::function<Vector(const Vector&, const Vector&)>& advance,
      const std::function<Vector(const Vector&, const Matrix&)>& scoreBatch,
      const std::function<Vector(const Vector&)>& weights);

  /**
   * Aggregate domain-randomized rollouts, one row per candidate and one
   * column per equally likely domain. tailFraction=1 gives the mean and
   * tailFraction=0 the maximum. Intermediate values give upper-tail CVaR.
   */
  static Vector riskCosts(const Matrix& domainCosts, double tailFraction);

  /// Number of controls in one physical action.
  size_t controlDim() const { return controlDim_; }

 private:
  friend class GPCProposal;
  Eigen::Index observationDim_, controlDim_, horizon_;
  Matrix inputWeights_, outputWeights_;
  Vector hiddenBias_, outputBias_;
  mutable std::mt19937_64 generator_;
  Vector firstMoment_, secondMoment_;
  size_t adamUpdates_ = 0;

  double fitBatch(const Matrix& observations, const Matrix& previousPlans,
                  const Matrix& targets, size_t gradientSteps, size_t batchSize,
                  double learningRate, double cosineWeight, bool useAdamW);

  Matrix generateWithRng(const Vector& observation, const Matrix& previous,
                         double warmStart, size_t flowSteps,
                         std::mt19937_64& generator) const;
  Vector features(const Vector& sequence, const Vector& observation,
                  double time) const;
  void checkInputs(const Vector& observation, const Matrix& previous) const;
};

/**
 * GPC+ proposal for SamplingPlanner. Gaussian candidates use the current
 * fitted search distribution and additional candidates come from a trained
 * flow. The proposal seed drives both candidate sources, so copies produce
 * paired draws for identical observations. The fitted policy is shared with the
 * proposal. Construct with the observation for the current MPC tick.
 */
class GTSAM_UNSTABLE_EXPORT GPCProposal : public PolicyAugmentedProposal {
 public:
  /// Configure both candidate sources for one observed state.
  GPCProposal(std::shared_ptr<GenerativePredictiveControl> policy,
              const Vector& observation, const Matrix& covariance,
              size_t gaussianSamples, size_t policySamples, double warmStart,
              size_t flowSteps, size_t seed = 42);

  /// Preserve the fitted policy and both proposal streams for coupled rollouts.
  std::unique_ptr<GaussianProposal> clone() const override {
    return std::make_unique<GPCProposal>(*this);
  }
};

}  // namespace gtsam

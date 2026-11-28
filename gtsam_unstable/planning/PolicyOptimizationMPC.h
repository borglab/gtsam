// Copyright (c) Georgia Tech Research Corporation. See LICENSE for details.
/**
 * @file PolicyOptimizationMPC.h
 * @author Frank Dellaert
 * @author Akash Ratheesh
 */
#pragma once

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/planning/EliteMPPI.h>
#include <gtsam/planning/GaussianNoiseSampler.h>
#include <gtsam_unstable/dllexport.h>

#include <limits>
#include <memory>

namespace gtsam {

/// Hyperparameters for PO-MPC with fully observed vector states and Gaussian
/// actions.
struct GTSAM_UNSTABLE_EXPORT PolicyOptimizationMPCParams {
  size_t horizon = 5, samples = 64, policySamples = 16, elites = 12;
  size_t iterations = 3, hiddenSize = 32, valueBins = 51;
  size_t replayCapacity = 10000, batchSize = 16, updateInterval = 1;
  size_t reanalyzeInterval = 10, warmupSteps = 0, seed = 42;
  double temperature = 0.5, minStddev = 0.05, maxStddev = 1.0;
  double discount = 0.99, temporalDecay = 0.5, klWeight = 0.1;
  double entropyWeight = 0.001, learningRate = 0.001, targetMix = 0.01;
  double valueLimit = 100.0;
  /// Symmetric bound on each physical action coordinate, applied before
  /// rollout.
  double actionLimit = std::numeric_limits<double>::infinity();
  bool learnedPrior = true, forwardPriorKL = false;
  /// Use Adam on clipped mean minibatch gradients for all learned networks.
  bool useAdam = false;
};

/// Selected action and fitted Gaussian parameters along the planning horizon.
struct GTSAM_UNSTABLE_EXPORT PolicyOptimizationPlan {
  Vector action;
  Matrix mean, stddev;
  size_t modelRollouts =
      0;  ///< Completed learned-model trajectories in this search.

 private:
  friend class PolicyOptimizationMPC;
  std::shared_ptr<const NonlinearFactorGraph> taskCost_;
};

/**
 * Policy Optimization–MPC (Serra-Gomez et al., ICML 2026, Algorithm 2).
 * A learned transition and reward model, two categorical critics with target
 * networks, and diagonal Gaussian actor and adaptive prior are trained on
 * replay. This CPU implementation uses identity state encoding and single
 * hidden-layer tanh networks. All plan state/control keys hold gtsam::Vector.
 * Instances own their random streams, warm start, networks, and replay.
 */
class GTSAM_UNSTABLE_EXPORT PolicyOptimizationMPC {
 public:
  /// Construct and seed all networks for the given continuous spaces.
  PolicyOptimizationMPC(size_t stateDim, size_t controlDim,
                        const PolicyOptimizationMPCParams& params);
  ~PolicyOptimizationMPC();
  PolicyOptimizationMPC(const PolicyOptimizationMPC&) = delete;
  PolicyOptimizationMPC& operator=(const PolicyOptimizationMPC&) = delete;

  /// Plan using predicted discounted rewards and the terminal bootstrap critic.
  PolicyOptimizationPlan plan(const Vector& state);
  /// Subtract a known factor-graph cost over X(0:H) and U(0:H-1) from each
  /// return.
  PolicyOptimizationPlan plan(const Vector& state,
                              const NonlinearFactorGraph& taskCost);
  /// Search from an explicit nominal sequence, for decision intervals shorter
  /// than a model transition. Each column is one model step.
  PolicyOptimizationPlan planFrom(const Vector& state, const Matrix& nominal);
  /** Evaluate a supplied search budget on the current networks. The sampler
   * supplies the Gaussian controls and its initial random-stream position.
   * The actor supplies the first policySamples trajectories of each batch.
   * This explicit search leaves the stored receding-horizon warm start intact.
   */
  PolicyOptimizationPlan searchFrom(const Vector& state, const Matrix& nominal,
                                    const GaussianNoiseSampler& sampler,
                                    size_t policySamples,
                                    const EliteMPPI& update, size_t iterations);
  /// Copy fitted networks into a separately seeded instance with an empty
  /// replay buffer and a fresh planning warm start.
  std::shared_ptr<PolicyOptimizationMPC> fork(size_t seed) const;
  /// Predict the next state followed by the scalar reward for one model step.
  Vector predict(const Vector& state, const Vector& action) const;
  /// Record execution of plan.action, update on schedule and reset warm start
  /// at termination.
  void observe(const Vector& state, const PolicyOptimizationPlan& plan,
               double reward, const Vector& nextState, bool terminal);
  /// Draw replay batches and update model, critics, prior, actor and target
  /// networks.
  void train(size_t updates);
  /// Start a new episode and clear the planning warm start. Replay sequences
  /// end at the last observation. A nonterminal observation still bootstraps
  /// from its recorded next state, as required at a time limit.
  void reset();
  /// Number of transitions available for training.
  size_t replaySize() const;
  /// Mean action of the learned sampling policy for the given state.
  Vector policyMean(const Vector& state) const;
  /// Unregularized terminal action value used for bootstrapping trajectory
  /// returns.
  double bootstrapValue(const Vector& state, const Vector& action) const;
  /// KL-regularized action value used in the sampling-policy objective.
  double regularizedValue(const Vector& state, const Vector& action) const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace gtsam

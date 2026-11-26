// Copyright (c) Georgia Tech Research Corporation. See LICENSE for details.
/**
 * @file TrackingFeedback.h
 * @author Frank Dellaert
 * @author Akash Ratheesh
 */
#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/linear/HessianFactor.h>

#include <array>
#include <unordered_map>
#include <utility>
#include <vector>

namespace gtsam {

/**
 * Minimum-norm contraction-metric feedback for a control-affine system.
 * error is actual minus reference; driftDifference is the velocity
 * difference when both states receive the same nominal control. metric is
 * a constant positive-definite matrix, inputJacobian is B at the actual state.
 */
GTSAM_EXPORT Vector contractionFeedback(const Vector& error,
                                        const Matrix& metric,
                                        const Matrix& inputJacobian,
                                        const Vector& driftDifference,
                                        double rate);

namespace internal {

using StageKeyMap = std::unordered_map<Key, std::pair<size_t, bool>>;

/// Affine controls du = feedforward + gain * dx, indexed in forward time.
struct TrackingControlLaw {
  Matrix feedforward;
  std::vector<Matrix> gains;
};

/// Linearized stage/terminal cost factors partitioned by horizon step from a
/// NonlinearFactorGraph, keeping native GTSAM GaussianFactors.
struct StageLinearizedCosts {
  std::vector<HessianFactor> stageFactors;
  HessianFactor terminalFactor;
};

/// Verify that a precision matrix is finite, symmetric, and PSD (or PD).
GTSAM_EXPORT void checkTrackingPrecision(const Matrix& precision,
                                         bool definite);

/// Validate optional componentwise control limits.
GTSAM_EXPORT void validateControlLimits(Eigen::Index controlDim,
                                        const Vector& lower,
                                        const Vector& upper);

/// Build a key -> (stageIndex, isControl) lookup table and validate horizon
/// key counts.
GTSAM_EXPORT StageKeyMap buildStageKeyMap(const KeyVector& stateKeys,
                                          const KeyVector& controlKeys);

/** Eliminate one deterministic transition and its control in a single stage.
 * Pull the incoming separator on X(k+1) through dx' = A dx + B du, add the
 * local costs and optional DDP curvature, then eliminate U(k). Return the
 * conditional P(du | dx) and leave the separator on X(k) in the stage factor.
 * A null conditional signals an indefinite control block.
 */
GTSAM_EXPORT GaussianConditional::shared_ptr eliminateControlStage(
    const Matrix& A, const Matrix& B, HessianFactor& stage, double damping,
    const HessianFactor& valueToGo,
    const HessianFactor* dynamicsCurvature = nullptr);

/** Eliminate controls from the last stage to the first. Each conditional
 * depends on its stage state; the remaining factor becomes the preceding
 * stage's cost-to-go. An empty policy signals an indefinite control block.
 */
template <class EliminateStage>
GaussianBayesNet eliminateTrackingChain(const StageLinearizedCosts& costs,
                                        const EliminateStage& eliminateStage) {
  GaussianBayesNet policy;
  policy.resize(costs.stageFactors.size());
  // Cholesky leaves the separator in its input factor. Alternating two
  // stage factors lets the next elimination read it directly.
  std::array<HessianFactor, 2> work;
  const HessianFactor* valueToGo = &costs.terminalFactor;
  for (size_t k = policy.size(); k-- > 0;) {
    auto& stage = work[k % 2];
    stage = costs.stageFactors[k];
    policy[k] = eliminateStage(k, stage, *valueToGo);
    if (!policy[k]) return {};
    valueToGo = &stage;
  }
  return policy;
}

/// Retry a failed sequential elimination with increasing control damping.
template <class Eliminate, class IncreaseDamping>
GaussianBayesNet eliminateWithDamping(double& damping, double limit,
                                      const Eliminate& eliminate,
                                      const IncreaseDamping& increaseDamping) {
  auto policy = eliminate(damping);
  while (policy.empty() && damping < limit) {
    damping = increaseDamping(damping);
    policy = eliminate(damping);
  }
  return policy;
}

/** Solve each conditional R du = d - S dx for its affine control law. The
 * forward rollout substitutes successive state deviations into these laws.
 */
GTSAM_EXPORT TrackingControlLaw
solveConditionals(const GaussianBayesNet& policy);

}  // namespace internal
}  // namespace gtsam

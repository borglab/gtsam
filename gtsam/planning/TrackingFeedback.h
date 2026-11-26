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

struct TrackingSweep {
  GaussianBayesNet policy;
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
 * conditional P(du | dx) and replace valueToGo with the separator on X(k).
 * A null conditional signals an indefinite control block.
 */
GTSAM_EXPORT GaussianConditional::shared_ptr eliminateControlStage(
    const Matrix& A, const Matrix& B, const HessianFactor& stageCost,
    double damping, HessianFactor& valueToGo,
    const HessianFactor* dynamicsCurvature = nullptr);

/// Extract feedforward step and feedback gain from a GaussianConditional
/// P(du_k | dx_k) using an upper-triangular solve on R.
GTSAM_EXPORT void extractControlLaw(const GaussianConditional& conditional,
                                    Eigen::Ref<Vector> feedforward,
                                    Matrix& gain);

}  // namespace internal
}  // namespace gtsam

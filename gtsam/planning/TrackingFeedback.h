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
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

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
  std::vector<GaussianFactorGraph> stageFactors;
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

/// Linearize a NonlinearFactorGraph at values and partition its stage/terminal
/// GaussianFactors by time step.
GTSAM_EXPORT StageLinearizedCosts
linearizeStageCosts(const NonlinearFactorGraph& cost, const Values& values,
                    const KeyVector& stateKeys, const KeyVector& controlKeys,
                    const StageKeyMap& keyMap, Eigen::Index stateDim);

/// Pull a unary cost-to-go HessianFactor on X(k+1) back through linear
/// dynamics dx_{k+1} = A dx_k + B du_k into a binary HessianFactor on
/// (controlKey, stateKey).
GTSAM_EXPORT HessianFactor pullBackDynamics(Key stateKey, Key controlKey,
                                            const Matrix& A, const Matrix& B,
                                            const HessianFactor& nextValueToGo);

/// Eliminate controlKey from a binary HessianFactor on (controlKey, stateKey)
/// via HessianFactor::eliminateCholesky, returning the conditional
/// P(du_k | dx_k) (or nullptr if indefinite) and the remaining
/// Schur-complement HessianFactor on stateKey.
GTSAM_EXPORT std::pair<GaussianConditional::shared_ptr, HessianFactor>
eliminateControlFactor(HessianFactor stageFactor, Key controlKey);

/// Extract feedforward step and feedback gain from a GaussianConditional
/// P(du_k | dx_k) using an upper-triangular solve on R.
GTSAM_EXPORT void extractControlLaw(const GaussianConditional& conditional,
                                    Eigen::Ref<Vector> feedforward,
                                    Matrix& gain);

}  // namespace internal
}  // namespace gtsam

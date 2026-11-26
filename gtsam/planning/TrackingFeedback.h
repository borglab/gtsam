// Copyright (c) Georgia Tech Research Corporation. See LICENSE for details.
/**
 * @file TrackingFeedback.h
 * @author Frank Dellaert
 * @author Akash Ratheesh
 */
#pragma once
#include <gtsam/base/Matrix.h>

#include <functional>
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

/**
 * Differential dynamic programming for Euclidean trajectory tracking with a
 * supplied discrete transition. Central differences provide first and second
 * dynamics derivatives. The backward sweep computes feedforward corrections
 * and a time-varying feedback gain. The forward pass searches step lengths.
 */
class GTSAM_EXPORT DDPTracking {
 public:
  using Transition = std::function<Vector(const Vector&, const Vector&)>;
  /// Supply running state/control and terminal state precision matrices.
  DDPTracking(const Matrix& statePrecision, const Matrix& controlPrecision,
              const Matrix& terminalPrecision);
  /** Fit H controls and H gains to H+1 target states from the actual initial
   * state. Optional componentwise control limits are applied by clamping
   * during every forward rollout, including the initialization. Note that this
   * is a forward-rollout projection rather than a box-constrained backward
   * pass, so active bounds can cause the search to stall at a non-stationary
   * point.
   */
  Matrix optimize(const Vector& initial, const Matrix& targetStates,
                  const Matrix& targetControls, const Transition& transition,
                  size_t iterations, const Vector& lower = {},
                  const Vector& upper = {});
  /// Optimized state trajectory with columns ordered in time.
  const Matrix& states() const { return states_; }
  /// Gain multiplying actual minus reference state at a given step.
  const Matrix& gain(size_t step) const { return gains_.at(step); }

 private:
  Matrix q_, r_, terminal_, states_;
  std::vector<Matrix> gains_;
};

}  // namespace gtsam

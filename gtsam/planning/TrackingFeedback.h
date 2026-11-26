// Copyright (c) Georgia Tech Research Corporation. See LICENSE for details.
/**
 * @file TrackingFeedback.h
 * @author Frank Dellaert
 * @author Akash Ratheesh
 */
#pragma once

#include <gtsam/base/Matrix.h>

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

struct TrackingSweep {
  Matrix feedforward;
  std::vector<Matrix> gains;
};

/// Verify that a precision matrix is finite, symmetric, and PSD (or PD).
GTSAM_EXPORT void checkTrackingPrecision(const Matrix& precision,
                                         bool definite);

/// Validate optional componentwise control limits.
GTSAM_EXPORT void validateControlLimits(Eigen::Index controlDim,
                                        const Vector& lower,
                                        const Vector& upper);

}  // namespace internal
}  // namespace gtsam

/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */

/**
 * @file ConstrainedQR.h
 * Private adaptation of Eigen QR to constrained elimination fronts of any size.
 * GTSAM handles exact equations and rank decisions; Eigen handles orthogonal
 * transformations. Keeping the workspace here separates it from noise models.
 */

#pragma once

#include <gtsam/linear/NoiseModel.h>

namespace gtsam::noiseModel::internal {

/**
 * Eliminate a local augmented Jacobian in its original column order. Exact
 * equations take priority over finite observations. A suitable finite remainder
 * is delegated to Eigen::HouseholderQR. Rank-deficient or poorly scaled systems
 * use Eigen Householder operations one column at a time, with row pivoting.
 * No normal equations are formed and the original weighted rank test is kept.
 */
SharedDiagonal constrainedQR(Matrix& augmented, const Vector& sigmas,
                             const Vector& mu);

}  // namespace gtsam::noiseModel::internal

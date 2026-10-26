/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file CholeskyCheck.h
 * @brief Compare Cholesky pivots with local or operation-wide reference diagonals.
 * @author Frank Dellaert
 */

#pragma once

#include <limits>

namespace gtsam::internal {

/** Shared, dimensionless safeguard for normal-equation Cholesky pivots. */
struct CholeskyCheck {
  // Empirical safeguard, not a rank or condition-number estimate. This admits
  // the usable ~1e-8 pivots in #2856 while rejecting the ~2e-10 regression in
  // #2653. Keep this value shared by every Cholesky implementation.
  static constexpr double minimumNormalizedPivotSquared = 0x1p-28;

  /// Test a squared pivot against the diagonal selected by the caller's policy.
  static bool acceptSquared(
      double pivotSquared, double referenceDiagonal) {
    return positiveFinite(referenceDiagonal) && positiveFinite(pivotSquared) &&
           pivotSquared >= minimumNormalizedPivotSquared * referenceDiagonal;
  }

  /// Test a Cholesky diagonal without overflowing when squaring it.
  static bool accept(double pivot,
                     double referenceDiagonal) {
    if (!positiveFinite(pivot) || !positiveFinite(referenceDiagonal))
      return false;
    // Division before multiplication also handles very small valid systems.
    return (pivot / referenceDiagonal) * pivot >= minimumNormalizedPivotSquared;
  }

 private:
  /// Return whether a value is finite and strictly positive.
  static bool positiveFinite(double value) {
    return value > 0.0 && value <= std::numeric_limits<double>::max();
  }
};

}  // namespace gtsam::internal

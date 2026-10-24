/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. See LICENSE for the license information.
 * -------------------------------------------------------------------------- */

#pragma once

#include <limits>

#ifdef __CUDACC__
#define GTSAM_CHOLESKY_HOST_DEVICE __host__ __device__
#else
#define GTSAM_CHOLESKY_HOST_DEVICE
#endif

namespace gtsam::internal {

/** Shared, dimensionless safeguard for normal-equation Cholesky pivots. */
struct CholeskyCheck {
  // Empirical safeguard, not a rank or condition-number estimate. This admits
  // the usable ~1e-8 pivots in #2856 while rejecting the ~2e-10 regression in
  // #2653. Keep this value shared by every Cholesky implementation.
  static constexpr double minimumNormalizedPivotSquared = 0x1p-28;

  /** Test a squared pivot against its diagonal before any elimination. */
  GTSAM_CHOLESKY_HOST_DEVICE static bool acceptSquared(
      double pivotSquared, double referenceDiagonal) {
    return positiveFinite(referenceDiagonal) && positiveFinite(pivotSquared) &&
           pivotSquared >= minimumNormalizedPivotSquared * referenceDiagonal;
  }

  /** Test a Cholesky diagonal without overflowing when squaring it. */
  GTSAM_CHOLESKY_HOST_DEVICE static bool accept(double pivot,
                                                double referenceDiagonal) {
    if (!positiveFinite(pivot) || !positiveFinite(referenceDiagonal))
      return false;
    // Division before multiplication also handles very small valid systems.
    return (pivot / referenceDiagonal) * pivot >= minimumNormalizedPivotSquared;
  }

 private:
  GTSAM_CHOLESKY_HOST_DEVICE static bool positiveFinite(double value) {
    return value > 0.0 && value <= std::numeric_limits<double>::max();
  }
};

}  // namespace gtsam::internal

#undef GTSAM_CHOLESKY_HOST_DEVICE

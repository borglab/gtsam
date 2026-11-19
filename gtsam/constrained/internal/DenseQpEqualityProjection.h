/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */

/**
 * @file DenseQpEqualityProjection.h
 * Private preparation of a dense QP in equality-feasible coordinates.
 * Keeping this setup separate from DenseQpSolver isolates one-time affine
 * elimination from the mutable inequality working-system factorization.
 */
#pragma once

#include <gtsam/constrained/internal/DenseQpSolver.h>
#include <optional>

namespace gtsam::internal {

/** Per-call reduction x = point + nullspace*y, with orthonormal columns.
 * Singleton equalities are substituted exactly so fixed coordinates have
 * literal zero rows in the null-space matrix. Original input rows are retained
 * by the owning solver for consistency checks and multiplier recovery.
 */
struct DenseQpEqualityProjection {
  DenseQpData reduced;
  Vector point;
  Matrix nullspace;
  // Retain the equality QR already computed during projection for multipliers.
  Matrix equalityQ, equalityR;
  Vector equalityScales;
  std::vector<Eigen::Index> freeColumns, fixedRows, equalityBasis;
  size_t numericalFactorizations = 0;
  /** Recover original equality multipliers from this elimination, including
   * exact fixed coordinates. The gradient has inequality forcing subtracted.
   * No new factorization is needed; the caller checks original stationarity.
   */
  Vector multipliers(const DenseQpData& source, const Vector& gradient) const;
};

/** Eliminate permanent equalities once, or return nullopt when there are none.
 * Project square-root objective rows directly, compact only exactly constant
 * residuals, and preserve the requested proximal weight in orthonormal
 * coordinates. An ambiguous roundoff-sized constraint projection retains the
 * original dense working system instead of truncating coefficients.
 * Inconsistent original equalities throw std::runtime_error.
 */
std::optional<DenseQpEqualityProjection> projectDenseQpEqualities(
    const DenseQpData& source);

/** Check original equalities and selected inequalities as boundary equations.
 * Uses the same scale-aware consistency allowance as the dense working solver;
 * no original row is discarded by the reduced numerical representation.
 */
void checkDenseQpOriginalWorkingSet(const DenseQpData& source,
                                  const Vector& value,
                                  const std::vector<bool>& active);

}  // namespace gtsam::internal

/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
#pragma once

#include <gtsam/constrained/internal/DenseQpSolver.h>
#include <gtsam/constrained/internal/SparseQpSolver.h>
#include <memory>

namespace gtsam::internal {

/** Per-call dual active-set workspace. Both backends eliminate permanent
 * equalities before exploring inequalities. Sparse applies cached constrained
 * inverse solves and stores responses only for working rows.
 * Mathematical multipliers here are nonnegative for A*x <= b.
 */
class GTSAM_EXPORT DualQpSolver {
 public:
  /** Diagnostic counters for native tests and benchmarks, not public wrappers. */
  struct Statistics {
    size_t objectiveFactorizations = 0, responseSolves = 0, rhsSolves = 0;
    size_t rowAdditions = 0, rowRemovals = 0, workingRebuilds = 0;
  };
  /** Construct sparse algebra, or use the supplied Dense workspace and its
   * packing order. The Dense workspace must outlive this object.
   */
  explicit DualQpSolver(SparseQpData data, DenseQpSolver* dense = nullptr,
                        const Ordering& denseOrdering = {});
  /// Release all per-call caches.
  ~DualQpSolver();
  /// Rank-aware dimension of the permanent equality tangent space.
  size_t reducedDimension() const;
  /// Factor proximal curvature; return false only for singular curvature.
  bool prepare();
  /** Solve one proximal QP, updating original-row nonnegative multipliers and
   * the working mask. Every working solve consumes the shared iteration budget.
   * Initial multipliers must be finite, nonnegative and zero outside the mask.
   */
  VectorValues solve(const VectorValues& center, Vector* multipliers,
                     std::vector<bool>* active, size_t* iterations,
                     size_t maxIterations, double feasibilityTolerance,
                     double multiplierTolerance);
  /// Recover original equality multipliers from residual stationarity forcing.
  Vector equalityMultipliers(const VectorValues& gradient, double tolerance) const;
  /// Inspect numerical reuse without changing this workspace.
  const Statistics& statistics() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace gtsam::internal

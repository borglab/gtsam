/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
#pragma once

#include <gtsam/constrained/internal/DualQpSolver.h>
#include <stdexcept>

namespace gtsam::internal {

/** An uncertain transformed solve requests the existing original-coordinate
 * algorithm. Verified infeasibility and iteration limits never use this exit.
 */
class WhiteningFailure : public std::runtime_error {
 public:
  /// Describe a numerical reason for retaining the original-coordinate solver.
  explicit WhiteningFailure(const char* reason) : std::runtime_error(reason) {}
};

/** DAQP Algorithm 1 in square-root coordinates. Equality rows are immutable
 * and their multipliers unrestricted. Only certification recovers variables.
 * This workspace owns working-row LDLT updates, never Sparse numerical state.
 */
class GTSAM_EXPORT DenseDualQpSolver {
 public:
  /** Use curvature and transformed constraints cached by the Dense workspace.
   * Both references must outlive this workspace.
   */
  DenseDualQpSolver(DenseQpSolver& dense, DualQpSolver::Statistics& statistics);
  /// Release this call's working-set cache.
  ~DenseDualQpSolver();
  /// Solve one proximal center using the shared working-solve budget.
  Vector solve(const Vector& center, Vector* multipliers, std::vector<bool>* active,
               size_t* iterations, size_t maxIterations,
               double feasibilityTolerance, double multiplierTolerance);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace gtsam::internal

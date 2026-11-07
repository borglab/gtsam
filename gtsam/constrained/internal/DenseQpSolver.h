/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file DenseQpSolver.h
 * Internal production backend selected by QpSolverType::Dense.
 *
 * ActiveSetSolver packs the factor-graph inputs and owns feasibility search,
 * initialization, working-set decisions, and original-QP convergence checks.
 * This header separates those shared decisions from dense factorization reuse
 * through immutable packed inputs and a workspace owned by each solve call.
 * The declarations stay in gtsam::internal and are excluded from the language
 * wrappers; callers select Dense through the existing public QP options.
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/dllexport.h>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace gtsam {
namespace internal {

/** Packed dense working-system inputs, internal to ActiveSetSolver. The
 * objective is 0.5*||jacobian*x-rhs||^2-linear.dot(x); all original constraint
 * rows remain in normals/bounds. Equalities precede inequality rows.
 */
struct DenseQpData {
  Matrix jacobian;
  Vector rhs, linear;
  Matrix normals;
  Vector bounds;
  size_t equalities = 0;
  double regularization = 0;
};

/** Production dense workspace constructed by ActiveSetSolver for Dense mode.
 * This internal contract keeps
 * factorization choices out of the public QP API. It never changes objective
 * curvature, constraint rows, or the requested proximal weight.
 */
class GTSAM_EXPORT DenseQpSolver {
 public:
  /** Internal counters used by native regression tests and local benchmarks;
   * these are not exposed through ActiveSetSolver or language wrappers.
   */
  struct Statistics {
    size_t numericalFactorizations = 0, objectiveFactorizations = 0;
    size_t workingRebuilds = 0, rowAdditions = 0, rowRemovals = 0;
    size_t transformedNormals = 0, dependentRows = 0, rhsOnlySolves = 0, solves = 0;
    std::map<std::string, size_t> rebuildReasons;
  };
  /// Read this call's counters without changing the solver state.
  const Statistics& statistics() const;
  /** Cache curvature and update independent constraint normals, retaining a
   * cached null-space fallback. The immutable data must outlive this workspace.
   */
  explicit DenseQpSolver(const DenseQpData& data);
  /// Release this optimization call's numerical workspace.
  ~DenseQpSolver();
  /** Solve the specified working face, recentering only the objective RHS.
   * The returned point is checked against every selected original row.
   */
  Vector solve(const std::vector<bool>& active, const Vector& center);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace internal
}  // namespace gtsam

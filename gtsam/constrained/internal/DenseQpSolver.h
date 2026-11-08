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
 * Permanent affine equalities can be eliminated once; only inequality boundaries
 * participate in subsequent working-system updates. Original rows remain
 * authoritative for feasibility checks and multiplier recovery.
 * The declarations stay in gtsam::internal and are excluded from the language
 * wrappers; callers select Dense through the existing public QP options.
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/dllexport.h>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace gtsam {
namespace internal {

struct DenseQpEqualityProjection;

/** Packed dense working-system inputs, internal to ActiveSetSolver. The
 * objective is 0.5*||jacobian*x-rhs||^2-linear.dot(x); all original constraint
 * rows remain in normals/bounds. Equalities precede inequality rows.
 */
struct DenseQpData {
  Matrix jacobian;
  /// Optional well-conditioned, positive-definite native Hessian. Present only
  /// for Hessian-only objectives without equalities; jacobian retains its root.
  Matrix nativeHessian;
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
    size_t choleskyFactorizations = 0;
    size_t workingRebuilds = 0, rowAdditions = 0, rowRemovals = 0;
    size_t transformedNormals = 0, dependentRows = 0, rhsOnlySolves = 0, solves = 0;
    std::map<std::string, size_t> rebuildReasons;
  };
  /// Read this call's counters without changing the solver state.
  const Statistics& statistics() const;
  /** Optionally eliminate permanent equalities, cache curvature, and update
   * independent inequality normals, retaining a cached null-space fallback. Exact fixed
   * coordinates and supplied square-root costs are preserved. Working LDLT
   * factors support bordering, deletion and RHS-only solves; the objective
   * remains in square-root form. Original residuals refine cached solves.
   * With projectEqualities=false, retain original coordinates and exact
   * equality pivots. The immutable data must outlive this workspace.
   */
  explicit DenseQpSolver(const DenseQpData& data,
                          bool projectEqualities = true);
  /// Release this optimization call's numerical workspace.
  ~DenseQpSolver();
  /** Solve the specified working face, recentering only the objective RHS.
   * The returned point is checked against every selected original row.
   */
  Vector solve(const std::vector<bool>& active, const Vector& center);

  /** Read this call's permanent equality preparation without recomputing it.
   * Feasibility and tangent-cone initialization may share its affine point and
   * orthonormal nullspace with the main working solves. Return nullptr when
   * projection is disabled, there are no equalities, or their projection is
   * numerically ambiguous.
   * The returned pointer remains valid for this workspace's lifetime.
   */
  const DenseQpEqualityProjection* equalityProjection() const;

  /** Whether the last working solve has reusable primal/equality factors.
   * Singular objectives retain graph recovery. For dependent inequalities,
   * only a gauge with valid signs may be reused; other gauges retain graph
   * recovery and its existing row-release decisions.
   */
  bool canRecoverMultipliers() const;
  /** Reuse the primal LDLT multipliers and the cached equality QR, in original
   * row order, with zeros for inactive rows. The caller supplies the candidate
   * and original, unregularized gradient in packed variable order. No numerical
   * factorization is added. Return nullopt when caches are unsuitable or the
   * original stationarity or conservative absolute-gap reuse guard fails.
   */
  std::optional<Vector> multipliers(const Vector& value, const Vector& gradient,
                                    double tolerance) const;

  /// Remaining dimension after permanent equality elimination.
  size_t reducedDimension() const;
  /// Prepare square-root curvature; return false if it is not invertible.
  bool prepareDual();
  /** Minimize the equality-constrained objective minus forcing.dot(x).
   * Reuse fixed curvature and equality factors, ignoring inequalities.
   */
  Vector solveDual(const Vector& center, const Vector& forcing);
  /// Apply the equality-constrained inverse to a homogeneous linear forcing.
  Vector dualResponse(const Vector& forcing) const;
  /// Recover equality multipliers from a gradient with inequality forcing removed.
  Vector equalityMultipliers(const Vector& gradient) const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace internal
}  // namespace gtsam

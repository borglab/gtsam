/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file SparseQpSolver.h
 * Internal production backend selected by QpSolverType::Sparse.
 *
 * This workspace owns sparse working-graph assembly, ordering, and optional
 * GaussianISAM2 reuse for one optimization call. ActiveSetSolver retains
 * feasibility search, initialization, blocking, multiplier recovery, and the
 * original-QP convergence checks. There is no separate active-set algorithm
 * or public backend hierarchy here.
 */

#pragma once

#include <gtsam/dllexport.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>

#include <map>
#include <memory>
#include <optional>
#include <vector>

namespace gtsam {
namespace internal {

/** Factors, variable dimensions, and options for sparse QP subproblems.
 * Graph containers hold shared pointers to factors supplied by ActiveSetSolver;
 * copying this data copies the pointers rather than the factors or matrices.
 * The backend leaves these inputs unchanged while updating separate caches.
 * Constraint graphs contain normalized scalar exact rows in their original
 * multiplier order. The caller supplies validated dimensions and a complete
 * optional ordering.
 */
struct SparseQpData {
  GaussianFactorGraph objective, equalities, inequalities;
  std::map<Key, size_t> keyDimensions;
  double regularization = 0;
  bool useIncremental = true;
  std::optional<Ordering> ordering;
};

/** Per-call sparse QP workspace, used by ActiveSetSolver in Sparse mode.
 * Jacobian-native objectives retain square-root elimination. Active row changes
 * update the cached tree; proximal-center changes use its cached RHS solve.
 * Both paths apply one original-gradient residual correction through cached
 * triangular solves to improve weak-direction accuracy without another
 * numerical elimination. The batch option rebuilds the same working graph
 * and factorization for each call, using the same elimination policy.
 */
class GTSAM_EXPORT SparseQpSolver {
 public:
  /** Take ownership of the graph containers and options without elimination.
   * Factor objects remain shared and are never modified by this backend.
   * Mutable orderings and tree caches belong exclusively to this workspace.
   */
  explicit SparseQpSolver(SparseQpData data);

  /// Release this optimization call's sparse numerical workspace.
  ~SparseQpSolver();

  /** Solve the selected working face around the supplied proximal center.
   * The mask follows the original inequality graph order; all equalities stay
   * enforced. Restrict the requested ordering and recreate the tree only when
   * the graph's variable set changes. Do not add curvature beyond the supplied
   * regularization. Numerical failures propagate; discard a failed workspace.
   */
  VectorValues solve(const std::vector<bool>& active, const VectorValues& center);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace internal
}  // namespace gtsam

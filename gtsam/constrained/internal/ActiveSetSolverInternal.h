/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file ActiveSetSolverInternal.h
 * Definitions shared by the ActiveSetSolver implementation files.
 *
 * The backend workspaces keep their existing per-call ownership. Small inline
 * row and ordering operations allow setup, initialization, and iteration to
 * use the same private state without introducing another cache or data copy.
 * This header is an implementation detail and is excluded from the wrappers.
 */

#pragma once

#include <gtsam/constrained/ActiveSetSolver.h>
#include <gtsam/constrained/internal/DenseQpSolver.h>
#include <gtsam/constrained/internal/DenseQpEqualityProjection.h>
#include <gtsam/constrained/internal/SparseQpSolver.h>
#include <gtsam/constrained/internal/DualQpSolver.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace gtsam {

// Dense inputs are packed once behind the factor-graph API. Mutable numerical
// workspaces live in SolveWorkspace, which belongs to one optimization call.
struct ActiveSetSolver::DenseQpWorkspace {
  Ordering hessianOrdering;
  std::map<Key, size_t> keyOffsets;
  size_t totalDim = 0;
  internal::DenseQpData data;
};

// Each optimization call owns its backend and multiplier caches. LPs retain
// their batch projection ordering; both QP backends share multiplier recovery.
struct ActiveSetSolver::SolveWorkspace {
  std::unique_ptr<internal::DenseQpSolver> dense;
  std::unique_ptr<internal::SparseQpSolver> sparse;
  std::unique_ptr<internal::DualQpSolver> dual;
  std::optional<VectorValues> fixedPoint;
  bool formulationPrepared = false;
  KeySet lpKeys, dualKeys;
  Ordering lpOrdering, dualOrdering;
};

namespace internal {

/** Preserve the runtime_error contract while distinguishing exhausted auxiliary
 * solves from numerical failure in computeFeasibleStep.
 */
struct IterationLimitError : std::runtime_error {
  using std::runtime_error::runtime_error;
};

/** Restrict a complete requested ordering to the current graph keys while
 * preserving their relative order.
 */
inline Ordering restrictOrdering(const Ordering& requested, const KeySet& keys) {
  Ordering result;
  for (Key key : requested)
    if (keys.count(key)) result.push_back(key);
  return result;
}

/// Represent a scalar right-hand side as a one-element vector.
inline Vector1 SingleValue(double value) { return Vector1{value}; }

/// Evaluate the normalized scalar constraint residual A*x-b.
inline double RowResidual(const JacobianFactor& factor,
                          const VectorValues& values) {
  return factor.unweighted_error(values)(0);
}

/// Choose a synthetic key after the original problem keys.
inline Key NextAvailableKey(const std::map<Key, size_t>& keyDims) {
  if (keyDims.empty()) return 1;
  return keyDims.rbegin()->first + 1;
}

/** Check every original equality and inequality with the supplied tolerance.
 * Accept private ConstraintRow containers without exposing their row type.
 */
template <typename Row>
bool primalFeasible(const std::vector<Row>& equalities,
                    const std::vector<Row>& inequalities,
                    const VectorValues& values, double tolerance) {
  for (const Row& row : equalities) {
    const double residual = RowResidual(*row.factor, values);
    if (!std::isfinite(residual) || std::abs(residual) > tolerance)
      return false;
  }
  for (const Row& row : inequalities) {
    const double residual = RowResidual(*row.factor, values);
    if (!std::isfinite(residual) || residual > tolerance) return false;
  }
  return true;
}

/// Measure an infinity-norm displacement, rejecting nonfinite values.
inline double maximumStep(const VectorValues& first, const VectorValues& second) {
  double step = 0;
  for (const auto& [key, value] : first) {
    if (!value.allFinite() || !second.at(key).allFinite())
      throw std::runtime_error("ActiveSetSolver: non-finite feasible step.");
    step = std::max(step, (value - second.at(key)).lpNorm<Eigen::Infinity>());
  }
  return step;
}

}  // namespace internal
}  // namespace gtsam

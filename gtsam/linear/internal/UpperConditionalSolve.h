/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file UpperConditionalSolve.h
 * @brief Internal allocation-aware upper-conditional solve kernel.
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/inference/Key.h>
#include <gtsam/linear/linearExceptions.h>

namespace gtsam::internal {

/** Solve R*x = d - S*parents and reject an indeterminate result. */
template <class RDerived, class SDerived, class DDerived>
void solveUpperConditional(const Eigen::MatrixBase<RDerived>& R,
                           const Eigen::MatrixBase<SDerived>& S,
                           const Eigen::MatrixBase<DDerived>& d,
                           const Vector* parents, Key nearbyKey,
                           Vector* result) {
  result->resize(d.rows());
  result->noalias() = d;
  if (parents && parents->size() != 0) {
    result->noalias() -= S * (*parents);
  }
  R.derived().template triangularView<Eigen::Upper>().solveInPlace(*result);
  if (result->hasNaN()) throw IndeterminateSystemException(nearbyKey);
}

}  // namespace gtsam::internal

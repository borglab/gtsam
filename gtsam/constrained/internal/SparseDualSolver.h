/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SparseDualSolver.h
 * Sparse multiplier linear solves shared by both QP backends and LPs.
 *
 * This internal function solves a supplied stationarity factor graph and reports
 * its residual. It preserves the graph QR fallback for nonunique multipliers;
 * ActiveSetSolver decides whether to release a row or accept a QP certificate.
 */

#pragma once

#include <gtsam/dllexport.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>

namespace gtsam {

namespace internal {

/** A multiplier solution and its measured stationarity error.
 * Multipliers may be nonunique. The residual distinguishes a valid multiplier
 * set from an inconsistent stationarity equation.
 */
struct DualSolveResult {
  VectorValues multipliers;
  double residual = 0.0;
  double threshold = 0.0;

  /// Check the measured stationarity residual against its acceptance threshold.
  bool consistent() const { return residual <= threshold; }
};

/** Solve the multiplier graph using the supplied ordering and tolerances.
 * Rank-deficient systems use clique-local constrained QR to select free
 * multiplier coordinates. The original graph residual certifies consistency.
 * Disabling the stationarity check preserves the LP fast path: an ordinary
 * successful solve returns immediately. The rank-deficient fallback always
 * checks consistency. Neither path adds regularization.
 */
GTSAM_EXPORT DualSolveResult solveSparseDualGraph(
    const GaussianFactorGraph& graph, const Ordering& ordering,
    double tolerance, bool checkStationarity = true);

}  // namespace internal
}  // namespace gtsam

/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file LMDampingParams.h
 * @brief Parameters controlling LM-style damping for NonlinearMultifrontalSolver.
 * @author Frank Dellaert
 * @date   January 2026
 */

#pragma once

namespace gtsam {

/**
 * Parameters controlling LM-style damping as applied by
 * `gtsam::NonlinearMultifrontalSolver`.
 *
 * @note These parameters are intentionally in a standalone header so they can
 * be referenced from nonlinear optimizer parameter types without pulling in
 * the full solver headers.
 */
struct LMDampingParams {
  /// If true, use diagonal damping (LM) instead of identity damping.
  bool diagonalDamping = false;

  /**
   * If true, use the exact diagonal of the linearized system Hessian
   * `diag(J^T J)` (computed via `GaussianFactorGraph::hessianDiagonal()`) when
   * applying diagonal damping.
   *
   * If false, use the diagonal of each clique's assembled information matrix,
   * which includes Schur complement contributions and can differ from
   * `diag(J^T J)`.
   */
  bool exactHessianDiagonal = false;

  /// Clamp minimum diagonal value used for diagonal damping.
  double minDiagonal = 1e-6;

  /// Clamp maximum diagonal value used for diagonal damping.
  double maxDiagonal = 1e32;
};

}  // namespace gtsam


/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SuiteSparseSolver.h
 *
 * @brief SuiteSparse based linear solver backend for GTSAM
 *
 * Generates a sparse matrix with given ordering and calls the SuiteSparse
 * solver to solve it.
 *
 * @date Jun 2020
 * @author Fan Jiang
 */

#pragma once

#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/LinearSolver.h>
#include <gtsam/linear/VectorValues.h>

#include <string>

namespace gtsam {

/**
 * Eigen SparseSolver based Backend class
 */
class GTSAM_EXPORT SuiteSparseSolver : public LinearSolver {
 protected:
  Ordering ordering_;

 public:
  explicit SuiteSparseSolver(const Ordering &ordering);

  /** Solves the GaussianFactorGraph using a sparse matrix solver
   *
   * Uses elimination ordering during sparse matrix generation
   */
  VectorValues solve(const GaussianFactorGraph &gfg) const override;
};
}  // namespace gtsam

/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SparseEigenSolver.h
 *
 * @brief Eigen SparseSolver based linear solver backend for GTSAM
 *
 * Generates a sparse matrix with given ordering and calls the Eigen sparse
 * matrix solver to solve it.
 *
 * @date Aug 2019
 * @author Mandy Xie
 * @author Fan Jiang
 * @author Frank Dellaert
 * @author Gerry Chen
 */

#pragma once

#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/LinearSolver.h>
#include <gtsam/linear/VectorValues.h>

#include <Eigen/Sparse>
#include <string>

namespace gtsam {

/**
 * Eigen SparseSolver based Backend class
 */
class GTSAM_EXPORT SparseEigenSolver : public LinearSolver {
 public:
  typedef enum {
    QR,
    CHOLESKY
  } SparseEigenSolverType;

 protected:
  SparseEigenSolverType solverType_ = QR;
  Ordering ordering_;

 public:
  explicit SparseEigenSolver(SparseEigenSolver::SparseEigenSolverType type,
                             const Ordering &ordering)
      : solverType_(type), ordering_(ordering) {}

  /** Solves the GaussianFactorGraph using a sparse matrix solver
   *
   * Uses elimination ordering during sparse matrix generation in `solve(gfg)`
   */
  VectorValues solve(const GaussianFactorGraph &gfg) const override;
};
}  // namespace gtsam

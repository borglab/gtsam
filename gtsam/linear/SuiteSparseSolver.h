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
 public:
  typedef enum {
    QR,
    CHOLESKY
  } SuiteSparseSolverType;

 protected:
  SuiteSparseSolverType solverType_ = QR;
  Ordering ordering_;

 public:
  explicit SuiteSparseSolver(SuiteSparseSolver::SuiteSparseSolverType type,
                             const Ordering &ordering);

  bool isIterative() const override;

  bool isSequential() const override;

  VectorValues solve(const GaussianFactorGraph &gfg) const override;
};
}  // namespace gtsam

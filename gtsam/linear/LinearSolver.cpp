/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    LinearSolver.cpp
 * @brief   Common Interface for Linear Solvers
 * @author  Fan Jiang
 */

#include <gtsam/linear/LinearSolver.h>
#include <gtsam/linear/SparseEigenSolver.h>
#include <gtsam/linear/SuiteSparseSolver.h>
#include <gtsam/linear/CuSparseSolver.h>
#include <gtsam/linear/GlobalConstraintSequentialSolver.h>

namespace gtsam {

LinearSolver::LinearSolver() = default;

boost::shared_ptr<LinearSolver> LinearSolver::fromLinearSolverParams(
    const LinearSolverParams &params) {
  switch (params.linearSolverType) {
    case LinearSolverParams::EIGEN_QR:
      return boost::shared_ptr<SparseEigenSolver>(new SparseEigenSolver(
          SparseEigenSolver::SparseEigenSolverType::QR, *params.ordering));
    case LinearSolverParams::EIGEN_CHOLESKY:
      return boost::shared_ptr<SparseEigenSolver>(new SparseEigenSolver(
          SparseEigenSolver::SparseEigenSolverType::CHOLESKY,
          *params.ordering));
    case LinearSolverParams::SUITESPARSE_CHOLESKY:
      return boost::shared_ptr<SuiteSparseSolver>(new SuiteSparseSolver(
          SuiteSparseSolver::SuiteSparseSolverType::CHOLESKY,
          *params.ordering));
    case LinearSolverParams::CUSPARSE_CHOLESKY:
      return boost::shared_ptr<CuSparseSolver>(new CuSparseSolver(
          CuSparseSolver::CuSparseSolverType::CHOLESKY, *params.ordering));
    case LinearSolverParams::GLOBALCONSTRAINT_SEQUENTIAL_QR:
    case LinearSolverParams::GLOBALCONSTRAINT_SEQUENTIAL_CHOLESKY:
      return boost::shared_ptr<GlobalConstraintSequentialSolver>(
          new GlobalConstraintSequentialSolver(params));
    default:
      throw std::invalid_argument(
          "Invalid linear solver type parameter passed");
  }
}
}  // namespace gtsam

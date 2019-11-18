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

namespace gtsam {

  LinearSolver::LinearSolver() = default;

  std::shared_ptr<LinearSolver> LinearSolver::fromLinearSolverParams(const LinearSolverParams &params) {

    boost::optional<const Ordering &> optionalOrdering;
    if (params.ordering) optionalOrdering.reset(*params.ordering);

    if (params.solverType == EIGEN_QR) {
      return std::shared_ptr<SparseEigenSolver>(
              new SparseEigenSolver(SparseEigenSolver::SparseEigenSolverType::QR, *optionalOrdering));
    } else if (params.solverType == EIGEN_CHOLESKY) {
      return std::shared_ptr<SparseEigenSolver>(
              new SparseEigenSolver(SparseEigenSolver::SparseEigenSolverType::CHOLESKY, *optionalOrdering));
    }

    throw std::runtime_error(
            "Invalid parameters passed");

  }
}
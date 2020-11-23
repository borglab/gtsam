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
 * @author  Fan Jiang, Gerry Chen, Mandy Xie, and Frank Dellaert
 */

#include <gtsam/linear/LinearSolver.h>
#include <gtsam/linear/EliminationSolver.h>
#include <gtsam/linear/IterativeSolver.h>
#include <gtsam/linear/PCGSolver.h>
#include <gtsam/linear/SubgraphSolver.h>
#include <gtsam/linear/SparseEigenSolver.h>
#include <gtsam/linear/SuiteSparseSolver.h>
#include <gtsam/linear/CuSparseSolver.h>

namespace gtsam {

boost::shared_ptr<LinearSolver> LinearSolver::CreateFromParameters(
    const LinearSolverParams &params) {
  switch (params.linearSolverType) {
    case LinearSolverParams::MULTIFRONTAL_QR:
    case LinearSolverParams::MULTIFRONTAL_CHOLESKY:
    case LinearSolverParams::SEQUENTIAL_QR:
    case LinearSolverParams::SEQUENTIAL_CHOLESKY:
      return boost::make_shared<EliminationSolver>(params);
    case LinearSolverParams::Iterative:
      return IterativeSolver::CreateFromParameters(params);
    case LinearSolverParams::PCG:
      return boost::make_shared<PCGSolver>(params);
    case LinearSolverParams::SUBGRAPH:
      return boost::make_shared<SubgraphSolverWrapper>(params);
    case LinearSolverParams::EIGEN_QR:
      return boost::make_shared<SparseEigenSolver>(
          SparseEigenSolver::SparseEigenSolverType::QR, *params.ordering);
    case LinearSolverParams::EIGEN_CHOLESKY:
      return boost::make_shared<SparseEigenSolver>(
          SparseEigenSolver::SparseEigenSolverType::CHOLESKY, *params.ordering);
    case LinearSolverParams::SUITESPARSE:
      return boost::make_shared<SuiteSparseSolver>(*params.ordering);
    case LinearSolverParams::CUSPARSE:
      return boost::make_shared<CuSparseSolver>(*params.ordering);
    case LinearSolverParams::CHOLMOD:
    default:
      throw std::runtime_error("Invalid parameters passed");
  }
}
}  // namespace gtsam

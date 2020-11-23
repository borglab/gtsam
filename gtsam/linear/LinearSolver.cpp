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
#include <gtsam/linear/EliminationSolver.h>
#include <gtsam/linear/IterativeSolver.h>
#include <gtsam/linear/PCGSolver.h>
#include <gtsam/linear/SubgraphSolver.h>
#include <gtsam/linear/SparseEigenSolver.h>
#include <gtsam/linear/SuiteSparseSolver.h>
#include <gtsam/linear/CuSparseSolver.h>

namespace gtsam {

boost::shared_ptr<LinearSolver> LinearSolver::FromLinearSolverParams(
    const LinearSolverParams &params) {
  switch (params.linearSolverType) {
    case LinearSolverParams::MULTIFRONTAL_QR:
    case LinearSolverParams::MULTIFRONTAL_CHOLESKY:
    case LinearSolverParams::SEQUENTIAL_QR:
    case LinearSolverParams::SEQUENTIAL_CHOLESKY:
      return boost::make_shared<EliminationSolver>(params);
    case LinearSolverParams::Iterative:
      return IterativeSolver::FromLinearSolverParams(params);
    case LinearSolverParams::PCG:
      if (!params.iterativeParams)
        throw std::runtime_error(
            "LinearSolver::FromLinearSolverParams: iterative params has to be "
            "assigned ...");
      return boost::make_shared<PCGSolver>(
          *boost::static_pointer_cast<PCGSolverParameters>(
              params.iterativeParams));
    case LinearSolverParams::SUBGRAPH:
      if (!params.iterativeParams)
        throw std::runtime_error(
            "LinearSolver::FromLinearSolverParams: iterative params has to be "
            "assigned ...");
      if (!params.ordering)
        throw std::runtime_error(
            "LinearSolver::FromLinearSolverParams: SubgraphSolver needs an "
            "ordering");
      return boost::make_shared<SubgraphSolverWrapper>(
          *boost::static_pointer_cast<SubgraphSolverParameters>(
              params.iterativeParams),
          *params.ordering);
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
    case LinearSolverParams::CHOLMOD:
    default:
      throw std::runtime_error("Invalid parameters passed");
  }
}
}  // namespace gtsam

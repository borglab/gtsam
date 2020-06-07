/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SuiteSparseSolver.cpp
 *
 * @brief SuiteSparse based linear solver backend for GTSAM
 *
 * @date Jun 2020
 * @author Fan Jiang
 */

#include "gtsam/linear/SuiteSparseSolver.h"

#include "gtsam/linear/SparseEigenSolver.h"

#ifdef GTSAM_USE_SUITESPARSE
#include <Eigen/CholmodSupport>
#endif

namespace gtsam {
  SuiteSparseSolver::SuiteSparseSolver(SuiteSparseSolver::SuiteSparseSolverType type,
                                              const Ordering &ordering) {
    solverType = type;
    this->ordering = ordering;
  }

  bool SuiteSparseSolver::isIterative() {
    return false;
  }

  bool SuiteSparseSolver::isSequential() {
    return false;
  }

#ifdef GTSAM_USE_SUITESPARSE
  VectorValues SuiteSparseSolver::solve(const gtsam::GaussianFactorGraph &gfg) {
    if (solverType == QR) {
      throw std::invalid_argument("This solver does not support QR.");
    } else if (solverType == CHOLESKY) {
      gttic_(SuiteSparseSolver_optimizeEigenCholesky);
      Eigen::SparseMatrix<double>
          Ab = SparseEigenSolver::sparseJacobianEigen(gfg, ordering);
      auto rows = Ab.rows(), cols = Ab.cols();
      auto A = Ab.block(0, 0, rows, cols - 1);
      auto At = A.transpose();
      auto b = Ab.col(cols - 1);

      Eigen::SparseMatrix<double>
          AtA(A.cols(), A.cols());
      AtA.selfadjointView<Eigen::Upper>().rankUpdate(At);

      gttic_(SuiteSparseSolver_optimizeEigenCholesky_create_solver);
      // Solve A*x = b using sparse Cholesky from Eigen
      Eigen::CholmodSimplicialLDLT<Eigen::SparseMatrix<double>
          , Eigen::Upper>
          solver;
      solver.cholmod().nmethods = 1;
      solver.cholmod().method[0].ordering = CHOLMOD_NATURAL;
      solver.cholmod().postorder = false;

      solver.compute(AtA);
      gttoc_(SuiteSparseSolver_optimizeEigenCholesky_create_solver);

      gttic_(SuiteSparseSolver_optimizeEigenCholesky_solve);
      Matrix Atb = (At * b).eval();
      Eigen::VectorXd x = solver.solve(Atb);
      gttoc_(SuiteSparseSolver_optimizeEigenCholesky_solve);

      // NOTE: b is reordered now, so we need to transform back the order.
      // First find dimensions of each variable
      std::map<Key, size_t> dims;
      for (const boost::shared_ptr<GaussianFactor> &factor : gfg) {
        if (!static_cast<bool>(factor))
          continue;

        for (auto it = factor->begin(); it != factor->end(); ++it) {
          dims[*it] = factor->getDim(it);
        }
      }

      VectorValues vv;

      std::map<Key, size_t> columnIndices;

      {
        size_t currentColIndex = 0;
        for (const auto key : ordering) {
          columnIndices[key] = currentColIndex;
          currentColIndex += dims[key];
        }
      }

      for (const std::pair<const Key, unsigned long> keyDim : dims) {
        vv.insert(keyDim.first, x.segment(columnIndices[keyDim.first], keyDim.second));
      }

      return vv;
    }

    throw std::exception();
  }
#else
  VectorValues SuiteSparseSolver::solve(const gtsam::GaussianFactorGraph &gfg) {
    throw std::invalid_argument("This GTSAM is compiled without Cholmod support");
  }
#endif
}

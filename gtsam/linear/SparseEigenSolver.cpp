/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SparseEigenSolver.cpp
 *
 * @brief Eigen SparseSolver based linear solver backend for GTSAM
 *
 * @date Aug 2019
 * @author Mandy Xie
 * @author Fan Jiang
 * @author Frank Dellaert
 * @author Gerry Chen
 */

#include <gtsam/base/timing.h>
#include <gtsam/linear/SparseEigenSolver.h>
#include <gtsam/linear/LinearSolverParams.h>
#include <vector>

#include <boost/shared_ptr.hpp>

using namespace std;

namespace gtsam {

  // Solves using sparse QR (used when solverType_ == QR)
  VectorValues solveQr(const GaussianFactorGraph &gfg,
                       const Ordering &ordering) {
    gttic_(EigenOptimizer_optimizeEigenQR);

    // get sparse matrix from factor graph + ordering
    size_t rows, cols;
    SparseMatrixEigen Ab;
    std::tie(rows, cols, Ab) = gfg.sparseJacobian<SparseMatrixEigen>(ordering);

    // Solve A*x = b using sparse QR from Eigen
    gttic_(create_solver);
    Eigen::SparseQR<SparseMatrixEigen, Eigen::NaturalOrdering<int>>
        solver(Ab.block(0, 0, rows, cols - 1));
    gttoc_(create_solver);

    gttic_(solve);
    Eigen::VectorXd x = solver.solve(Ab.col(cols-1));
    gttoc_(solve);

    return VectorValues(x, gfg.getKeyDimMap());
  }

  // Solves using sparse Cholesky (used when solverType_ == CHOLESKY)
  VectorValues solveCholesky(const GaussianFactorGraph &gfg,
                             const Ordering &ordering) {
    gttic_(EigenOptimizer_optimizeEigenCholesky);

    // get sparse matrices A|b from factor graph + ordering
    size_t rows, cols;
    SparseMatrixEigen Ab;
    std::tie(rows, cols, Ab) = gfg.sparseJacobian<SparseMatrixEigen>(ordering);

    auto A = Ab.block(0, 0, rows, cols - 1);
    auto At = A.transpose();
    auto b = Ab.col(cols - 1);

    SparseMatrixEigen AtA(A.cols(), A.cols());
    AtA.selfadjointView<Eigen::Upper>().rankUpdate(At);

    gttic_(create_solver);
    // Solve A*x = b using sparse Cholesky from Eigen
    Eigen::SimplicialLDLT<SparseMatrixEigen, Eigen::Upper,
                          Eigen::NaturalOrdering<int>>
        solver(AtA);
    gttoc_(create_solver);

    gttic_(solve);
    Eigen::VectorXd x = solver.solve(At * b);
    gttoc_(solve);

    // NOTE: b is reordered now, so we need to transform back the order.
    // First find dimensions of each variable
    std::map<Key, size_t> dims;
    for (const boost::shared_ptr<GaussianFactor> &factor : gfg) {
      if (!static_cast<bool>(factor)) continue;

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

    for (const pair<const Key, unsigned long> keyDim : dims) {
      vv.insert(keyDim.first,
                x.segment(columnIndices[keyDim.first], keyDim.second));
    }

    return vv;
  }

  VectorValues SparseEigenSolver::solve(const GaussianFactorGraph &gfg) const {
    if (solverType_ == QR) {
      return solveQr(gfg, ordering_);
    } else if (solverType_ == CHOLESKY) {
      return solveCholesky(gfg, ordering_);
    }

    throw std::exception();
  }

}  // namespace gtsam

/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file EigenOptimizer.cpp
 *
 * @brief optimizer linear factor graph using eigen solver as backend
 *
 * @date Aug 2019
 * @author Mandy Xie
 */

#include <gtsam/base/timing.h>
#include <gtsam/linear/EigenOptimizer.h>
#include <Eigen/MetisSupport>
#include <vector>

using namespace std;

namespace gtsam {
using SpMat = Eigen::SparseMatrix<double>;
/// obtain sparse matrix for eigen sparse solver
SpMat obtainSparseMatrix(const GaussianFactorGraph &gfg) {
  gttic_(EigenOptimizer_obtainSparseMatrix);
  // Get sparse entries of Jacobian [A|b] augmented with RHS b.
  auto entries = gfg.sparseJacobian();

  // Convert boost tuples to Eigen triplets
  vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(entries.size());
  size_t rows = 0, cols = 0;
  for (const auto &e : entries) {
    size_t temp_rows = e.get<0>(), temp_cols = e.get<1>();
    triplets.emplace_back(temp_rows, temp_cols, e.get<2>());
    rows = std::max(rows, temp_rows);
    cols = std::max(cols, temp_cols);
  }

  // ...and make a sparse matrix with it.
  using SpMat = Eigen::SparseMatrix<double>;
  SpMat Ab(rows + 1, cols + 1);
  Ab.setFromTriplets(triplets.begin(), triplets.end());
  Ab.makeCompressed();
  return Ab;
}

template <typename EigenSolverType>
Eigen::VectorXd createQR(const SpMat &Ab) {
  gttic_(EigenOptimizer_createQR);
  size_t rows = Ab.rows();
  size_t cols = Ab.cols();
  EigenSolverType solver(Ab.block(0, 0, rows, cols - 1));
  return solver.solve(Ab.col(cols - 1));
}

/* ************************************************************************* */
VectorValues optimizeEigenQR(const GaussianFactorGraph &gfg,
                             const std::string &orderingType) {
  gttic_(EigenOptimizer_optimizeEigenQR);
  SpMat Ab = obtainSparseMatrix(gfg);
  // Solve A*x = b using sparse QR from Eigen
  Eigen::VectorXd x;
  if (orderingType == "AMD") {
    gttic_(EigenOptimizer_optimizeEigenQR_AMD);
    x = createQR<Eigen::SparseQR<SpMat, Eigen::AMDOrdering<int>>>(Ab);
  } else if (orderingType == "COLAMD") {
    gttic_(EigenOptimizer_optimizeEigenQR_COLAMD);
    x = createQR<Eigen::SparseQR<SpMat, Eigen::COLAMDOrdering<int>>>(Ab);
  } else if (orderingType == "NATURAL") {
    gttic_(EigenOptimizer_optimizeEigenQR_NATURAL);
    x = createQR<Eigen::SparseQR<SpMat, Eigen::NaturalOrdering<int>>>(Ab);
  } else if (orderingType == "METIS") {
    gttic_(EigenOptimizer_optimizeEigenQR_METIS);
    x = createQR<Eigen::SparseQR<SpMat, Eigen::MetisOrdering<int>>>(Ab);
  }
  return VectorValues(x, gfg.getKeyDimMap());
}

template <typename EigenSolverType>
Eigen::VectorXd createCholesky(const SpMat &Ab) {
  gttic_(EigenOptimizer_createCholesky);
  size_t rows = Ab.rows();
  size_t cols = Ab.cols();
  auto A = Ab.block(0, 0, rows, cols - 1);
  auto At = A.transpose();
  auto b = Ab.col(cols - 1);
  EigenSolverType solver(At * A);
  return solver.solve(At * b);
}

/* *************************************************************************
 */
VectorValues optimizeEigenCholesky(const GaussianFactorGraph &gfg,
                                   const std::string &orderingType) {
  gttic_(EigenOptimizer_optimizeEigenCholesky);
  SpMat Ab = obtainSparseMatrix(gfg);
  // Solve A*x = b using sparse QR from Eigen
  Eigen::VectorXd x;
  if (orderingType == "AMD") {
    gttic_(EigenOptimizer_optimizeEigenCholesky_AMD);
    x = createCholesky<
        Eigen::SimplicialLDLT<SpMat, Eigen::Lower, Eigen::AMDOrdering<int>>>(
        Ab);
  } else if (orderingType == "COLAMD") {
    gttic_(EigenOptimizer_optimizeEigenCholesky_COLAMD);
    x = createCholesky<
        Eigen::SimplicialLDLT<SpMat, Eigen::Lower, Eigen::COLAMDOrdering<int>>>(
        Ab);
  } else if (orderingType == "NATURAL") {
    gttic_(EigenOptimizer_optimizeEigenCholesky_NATURAL);
    x = createCholesky<Eigen::SimplicialLDLT<SpMat, Eigen::Lower,
                                            Eigen::NaturalOrdering<int>>>(Ab);
  } else if (orderingType == "METIS") {
    gttic_(EigenOptimizer_optimizeEigenCholesky_METIS);
    x = createCholesky<
        Eigen::SimplicialLDLT<SpMat, Eigen::Lower, Eigen::MetisOrdering<int>>>(
        Ab);
  }
  return VectorValues(x, gfg.getKeyDimMap());
}

}  // namespace gtsam
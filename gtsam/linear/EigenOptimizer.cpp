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

#include <gtsam/linear/EigenOptimizer.h>
#include <vector>

using namespace std;

namespace gtsam {
using SpMat = Eigen::SparseMatrix<double>;
/// obtain sparse matrix for eigen sparse solver
SpMat obtainSparseMatrix(const GaussianFactorGraph &gfg) {
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

/* ************************************************************************* */
VectorValues optimizeEigenQR(const GaussianFactorGraph &gfg) {
  SpMat Ab = obtainSparseMatrix(gfg);
  size_t rows = Ab.rows();
  size_t cols = Ab.cols();
  // Solve A*x = b using sparse QR from Eigen
  Eigen::SparseQR<SpMat, Eigen::COLAMDOrdering<int>> qr(
      Ab.block(0, 0, rows, cols - 1));
  Eigen::VectorXd x = qr.solve(Ab.col(cols - 1));

  return VectorValues(x, gfg.getKeyDimMap());
}

/* ************************************************************************* */
VectorValues optimizeEigenCholesky(const GaussianFactorGraph &gfg) {
  SpMat Ab = obtainSparseMatrix(gfg);
  size_t rows = Ab.rows();
  size_t cols = Ab.cols();
  // Solve A*x = b using sparse QR from Eigen
  Eigen::SimplicialLDLT<SpMat> ldlt(Ab.block(0, 0, rows, cols - 1));
  Eigen::VectorXd x = ldlt.solve(Ab.col(cols - 1));

  return VectorValues(x, gfg.getKeyDimMap());
}

} //namespace gtsam
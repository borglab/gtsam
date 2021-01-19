/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SparseEigen.h
 *
 * @brief Utilities for converting to Eigen's sparse matrix representations
 *
 * @date Aug 2019
 * @author Mandy Xie
 * @author Fan Jiang
 * @author Gerry Chen
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>

#include <Eigen/Sparse>

namespace gtsam {

typedef Eigen::SparseMatrix<double> SpMat;

SpMat sparseJacobianEigen(
    const GaussianFactorGraph &gfg, const Ordering &ordering) {
  // First find dimensions of each variable
  std::map<Key, size_t> dims;
  for (const boost::shared_ptr<GaussianFactor> &factor : gfg) {
    if (!static_cast<bool>(factor)) continue;

    for (auto it = factor->begin(); it != factor->end(); ++it) {
      dims[*it] = factor->getDim(it);
    }
  }

  // Compute first scalar column of each variable
  size_t currentColIndex = 0;
  std::map<Key, size_t> columnIndices;
  for (const auto key : ordering) {
    columnIndices[key] = currentColIndex;
    currentColIndex += dims[key];
  }

  // Iterate over all factors, adding sparse scalar entries
  std::vector<Eigen::Triplet<double>> entries;
  entries.reserve(60 * gfg.size());

  size_t row = 0;
  for (const boost::shared_ptr<GaussianFactor> &factor : gfg) {
    if (!static_cast<bool>(factor)) continue;

    // Convert to JacobianFactor if necessary
    JacobianFactor::shared_ptr jacobianFactor(
        boost::dynamic_pointer_cast<JacobianFactor>(factor));
    if (!jacobianFactor) {
      HessianFactor::shared_ptr hessian(
          boost::dynamic_pointer_cast<HessianFactor>(factor));
      if (hessian)
        jacobianFactor.reset(new JacobianFactor(*hessian));
      else
        throw std::invalid_argument(
            "GaussianFactorGraph contains a factor that is neither a "
            "JacobianFactor nor a HessianFactor.");
    }

    // Whiten the factor and add entries for it
    // iterate over all variables in the factor
    const JacobianFactor whitened(jacobianFactor->whiten());
    for (JacobianFactor::const_iterator key = whitened.begin();
         key < whitened.end(); ++key) {
      JacobianFactor::constABlock whitenedA = whitened.getA(key);
      // find first column index for this key
      size_t column_start = columnIndices[*key];
      for (size_t i = 0; i < (size_t)whitenedA.rows(); i++)
        for (size_t j = 0; j < (size_t)whitenedA.cols(); j++) {
          double s = whitenedA(i, j);
          if (std::abs(s) > 1e-12)
            entries.emplace_back(row + i, column_start + j, s);
        }
    }

    JacobianFactor::constBVector whitenedb(whitened.getb());
    size_t bcolumn = currentColIndex;
    for (size_t i = 0; i < (size_t)whitenedb.size(); i++) {
      double s = whitenedb(i);
      if (std::abs(s) > 1e-12) entries.emplace_back(row + i, bcolumn, s);
    }

    // Increment row index
    row += jacobianFactor->rows();
  }

  // ...and make a sparse matrix with it.
  SpMat Ab(row + 1, currentColIndex + 1);
  Ab.setFromTriplets(entries.begin(), entries.end());
  return Ab;
}

SpMat sparseJacobianEigen(const GaussianFactorGraph &gfg) {
  return sparseJacobianEigen(gfg, Ordering(gfg.keys()));
}

// /// obtain sparse matrix for eigen sparse solver
// std::pair<SpMat, Eigen::VectorXd> obtainSparseMatrix(
//     const GaussianFactorGraph &gfg, const Ordering &ordering) {
//   gttic_(EigenOptimizer_obtainSparseMatrix);

//   // Get sparse entries of Jacobian [A|b] augmented with RHS b.
//   auto entries = gfg.sparseJacobian(ordering);

//   gttic_(EigenOptimizer_convertSparse);
//   // Convert boost tuples to Eigen triplets
//   vector<Eigen::Triplet<double>> triplets;
//   triplets.reserve(entries.size());
//   size_t rows = 0, cols = 0;
//   for (const auto &e : entries) {
//     size_t temp_rows = e.get<0>(), temp_cols = e.get<1>();
//     triplets.emplace_back(temp_rows, temp_cols, e.get<2>());
//     rows = std::max(rows, temp_rows);
//     cols = std::max(cols, temp_cols);
//   }

//   // ...and make a sparse matrix with it.
//   SpMat Ab(rows + 1, cols + 1);
//   Ab.setFromTriplets(triplets.begin(), triplets.end());
//   Ab.makeCompressed();
//   gttoc_(EigenOptimizer_convertSparse);

//   gttoc_(EigenOptimizer_obtainSparseMatrix);

//   return make_pair<SpMat, Eigen::VectorXd>(Ab.block(0, 0, rows + 1, cols),
//                                            Ab.col(cols));
// }

}  // namespace gtsam

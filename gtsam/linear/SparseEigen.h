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
 * @brief Utilities for creating Eigen sparse matrices (gtsam::SparseEigen)
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

typedef Eigen::SparseMatrix<double> SparseEigen;

/// Constructs an Eigen-format SparseMatrix of a GaussianFactorGraph
SparseEigen sparseJacobianEigen(
    const GaussianFactorGraph &gfg, const Ordering &ordering) {
  // TODO(gerry): eliminate copy/pasta by making GaussianFactorGraph version
  // more general, or by creating an Eigen::Triplet compatible wrapper for
  // boost::tuple return type

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
  SparseEigen Ab(row, currentColIndex + 1);
  Ab.setFromTriplets(entries.begin(), entries.end());
  return Ab;
}

SparseEigen sparseJacobianEigen(const GaussianFactorGraph &gfg) {
  return sparseJacobianEigen(gfg, Ordering(gfg.keys()));
}

}  // namespace gtsam

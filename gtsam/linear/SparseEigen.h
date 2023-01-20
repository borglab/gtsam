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

/// Eigen-format sparse matrix.  Note: ColMajor is ~20% faster since
/// InnerIndices must be sorted
typedef Eigen::SparseMatrix<double, Eigen::ColMajor, int> SparseEigen;

/// Constructs an Eigen-format SparseMatrix of a GaussianFactorGraph
SparseEigen sparseJacobianEigen(
    const GaussianFactorGraph &gfg, const Ordering &ordering) {
  gttic_(SparseEigen_sparseJacobianEigen);
  // intermediate `entries` vector is kind of unavoidable due to how expensive
  // factor->rows() is, which prevents us from populating SparseEigen directly.
  size_t nrows, ncols;
  auto entries = gfg.sparseJacobian(ordering, nrows, ncols);
  // declare sparse matrix
  SparseEigen Ab(nrows, ncols);
  // See Eigen::set_from_triplets.  This is about 5% faster.
  // pass 1: count the nnz per inner-vector
  std::vector<int> nnz(ncols, 0);
  for (const auto &entry : entries) nnz[std::get<1>(entry)]++;
  Ab.reserve(nnz);
  // pass 2: insert the elements
  for (const auto &entry : entries)
    Ab.insert(std::get<0>(entry), std::get<1>(entry)) = std::get<2>(entry);
  return Ab;
}

SparseEigen sparseJacobianEigen(const GaussianFactorGraph &gfg) {
  gttic_(SparseEigen_sparseJacobianEigen_defaultOrdering);
  return sparseJacobianEigen(gfg, Ordering(gfg.keys()));
}

}  // namespace gtsam

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
typedef Eigen::SparseMatrix<double, Eigen::ColMajor> SparseEigen;

/// Constructs an Eigen-format SparseMatrix of a GaussianFactorGraph
SparseEigen sparseJacobianEigen(
    const GaussianFactorGraph &gfg, const Ordering &ordering) {
  gttic_(SparseEigen_sparseJacobianEigen);
  // intermediate `entries` vector is kind of unavoidable due to how expensive
  // factor->rows() is, which prevents us from populating SparseEigen directly
  std::vector<Eigen::Triplet<double>> entries;
  entries.reserve(60 * gfg.size());
  size_t nrows, ncols;
  gfg.sparseJacobianInPlace(entries, ordering, nrows, ncols);
  // create sparse matrix
  SparseEigen Ab(nrows, ncols);
  Ab.setFromTriplets(entries.begin(), entries.end());
  return Ab;
}

SparseEigen sparseJacobianEigen(const GaussianFactorGraph &gfg) {
  gttic_(SparseEigen_sparseJacobianEigen_defaultOrdering);
  return sparseJacobianEigen(gfg, Ordering(gfg.keys()));
}

}  // namespace gtsam

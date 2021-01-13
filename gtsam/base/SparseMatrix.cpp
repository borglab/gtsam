/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SparseMatrix.h
 * @brief   Interface utilities with Eigen's SparseMatrix representation
 * @author  Gerry Chen
 */

#include <gtsam/base/SparseMatrix.h>

namespace gtsam {

/** Returns the sparse jacobian of a factor graph as an Eigen::SparseMatrix
 * The standard deviations are baked into A and b
 */
SparseMatrixEigen SparseMatrix::JacobianEigen(const GaussianFactorGraph& graph,
                                              const Ordering& ordering,
                                              size_t& nrows, size_t& ncols) {
  gttic_(Jacobian);
  gttic_(obtainSparseJacobian);
  auto entries = JacobianBoostTriplets(graph, ordering, nrows, ncols);
  gttoc_(obtainSparseJacobian);
  gttic_(convertSparseJacobian);
  SparseMatrixEigen Ab(nrows, ncols);
  Ab.reserve(entries.size());
  for (auto entry : entries) {
    Ab.insert(entry.get<0>(), entry.get<1>()) = entry.get<2>();
  }
  Ab.makeCompressed();
  // TODO(gerry): benchmark to see if setFromTriplets is faster
  // Ab.setFromTriplets(entries.begin(), entries.end());
  return Ab;
}

/* ******************************* Overloads ******************************* */
SparseMatrixEigen SparseMatrix::JacobianEigen(const GaussianFactorGraph& graph,
                                              const Ordering& ordering) {
  size_t dummy1, dummy2;
  return JacobianEigen(graph, ordering, dummy1, dummy2);
}
SparseMatrixEigen SparseMatrix::JacobianEigen(const GaussianFactorGraph& graph,
                                              size_t& nrows, size_t& ncols) {
  return JacobianEigen(graph, Ordering(graph.keys()), nrows, ncols);
}
SparseMatrixEigen SparseMatrix::JacobianEigen(
    const GaussianFactorGraph& graph) {
  return JacobianEigen(graph, Ordering(graph.keys()));
}

}  // namespace gtsam

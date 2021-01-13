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

#pragma once

#include "gtsam/linear/GaussianFactorGraph.h"

#include <Eigen/Sparse>

namespace gtsam {

/// Sparse matrix as Eigen::SparseMatrix
//  note: Eigen only supports signed (e.g. not size_t) StorageIndex
typedef Eigen::SparseMatrix<double, Eigen::ColMajor, int> SparseMatrixEigen;

class SparseMatrix {
 public:
  /** Returns the sparse augmented jacobian of a factor graph as a vector of
   * boost tuples (row, col, entry).  Equivalent to graph.sparseJacobian().
   * The standard deviations are baked into A and b
   */
  template <typename... Args>
  static SparseMatrixBoostTriplets JacobianBoostTriplets(
      const GaussianFactorGraph& graph, Args&&... args) {
    return graph.sparseJacobian(std::forward<Args>(args)...);
  }

  /** Returns the sparse augmented jacobian of a factor graph as a matlab
   * `sparse` -compatible 3xm matrix with each column representing an entry of
   * the form [row; col; entry].  graph.sparseJacobian_().
   * The standard deviations are baked into A and b
   */
  template <typename... Args>
  static Matrix JacobianMatrix(const GaussianFactorGraph& graph,
                               Args&&... args) {
    return graph.sparseJacobian_(std::forward<Args>(args)...);
  }

  /** Returns the sparse augmented jacobian of a factor graph as an
   * Eigen::SparseMatrix
   * The standard deviations are baked into A and b
   */
  static SparseMatrixEigen JacobianEigen(const GaussianFactorGraph& graph,
                                         const Ordering& ordering,
                                         size_t& nrows, size_t& ncols);

  // Overloads
  static SparseMatrixEigen JacobianEigen(const GaussianFactorGraph& graph,
                                         const Ordering& ordering);
  static SparseMatrixEigen JacobianEigen(const GaussianFactorGraph& graph,
                                         size_t& nrows, size_t& ncols);
  static SparseMatrixEigen JacobianEigen(const GaussianFactorGraph& graph);
};
}  // namespace gtsam

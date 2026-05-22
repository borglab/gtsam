/** A set of typedefs describing the basic types that will be used throughout
 * the Preconditioners library.
 *
 *  Copyright (C) 2020 by David M. Rosen (dmrosen@mit.edu)
 */

#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <utility>

namespace Preconditioners {

/// Some useful typedefs for the Preconditioners library

/// Linear algebra types
typedef double Scalar;
typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> Vector;
typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Matrix;

/// Typedef for a (fixed-size) 2x2 matrix
typedef Eigen::Matrix2d Matrix2d;

/** We use row-major storage order to take advantage of fast (sparse-matrix)
 * (dense-vector) multiplications when OpenMP is available (cf. the Eigen
 * documentation page on "Eigen and Multithreading") */
typedef Eigen::SparseMatrix<Scalar, Eigen::RowMajor> SparseMatrix;

typedef Eigen::PermutationMatrix<Eigen::Dynamic> Permutation;
typedef Permutation::IndicesType PermutationVector;

/// Convenience typedef for a pair containing the number of positive and
/// negative eigenvalues of a matrix
typedef std::pair<size_t, size_t> Inertia;

} // namespace Preconditioners

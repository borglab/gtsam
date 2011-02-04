/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    cholesky.h
 * @brief   Efficient incomplete Cholesky on rank-deficient matrices, todo: constrained Cholesky
 * @author  Richard Roberts
 * @created Nov 5, 2010
 */
#pragma once

#include <gtsam/base/Matrix.h>

namespace gtsam {

/** Plain Cholesky on a symmetric positive semi-definite matrix, in place. */
void cholesky_inplace(MatrixColMajor& I);

/**
 * Factor an underdetermined Gaussian into a Gaussian conditional.  This means
 * for a Gaussian exp(-1/2 * ||Ax - b||^2), with a "wide" A, i.e. m < n, to
 * find an upper-triangular m x m R, rectangular m x (n-m) S, and m-vector d,
 * such that ||Ax - b||^2 == || [R S]x - d ||^2.
 *
 * The matrices [ R S ] and [ R S d ] are each upper-trapazoidal.
 *
 * This returns the same upper-trapazoidal factor as QR, but uses Cholesky for
 * efficiency.  Given a matrix [ F G b ], with F square, this first computes
 * the upper-triangular R = chol(F), i.e. R'R == F'F.  It then computes the
 * upper-trapazoidal factor [ R S d ], with [ S d ] = inv(R') * F' * [ G b ].
 *
 * Note that this operates on the "canonical" A matrix, not the symmetric
 * information matrix like plain Cholesky.
 *
 * This function returns the rank of the factor.
 */
size_t choleskyFactorUnderdetermined(MatrixColMajor& Ab, size_t nFrontal);

/**
 * "Careful" Cholesky computes the positive square-root of a positive symmetric
 * semi-definite matrix (i.e. that may be rank-deficient).  Unlike standard
 * Cholesky, the square-root factor may have all-zero rows for free variables.
 *
 * Additionally, this function returns the index of the row after the last
 * non-zero row in the computed factor, so that it may be truncated to an
 * upper-trapazoidal matrix.
 *
 * Note that this returned index is the rank of the matrix if and only if all
 * of the zero-rows of the factor occur after any non-zero rows.  This is
 * (always?) the case during elimination of a fully-constrained least-squares
 * problem.
 */
std::pair<size_t,bool> choleskyCareful(MatrixColMajor& ATA, int order = -1);

/**
 * Partial Cholesky computes a factor [R S  such that [R' 0  [R S  = [A  B
 *                                     0 L]            S' I]  0 L]    B' C].
 * The input to this function is the matrix ABC = [A  B], and the parameter
 *                                                [B' C]
 * nFrontal determines the split between A, B, and C, with A being of size
 * nFrontal x nFrontal.
 */
void choleskyPartial(MatrixColMajor& ABC, size_t nFrontal);

}


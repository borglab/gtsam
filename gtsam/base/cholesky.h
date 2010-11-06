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
 */
void choleskyFactorUnderdetermined(MatrixColMajor& Ab);

}


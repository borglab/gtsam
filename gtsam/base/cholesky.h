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
 * @date    Nov 5, 2010
 */
#pragma once

#include <gtsam/base/Matrix.h>

namespace gtsam {

/**
 * "Careful" Cholesky computes the positive square-root of a positive symmetric
 * semi-definite matrix (i.e. that may be rank-deficient).  Unlike standard
 * Cholesky, the square-root factor may have all-zero rows for free variables.
 *
 * Additionally, this function returns the index of the row after the last
 * non-zero row in the computed factor, so that it may be truncated to an
 * upper-trapazoidal matrix.
 *
 * The second element of the return value is \c true if the matrix was factored
 * successfully, or \c false if it was non-positive-semidefinite (i.e.
 * indefinite or negative-(semi-)definite.
 *
 * Note that this returned index is the rank of the matrix if and only if all
 * of the zero-rows of the factor occur after any non-zero rows.  This is
 * (always?) the case during elimination of a fully-constrained least-squares
 * problem.
 *
 * The optional order argument specifies the size of the square upper-left
 * submatrix to operate on, ignoring the rest of the matrix.
 *
 *
 */
GTSAM_EXPORT std::pair<size_t,bool> choleskyCareful(Matrix& ATA, int order = -1);

/**
 * Partial Cholesky computes a factor [R S  such that [R' 0  [R S  = [A  B
 *                                     0 L]            S' I]  0 L]    B' C].
 * The input to this function is the matrix ABC = [A  B], and the parameter
 *                                                [B' C]
 * nFrontal determines the split between A, B, and C, with A being of size
 * nFrontal x nFrontal.
 *
 * if non-zero, factorization proceeds in bottom-right corner starting at topleft
 *
 * @return \c true if the decomposition is successful, \c false if \c A was
 * not positive-definite.
 */
GTSAM_EXPORT bool choleskyPartial(Matrix& ABC, size_t nFrontal, size_t topleft=0);

}


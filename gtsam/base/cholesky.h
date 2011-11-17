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
#include <boost/shared_ptr.hpp>

namespace gtsam {

/**
 * An exception indicating an attempt to factor a negative or indefinite matrix.
 * If detailed exceptions are enabled
 * \todo fill this in
 */
struct NegativeMatrixException : public std::exception {
  /// Detail for NegativeMatrixException
  struct Detail {
    Matrix A; ///< The original matrix attempted to factor
    Matrix U; ///< The produced upper-triangular factor
    Matrix D; ///< The produced diagonal factor
    Detail(const Matrix& _A, const Matrix& _U, const Matrix& _D) /**< Detail constructor */ : A(_A), U(_U), D(_D) {}
    void print(const std::string& str = "") const {
      std::cout << str << "\n";
      gtsam::print(A, "  A: ");
      gtsam::print(U, "  U: ");
      gtsam::print(D, "  D: ");
    }
  };
  const boost::shared_ptr<const Detail> detail; ///< Detail
  NegativeMatrixException() /**< Constructor with no detail */ {}
  NegativeMatrixException(const Detail& _detail) /**< Constructor with detail */ : detail(new Detail(_detail)) {}
  NegativeMatrixException(const boost::shared_ptr<Detail>& _detail) /**< Constructor with detail */ : detail(_detail) {}
  virtual ~NegativeMatrixException() throw() {}
};

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
 *
 * The optional order argument specifies the size of the square upper-left
 * submatrix to operate on, ignoring the rest of the matrix.
 */
std::pair<size_t,bool> choleskyCareful(Matrix& ATA, int order = -1);

/**
 * Partial Cholesky computes a factor [R S  such that [R' 0  [R S  = [A  B
 *                                     0 L]            S' I]  0 L]    B' C].
 * The input to this function is the matrix ABC = [A  B], and the parameter
 *                                                [B' C]
 * nFrontal determines the split between A, B, and C, with A being of size
 * nFrontal x nFrontal.
 */
void choleskyPartial(Matrix& ABC, size_t nFrontal);

/**
 * Partial LDL computes a factor [R S  such that [P'R' 0  [RP S  = [A  B
 *                                0 F]            S'   I]  0  F]    B' C].
 * The input to this function is the matrix ABC = [A  B], and the parameter
 *                                                [B' C]
 * nFrontal determines the split between A, B, and C, with A being of size
 * nFrontal x nFrontal.
 * P is a permutation matrix obtained from the pivoting process while doing LDL'.
 *
 * Specifically, if A = P'U'DUP is the LDL' factorization of A,
 * then R = sqrt(D)*U is the permuted upper-triangular matrix.
 * The permutation is returned so that it can be used in the backsubstitution
 * process to return the correct order of variables, and in creating the
 * Jacobian factor by permuting R correctly.
 *
 * Note that S and F are not permuted (in correct original ordering).
 */
Eigen::LDLT<Matrix>::TranspositionType ldlPartial(Matrix& ABC, size_t nFrontal);

}


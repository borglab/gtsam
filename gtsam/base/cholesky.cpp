/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    cholesky.cpp
 * @brief   Efficient incomplete Cholesky on rank-deficient matrices, todo: constrained Cholesky
 * @author  Richard Roberts
 * @created Nov 5, 2010
 */

#include <gtsam/base/cholesky.h>
#include <gtsam/base/lapack.h>

#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost/numeric/ublas/blas.hpp>
#include <boost/format.hpp>

namespace ublas = boost::numeric::ublas;

namespace gtsam {

/* ************************************************************************* */
void cholesky_inplace(MatrixColMajor& I) {

  // We do not check for symmetry but we do check for squareness
  assert(I.size1() == I.size2());

  // Do Cholesky, return value info as follows (from dpotrf.f):
  // 00054 *  INFO    (output) INTEGER
  // 00055 *          = 0:  successful exit
  // 00056 *          < 0:  if INFO = -i, the i-th argument had an illegal value
  // 00057 *          > 0:  if INFO = i, the leading minor of order i is not
  // 00058 *                positive definite, and the factorization could not be
  // 00059 *                completed.
  int info = lapack_dpotrf('U', I.size1(), &I(0,0), I.size1());
  if(info != 0) {
    if(info < 0)
      throw std::domain_error(boost::str(boost::format(
          "Bad input to cholesky_inplace, dpotrf returned %d.\n")%info));
    else
      throw std::domain_error("The matrix passed into cholesky_inplace is rank-deficient");
  }
}

/* ************************************************************************* */
size_t choleskyFactorUnderdetermined(MatrixColMajor& Ab) {

  bool debug = false;

  size_t m = Ab.size1();
  size_t n = Ab.size2();

  // If m >= n, this function will just compute the plain Cholesky
  // factorization of A'A.  If m < n, A'A is rank-deficient this function
  // instead computes the upper-trapazoidal factor [ R S ], as described in the
  // header file comment.
  size_t rank = std::min(m,n-1);

  if(rank > 0) {

    // F is the first 'rank' columns of Ab, G is the remaining columns
    ublas::matrix_range<MatrixColMajor> F(ublas::project(Ab, ublas::range(0,m), ublas::range(0,rank)));
    ublas::matrix_range<MatrixColMajor> G(ublas::project(Ab, ublas::range(0,m), ublas::range(rank,n)));

    if(debug) {
      print(F, "F: ");
      print(G, "G: ");
    }

    ublas::matrix_range<MatrixColMajor> R(ublas::project(Ab, ublas::range(0,rank), ublas::range(0,rank)));
    ublas::matrix_range<MatrixColMajor> S(ublas::project(Ab, ublas::range(0,rank), ublas::range(rank,n)));

    // First compute F' * G (ublas makes a copy here to avoid aliasing)
    if(S.size2() > 0)
      S = ublas::prod(ublas::trans(F), G);

    // ublas makes a copy to avoid aliasing on this assignment
    R = ublas::prod(ublas::trans(F), F);

    // Compute the values of R from F'F
    int info = lapack_dpotrf('U', rank, &R(0,0), Ab.size1());
    if(info != 0) {
      if(info < 0)
        throw std::domain_error(boost::str(boost::format(
            "Bad input to choleskyFactorUnderdetermined, dpotrf returned %d.\n")%info));
      else
        throw std::domain_error(boost::str(boost::format(
            "The matrix passed into choleskyFactorUnderdetermined is numerically rank-deficient, dpotrf returned rank=%d, expected rank was %d.\n")%(info-1)%rank));
    }

    // Compute S = inv(R') * F' * G, i.e. solve S when R'S = F'G
    if(S.size2() > 0)
      cblas_dtrsm(CblasColMajor, CblasLeft, CblasUpper, CblasTrans, CblasNonUnit, S.size1(), S.size2(), 1.0, &R(0,0), m, &S(0,0), m);

    if(debug) {
      print(R, "R: ");
      print(S, "S: ");
    }

    return rank;
  } else
    return 0;
}

}

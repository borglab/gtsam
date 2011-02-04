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
#include <gtsam/base/timing.h>

#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost/numeric/ublas/symmetric.hpp>
#include <boost/numeric/ublas/blas.hpp>
#include <boost/format.hpp>

namespace ublas = boost::numeric::ublas;

using namespace std;

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
size_t choleskyFactorUnderdetermined(MatrixColMajor& Ab, size_t nFrontal) {

  static const bool debug = false;

  size_t m = Ab.size1();
  size_t n = Ab.size2();

  // If m >= n, this function will just compute the plain Cholesky
  // factorization of A'A.  If m < n, A'A is rank-deficient this function
  // instead computes the upper-trapazoidal factor [ R S ], as described in the
  // header file comment.
//  size_t rank = std::min(m,n-1);
  size_t rank = nFrontal;

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

    return m;
  } else
    return 0;
}

/* ************************************************************************* */
static inline bool choleskyStep(MatrixColMajor& ATA, size_t k) {

  // Tolerance for being equal to zero
//  static const double zeroTol = numeric_limits<double>::epsilon();
  static const double zeroTol = 1.e-15;

  const size_t n = ATA.size1();

  const double alpha = ATA(k,k);
  const double beta = sqrt(alpha);

  if(beta > zeroTol) {
    const double betainv = 1.0 / beta;

    // Update k,k
    ATA(k,k) = beta;

    if(k < (n-1)) {
      // Update A(k,k+1:end) <- A(k,k+1:end) / beta
      cblas_dscal(n-k-1, betainv, &(ATA(k,k+1)), n);

      // Update A(k+1:end, k+1:end) <- A(k+1:end, k+1:end) - v*v' / alpha
      cblas_dsyr(CblasColMajor, CblasUpper, n-k-1, -1.0, &(ATA(k,k+1)), n, &(ATA(k+1,k+1)), n);
    }

    return true;
  } else
    return false;
}

/* ************************************************************************* */
size_t choleskyCareful(MatrixColMajor& ATA) {

  static const bool debug = false;

//  // Tolerance for being equal to zero
////  static const double zeroTol = numeric_limits<double>::epsilon();
//  static const double zeroTol = 1.e-15;

  // Check that the matrix is square (we do not check for symmetry)
  assert(ATA.size1() == ATA.size2());

  // Number of rows/columns
  const size_t n = ATA.size1();

  // The index of the row after the last non-zero row of the square-root factor
  size_t maxrank = 0;

  for(size_t k = 0; k < n; ++k) {

    if(choleskyStep(ATA, k)) {
      if(debug) cout << "choleskyCareful:  Factored through " << k << endl;
      if(debug) print(ATA, "ATA: ");
      maxrank = k+1;
    } else {
      if(debug) cout << "choleskyCareful:  Skipping " << k << endl;
    }

//    // If the diagonal element is not zero, run Cholesky as far as possible -
//    // Cholesky will stop on the first zero diagonal element.  Because ATA is
//    // symmetric positive semi-definite, a zero diagonal element implies
//    // a corresponding row and column of zeros, thus we need only check the
//    // diagonal.
//    if(ATA(k,k) > zeroTol || ATA(k,k) < -zeroTol) {
//
//      // Try to do Cholesky on the remaining lower-right square submatrix.
//      int info = lapack_dpotf2('U', n-k, &(ATA(k,k)), n);
//
//      if(info > 0) {
//        // The submatrix is rank-deficient, but Cholesky factored the first
//        // subRank rows/columns, leaving a positive semi-definite matrix
//        // starting at subRank.  (we're speaking in zero-based indexices).
//        size_t subRank = info - 1;
//
//        // The row/column after the last nonzero one.
//        maxrank = k + subRank;
//
//        // We know that the row/column just after the factored part is zero, so
//        // skip it.  Note that after this statement k will be the next
//        // row/column to process.
//        k += subRank + 1;
//
//        if(debug) cout << "choleskyCareful:  Factored until " << maxrank << ", skipping next." << endl;
//
//      } else if(info == 0) {
//        // Cholesky successfully factored the rest of the matrix.  Note that
//        // after this statement k will be the last processed row/column, and
//        // will be incremented by 1 by the 'for' loop.
//        k += n - k;
//
//        // The last row/column is nonzero.
//        maxrank = n;
//
//        if(debug) cout << "choleskyCareful:  Factored the remainder" << endl;
//
//      } else {
//        throw std::domain_error(boost::str(boost::format(
//            "Bad input to choleskyFactorUnderdetermined, dpotrf returned %d.\n")%info));
//      }
//    } else {
//
//      if(debug) cout << "choleskyCareful:  Skipping " << k << endl;
//
//      // The diagonal element is numerically zero, so skip this row/column.
//      ++ k;
//    }
//
//    if(debug) print(ATA, "ATA: ");
  }

  return maxrank;
}

/* ************************************************************************* */
void choleskyPartial(MatrixColMajor& ABC, size_t nFrontal) {

  static const bool debug = false;

  assert(ABC.size1() == ABC.size2());
  assert(nFrontal <= ABC.size1());

  const size_t n = ABC.size1();

  // Compute Cholesky factorization of A, overwrites A.
  tic(1, "dpotrf");
  int info = lapack_dpotrf('U', nFrontal, &ABC(0,0), n);
  if(info != 0) {
    if(info < 0)
      throw std::domain_error(boost::str(boost::format(
          "Bad input to choleskyFactorUnderdetermined, dpotrf returned %d.\n")%info));
    else
      throw std::domain_error(boost::str(boost::format(
          "The matrix passed into choleskyFactorUnderdetermined is numerically rank-deficient, dpotrf returned rank=%d, expected rank was %d.\n")%(info-1)%nFrontal));
  }
  toc(1, "dpotrf");

  // Views of R, S, and L submatrices.
  ublas::matrix_range<MatrixColMajor> R(ublas::project(ABC, ublas::range(0,nFrontal), ublas::range(0,nFrontal)));
  ublas::matrix_range<MatrixColMajor> S(ublas::project(ABC, ublas::range(0,nFrontal), ublas::range(nFrontal,n)));
  ublas::matrix_range<MatrixColMajor> Lf(ublas::project(ABC, ublas::range(nFrontal,n), ublas::range(nFrontal,n)));
  ublas::symmetric_adaptor<typeof(Lf), ublas::upper> L(Lf);

  // Compute S = inv(R') * B
  tic(2, "compute S");
  if(S.size2() > 0)
    cblas_dtrsm(CblasColMajor, CblasLeft, CblasUpper, CblasTrans, CblasNonUnit, S.size1(), S.size2(), 1.0, &R(0,0), n, &S(0,0), n);
  if(debug) gtsam::print(S, "S: ");
  toc(2, "compute S");

  // Compute L = C - S' * S
  tic(3, "compute L");
  if(debug) gtsam::print(L, "C: ");
  if(L.size2() > 0)
    L -= ublas::prod(ublas::trans(S), S);
  if(debug) gtsam::print(L, "L: ");
  toc(3, "compute L");
}

}

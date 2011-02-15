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

#include <gtsam/base/debug.h>
#include <gtsam/base/cholesky.h>
#include <gtsam/base/lapack.h>
#include <gtsam/base/timing.h>

#include <gtsam/3rdparty/Eigen/Core>
#include <gtsam/3rdparty/Eigen/Dense>

#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost/numeric/ublas/symmetric.hpp>
#include <boost/numeric/ublas/blas.hpp>
#include <boost/format.hpp>

namespace ublas = boost::numeric::ublas;

using namespace std;

namespace gtsam {

  static const double negativePivotThreshold = -1e-1;
  static const double zeroPivotThreshold = 1e-6;
  static const double underconstrainedPrior = 1e-5;

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
static inline bool choleskyStep(MatrixColMajor& ATA, size_t k, size_t order) {

  // Get pivot value
  double alpha = ATA(k,k);

  // Correct negative pivots from round-off error
  if(alpha < negativePivotThreshold) {
    cout << "pivot = " << alpha << endl;
    print(ATA, "Partially-factorized matrix: ");
    throw(invalid_argument("The matrix was found to be non-positive-semidefinite when factoring with careful Cholesky."));
  } else if(alpha < 0.0)
    alpha = 0.0;
    
  const double beta = sqrt(alpha);

  if(beta > zeroPivotThreshold) {
    const double betainv = 1.0 / beta;

    // Update k,k
    ATA(k,k) = beta;

    if(k < (order-1)) {
      ublas::matrix_row<MatrixColMajor> Vf(ublas::row(ATA, k));
      ublas::vector_range<typeof(Vf)> V(ublas::subrange(Vf, k+1,order));

      // Update A(k,k+1:end) <- A(k,k+1:end) / beta
      V *= betainv;

      // Update A(k+1:end, k+1:end) <- A(k+1:end, k+1:end) - v*v' / alpha
      ublas::matrix_range<MatrixColMajor> L(ublas::subrange(ATA, k+1,order, k+1,order));
      L -= ublas::outer_prod(V, V);
    }

    return true;
  } else {
    // For zero pivots, add the underconstrained variable prior
    ATA(k,k) = underconstrainedPrior;
    for(size_t j=k+1; j<order; ++j)
      ATA(k,j) = 0.0;
    return false;
  }
}

/* ************************************************************************* */
pair<size_t,bool> choleskyCareful(MatrixColMajor& ATA, int order) {

  const bool debug = ISDEBUG("choleskyCareful");

  // Check that the matrix is square (we do not check for symmetry)
  assert(ATA.size1() == ATA.size2());

  // Number of rows/columns
  const size_t n = ATA.size1();

  // Negative order means factor the entire matrix
  if(order < 0)
    order = n;

  assert(size_t(order) <= n);

  // The index of the row after the last non-zero row of the square-root factor
  size_t maxrank = 0;
  bool fullRank = true;

  // Factor row-by-row
  for(size_t k = 0; k < size_t(order); ++k) {
    if(choleskyStep(ATA, k, size_t(order))) {
      if(debug) cout << "choleskyCareful:  Factored through " << k << endl;
      if(debug) print(ATA, "ATA: ");
      maxrank = k+1;
    } else {
      fullRank = false;
      if(debug) cout << "choleskyCareful:  Skipping " << k << endl;
    }
  }

  return make_pair(maxrank, fullRank);
}

/* ************************************************************************* */
void choleskyPartial(MatrixColMajor& ABCublas, size_t nFrontal) {

  const bool debug = ISDEBUG("choleskyPartial");

  Eigen::Map<Eigen::MatrixXd> ABC(&ABCublas(0,0), ABCublas.size1(), ABCublas.size2());

  assert(ABC.rows() == ABC.cols());
  assert(ABC.rows() >= 0 && nFrontal <= size_t(ABC.rows()));

  const size_t n = ABC.rows();

  // Compute Cholesky factorization of A, overwrites A.
  tic(1, "lld");
  ABC.block(0,0,nFrontal,nFrontal).triangularView<Eigen::Upper>() =
      ABC.block(0,0,nFrontal,nFrontal).selfadjointView<Eigen::Upper>().llt().matrixU();
  toc(1, "lld");

  if(debug) cout << "R:\n" << Eigen::MatrixXd(ABC.topLeftCorner(nFrontal,nFrontal).triangularView<Eigen::Upper>()) << endl;

  // Compute S = inv(R') * B
  tic(2, "compute S");
  if(n - nFrontal > 0) {
    ABC.topLeftCorner(nFrontal,nFrontal).triangularView<Eigen::Upper>().transpose().solveInPlace(
        ABC.topRightCorner(nFrontal, n-nFrontal));
  }
  if(debug) cout << "S:\n" << ABC.topRightCorner(nFrontal, n-nFrontal) << endl;
  toc(2, "compute S");

  // Compute L = C - S' * S
  tic(3, "compute L");
  if(debug) cout << "C:\n" << Eigen::MatrixXd(ABC.bottomRightCorner(n-nFrontal,n-nFrontal).selfadjointView<Eigen::Upper>()) << endl;
  if(n - nFrontal > 0)
    ABC.bottomRightCorner(n-nFrontal,n-nFrontal).selfadjointView<Eigen::Upper>().rankUpdate(
        ABC.topRightCorner(nFrontal, n-nFrontal).transpose(), -1.0);
  if(debug) cout << "L:\n" << Eigen::MatrixXd(ABC.bottomRightCorner(n-nFrontal,n-nFrontal).selfadjointView<Eigen::Upper>()) << endl;
  toc(3, "compute L");
}

}

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
 * @date    Nov 5, 2010
 */

#include <gtsam/base/debug.h>
#include <gtsam/base/cholesky.h>
#include <gtsam/base/timing.h>

#include <gtsam/3rdparty/Eigen/Eigen/Core>
#include <gtsam/3rdparty/Eigen/Eigen/Dense>

#include <boost/format.hpp>
#include <cmath>

using namespace std;

namespace gtsam {

  static const double negativePivotThreshold = -1e-1;
  static const double zeroPivotThreshold = 1e-6;
  static const double underconstrainedPrior = 1e-5;
	static const int underconstrainedExponentDifference = 12;

/* ************************************************************************* */
static inline int choleskyStep(Matrix& ATA, size_t k, size_t order) {

	const bool debug = ISDEBUG("choleskyCareful");

  // Get pivot value
  double alpha = ATA(k,k);

  // Correct negative pivots from round-off error
  if(alpha < negativePivotThreshold) {
		if(debug) {
			cout << "pivot = " << alpha << endl;
			print(ATA, "Partially-factorized matrix: ");
		}
    return -1;
  } else if(alpha < 0.0)
    alpha = 0.0;
    
  const double beta = sqrt(alpha);

  if(beta > zeroPivotThreshold) {
    const double betainv = 1.0 / beta;

    // Update k,k
    ATA(k,k) = beta;

    if(k < (order-1)) {
      // Update A(k,k+1:end) <- A(k,k+1:end) / beta
    	typedef Matrix::RowXpr::SegmentReturnType BlockRow;
    	BlockRow V = ATA.row(k).segment(k+1, order-(k+1));
    	V *= betainv;

      // Update A(k+1:end, k+1:end) <- A(k+1:end, k+1:end) - v*v' / alpha
    	ATA.block(k+1, k+1, order-(k+1), order-(k+1)) -= V.transpose() * V;
//    	ATA.bottomRightCorner(order-(k+1), order-(k+1)).selfadjointView<Eigen::Upper>()
//    	    .rankUpdate(V.adjoint(), -1);
    }
		return 1;
  } else {
    // For zero pivots, add the underconstrained variable prior
    ATA(k,k) = underconstrainedPrior;
    for(size_t j=k+1; j<order; ++j)
      ATA(k,j) = 0.0;
		if(debug) cout << "choleskyCareful:  Skipping " << k << endl;
		return 0;
  }
}

/* ************************************************************************* */
pair<size_t,bool> choleskyCareful(Matrix& ATA, int order) {

  const bool debug = ISDEBUG("choleskyCareful");

  // Check that the matrix is square (we do not check for symmetry)
  assert(ATA.rows() == ATA.cols());

  // Number of rows/columns
  const size_t n = ATA.rows();

  // Negative order means factor the entire matrix
  if(order < 0)
    order = int(n);

  assert(size_t(order) <= n);

  // The index of the row after the last non-zero row of the square-root factor
  size_t maxrank = 0;
	bool success = true;

  // Factor row-by-row
  for(size_t k = 0; k < size_t(order); ++k) {
    int stepResult = choleskyStep(ATA, k, size_t(order));
		if(stepResult == 1) {
      if(debug) cout << "choleskyCareful:  Factored through " << k << endl;
      if(debug) print(ATA, "ATA: ");
      maxrank = k+1;
    } else if(stepResult == -1) {
      success = false;
			break;
    } /* else if(stepResult == 0) Found zero pivot */
  }

  return make_pair(maxrank, success);
}

/* ************************************************************************* */
bool choleskyPartial(Matrix& ABC, size_t nFrontal) {

  const bool debug = ISDEBUG("choleskyPartial");

  assert(ABC.rows() == ABC.cols());
  assert(ABC.rows() >= 0 && nFrontal <= size_t(ABC.rows()));

  const size_t n = ABC.rows();

  // Compute Cholesky factorization of A, overwrites A.
  tic(1, "lld");
	Eigen::LLT<Matrix, Eigen::Upper> llt = ABC.block(0,0,nFrontal,nFrontal).selfadjointView<Eigen::Upper>().llt();
  ABC.block(0,0,nFrontal,nFrontal).triangularView<Eigen::Upper>() = llt.matrixU();
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

	// Check last diagonal element - Eigen does not check it
	bool ok;
	if(llt.info() == Eigen::Success) {
		if(nFrontal >= 2) {
			int exp2, exp1;
			(void)frexp(ABC(nFrontal-2, nFrontal-2), &exp2);
			(void)frexp(ABC(nFrontal-1, nFrontal-1), &exp1);
			ok = (exp2 - exp1 < underconstrainedExponentDifference);
		} else if(nFrontal == 1) {
			int exp1;
			(void)frexp(ABC(0,0), &exp1);
			ok = (exp1 > -underconstrainedExponentDifference);
		} else {
			ok = true;
		}
	} else {
		ok = false;
	}

	return ok;
}

}

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
 * @author  Frank Dellaert
 * @date    Nov 5, 2010
 */

#include <gtsam/base/cholesky.h>
#include <gtsam/base/timing.h>

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
  // Get pivot value
  double alpha = ATA(k, k);

  // Correct negative pivots from round-off error
  if (alpha < negativePivotThreshold) {
    return -1;
  } else if (alpha < 0.0)
    alpha = 0.0;

  const double beta = sqrt(alpha);

  if (beta > zeroPivotThreshold) {
    const double betainv = 1.0 / beta;

    // Update k,k
    ATA(k, k) = beta;

    if (k < (order - 1)) {
      // Update A(k,k+1:end) <- A(k,k+1:end) / beta
      typedef Matrix::RowXpr::SegmentReturnType BlockRow;
      BlockRow V = ATA.row(k).segment(k + 1, order - (k + 1));
      V *= betainv;

      // Update A(k+1:end, k+1:end) <- A(k+1:end, k+1:end) - v*v' / alpha
      ATA.block(k + 1, k + 1, order - (k + 1), order - (k + 1)) -= V.transpose() * V;
      //      ATA.bottomRightCorner(order-(k+1), order-(k+1)).selfadjointView<Eigen::Upper>()
      //          .rankUpdate(V.adjoint(), -1);
    }
    return 1;
  } else {
    // For zero pivots, add the underconstrained variable prior
    ATA(k, k) = underconstrainedPrior;
    for (size_t j = k + 1; j < order; ++j)
      ATA(k, j) = 0.0;
    return 0;
  }
}

/* ************************************************************************* */
pair<size_t, bool> choleskyCareful(Matrix& ATA, int order) {
  // Check that the matrix is square (we do not check for symmetry)
  assert(ATA.rows() == ATA.cols());

  // Number of rows/columns
  const size_t n = ATA.rows();

  // Negative order means factor the entire matrix
  if (order < 0)
    order = int(n);

  assert(size_t(order) <= n);

  // The index of the row after the last non-zero row of the square-root factor
  size_t maxrank = 0;
  bool success = true;

  // Factor row-by-row
  for (size_t k = 0; k < size_t(order); ++k) {
    int stepResult = choleskyStep(ATA, k, size_t(order));
    if (stepResult == 1) {
      maxrank = k + 1;
    } else if (stepResult == -1) {
      success = false;
      break;
    } /* else if(stepResult == 0) Found zero pivot */
  }

  return make_pair(maxrank, success);
}

/* ************************************************************************* */
bool choleskyPartial(Matrix& ABC, size_t nFrontal, size_t topleft) {
  gttic(choleskyPartial);
  if (nFrontal == 0)
    return true;

  assert(ABC.cols() == ABC.rows());
  assert(size_t(ABC.rows()) >= topleft);
  const size_t n = static_cast<size_t>(ABC.rows() - topleft);
  assert(nFrontal <= size_t(n));

  // Create views on blocks
  auto A = ABC.block(topleft, topleft, nFrontal, nFrontal);
  auto B = ABC.block(topleft, topleft + nFrontal, nFrontal, n - nFrontal);
  auto C = ABC.block(topleft + nFrontal, topleft + nFrontal, n - nFrontal, n - nFrontal);

  // Compute Cholesky factorization A = R'*R, overwrites A.
  gttic(LLT);
  Eigen::LLT<Matrix, Eigen::Upper> llt(A);
  Eigen::ComputationInfo lltResult = llt.info();
  if (lltResult != Eigen::Success)
    return false;
  auto R = A.triangularView<Eigen::Upper>();
  R = llt.matrixU();
  gttoc(LLT);

  // Compute S = inv(R') * B
  gttic(compute_S);
  if (nFrontal < n)
    R.transpose().solveInPlace(B);
  gttoc(compute_S);

  // Compute L = C - S' * S
  gttic(compute_L);
  if (nFrontal < n)
    C.selfadjointView<Eigen::Upper>().rankUpdate(B.transpose(), -1.0);
  gttoc(compute_L);

  // Check last diagonal element - Eigen does not check it
  if (nFrontal >= 2) {
    int exp2, exp1;
    // NOTE(gareth): R is already the size of A, so we don't need to add topleft here.
    (void)frexp(R(nFrontal - 2, nFrontal - 2), &exp2);
    (void)frexp(R(nFrontal - 1, nFrontal - 1), &exp1);
    return (exp2 - exp1 < underconstrainedExponentDifference);
  } else if (nFrontal == 1) {
    int exp1;
    (void)frexp(R(0, 0), &exp1);
    return (exp1 > -underconstrainedExponentDifference);
  } else {
    return true;
  }
}
}  // namespace gtsam

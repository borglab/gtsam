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

#include <Eigen/Cholesky>
#include <cassert>
#include <cmath>
#include <limits>

using namespace std;

namespace gtsam {

static const double negativePivotThreshold = -1e-1;
static const double zeroPivotThreshold = 1e-6;
static const double underconstrainedPrior = 1e-5;
// Limit the estimated relative error kappa(D*A*D) * epsilon to sqrt(epsilon),
// where D equilibrates the diagonal. This is invariant under diagonal changes
// of variable units and symmetric permutations.
static const double minimumReciprocalConditionNumber =
    std::sqrt(std::numeric_limits<double>::epsilon());

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

  // Equilibrate the diagonal before checking conditioning. If E = D*A*D and
  // E = Re'*Re, then A = R'*R with R = Re*inv(D), which only rescales the
  // columns of the upper-triangular factor.
  gttic(LLT);
  if (!A.diagonal().allFinite() || (A.diagonal().array() <= 0.0).any()) {
    return false;
  }
  const Vector inverseDiagonalSqrt =
      A.diagonal().cwiseSqrt().cwiseInverse();
  // A is usually a strided block, so pack its upper triangle before the
  // in-place factorization to retain Eigen's contiguous-storage performance.
  Matrix factorStorage(nFrontal, nFrontal);
  factorStorage.triangularView<Eigen::Upper>() = A;
  for (DenseIndex column = 0; column < factorStorage.cols(); ++column) {
    auto upperColumn = factorStorage.col(column).head(column + 1);
    upperColumn.array() *= inverseDiagonalSqrt.head(column + 1).array();
    upperColumn *= inverseDiagonalSqrt(column);
  }

  Eigen::Map<Matrix> factorStorageMap(factorStorage.data(), nFrontal,
                                     nFrontal);
  Eigen::LLT<Eigen::Map<Matrix>, Eigen::Upper> llt(factorStorageMap);
  Eigen::ComputationInfo lltResult = llt.info();
  if (lltResult != Eigen::Success) {
    return false;
  }
  if (llt.rcond() < minimumReciprocalConditionNumber) {
    return false;
  }
  for (DenseIndex column = 0; column < factorStorage.cols(); ++column) {
    factorStorage.col(column).head(column + 1) /=
        inverseDiagonalSqrt(column);
  }
  auto R = A.triangularView<Eigen::Upper>();
  R = factorStorage;
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

  return true;
}
}  // namespace gtsam

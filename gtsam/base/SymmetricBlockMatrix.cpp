/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SymmetricBlockMatrix.cpp
 * @brief   Access to matrices via blocks of pre-defined sizes.  Used in GaussianFactor and GaussianConditional.
 * @author  Richard Roberts
 * @date    Sep 18, 2010
 */

#include <gtsam/base/SymmetricBlockMatrix.h>
#include <gtsam/base/VerticalBlockMatrix.h>
#include <gtsam/base/cholesky.h>
#include <gtsam/base/timing.h>
#include <gtsam/base/ThreadsafeException.h>

namespace gtsam {

/* ************************************************************************* */
SymmetricBlockMatrix SymmetricBlockMatrix::LikeActiveViewOf(
    const SymmetricBlockMatrix& other) {
  SymmetricBlockMatrix result;
  result.variableColOffsets_.resize(other.nBlocks() + 1);
  for (size_t i = 0; i < result.variableColOffsets_.size(); ++i)
    result.variableColOffsets_[i] = other.variableColOffsets_[other.blockStart_
        + i] - other.variableColOffsets_[other.blockStart_];
  result.matrix_.resize(other.cols(), other.cols());
  result.assertInvariants();
  return result;
}

/* ************************************************************************* */
SymmetricBlockMatrix SymmetricBlockMatrix::LikeActiveViewOf(
    const VerticalBlockMatrix& other) {
  SymmetricBlockMatrix result;
  result.variableColOffsets_.resize(other.nBlocks() + 1);
  for (size_t i = 0; i < result.variableColOffsets_.size(); ++i)
    result.variableColOffsets_[i] = other.variableColOffsets_[other.blockStart_
        + i] - other.variableColOffsets_[other.blockStart_];
  result.matrix_.resize(other.cols(), other.cols());
  result.assertInvariants();
  return result;
}

/* ************************************************************************* */
Matrix SymmetricBlockMatrix::block(DenseIndex I, DenseIndex J) const {
  if (I == J) {
    return diagonalBlock(I);
  } else if (I < J) {
    return aboveDiagonalBlock(I, J);
  } else {
    return aboveDiagonalBlock(J, I).transpose();
  }
}

/* ************************************************************************* */
void SymmetricBlockMatrix::choleskyPartial(DenseIndex nFrontals) {
  gttic(VerticalBlockMatrix_choleskyPartial);
  DenseIndex topleft = variableColOffsets_[blockStart_];
  if (!gtsam::choleskyPartial(matrix_, offset(nFrontals) - topleft, topleft)) {
    throw CholeskyFailed();
  }
}

/* ************************************************************************* */
VerticalBlockMatrix SymmetricBlockMatrix::split(DenseIndex nFrontals) {
  gttic(VerticalBlockMatrix_split);

  // Construct a VerticalBlockMatrix that contains [R Sd]
  const size_t n1 = offset(nFrontals);
  VerticalBlockMatrix RSd = VerticalBlockMatrix::LikeActiveViewOf(*this, n1);

  // Copy into it.
  RSd.full() = matrix_.topRows(n1);
  RSd.full().triangularView<Eigen::StrictlyLower>().setZero();

  // Take lower-right block of Ab_ to get the remaining factor
  blockStart() = nFrontals;

  return RSd;
}

/* ************************************************************************* */

} //\ namespace gtsam


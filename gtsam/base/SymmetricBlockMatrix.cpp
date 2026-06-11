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
SymmetricBlockMatrix::SymmetricBlockMatrix() : blockStart_(0) {
  variableColOffsets_.push_back(0);
  assertInvariants();
}

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
void SymmetricBlockMatrix::negate() {
  full().triangularView<Eigen::Upper>() *= -1.0;
}

/* ************************************************************************* */
void SymmetricBlockMatrix::invertInPlace() {
  const auto identity = Matrix::Identity(rows(), rows());
  full().triangularView<Eigen::Upper>() =
      selfadjointView().llt().solve(identity).triangularView<Eigen::Upper>();
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
void SymmetricBlockMatrix::split(DenseIndex nFrontals,
                                 VerticalBlockMatrix* RSd) {
  gttic(VerticalBlockMatrix_split);
  assert(RSd);

  // Construct a VerticalBlockMatrix that contains [R Sd]
  const DenseIndex n1 = offset(nFrontals);
  assert(RSd->rows() == n1);
  assert(RSd->cols() == cols());
  assert(RSd->nBlocks() == nBlocks());

  // Copy into it.
  RSd->full() = matrix_.topRows(n1);
  RSd->full().triangularView<Eigen::StrictlyLower>().setZero();

  // Take lower-right block of Ab_ to get the remaining factor
  blockStart() = nFrontals;
}

VerticalBlockMatrix SymmetricBlockMatrix::split(DenseIndex nFrontals) {
  // Construct a VerticalBlockMatrix that contains [R Sd]
  const DenseIndex n1 = offset(nFrontals);
  VerticalBlockMatrix RSd = VerticalBlockMatrix::LikeActiveViewOf(*this, n1);
  split(nFrontals, &RSd);
  return RSd;
}

/* ************************************************************************* */
void SymmetricBlockMatrix::updateFromMappedBlocks(
    const SymmetricBlockMatrix& other,
    const std::vector<DenseIndex>& blockIndices) {
  assert(static_cast<DenseIndex>(blockIndices.size()) == other.nBlocks());
  const DenseIndex otherBlocks = other.nBlocks();
  for (DenseIndex i = 0; i < otherBlocks; ++i) {
    const DenseIndex I = blockIndices[i];
    if (I < 0) continue;
    assert(I < nBlocks());
    updateDiagonalBlock(I, other.diagonalBlock(i));
    for (DenseIndex j = i + 1; j < otherBlocks; ++j) {
      const DenseIndex J = blockIndices[j];
      if (J < 0) continue;
      assert(J < nBlocks());
      updateOffDiagonalBlock(I, J, other.aboveDiagonalBlock(i, j));
    }
  }
}

/* ************************************************************************* */
void SymmetricBlockMatrix::updateFromOuterProductBlocks(
    const VerticalBlockMatrix& other,
    const std::vector<DenseIndex>& blockIndices) {
  assert(static_cast<DenseIndex>(blockIndices.size()) == other.nBlocks());
  const DenseIndex otherBlocks = other.nBlocks();
  for (DenseIndex i = 0; i < otherBlocks; ++i) {
    const DenseIndex I = blockIndices[i];
    if (I < 0) continue;
    assert(I < nBlocks());
    const auto Si = other(i);
    Matrix diag = Si.transpose() * Si;
    updateDiagonalBlock(I, diag);
    for (DenseIndex j = i + 1; j < otherBlocks; ++j) {
      const DenseIndex J = blockIndices[j];
      if (J < 0) continue;
      assert(J < nBlocks());
      const auto Sj = other(j);
      Matrix off = Si.transpose() * Sj;
      updateOffDiagonalBlock(I, J, off);
    }
  }
}

/* ************************************************************************* */

} //\ namespace gtsam

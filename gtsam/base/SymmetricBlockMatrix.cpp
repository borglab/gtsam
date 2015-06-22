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
VerticalBlockMatrix SymmetricBlockMatrix::choleskyPartial(
    DenseIndex nFrontals) {
  // Do dense elimination
  if (blockStart() != 0)
    throw std::invalid_argument(
        "Can only do Cholesky when the SymmetricBlockMatrix is not a restricted view, i.e. when blockStart == 0.");
  if (!gtsam::choleskyPartial(matrix_, offset(nFrontals)))
    throw CholeskyFailed();

  // Split conditional

  // Create one big conditionals with many frontal variables.
  gttic(Construct_conditional);
  const size_t varDim = offset(nFrontals);
  VerticalBlockMatrix Ab = VerticalBlockMatrix::LikeActiveViewOf(*this, varDim);
  Ab.full() = matrix_.topRows(varDim);
  Ab.full().triangularView<Eigen::StrictlyLower>().setZero();
  gttoc(Construct_conditional);

  gttic(Remaining_factor);
  // Take lower-right block of Ab_ to get the remaining factor
  blockStart() = nFrontals;
  gttoc(Remaining_factor);

  return Ab;
}
/* ************************************************************************* */

} //\ namespace gtsam


/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    VerticalBlockMatrix.cpp
 * @brief   A matrix with column blocks of pre-defined sizes.  Used in JacobianFactor and
 *        GaussianConditional.
 * @author  Richard Roberts
 * @date    Sep 18, 2010 */

#include <gtsam/base/VerticalBlockMatrix.h>
#include <gtsam/base/SymmetricBlockMatrix.h>

namespace gtsam {

/* ************************************************************************* */
VerticalBlockMatrix VerticalBlockMatrix::LikeActiveViewOf(
    const VerticalBlockMatrix& other) {
  VerticalBlockMatrix result;
  result.variableColOffsets_.resize(other.nBlocks() + 1);
  for (size_t i = 0; i < result.variableColOffsets_.size(); ++i)
    result.variableColOffsets_[i] = other.variableColOffsets_[other.blockStart_
        + i] - other.variableColOffsets_[other.blockStart_];
  result.matrix_.resize(other.rows(), result.variableColOffsets_.back());
  result.rowEnd_ = other.rows();
  result.assertInvariants();
  return result;
}

/* ************************************************************************* */
VerticalBlockMatrix VerticalBlockMatrix::LikeActiveViewOf(
    const SymmetricBlockMatrix& other, DenseIndex height) {
  VerticalBlockMatrix result;
  result.variableColOffsets_.resize(other.nBlocks() + 1);
  for (size_t i = 0; i < result.variableColOffsets_.size(); ++i)
    result.variableColOffsets_[i] = other.variableColOffsets_[other.blockStart_
        + i] - other.variableColOffsets_[other.blockStart_];
  result.matrix_.resize(height, result.variableColOffsets_.back());
  result.rowEnd_ = height;
  result.assertInvariants();
  return result;
}

}

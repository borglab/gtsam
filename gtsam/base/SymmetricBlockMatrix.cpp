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

namespace gtsam {

  /* ************************************************************************* */
  SymmetricBlockMatrix SymmetricBlockMatrix::LikeActiveViewOf(const SymmetricBlockMatrix& other)
  {
    SymmetricBlockMatrix result;
    result.variableColOffsets_.resize(other.nBlocks() + 1);
    std::copy(other.variableColOffsets_.begin() + other.blockStart_, other.variableColOffsets_.end(),
      result.variableColOffsets_.begin());
    result.matrix_.resize(other.cols(), other.cols());
    result.assertInvariants();
    return result;
  }

  /* ************************************************************************* */
  SymmetricBlockMatrix SymmetricBlockMatrix::LikeActiveViewOf(const VerticalBlockMatrix& other)
  {
    SymmetricBlockMatrix result;
    result.variableColOffsets_.resize(other.nBlocks() + 1);
    std::copy(other.variableColOffsets_.begin() + other.blockStart_, other.variableColOffsets_.end(),
      result.variableColOffsets_.begin());
    result.matrix_.resize(other.cols(), other.cols());
    result.assertInvariants();
    return result;
  }

}

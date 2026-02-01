/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    HessianFactor-inl.h
 * @brief   Contains the HessianFactor class, a general quadratic factor
 * @author  Richard Roberts
 * @date    Dec 8, 2010
 */

#pragma once

namespace gtsam {

  /* ************************************************************************* */
  template<typename KEYS>
  HessianFactor::HessianFactor(const KEYS& keys, const SymmetricBlockMatrix& augmentedInformation) :
    GaussianFactor(keys), info_(augmentedInformation)
  {
    // Check number of variables
    if((DenseIndex)Base::keys_.size() != augmentedInformation.nBlocks() - 1)
      throw std::invalid_argument(
      "Error in HessianFactor constructor input.  Number of provided keys plus\n"
      "one for the information vector must equal the number of provided matrix blocks. ");

    // Check RHS dimension
    if(augmentedInformation.getDim(augmentedInformation.nBlocks() - 1) != 1)
      throw std::invalid_argument(
      "Error in HessianFactor constructor input.  The last provided matrix block\n"
      "must be the information vector, but the last provided block had more than one column.");
  }

}

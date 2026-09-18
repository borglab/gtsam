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
  inline void HessianFactor::CheckAugmentedInformation(
      size_t keyCount, const SymmetricBlockMatrix& augmentedInformation) {
    const DenseIndex numBlocks = augmentedInformation.nBlocks();
    if (numBlocks < 1 || static_cast<DenseIndex>(keyCount) != numBlocks - 1) {
      throw std::invalid_argument(
          "Error in HessianFactor constructor input.  Number of provided keys plus\n"
          "one for the information vector must equal the number of provided matrix blocks. ");
    }

    if (augmentedInformation.getDim(numBlocks - 1) != 1) {
      throw std::invalid_argument(
          "Error in HessianFactor constructor input.  The last provided matrix block\n"
          "must be the information vector, but the last provided block had more than one column.");
    }
  }

  /* ************************************************************************* */
  template<typename KEYS>
  HessianFactor::HessianFactor(const KEYS& keys, const SymmetricBlockMatrix& augmentedInformation) :
    GaussianFactor(keys), info_(augmentedInformation)
  {
    CheckAugmentedInformation(Base::keys_.size(), info_);
  }

  /* ************************************************************************* */
  template <typename KEYS>
  HessianFactor::HessianFactor(
      const KEYS& keys, SymmetricBlockMatrix&& augmentedInformation)
      : GaussianFactor(keys), info_(std::move(augmentedInformation)) {
    CheckAugmentedInformation(Base::keys_.size(), info_);
  }

}

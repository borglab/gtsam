/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GaussianConditional-inl.h
 * @brief   Conditional Gaussian Base class
 * @author  Christian Potthast
 */

// \callgraph

#pragma once

namespace gtsam {

  /* ************************************************************************* */
  template<typename TERMS>
  GaussianConditional::GaussianConditional(const TERMS& terms,
    size_t nrFrontals, const Vector& d, const SharedDiagonal& sigmas) :
  BaseFactor(terms, d, sigmas), BaseConditional(nrFrontals) {}

  /* ************************************************************************* */
  template<typename KEYS>
  GaussianConditional::GaussianConditional(
    const KEYS& keys, size_t nrFrontals, const VerticalBlockMatrix& augmentedMatrix, const SharedDiagonal& sigmas) :
  BaseFactor(keys, augmentedMatrix, sigmas), BaseConditional(nrFrontals) {}

} // gtsam

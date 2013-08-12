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

#include <boost/range/join.hpp>
#include <boost/assign/list_of.hpp>

namespace gtsam {

  /* ************************************************************************* */
  template<class PARENTS>
  GaussianConditional::GaussianConditional(Index key, const Vector& d,
    const Matrix& R, const PARENTS& parents, const SharedDiagonal& sigmas, const typename PARENTS::value_type*) :
  BaseFactor(boost::join(
    ListOfOne<typename PARENTS::value_type>(std::make_pair(key, R)),
    parents), d, sigmas),
    BaseConditional(1) {}

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

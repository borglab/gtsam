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
  template<typename PARENTS>
  GaussianConditionalUnordered::GaussianConditionalUnordered(Index key, const Vector& d,
    const Matrix& R, const PARENTS& parents, const SharedDiagonal& sigmas) :
  BaseFactor(boost::join(boost::assign::cref_list_of<1>(std::make_pair(key, R)), parents), d, sigmas),
    BaseConditional(1) {}

  /* ************************************************************************* */
  template<typename TERMS>
  GaussianConditionalUnordered::GaussianConditionalUnordered(const TERMS& terms,
    size_t nrFrontals, const Vector& d, const SharedDiagonal& sigmas) :
  BaseFactor(terms, d, sigmas), BaseConditional(nrFrontals) {}

  /* ************************************************************************* */
  template<typename KEYS>
  GaussianConditionalUnordered::GaussianConditionalUnordered(
    const KEYS& keys, size_t nrFrontals, const VerticalBlockMatrix& augmentedMatrix, const SharedDiagonal& sigmas) :
  BaseFactor(keys, augmentedMatrix, sigmas), BaseConditional(nrFrontals) {}

  /* ************************************************************************* */
  template<typename ITERATOR>
  GaussianConditionalUnordered::shared_ptr GaussianConditionalUnordered::Combine(ITERATOR firstConditional, ITERATOR lastConditional)
  {
    // TODO:  check for being a clique

    // Get dimensions from first conditional
    std::vector<size_t> dims;   dims.reserve((*firstConditional)->size() + 1);
    for(const_iterator j = (*firstConditional)->begin(); j != (*firstConditional)->end(); ++j)
      dims.push_back((*firstConditional)->dim(j));
    dims.push_back(1);

    // We assume the conditionals form clique, so the first n variables will be
    // frontal variables in the new conditional.
    size_t nFrontals = 0;
    size_t nRows = 0;
    for(ITERATOR c = firstConditional; c != lastConditional; ++c) {
      nRows += dims[nFrontals];
      ++ nFrontals;
    }

    // Allocate combined conditional, has same keys as firstConditional
    Matrix tempCombined;
    VerticalBlockView<Matrix> tempBlockView(tempCombined, dims.begin(), dims.end(), 0);
    GaussianConditional::shared_ptr combinedConditional(new GaussianConditional((*firstConditional)->begin(), (*firstConditional)->end(), nFrontals, tempBlockView, zero(nRows)));

    // Resize to correct number of rows
    combinedConditional->matrix_.resize(nRows, combinedConditional->matrix_.cols());
    combinedConditional->rsd_.rowEnd() = combinedConditional->matrix_.rows();

    // Copy matrix and sigmas
    const size_t totalDims = combinedConditional->matrix_.cols();
    size_t currentSlot = 0;
    for(ITERATOR c = firstConditional; c != lastConditional; ++c) {
      const size_t startRow = combinedConditional->rsd_.offset(currentSlot); // Start row is same as start column
      combinedConditional->rsd_.range(0, currentSlot).block(startRow, 0, dims[currentSlot], combinedConditional->rsd_.offset(currentSlot)).operator=(
        Matrix::Zero(dims[currentSlot], combinedConditional->rsd_.offset(currentSlot)));
      combinedConditional->rsd_.range(currentSlot, dims.size()).block(startRow, 0, dims[currentSlot], totalDims - startRow).operator=(
        (*c)->matrix_);
      combinedConditional->sigmas_.segment(startRow, dims[currentSlot]) = (*c)->sigmas_;
      ++ currentSlot;
    }

    return combinedConditional;
  }

} // gtsam

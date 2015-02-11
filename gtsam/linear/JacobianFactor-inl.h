/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    JacobianFactor.h
 * @author  Richard Roberts
 * @author  Christian Potthast
 * @author  Frank Dellaert
 * @date    Dec 8, 2010
 */
#pragma once

#include <gtsam/linear/linearExceptions.h>

namespace gtsam {

  /* ************************************************************************* */
  template<typename TERMS>
  JacobianFactor::JacobianFactor(const TERMS&terms, const Vector &b, const SharedDiagonal& model)
  {
    fillTerms(terms, b, model);
  }

  /* ************************************************************************* */
  template<typename KEYS>
  JacobianFactor::JacobianFactor(
    const KEYS& keys, const VerticalBlockMatrix& augmentedMatrix, const SharedDiagonal& model) :
  Base(keys), Ab_(augmentedMatrix)
  {
    // Check noise model dimension
    if(model && (DenseIndex)model->dim() != augmentedMatrix.rows())
      throw InvalidNoiseModel(augmentedMatrix.rows(), model->dim());

    // Check number of variables
    if((DenseIndex)Base::keys_.size() != augmentedMatrix.nBlocks() - 1)
      throw std::invalid_argument(
      "Error in JacobianFactor constructor input.  Number of provided keys plus\n"
      "one for the RHS vector must equal the number of provided matrix blocks.");

    // Check RHS dimension
    if(augmentedMatrix(augmentedMatrix.nBlocks() - 1).cols() != 1)
      throw std::invalid_argument(
      "Error in JacobianFactor constructor input.  The last provided matrix block\n"
      "must be the RHS vector, but the last provided block had more than one column.");

    // Take noise model
    model_ = model;
  }

  /* ************************************************************************* */
  template<typename TERMS>
  void JacobianFactor::fillTerms(const TERMS& terms, const Vector& b, const SharedDiagonal& noiseModel)
  {
    // Check noise model dimension
    if(noiseModel && (DenseIndex)noiseModel->dim() != b.size())
      throw InvalidNoiseModel(b.size(), noiseModel->dim());

    // Resize base class key vector
    Base::keys_.resize(terms.size());

    // Get dimensions of matrices
    std::vector<size_t> dimensions;
    dimensions.reserve(terms.size());
    for(typename TERMS::const_iterator it = terms.begin(); it != terms.end(); ++it) {
      const std::pair<Key, Matrix>& term = *it;
      const Matrix& Ai = term.second;
      dimensions.push_back(Ai.cols());
    }

    // Construct block matrix
    Ab_ = VerticalBlockMatrix(dimensions, b.size(), true);

    // Check and add terms
    DenseIndex i = 0; // For block index
    for(typename TERMS::const_iterator it = terms.begin(); it != terms.end(); ++it) {
      const std::pair<Key, Matrix>& term = *it;
      Key key = term.first;
      const Matrix& Ai = term.second;

      // Check block rows
      if(Ai.rows() != Ab_.rows())
        throw InvalidMatrixBlock(Ab_.rows(), Ai.rows());

      // Assign key and matrix
      Base::keys_[i] = key;
      Ab_(i) = Ai;

      // Increment block index
      ++ i;
    }

    // Assign RHS vector
    getb() = b;

    // Assign noise model
    model_ = noiseModel;
  }

} // gtsam


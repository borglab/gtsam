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
    keys_.resize(terms.size());

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
      keys_[i] = key;
      Ab_(i) = Ai;

      // Increment block index
      ++ i;
    }

    // Assign RHS vector
    getb() = b;


    // Assign noise model
    model_ = noiseModel;
  } //END JF::fillTerms()

//======================== MHJacobianFactor::MHJacobianFactor(terms, b_arr) ========================

  template<typename TERMS>
  MHJacobianFactor::MHJacobianFactor(const TERMS& mh_terms, const std::vector<Vector> &b_arr, HypoLayer* resulting_layer, const SharedDiagonal& model) { //mhsiao: NO model input when called in mhLinearize()

    //TODO: mhsiao: should allow MH-models as well
    fillTerms(mh_terms, b_arr, model);

    resulting_layer_ = resulting_layer;

  }
//======================== END MHJacobianFactor::MHJacobianFactor(terms, b_arr) ========================
//========================== MHJacobianFactor::fillTerms(terms, b_arr) ============================
  template<typename TERMS>
  void MHJacobianFactor::fillTerms(const TERMS& mh_terms, const std::vector<Vector>& b_arr, const SharedDiagonal& noiseModel)
  {

    //[MH-A]: hypo_list_ NOT being setup yet 
    //mhsiao: efficiency wasted by duplicating all terms[j] and all noiseModels
    for (size_t i = 0; i < b_arr.size(); ++i) { //i: hypo
      // Create terms...
      std::vector<std::pair<Key, Matrix> > terms(mh_terms.size());

      for (size_t j = 0; j < terms.size(); ++j) { //j: key
        terms[j].first = mh_terms[j].first;

        terms[j].second.swap((mh_terms[j].second)[i]);

      }

      jacobian_list_.push_back(JacobianFactor(terms, b_arr[i], noiseModel)); //redundant in setting up Keys in terms...
    }
    
    //[MH-A]: setup Keys of this Factor
    keys_.resize(mh_terms.size());
    for (size_t j = 0; j < mh_terms.size(); ++j) {
      keys_[j] = mh_terms[j].first;
    }
  }
//========================== END MHJacobianFactor::fillTerms(terms, b_arr) ============================

} // gtsam


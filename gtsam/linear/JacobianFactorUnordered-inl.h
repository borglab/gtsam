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
#include <boost/range/adaptor/map.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/range/join.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/foreach.hpp>

namespace gtsam {

  /* ************************************************************************* */
  template<typename TERMS>
  JacobianFactorUnordered::JacobianFactorUnordered(const TERMS&terms, const Vector &b, const SharedDiagonal& model)
  {
    fillTerms(terms, b, model);
  }

  /* ************************************************************************* */
  template<typename KEYS>
  JacobianFactorUnordered::JacobianFactorUnordered(
    const KEYS& keys, const VerticalBlockMatrix& augmentedMatrix, const SharedDiagonal& model) :
  Base(keys)
  {
    // Check noise model dimension
    if(model && model->dim() != augmentedMatrix.rows())
      throw InvalidNoiseModel(augmentedMatrix.rows(), model->dim());

    // Check number of variables
    if(Base::keys_.size() != augmentedMatrix.nBlocks() - 1)
      throw std::invalid_argument(
      "Error in JacobianFactor constructor input.  Number of provided keys plus\n"
      "one for the RHS vector must equal the number of provided matrix blocks.");

    // Check RHS dimension
    if(augmentedMatrix(augmentedMatrix.nBlocks() - 1).cols() != 1)
      throw std::invalid_argument(
      "Error in JacobianFactor constructor input.  The last provided matrix block\n"
      "must be the RHS vector, but the last provided block had more than one column.");

    // Allocate and copy matrix - only takes the active view of the provided augmented matrix
    Ab_ = VerticalBlockMatrix::LikeActiveViewOf(augmentedMatrix);
    Ab_.full() = augmentedMatrix.full();

    // Take noise model
    model_ = model;
  }

  /* ************************************************************************* */
  namespace {
    const Matrix& _getPairSecond(const std::pair<Key,Matrix>& p) {
      return p.second;
    }
  }

  /* ************************************************************************* */
  template<typename TERMS>
  void JacobianFactorUnordered::fillTerms(const TERMS& terms, const Vector& b, const SharedDiagonal& noiseModel)
  {
    // Check noise model dimension
    if(noiseModel && noiseModel->dim() != b.size())
      throw InvalidNoiseModel(b.size(), noiseModel->dim());

    // Resize base class key vector
    Base::keys_.resize(terms.size());

    // Gather dimensions - uses boost range adaptors to take terms, extract .second which are the
    // matrices, then extract the number of columns e.g. dimensions in each matrix.  Then joins with
    // a single '1' to add a dimension for the b vector.
    using boost::adaptors::map_values;
    using boost::adaptors::transformed;
    using boost::join;
    Ab_ = VerticalBlockMatrix(join(
      terms
      | transformed(&_getPairSecond)
      | transformed(boost::mem_fn(&Matrix::cols)),
      boost::assign::cref_list_of<1,DenseIndex>(1)), b.size());

    // Check and add terms
    typedef std::pair<Key, Matrix> Term;
    DenseIndex i = 0; // For block index
    BOOST_FOREACH(const Term& term, terms) {
      // Check block rows
      if(term.second.rows() != b.size())
        throw InvalidMatrixBlock(b.size(), term.second.rows());
      // Assign key and matrix
      Base::keys_[i] = term.first;
      Ab_(i) = term.second;
      // Increment block index
      ++ i;
    }

    // Assign RHS vector
    getb() = b;

    // Assign noise model
    model_ = noiseModel;
  }

} // gtsam


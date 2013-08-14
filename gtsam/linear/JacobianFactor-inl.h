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
#include <boost/assign/list_of.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/range/join.hpp>
#include <boost/range/algorithm/for_each.hpp>
#include <boost/foreach.hpp>

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
  namespace {
    DenseIndex _getColsJF(const std::pair<Key,Matrix>& p) {
      return p.second.cols();
    }

    // Helper to fill terms from any Eigen matrix type, e.g. could be a matrix expression.  Also
    // handles the cases where the matrix part of the term pair is or isn't a reference, and also
    // assignment from boost::cref_list_of, where the term ends up wrapped in a assign_reference<T>
    // that is implicitly convertible to T&.  This was introduced to work around a problem where
    // BOOST_FOREACH over terms did not work on GCC.
    struct _fillTerm {
      std::vector<Key>& keys;
      VerticalBlockMatrix& Ab;
      DenseIndex& i;
      _fillTerm(std::vector<Key>& keys, VerticalBlockMatrix& Ab, DenseIndex& i)
        : keys(keys), Ab(Ab), i(i) {}

      template<class MATRIX>
      void operator()(const std::pair<Key, MATRIX>& term) const
      {
        // Check block rows
        if(term.second.rows() != Ab.rows())
          throw InvalidMatrixBlock(Ab.rows(), term.second.rows());
        // Assign key and matrix
        keys[i] = term.first;
        Ab(i) = term.second;
        // Increment block index
        ++ i;
      }

      template<class MATRIX>
      void operator()(const std::pair<int, MATRIX>& term) const
      {
        operator()(std::pair<Key, const MATRIX&>(term));
      }

      template<class T>
      void operator()(boost::assign_detail::assign_reference<T> assignReference) const
      {
        operator()(assignReference.get_ref());
      }
    };
  }

  /* ************************************************************************* */
  template<typename TERMS>
  void JacobianFactor::fillTerms(const TERMS& terms, const Vector& b, const SharedDiagonal& noiseModel)
  {
    using boost::adaptors::transformed;
    namespace br = boost::range;

    // Check noise model dimension
    if(noiseModel && (DenseIndex)noiseModel->dim() != b.size())
      throw InvalidNoiseModel(b.size(), noiseModel->dim());

    // Resize base class key vector
    Base::keys_.resize(terms.size());

    // Gather dimensions - uses boost range adaptors to take terms, extract .second which are the
    // matrices, then extract the number of columns e.g. dimensions in each matrix.  Then joins with
    // a single '1' to add a dimension for the b vector.
    {
      Ab_ = VerticalBlockMatrix(br::join(terms | transformed(&_getColsJF), ListOfOne((DenseIndex)1)), b.size());
    }

    // Check and add terms
    DenseIndex i = 0; // For block index
    br::for_each(terms, _fillTerm(Base::keys_, Ab_, i));

    // Assign RHS vector
    getb() = b;

    // Assign noise model
    model_ = noiseModel;
  }

} // gtsam


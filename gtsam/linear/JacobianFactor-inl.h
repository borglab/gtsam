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

#include <array>

#if defined(__GNUC__) && !defined(__clang__) && __GNUC__ >= 13
#pragma GCC diagnostic warning "-Wstringop-overread"
#endif

namespace gtsam {

  /* ************************************************************************* */
  template <typename TERMS>
  JacobianFactor::JacobianFactor(const TERMS& terms, const Vector& b,
                                const SharedDiagonal& model) {
    fillTerms(terms, b, model);
  }

  /* ************************************************************************* */
  template <int M, int N1, typename>
  JacobianFactor::JacobianFactor(Key i1, const Eigen::Matrix<double, M, N1>& A1,
                                const Eigen::Matrix<double, M, 1>& b,
                                const SharedDiagonal& model)
      : Base(std::array<Key, 1>{{i1}}) {
    const DenseIndex rows = static_cast<DenseIndex>(b.rows());
    if (model && (DenseIndex)model->dim() != rows)
      throw InvalidNoiseModel(rows, model->dim());

    const std::array<size_t, 1> dims = {
        static_cast<size_t>(A1.cols())};
    Ab_ = VerticalBlockMatrix(dims, rows, true);
    Ab_(0) = A1;
    getb() = b;
    model_ = model;
  }

  /* ************************************************************************* */
  template <int M, int N1, int N2, typename>
  JacobianFactor::JacobianFactor(Key i1, const Eigen::Matrix<double, M, N1>& A1,
                                Key i2, const Eigen::Matrix<double, M, N2>& A2,
                                const Eigen::Matrix<double, M, 1>& b,
                                const SharedDiagonal& model)
      : Base(std::array<Key, 2>{{i1, i2}}) {
    const DenseIndex rows = static_cast<DenseIndex>(b.rows());
    if (model && (DenseIndex)model->dim() != rows)
      throw InvalidNoiseModel(rows, model->dim());

    const std::array<size_t, 2> dims = {
        static_cast<size_t>(A1.cols()),
        static_cast<size_t>(A2.cols())};
    Ab_ = VerticalBlockMatrix(dims, rows, true);
    Ab_(0) = A1;
    Ab_(1) = A2;
    getb() = b;
    model_ = model;
  }

  /* ************************************************************************* */
  template <int M, int N1, int N2, int N3, typename>
  JacobianFactor::JacobianFactor(Key i1, const Eigen::Matrix<double, M, N1>& A1,
                                Key i2, const Eigen::Matrix<double, M, N2>& A2,
                                Key i3, const Eigen::Matrix<double, M, N3>& A3,
                                const Eigen::Matrix<double, M, 1>& b,
                                const SharedDiagonal& model)
      : Base(std::array<Key, 3>{{i1, i2, i3}}) {
    const DenseIndex rows = static_cast<DenseIndex>(b.rows());
    if (model && (DenseIndex)model->dim() != rows)
      throw InvalidNoiseModel(rows, model->dim());

    const std::array<size_t, 3> dims = {
        static_cast<size_t>(A1.cols()),
        static_cast<size_t>(A2.cols()),
        static_cast<size_t>(A3.cols())};
    Ab_ = VerticalBlockMatrix(dims, rows, true);
    Ab_(0) = A1;
    Ab_(1) = A2;
    Ab_(2) = A3;
    getb() = b;
    model_ = model;
  }

  /* ************************************************************************* */
  template <typename KEYS>
  JacobianFactor::JacobianFactor(const KEYS& keys,
                                 const VerticalBlockMatrix& augmentedMatrix,
                                 const SharedDiagonal& model)
      : Base(keys), Ab_(augmentedMatrix), model_(model) {
    checkAb(model, augmentedMatrix);
  }

  /* ************************************************************************* */
  template <typename KEYS>
  JacobianFactor::JacobianFactor(const KEYS& keys,
                                 VerticalBlockMatrix&& augmentedMatrix,
                                 const SharedDiagonal& model)
      : Base(keys), Ab_(std::move(augmentedMatrix)), model_(model) {
    checkAb(model, Ab_);
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

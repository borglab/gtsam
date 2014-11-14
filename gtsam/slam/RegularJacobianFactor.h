/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    RegularJacobianFactor.h
 * @brief   JacobianFactor class with fixed sized blcoks
 * @author  Sungtae An
 * @date    Nov 11, 2014
 */

#pragma once

#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/VectorValues.h>
#include <boost/foreach.hpp>
#include <vector>

namespace gtsam {

template<size_t D>
class RegularJacobianFactor: public JacobianFactor {

public:

  /** Construct an n-ary factor
   * @tparam TERMS A container whose value type is std::pair<Key, Matrix>, specifying the
   *         collection of keys and matrices making up the factor. */
  template<typename TERMS>
  RegularJacobianFactor(const TERMS& terms, const Vector& b,
      const SharedDiagonal& model = SharedDiagonal()) :
      JacobianFactor(terms, b, model) {
  }

  /** Constructor with arbitrary number keys, and where the augmented matrix is given all together
   *  instead of in block terms.  Note that only the active view of the provided augmented matrix
   *  is used, and that the matrix data is copied into a newly-allocated matrix in the constructed
   *  factor. */
  template<typename KEYS>
  RegularJacobianFactor(const KEYS& keys,
      const VerticalBlockMatrix& augmentedMatrix,
      const SharedDiagonal& sigmas = SharedDiagonal()) :
      JacobianFactor(keys, augmentedMatrix, sigmas) {
  }

  /// y += alpha * A'*A*x
  void multiplyHessianAdd(double alpha, const VectorValues& x,
      VectorValues& y) const {
    JacobianFactor::multiplyHessianAdd(alpha, x, y);
  }

  // Note: this is not assuming a fixed dimension for the variables,
  // but requires the vector accumulatedDims to tell the dimension of
  // each variable: e.g.: x0 has dim 3, x2 has dim 6, x3 has dim 2,
  // then accumulatedDims is [0 3 9 11 13]
  // NOTE: size of accumulatedDims is size of keys + 1!!
  void multiplyHessianAdd(double alpha, const double* x, double* y,
      const std::vector<size_t>& accumulatedDims) const {

    // Use eigen magic to access raw memory
    typedef Eigen::Matrix<double, Eigen::Dynamic, 1> DVector;
    typedef Eigen::Map<DVector> DMap;
    typedef Eigen::Map<const DVector> ConstDMap;

    if (empty())
      return;
    Vector Ax = zero(Ab_.rows());

    // Just iterate over all A matrices and multiply in correct config part (looping over keys)
    // E.g.: Jacobian A = [A0 A1 A2] multiplies x = [x0 x1 x2]'
    // Hence: Ax = A0 x0 + A1 x1 + A2 x2 (hence we loop over the keys and accumulate)
    for (size_t pos = 0; pos < size(); ++pos)
    {
      Ax += Ab_(pos)
          * ConstDMap(x + accumulatedDims[keys_[pos]], accumulatedDims[keys_[pos] + 1] - accumulatedDims[keys_[pos]]);
    }
    // Deal with noise properly, need to Double* whiten as we are dividing by variance
    if (model_) {
      model_->whitenInPlace(Ax);
      model_->whitenInPlace(Ax);
    }

    // multiply with alpha
    Ax *= alpha;

    // Again iterate over all A matrices and insert Ai^T into y
    for (size_t pos = 0; pos < size(); ++pos){
      DMap(y + accumulatedDims[keys_[pos]], accumulatedDims[keys_[pos] + 1] - accumulatedDims[keys_[pos]]) += Ab_(
          pos).transpose() * Ax;
    }
  }

  void multiplyHessianAdd(double alpha, const double* x, double* y) const {

    // Use eigen magic to access raw memory
    typedef Eigen::Matrix<double, D, 1> DVector;
    typedef Eigen::Map<DVector> DMap;
    typedef Eigen::Map<const DVector> ConstDMap;

    if (empty()) return;
    Vector Ax = zero(Ab_.rows());

    // Just iterate over all A matrices and multiply in correct config part
    for(size_t pos=0; pos<size(); ++pos)
      Ax += Ab_(pos) * ConstDMap(x + D * keys_[pos]);

    // Deal with noise properly, need to Double* whiten as we are dividing by variance
    if  (model_) { model_->whitenInPlace(Ax); model_->whitenInPlace(Ax); }

    // multiply with alpha
    Ax *= alpha;

    // Again iterate over all A matrices and insert Ai^e into y
    for(size_t pos=0; pos<size(); ++pos)
      DMap(y + D * keys_[pos]) += Ab_(pos).transpose() * Ax;
  }

  /// Return the diagonal of the Hessian for this factor
  VectorValues hessianDiagonal() const {
    return JacobianFactor::hessianDiagonal();
  }

  /// Raw memory access version of hessianDiagonal
  /* ************************************************************************* */
  // TODO: currently assumes all variables of the same size D (templated) and keys arranged from 0 to n
  void hessianDiagonal(double* d) const {
    // Use eigen magic to access raw memory
    typedef Eigen::Matrix<double, D, 1> DVector;
    typedef Eigen::Map<DVector> DMap;

    // Loop over all variables in the factor
    for (DenseIndex j = 0; j < (DenseIndex) size(); ++j) {
      // Get the diagonal block, and insert its diagonal
      DVector dj;
      for (size_t k = 0; k < D; ++k){
        if (model_){
          Vector column_k = Ab_(j).col(k);
          column_k = model_->whiten(column_k);
          dj(k) = dot(column_k, column_k);
        }else{
          dj(k) = Ab_(j).col(k).squaredNorm();
        }
      }
      DMap(d + D * j) += dj;
    }
  }

  VectorValues gradientAtZero() const {
    return JacobianFactor::gradientAtZero();
  }

  void gradientAtZero(double* d) const {
    //throw std::runtime_error("gradientAtZero not implemented for Jacobian factor");
  }

};

}


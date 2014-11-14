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

  void multiplyHessianAdd(double alpha, const double* x, double* y,
      std::vector<size_t> keys) const {

    // Use eigen magic to access raw memory
    typedef Eigen::Matrix<double, Eigen::Dynamic, 1> DVector;
    typedef Eigen::Map<DVector> DMap;
    typedef Eigen::Map<const DVector> ConstDMap;

    if (empty())
      return;
    Vector Ax = zero(Ab_.rows());

    // Just iterate over all A matrices and multiply in correct config part
    for (size_t pos = 0; pos < size(); ++pos)
    {
      std::cout << "pos: " << pos << " keys_[pos]: " << keys_[pos] << " keys[keys_[pos]]: " << keys[keys_[pos]] << std::endl;
      Ax += Ab_(pos)
          * ConstDMap(x + keys[keys_[pos]],
              keys[keys_[pos] + 1] - keys[keys_[pos]]);
    }
    // Deal with noise properly, need to Double* whiten as we are dividing by variance
    if (model_) {
      model_->whitenInPlace(Ax);
      model_->whitenInPlace(Ax);
    }

    // multiply with alpha
    Ax *= alpha;

    // Again iterate over all A matrices and insert Ai^e into y
    for (size_t pos = 0; pos < size(); ++pos)
      DMap(y + keys[keys_[pos]], keys[keys_[pos] + 1] - keys[keys_[pos]]) += Ab_(
          pos).transpose() * Ax;
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
  // TODO: currently assumes all variables of the same size 9 and keys arranged from 0 to n
  void hessianDiagonal(double* d) const {
    // Use eigen magic to access raw memory
    //typedef Eigen::Matrix<double, 9, 1> DVector;
    typedef Eigen::Matrix<double, D, 1> DVector;
    typedef Eigen::Map<DVector> DMap;

    // Loop over all variables in the factor
    for (DenseIndex j = 0; j < (DenseIndex) size(); ++j) {
      // Get the diagonal block, and insert its diagonal
      DVector dj;
      //for (size_t k = 0; k < 9; ++k)
      for (size_t k = 0; k < D; ++k)
        dj(k) = Ab_(j).col(k).squaredNorm();

      //DMap(d + 9 * j) += dj;
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


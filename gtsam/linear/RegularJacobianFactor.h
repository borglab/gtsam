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

namespace gtsam {

/**
 * JacobianFactor with constant sized blocks
 * Provides raw memory access versions of linear operator.
 * Is base class for JacobianQFactor, JacobianFactorQR, and JacobianFactorSVD
 */
template<size_t D>
class RegularJacobianFactor: public JacobianFactor {

private:

  // Use eigen magic to access raw memory
  typedef Eigen::Matrix<double, D, 1> DVector;
  typedef Eigen::Map<DVector> DMap;
  typedef Eigen::Map<const DVector> ConstDMap;

public:

  /// Default constructor
  RegularJacobianFactor() {}

  /** Construct an n-ary factor
   * @tparam TERMS A container whose value type is std::pair<Key, Matrix>, specifying the
   *         collection of keys and matrices making up the factor.
   * TODO Verify terms are regular
   */
  template<typename TERMS>
  RegularJacobianFactor(const TERMS& terms, const Vector& b,
      const SharedDiagonal& model = SharedDiagonal()) :
      JacobianFactor(terms, b, model) {
  }

  /** Constructor with arbitrary number keys, and where the augmented matrix is given all together
   *  instead of in block terms.  Note that only the active view of the provided augmented matrix
   *  is used, and that the matrix data is copied into a newly-allocated matrix in the constructed
   *  factor.
   *  TODO Verify complies to regular
   */
  template<typename KEYS>
  RegularJacobianFactor(const KEYS& keys,
      const VerticalBlockMatrix& augmentedMatrix, const SharedDiagonal& sigmas =
          SharedDiagonal()) :
      JacobianFactor(keys, augmentedMatrix, sigmas) {
  }

  using JacobianFactor::multiplyHessianAdd;

  /** y += alpha * A'*A*x */
  virtual void multiplyHessianAdd(double alpha, const VectorValues& x,
      VectorValues& y) const {
    JacobianFactor::multiplyHessianAdd(alpha, x, y);
  }

  /**
   * @brief double* Hessian-vector multiply, i.e. y += A'*(A*x)
   * RAW memory access! Assumes keys start at 0 and go to M-1, and x and and y are laid out that way
   */
  void multiplyHessianAdd(double alpha, const double* x, double* y) const {
    if (empty())
      return;
    Vector Ax = Vector::Zero(Ab_.rows());

    // Just iterate over all A matrices and multiply in correct config part
    for (size_t pos = 0; pos < size(); ++pos)
      Ax += Ab_(pos) * ConstDMap(x + D * keys_[pos]);

    // Deal with noise properly, need to Double* whiten as we are dividing by variance
    if (model_) {
      model_->whitenInPlace(Ax);
      model_->whitenInPlace(Ax);
    }

    // multiply with alpha
    Ax *= alpha;

    // Again iterate over all A matrices and insert Ai^e into y
    for (size_t pos = 0; pos < size(); ++pos)
      DMap(y + D * keys_[pos]) += Ab_(pos).transpose() * Ax;
  }

  /// Using the base method
  using GaussianFactor::hessianDiagonal;

  /// Raw memory access version of hessianDiagonal
  void hessianDiagonal(double* d) const {
    // Loop over all variables in the factor
    for (DenseIndex j = 0; j < (DenseIndex) size(); ++j) {
      // Get the diagonal block, and insert its diagonal
      DVector dj;
      for (size_t k = 0; k < D; ++k) {
        if (model_) {
          Vector column_k = Ab_(j).col(k);
          column_k = model_->whiten(column_k);
          dj(k) = dot(column_k, column_k);
        } else {
          dj(k) = Ab_(j).col(k).squaredNorm();
        }
      }
      DMap(d + D * j) += dj;
    }
  }

  /// Expose base class gradientAtZero
  virtual VectorValues gradientAtZero() const {
    return JacobianFactor::gradientAtZero();
  }

  /// Raw memory access version of gradientAtZero
  void gradientAtZero(double* d) const {

    // Get vector b not weighted
    Vector b = getb();

    // Whitening b
    if (model_) {
      b = model_->whiten(b);
      b = model_->whiten(b);
    }

    // Just iterate over all A matrices
    for (DenseIndex j = 0; j < (DenseIndex) size(); ++j) {
      DVector dj;
      // gradient -= A'*b/sigma^2
      // Computing with each column
      for (size_t k = 0; k < D; ++k) {
        Vector column_k = Ab_(j).col(k);
        dj(k) = -1.0 * dot(b, column_k);
      }
      DMap(d + D * j) += dj;
    }
  }

  /**
   * @brief double* Transpose Matrix-vector multiply, i.e. x += A'*e
   * RAW memory access! Assumes keys start at 0 and go to M-1, and y is laid out that way
   */
  void transposeMultiplyAdd(double alpha, const Vector& e, double* x) const {
    Vector E = alpha * (model_ ? model_->whiten(e) : e);
    // Just iterate over all A matrices and insert Ai^e into y
    for (size_t pos = 0; pos < size(); ++pos)
      DMap(x + D * keys_[pos]) += Ab_(pos).transpose() * E;
  }

  /**
   * @brief double* Matrix-vector multiply, i.e. y = A*x
   * RAW memory access! Assumes keys start at 0 and go to M-1, and x is laid out that way
   */
  Vector operator*(const double* x) const {
    Vector Ax = Vector::Zero(Ab_.rows());
    if (empty())
      return Ax;

    // Just iterate over all A matrices and multiply in correct config part
    for (size_t pos = 0; pos < size(); ++pos)
      Ax += Ab_(pos) * ConstDMap(x + D * keys_[pos]);

    return model_ ? model_->whiten(Ax) : Ax;
  }

};
// end class RegularJacobianFactor

}// end namespace gtsam

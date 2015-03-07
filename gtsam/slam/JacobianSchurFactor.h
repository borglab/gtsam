/*
 * @file  JacobianSchurFactor.h
 * @brief Jacobianfactor that combines and eliminates points
 * @date  Oct 27, 2013
 * @uthor Frank Dellaert
 */

#pragma once

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <boost/foreach.hpp>

namespace gtsam {
/**
 * JacobianFactor for Schur complement that uses Q noise model
 */
template<size_t D>
class JacobianSchurFactor: public JacobianFactor {

public:

  typedef Eigen::Matrix<double, 2, D> Matrix2D;
  typedef std::pair<Key, Matrix2D> KeyMatrix2D;

  // Use eigen magic to access raw memory
  typedef Eigen::Matrix<double, D, 1> DVector;
  typedef Eigen::Map<DVector> DMap;
  typedef Eigen::Map<const DVector> ConstDMap;

  /**
   * @brief double* Matrix-vector multiply, i.e. y = A*x
   * RAW memory access! Assumes keys start at 0 and go to M-1, and x is laid out that way
   */
  Vector operator*(const double* x) const {
    Vector Ax = zero(Ab_.rows());
    if (empty()) return Ax;

    // Just iterate over all A matrices and multiply in correct config part
    for(size_t pos=0; pos<size(); ++pos)
      Ax += Ab_(pos) * ConstDMap(x + D * keys_[pos]);

    return model_ ? model_->whiten(Ax) : Ax;
  }

  /**
   * @brief double* Transpose Matrix-vector multiply, i.e. x += A'*e
   * RAW memory access! Assumes keys start at 0 and go to M-1, and y is laid out that way
   */
  void transposeMultiplyAdd(double alpha, const Vector& e, double* x) const
  {
    Vector E = alpha * (model_ ? model_->whiten(e) : e);
    // Just iterate over all A matrices and insert Ai^e into y
    for(size_t pos=0; pos<size(); ++pos)
      DMap(x + D * keys_[pos]) += Ab_(pos).transpose() * E;
  }

  /** y += alpha * A'*A*x */
  void multiplyHessianAdd(double alpha, const VectorValues& x, VectorValues& y) const {
    JacobianFactor::multiplyHessianAdd(alpha,x,y);
  }

  /**
   * @brief double* Hessian-vector multiply, i.e. y += A'*(A*x)
   * RAW memory access! Assumes keys start at 0 and go to M-1, and x and and y are laid out that way
   */
  void multiplyHessianAdd(double alpha, const double* x, double* y) const {
//    Vector Ax = (*this)*x;
//    this->transposeMultiplyAdd(alpha,Ax,y);
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

}; // class

} // gtsam

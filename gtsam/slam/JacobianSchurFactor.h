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

  /** y += alpha * A'*A*x */
  void multiplyHessianAdd(double alpha, const VectorValues& x, VectorValues& y) const {
    JacobianFactor::multiplyHessianAdd(alpha,x,y);
  }

}; // class

} // gtsam

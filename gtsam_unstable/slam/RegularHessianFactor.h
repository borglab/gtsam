/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    RegularHessianFactor.h
 * @brief   HessianFactor class with constant sized blcoks
 * @author  Richard Roberts
 * @date    Dec 8, 2010
 */

#pragma once

#include <gtsam/linear/HessianFactor.h>
#include <boost/foreach.hpp>
#include <vector>

namespace gtsam {

template<size_t D>
class RegularHessianFactor: public HessianFactor {

private:

  typedef Eigen::Matrix<double, D, D> MatrixDD; // camera hessian block
  typedef Eigen::Matrix<double, D, 1> VectorD;

public:

  /** Construct an n-way factor.  Gs contains the upper-triangle blocks of the
   * quadratic term (the Hessian matrix) provided in row-order, gs the pieces
   * of the linear vector term, and f the constant term.
   */
  RegularHessianFactor(const std::vector<Key>& js,
      const std::vector<Matrix>& Gs, const std::vector<Vector>& gs, double f) :
      HessianFactor(js, Gs, gs, f) {
  }

  /** Constructor with an arbitrary number of keys and with the augmented information matrix
   *   specified as a block matrix. */
  template<typename KEYS>
  RegularHessianFactor(const KEYS& keys,
      const SymmetricBlockMatrix& augmentedInformation) :
      HessianFactor(keys, augmentedInformation) {
  }

  /** y += alpha * A'*A*x */
  void multiplyHessianAdd(double alpha, const VectorValues& x,
      VectorValues& y) const {
    HessianFactor::multiplyHessianAdd(alpha, x, y);
  }

  // Scratch space for multiplyHessianAdd
  typedef Eigen::Matrix<double, D, 1> DVector;
  mutable std::vector<DVector> y;

  void multiplyHessianAdd(double alpha, const double* x,
      double* yvalues) const {
    // Create a vector of temporary y values, corresponding to rows i
    y.resize(size());
    BOOST_FOREACH(DVector & yi, y)
      yi.setZero();

    typedef Eigen::Map<DVector> DMap;
    typedef Eigen::Map<const DVector> ConstDMap;

    // Accessing the VectorValues one by one is expensive
    // So we will loop over columns to access x only once per column
    // And fill the above temporary y values, to be added into yvalues after
    DVector xj(D);
    for (DenseIndex j = 0; j < (DenseIndex) size(); ++j) {
      Key key = keys_[j];
      const double* xj = x + key * D;
      DenseIndex i = 0;
      for (; i < j; ++i)
        y[i] += info_(i, j).knownOffDiagonal() * ConstDMap(xj);
      // blocks on the diagonal are only half
      y[i] += info_(j, j).selfadjointView() * ConstDMap(xj);
      // for below diagonal, we take transpose block from upper triangular part
      for (i = j + 1; i < (DenseIndex) size(); ++i)
        y[i] += info_(i, j).knownOffDiagonal() * ConstDMap(xj);
    }

    // copy to yvalues
    for (DenseIndex i = 0; i < (DenseIndex) size(); ++i) {
      Key key = keys_[i];
      DMap(yvalues + key * D) += alpha * y[i];
    }
  }

};

}


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

};

}


/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file factorTesting.h
 * @date September 18, 2014
 * @author Frank Dellaert
 * @author Paul Furgale
 * @brief Evaluate derivatives of a nonlinear factor numerically
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/numericalDerivative.h>

namespace gtsam {

/**
 * Linearize a nonlinear factor using numerical differentiation
 * The benefit of this method is that it does not need to know what types are
 * involved to evaluate the factor. If all the machinery of gtsam is working
 * correctly, we should get the correct numerical derivatives out the other side.
 */
JacobianFactor linearizeNumerically(const NoiseModelFactor& factor,
    const Values& values, double delta = 1e-5) {

  // We will fill a vector of key/Jacobians pairs (a map would sort)
  std::vector<std::pair<Key, Matrix> > jacobians;

  // Get size
  Eigen::VectorXd e = factor.whitenedError(values);
  const size_t rows = e.size();

  // Loop over all variables
  VectorValues dX = values.zeroVectors();
  BOOST_FOREACH(Key key, factor) {
    // Compute central differences using the values struct.
    const size_t cols = dX.dim(key);
    Matrix J = Matrix::Zero(rows, cols);
    for (size_t col = 0; col < cols; ++col) {
      Eigen::VectorXd dx = Eigen::VectorXd::Zero(cols);
      dx[col] = delta;
      dX[key] = dx;
      Values eval_values = values.retract(dX);
      Eigen::VectorXd left = factor.whitenedError(eval_values);
      dx[col] = -delta;
      dX[key] = dx;
      eval_values = values.retract(dX);
      Eigen::VectorXd right = factor.whitenedError(eval_values);
      J.col(col) = (left - right) * (1.0 / (2.0 * delta));
    }
    jacobians.push_back(std::make_pair(key,J));
  }

  // Next step...return JacobianFactor
  return JacobianFactor(jacobians, -e);
}

} // namespace gtsam

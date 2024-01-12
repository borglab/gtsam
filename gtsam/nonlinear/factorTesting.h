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
#include <string>
#include <vector>

namespace gtsam {

/**
 * Linearize a nonlinear factor using numerical differentiation
 * The benefit of this method is that it does not need to know what types are
 * involved to evaluate the factor. If all the machinery of gtsam is working
 * correctly, we should get the correct numerical derivatives out the other side.
 * NOTE(frank): factors that have non vector-space measurements use between or LocalCoordinates
 * to evaluate the error, and their derivatives will only be correct for near-zero errors.
 * This is fixable but expensive, and does not matter in practice as most factors will sit near
 * zero errors anyway. However, it means that below will only be exact for the correct measurement.
 */
inline JacobianFactor linearizeNumerically(const NoiseModelFactor& factor,
                                           const Values& values,
                                           double delta = 1e-5) {
  // We will fill a vector of key/Jacobians pairs (a map would sort)
  std::vector<std::pair<Key, Matrix> > jacobians;

  // Get size
  const Vector e = factor.whitenedError(values);
  const size_t rows = e.size();

  // Loop over all variables
  const double one_over_2delta = 1.0 / (2.0 * delta);
  for (Key key : factor) {
    // Compute central differences using the values struct.
    VectorValues dX = values.zeroVectors();
    const size_t cols = dX.dim(key);
    Matrix J = Matrix::Zero(rows, cols);
    for (size_t col = 0; col < cols; ++col) {
      Vector dx = Vector::Zero(cols);
      dx(col) = delta;
      dX[key] = dx;
      Values eval_values = values.retract(dX);
      const Vector left = factor.whitenedError(eval_values);
      dx(col) = -delta;
      dX[key] = dx;
      eval_values = values.retract(dX);
      const Vector right = factor.whitenedError(eval_values);
      J.col(col) = (left - right) * one_over_2delta;
    }
    jacobians.emplace_back(key, J);
  }

  // Next step...return JacobianFactor
  return JacobianFactor(jacobians, -e);
}

namespace internal {
// CPPUnitLite-style test for linearization of a factor
inline bool testFactorJacobians(const std::string& name_,
                                const NoiseModelFactor& factor,
                                const gtsam::Values& values, double delta,
                                double tolerance) {
  // Create expected value by numerical differentiation
  JacobianFactor expected = linearizeNumerically(factor, values, delta);

  // Create actual value by linearize
  auto actual =
      std::dynamic_pointer_cast<JacobianFactor>(factor.linearize(values));
  if (!actual) return false;

  // Check cast result and then equality
  bool equal = assert_equal(expected, *actual, tolerance);

  // if not equal, test individual jacobians:
  if (!equal) {
    for (size_t i = 0; i < actual->size(); i++) {
      bool i_good =
          assert_equal((Matrix)(expected.getA(expected.begin() + i)),
                       (Matrix)(actual->getA(actual->begin() + i)), tolerance);
      if (!i_good) {
        std::cout << "Mismatch in Jacobian " << i + 1
                  << " (base 1), as shown above" << std::endl;
      }
    }
  }

  return equal;
}
}  // namespace internal

/// \brief Check the Jacobians produced by a factor against finite differences.
/// \param factor The factor to test.
/// \param values Values filled in for testing the Jacobians.
/// \param numerical_derivative_step The step to use when computing the numerical derivative Jacobians
/// \param tolerance The numerical tolerance to use when comparing Jacobians.
#define EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, numerical_derivative_step, tolerance) \
    { EXPECT(gtsam::internal::testFactorJacobians(name_, factor, values, numerical_derivative_step, tolerance)); }

}  // namespace gtsam

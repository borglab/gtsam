/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   JacobianFactorGraph.cpp
 * @date   Jun 6, 2012
 * @brief  Linear Algebra Operations for a JacobianFactorGraph
 * @author Yong Dian Jian
 */
#pragma once

#include <gtsam/linear/Errors.h>
#include <gtsam/inference/FactorGraph.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/VectorValues.h>

namespace gtsam {

  /** return A*x */
  Errors operator*(const FactorGraph<JacobianFactor>& fg, const VectorValues& x);

  /** In-place version e <- A*x that overwrites e. */
  void multiplyInPlace(const FactorGraph<JacobianFactor>& fg, const VectorValues& x, Errors& e);

  /** In-place version e <- A*x that takes an iterator. */
  void multiplyInPlace(const FactorGraph<JacobianFactor>& fg, const VectorValues& x, const Errors::iterator& e);

  /** x += alpha*A'*e */
  void transposeMultiplyAdd(const FactorGraph<JacobianFactor>& fg, double alpha, const Errors& e, VectorValues& x);

  /**
   * Compute the gradient of the energy function,
   * \f$ \nabla_{x=x_0} \left\Vert \Sigma^{-1} A x - b \right\Vert^2 \f$,
   * centered around \f$ x = x_0 \f$.
   * The gradient is \f$ A^T(Ax-b) \f$.
   * @param fg The Jacobian factor graph $(A,b)$
   * @param x0 The center about which to compute the gradient
   * @return The gradient as a VectorValues
   */
  VectorValues gradient(const FactorGraph<JacobianFactor>& fg, const VectorValues& x0);

  /**
   * Compute the gradient of the energy function,
   * \f$ \nabla_{x=0} \left\Vert \Sigma^{-1} A x - b \right\Vert^2 \f$,
   * centered around zero.
   * The gradient is \f$ A^T(Ax-b) \f$.
   * @param fg The Jacobian factor graph $(A,b)$
   * @param [output] g A VectorValues to store the gradient, which must be preallocated, see allocateVectorValues
   * @return The gradient as a VectorValues
   */
  void gradientAtZero(const FactorGraph<JacobianFactor>& fg, VectorValues& g);

  /* matrix-vector operations */
  void residual(const FactorGraph<JacobianFactor>& fg, const VectorValues &x, VectorValues &r);
  void multiply(const FactorGraph<JacobianFactor>& fg, const VectorValues &x, VectorValues &r);
  void transposeMultiply(const FactorGraph<JacobianFactor>& fg, const VectorValues &r, VectorValues &x);

}

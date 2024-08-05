/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    EqualityConstraint-inl.h
 * @brief   Equality constraints in constrained optimization.
 * @author  Yetong Zhang, Frank Dellaert
 * @date    Aug 3, 2024 */

#pragma once

#include <gtsam/constraint/NonlinearEqualityConstraint.h>

namespace gtsam {

/* ************************************************************************* */
template <typename T>
ExpressionEqualityConstraint<T>::ExpressionEqualityConstraint(const Expression<T>& expression,
                                                              const T& rhs,
                                                              const Vector& sigmas)
    : Base(constrainedNoise(sigmas), expression.keysAndDims().first),
      expression_(expression),
      rhs_(rhs),
      dims_(expression.keysAndDims().second) {}

/* ************************************************************************* */
template <typename T>
Vector ExpressionEqualityConstraint<T>::unwhitenedError(const Values& x,
                                                        OptionalMatrixVecType H) const {
  // Copy-paste from ExpressionFactor.
  if (H) {
    const T value = expression_.valueAndDerivatives(x, keys_, dims_, *H);
    return -traits<T>::Local(value, rhs_);
  } else {
    const T value = expression_.value(x);
    return -traits<T>::Local(value, rhs_);
  }
}

/* ************************************************************************* */
template <typename T>
NoiseModelFactor::shared_ptr ExpressionEqualityConstraint<T>::penaltyFactor(const double mu) const {
  return std::make_shared<ExpressionFactor<T>>(penaltyNoise(mu), rhs_, expression_);
}

}  // namespace gtsam

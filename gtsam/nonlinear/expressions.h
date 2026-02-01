/**
 * @file expressions.h
 * @brief Common expressions, both linear and non-linear
 * @date Nov 23, 2014
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/nonlinear/Expression.h>
#include <gtsam/base/Lie.h>

namespace gtsam {

// Generic between, assumes existence of traits<T>::Between
template <typename T>
Expression<T> between(const Expression<T>& t1, const Expression<T>& t2) {
  return Expression<T>(traits<T>::Between, t1, t2);
}

// Generic compose, assumes existence of traits<T>::Compose
template <typename T>
Expression<T> compose(const Expression<T>& t1, const Expression<T>& t2) {
  return Expression<T>(traits<T>::Compose, t1, t2);
}

// Some typedefs
typedef Expression<double> Double_;
typedef Expression<Vector1> Vector1_;
typedef Expression<Vector2> Vector2_;
typedef Expression<Vector3> Vector3_;
typedef Expression<Vector4> Vector4_;
typedef Expression<Vector5> Vector5_;
typedef Expression<Vector6> Vector6_;
typedef Expression<Vector7> Vector7_;
typedef Expression<Vector8> Vector8_;
typedef Expression<Vector9> Vector9_;

}  // \namespace gtsam

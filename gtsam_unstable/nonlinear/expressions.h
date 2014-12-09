/**
 * @file expressions.h
 * @brief Common expressions, both linear and non-linear
 * @date Nov 23, 2014
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/nonlinear/Expression.h>
#include <boost/bind.hpp>

namespace gtsam {

// Generics
template<typename T>
Expression<T> between(const Expression<T>& t1, const Expression<T>& t2) {
  return Expression<T>(t1, &T::between, t2);
}

typedef Expression<double> double_;
typedef Expression<Vector3> Vector3_;

} // \namespace gtsam


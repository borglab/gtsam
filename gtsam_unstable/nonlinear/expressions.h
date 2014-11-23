/**
 * @file expressions.h
 * @brief Common expressions, both linear and non-linear
 * @date Oct 1, 2014
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam_unstable/nonlinear/Expression.h>
#include <boost/bind.hpp>

namespace gtsam {

// Generics

template<class T>
Expression<T> between(const Expression<T>& t1, const Expression<T>& t2) {
  return Expression<T>(t1, &T::between, t2);
}

typedef Expression<double> double_;

} // \namespace gtsam


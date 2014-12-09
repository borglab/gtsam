/**
 * @file expressions.h
 * @brief Common expressions, both linear and non-linear
 * @date Nov 23, 2014
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/nonlinear/Expression.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <boost/bind.hpp>

namespace gtsam {

// 2D Geometry
  typedef Expression<Pose2> Pose2_;

  Pose2_ between(const Pose2_& x, const Pose2_& p) {
    Pose2(Pose2::*transform)(const Pose2& p,
      boost::optional<Matrix3&> H1,
      boost::optional<Matrix3&> H2) const = &Pose2::between;

    return Pose2_(x, transform, p);
  }

// Generics

template<typename T>
Expression<T> between(const Expression<T>& t1, const Expression<T>& t2) {
  return Expression<T>(t1, &T::between, t2);
}

typedef Expression<double> double_;
typedef Expression<Vector3> Vector3_;

} // \namespace gtsam


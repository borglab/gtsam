/**
 * @file gtsam/nonlinear/expressions.h
 * @brief Common expressions, both linear and non-linear
 * @date Nov 23, 2014
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/base/Lie.h>
#include <gtsam/nonlinear/Expression.h>

#include <type_traits>

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

namespace internal {

/** Adapt a traits exponential map to Expression's unary function signature. */
template <typename T>
T expmapValue(const typename traits<T>::TangentVector& tangent,
              typename MakeOptionalJacobian<
                  T, typename traits<T>::TangentVector>::type H = {}) {
  return traits<T>::Expmap(tangent, H);
}

}  // namespace internal

/** Apply an exponential-map increment to a Lie-group expression. */
template <typename T>
Expression<T> expmap(
    const Expression<T>& origin,
    const Expression<typename traits<T>::TangentVector>& tangent) {
  return compose(origin, Expression<T>(&internal::expmapValue<T>, tangent));
}

namespace internal {

/** Scale a fixed-size vector-like value and provide both Jacobians. */
template <typename T>
T scaleVector(double scale, const T& vector,
              typename MakeOptionalJacobian<T, double>::type Hscale,
              typename MakeOptionalJacobian<T, T>::type Hvector) {
  using Jacobian = typename MakeJacobian<T, T>::type;
  if (Hscale) *Hscale = vector;
  if (Hvector) *Hvector = scale * Jacobian::Identity();
  return scale * vector;
}

}  // namespace internal

/** Multiply a non-scalar expression by a scalar expression. */
template <typename T, typename std::enable_if<
                          !std::is_same<T, double>::value>::type* = nullptr>
Expression<T> operator*(const Expression<double>& scale,
                        const Expression<T>& vector) {
  return Expression<T>(internal::scaleVector<T>, scale, vector);
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

}  // namespace gtsam

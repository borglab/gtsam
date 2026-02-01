/**
 * @file expressions.h
 * @brief Common expressions for solving navigation problems
 * @date May, 2019
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/geometry/Rot3.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/slam/expressions.h>

namespace gtsam {

typedef Expression<NavState> NavState_;
typedef Expression<Velocity3> Velocity3_;

namespace internal {
// define getters that return a value rather than a reference
inline Rot3 attitude(const NavState& X, OptionalJacobian<3, 9> H) {
  return X.attitude(H);
}
inline Point3 position(const NavState& X, OptionalJacobian<3, 9> H) {
  return X.position(H);
}
inline Velocity3 velocity(const NavState& X, OptionalJacobian<3, 9> H) {
  return X.velocity(H);
}
}  // namespace internal

// overloads for getters
inline Rot3_ attitude(const NavState_& X) {
  return Rot3_(internal::attitude, X);
}
inline Point3_ position(const NavState_& X) {
  return Point3_(internal::position, X);
}
inline Velocity3_ velocity(const NavState_& X) {
  return Velocity3_(internal::velocity, X);
}

}  // namespace gtsam

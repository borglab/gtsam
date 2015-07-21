/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    NavState.h
 * @brief   Navigation state composing of attitude, position, and velocity
 * @author  Frank Dellaert
 * @date    July 2015
 **/

#include <gtsam/navigation/NavState.h>

namespace gtsam {

#define TIE(R,t,v,x) const Rot3& R = (x).R_;const Point3& t = (x).t_;const Velocity3& v = (x).v_;

Matrix7 NavState::matrix() const {
  Matrix3 R = this->R();
  Matrix7 T;
  T << R, Z_3x3, t(), Z_3x3, R, v(), Vector6::Zero().transpose(), 1.0;
  return T;
}

void NavState::print(const std::string& s) const {
  attitude().print(s + ".R");
  position().print(s + ".p");
  gtsam::print((Vector) v_, s + ".v");
}

bool NavState::equals(const NavState& other, double tol) const {
  return R_.equals(other.R_, tol) && t_.equals(other.t_, tol)
      && equal_with_abs_tol(v_, other.v_, tol);
}

NavState NavState::inverse() const {
  TIE(nRb, n_t, n_v, *this);
  const Rot3 bRn = nRb.inverse();
  return NavState(bRn, -(bRn * n_t), -(bRn * n_v));
}

NavState NavState::operator*(const NavState& bTc) const {
  TIE(nRb, n_t, n_v, *this);
  TIE(bRc, b_t, b_v, bTc);
  return NavState(nRb * bRc, nRb * b_t + n_t, nRb * b_v + n_v);
}

NavState::PositionAndVelocity //
NavState::operator*(const PositionAndVelocity& b_tv) const {
  TIE(nRb, n_t, n_v, *this);
  const Point3& b_t = b_tv.first;
  const Velocity3& b_v = b_tv.second;
  return PositionAndVelocity(nRb * b_t + n_t, nRb * b_v + n_v);
}

Point3 NavState::operator*(const Point3& b_t) const {
  return Point3(R_ * b_t + t_);
}

Velocity3 NavState::operator*(const Velocity3& b_v) const {
  return Velocity3(R_ * b_v + v_);
}

NavState NavState::ChartAtOrigin::Retract(const Vector9& xi,
    OptionalJacobian<9, 9> H) {
  Matrix3 D_R_xi;
  const Rot3 R = Rot3::Expmap(dR(xi), H ? &D_R_xi : 0);
  const Point3 p = Point3(dP(xi));
  const Vector v = dV(xi);
  const NavState result(R, p, v);
  if (H) {
    *H << D_R_xi, Z_3x3, Z_3x3, //
    Z_3x3, R.transpose(), Z_3x3, //
    Z_3x3, Z_3x3, R.transpose();
  }
  return result;
}

Vector9 NavState::ChartAtOrigin::Local(const NavState& x,
    OptionalJacobian<9, 9> H) {
  if (H)
    throw std::runtime_error(
        "NavState::ChartOrigin::Local derivative not implemented yet");
  Vector9 xi;
  xi << Rot3::Logmap(x.R_), x.t(), x.v();
  return xi;
}

NavState NavState::Expmap(const Vector9& xi, OptionalJacobian<9, 9> H) {
  if (H)
    throw std::runtime_error("NavState::Expmap derivative not implemented yet");
  // use brute force matrix exponential
  return expm<NavState>(xi);
}

Vector9 NavState::Logmap(const NavState& p, OptionalJacobian<9, 9> H) {
  if (H)
    throw std::runtime_error("NavState::Logmap derivative not implemented yet");
  return Vector9::Zero();
}

Matrix9 NavState::AdjointMap() const {
  throw std::runtime_error("NavState::AdjointMap not implemented yet");
}

Matrix7 NavState::wedge(const Vector9& xi) {
  const Matrix3 Omega = skewSymmetric(dR(xi));
  Matrix7 T;
  T << Omega, Z_3x3, dP(xi), Z_3x3, Omega, dV(xi), Vector6::Zero().transpose(), 1.0;
  return T;
}

} /// namespace gtsam

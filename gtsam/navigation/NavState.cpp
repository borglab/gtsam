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

using namespace std;

namespace gtsam {

#define TIE(R,t,v,x) const Rot3& R = (x).R_;const Point3& t = (x).t_;const Velocity3& v = (x).v_;

const Rot3& NavState::attitude(OptionalJacobian<3, 9> H) const {
  if (H)
    *H << I_3x3, Z_3x3, Z_3x3;
  return R_;
}

const Point3& NavState::position(OptionalJacobian<3, 9> H) const {
  if (H)
    *H << Z_3x3, R(), Z_3x3;
  return t_;
}

const Vector3& NavState::velocity(OptionalJacobian<3, 9> H) const {
  if (H)
    *H << Z_3x3, Z_3x3, R();
  return v_;
}

Matrix7 NavState::matrix() const {
  Matrix3 R = this->R();
  Matrix7 T;
  T << R, Z_3x3, t(), Z_3x3, R, v(), Vector6::Zero().transpose(), 1.0;
  return T;
}

void NavState::print(const string& s) const {
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
    throw runtime_error(
        "NavState::ChartOrigin::Local derivative not implemented yet");
  Vector9 xi;
  xi << Rot3::Logmap(x.R_), x.t(), x.v();
  return xi;
}

NavState NavState::Expmap(const Vector9& xi, OptionalJacobian<9, 9> H) {
  if (H)
    throw runtime_error("NavState::Expmap derivative not implemented yet");

  Eigen::Block<const Vector9, 3, 1> n_omega_nb = dR(xi);
  Eigen::Block<const Vector9, 3, 1> v = dP(xi);
  Eigen::Block<const Vector9, 3, 1> a = dV(xi);

  // NOTE(frank): See Pose3::Expmap
  Rot3 nRb = Rot3::Expmap(n_omega_nb);
  double theta2 = n_omega_nb.dot(n_omega_nb);
  if (theta2 > numeric_limits<double>::epsilon()) {
    // Expmap implements a "screw" motion in the direction of n_omega_nb
    Vector3 n_t_parallel = n_omega_nb * n_omega_nb.dot(v); // component along rotation axis
    Vector3 omega_cross_v = n_omega_nb.cross(v); // points towards axis
    Point3 n_t = Point3(omega_cross_v - nRb * omega_cross_v + n_t_parallel)
        / theta2;
    Vector3 n_v_parallel = n_omega_nb * n_omega_nb.dot(a); // component along rotation axis
    Vector3 omega_cross_a = n_omega_nb.cross(a); // points towards axis
    Vector3 n_v = (omega_cross_a - nRb * omega_cross_a + n_v_parallel) / theta2;
    return NavState(nRb, n_t, n_v);
  } else {
    return NavState(nRb, Point3(v), a);
  }
}

Vector9 NavState::Logmap(const NavState& nTb, OptionalJacobian<9, 9> H) {
  if (H)
    throw runtime_error("NavState::Logmap derivative not implemented yet");

  TIE(nRb, n_p, n_v, nTb);
  Vector3 n_t = n_p.vector();

  // NOTE(frank): See Pose3::Logmap
  Vector9 xi;
  Vector3 n_omega_nb = Rot3::Logmap(nRb);
  double theta = n_omega_nb.norm();
  if (theta * theta <= numeric_limits<double>::epsilon()) {
    xi << n_omega_nb, n_t, n_v;
  } else {
    Matrix3 W = skewSymmetric(n_omega_nb / theta);
    // Formula from Agrawal06iros, equation (14)
    // simplified with Mathematica, and multiplying in n_t to avoid matrix math
    double factor = (1 - theta / (2. * tan(0.5 * theta)));
    Vector3 n_x = W * n_t;
    Vector3 v = n_t - (0.5 * theta) * n_x + factor * (W * n_x);
    Vector3 n_y = W * n_v;
    Vector3 a = n_v - (0.5 * theta) * n_y + factor * (W * n_y);
    xi << n_omega_nb, v, a;
  }
  return xi;
}

Matrix9 NavState::AdjointMap() const {
  // NOTE(frank): See Pose3::AdjointMap
  const Matrix3 nRb = R();
  Matrix3 pAr = skewSymmetric(t()) * nRb;
  Matrix3 vAr = skewSymmetric(v()) * nRb;
  Matrix9 adj;
  //     nR/bR nR/bP nR/bV nP/bR nP/bP nP/bV nV/bR nV/bP nV/bV
  adj << nRb, Z_3x3, Z_3x3, pAr, nRb, Z_3x3, vAr, Z_3x3, nRb;
  return adj;
}

Matrix7 NavState::wedge(const Vector9& xi) {
  const Matrix3 Omega = skewSymmetric(dR(xi));
  Matrix7 T;
  T << Omega, Z_3x3, dP(xi), Z_3x3, Omega, dV(xi), Vector6::Zero().transpose(), 1.0;
  return T;
}

// sugar for derivative blocks
#define D_R_R(H) H->block<3,3>(0,0)
#define D_t_t(H) H->block<3,3>(3,3)
#define D_t_v(H) H->block<3,3>(3,6)
#define D_v_t(H) H->block<3,3>(6,3)
#define D_v_v(H) H->block<3,3>(6,6)

Vector9 NavState::coriolis(const Vector3& omega, double dt, bool secondOrder,
    OptionalJacobian<9, 9> H) const {
  Vector9 result;
  const Rot3& nRb = attitude();
  const Point3& n_t = position(); // derivative is R() !
  const Vector3& n_v = velocity(); // ditto
  const double dt2 = dt * dt;

  const Vector3 omega_cross_vel = omega.cross(n_v);
  Matrix3 D_dP_R;
  dR(result) << -nRb.unrotate(omega * dt, H ? &D_dP_R : 0);
  dP(result) << ((-dt2) * omega_cross_vel); // NOTE(luca): we got rid of the 2 wrt INS paper
  dV(result) << ((-2.0 * dt) * omega_cross_vel);
  if (secondOrder) {
    const Vector3 omega_cross2_t = omega.cross(omega.cross(n_t.vector()));
    dP(result) -= (0.5 * dt2) * omega_cross2_t;
    dV(result) -= dt * omega_cross2_t;
  }
  if (H) {
    const Matrix3 Omega = skewSymmetric(omega);
    const Matrix3 D_cross_state = Omega * R();
    H->setZero();
    D_R_R(H) << -D_dP_R;
    D_t_v(H) << (-dt2) * D_cross_state;
    D_v_v(H) << (-2.0 * dt) * D_cross_state;
    if (secondOrder) {
      const Matrix3 D_cross2_state = Omega * D_cross_state;
      D_t_t(H) -= (0.5 * dt2) * D_cross2_state;
      D_v_t(H) -= dt * D_cross2_state;
    }
  }
  return result;
}

} /// namespace gtsam

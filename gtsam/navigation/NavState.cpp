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

//------------------------------------------------------------------------------
NavState NavState::FromPoseVelocity(const Pose3& pose, const Vector3& vel,
    OptionalJacobian<9, 6> H1, OptionalJacobian<9, 3> H2) {
  if (H1)
    *H1 << I_3x3, Z_3x3, Z_3x3, I_3x3, Z_3x3, Z_3x3;
  if (H2)
    *H2 << Z_3x3, Z_3x3, pose.rotation().transpose();
  return NavState(pose, vel);
}

//------------------------------------------------------------------------------
const Rot3& NavState::attitude(OptionalJacobian<3, 9> H) const {
  if (H)
    *H << I_3x3, Z_3x3, Z_3x3;
  return R_;
}

//------------------------------------------------------------------------------
const Point3& NavState::position(OptionalJacobian<3, 9> H) const {
  if (H)
    *H << Z_3x3, R(), Z_3x3;
  return t_;
}

//------------------------------------------------------------------------------
const Vector3& NavState::velocity(OptionalJacobian<3, 9> H) const {
  if (H)
    *H << Z_3x3, Z_3x3, R();
  return v_;
}

//------------------------------------------------------------------------------
Vector3 NavState::bodyVelocity(OptionalJacobian<3, 9> H) const {
  const Rot3& nRb = R_;
  const Vector3& n_v = v_;
  Matrix3 D_bv_nRb;
  Vector3 b_v = nRb.unrotate(n_v, H ? &D_bv_nRb : 0);
  if (H)
    *H << D_bv_nRb, Z_3x3, I_3x3;
  return b_v;
}

//------------------------------------------------------------------------------
Matrix7 NavState::matrix() const {
  Matrix3 R = this->R();
  Matrix7 T;
  T << R, Z_3x3, t(), Z_3x3, R, v(), Vector6::Zero().transpose(), 1.0;
  return T;
}

//------------------------------------------------------------------------------
void NavState::print(const string& s) const {
  attitude().print(s + ".R");
  position().print(s + ".p");
  gtsam::print((Vector) v_, s + ".v");
}

//------------------------------------------------------------------------------
bool NavState::equals(const NavState& other, double tol) const {
  return R_.equals(other.R_, tol) && t_.equals(other.t_, tol)
      && equal_with_abs_tol(v_, other.v_, tol);
}

//------------------------------------------------------------------------------
NavState NavState::inverse() const {
  TIE(nRb, n_t, n_v, *this);
  const Rot3 bRn = nRb.inverse();
  return NavState(bRn, -(bRn * n_t), -(bRn * n_v));
}

//------------------------------------------------------------------------------
NavState NavState::operator*(const NavState& bTc) const {
  TIE(nRb, n_t, n_v, *this);
  TIE(bRc, b_t, b_v, bTc);
  return NavState(nRb * bRc, nRb * b_t + n_t, nRb * b_v + n_v);
}

//------------------------------------------------------------------------------
NavState::PositionAndVelocity //
NavState::operator*(const PositionAndVelocity& b_tv) const {
  TIE(nRb, n_t, n_v, *this);
  const Point3& b_t = b_tv.first;
  const Velocity3& b_v = b_tv.second;
  return PositionAndVelocity(nRb * b_t + n_t, nRb * b_v + n_v);
}

//------------------------------------------------------------------------------
Point3 NavState::operator*(const Point3& b_t) const {
  return Point3(R_ * b_t + t_);
}

//------------------------------------------------------------------------------
Velocity3 NavState::operator*(const Velocity3& b_v) const {
  return Velocity3(R_ * b_v + v_);
}

//------------------------------------------------------------------------------
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

//------------------------------------------------------------------------------
Vector9 NavState::ChartAtOrigin::Local(const NavState& x,
    OptionalJacobian<9, 9> H) {
  Vector9 xi;
  Matrix3 D_xi_R;
  xi << Rot3::Logmap(x.R_, H ? &D_xi_R : 0), x.t(), x.v();
  if (H) {
    *H << D_xi_R, Z_3x3, Z_3x3, //
    Z_3x3, x.R(), Z_3x3, //
    Z_3x3, Z_3x3, x.R();
  }
  return xi;
}

//------------------------------------------------------------------------------
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

//------------------------------------------------------------------------------
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

//------------------------------------------------------------------------------
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

//------------------------------------------------------------------------------
Matrix7 NavState::wedge(const Vector9& xi) {
  const Matrix3 Omega = skewSymmetric(dR(xi));
  Matrix7 T;
  T << Omega, Z_3x3, dP(xi), Z_3x3, Omega, dV(xi), Vector6::Zero().transpose(), 1.0;
  return T;
}

//------------------------------------------------------------------------------
// sugar for derivative blocks
#define D_R_R(H) (H)->block<3,3>(0,0)
#define D_R_t(H) (H)->block<3,3>(0,3)
#define D_R_v(H) (H)->block<3,3>(0,6)
#define D_t_R(H) (H)->block<3,3>(3,0)
#define D_t_t(H) (H)->block<3,3>(3,3)
#define D_t_v(H) (H)->block<3,3>(3,6)
#define D_v_R(H) (H)->block<3,3>(6,0)
#define D_v_t(H) (H)->block<3,3>(6,3)
#define D_v_v(H) (H)->block<3,3>(6,6)

//------------------------------------------------------------------------------
NavState NavState::update(const Vector3& b_acceleration, const Vector3& b_omega,
    const double dt, OptionalJacobian<9, 9> F, OptionalJacobian<9, 3> G1,
    OptionalJacobian<9, 3> G2) const {

  Vector9 xi;
  Matrix39 D_xiP_state;
  Vector3 b_v = bodyVelocity(F ? &D_xiP_state : 0);
  double dt22 = 0.5 * dt * dt;

  // Integrate on tangent space. TODO(frank): coriolis?
  dR(xi) << dt * b_omega;
  dP(xi) << dt * b_v + dt22 * b_acceleration;
  dV(xi) << dt * b_acceleration;

  // Bring back to manifold
  Matrix9 D_newState_xi;
  NavState newState = retract(xi, F, G1 || G2 ? &D_newState_xi : 0);

  // Derivative wrt state is computed by retract directly
  // However, as dP(xi) also depends on state, we need to add that contribution
  if (F) {
    F->middleRows<3>(3) += dt * D_t_t(F) * D_xiP_state;
  }
  // derivative wrt acceleration
  if (G1) {
    // D_newState_dPxi = D_newState_xi.middleCols<3>(3)
    // D_dPxi_acc = dt22 * I_3x3
    // D_newState_dVxi = D_newState_xi.rightCols<3>()
    // D_dVxi_acc = dt * I_3x3
    // *G2 = D_newState_acc = D_newState_dPxi * D_dPxi_acc + D_newState_dVxi * D_dVxi_acc
    *G1 = D_newState_xi.middleCols<3>(3) * dt22
        + D_newState_xi.rightCols<3>() * dt;
  }
  // derivative wrt omega
  if (G2) {
    // D_newState_dRxi = D_newState_xi.leftCols<3>()
    // D_dRxi_omega = dt * I_3x3
    // *G1 = D_newState_omega = D_newState_dRxi * D_dRxi_omega
    *G2 = D_newState_xi.leftCols<3>() * dt;
  }
  return newState;
}

//------------------------------------------------------------------------------
Vector9 NavState::coriolis(double dt, const Vector3& omega, bool secondOrder,
    OptionalJacobian<9, 9> H) const {
  TIE(nRb, n_t, n_v, *this);
  const double dt2 = dt * dt;
  const Vector3 omega_cross_vel = omega.cross(n_v);

  Vector9 xi;
  Matrix3 D_dP_R;
  dR(xi) << nRb.unrotate((-dt) * omega, H ? &D_dP_R : 0);
  dP(xi) << ((-dt2) * omega_cross_vel); // NOTE(luca): we got rid of the 2 wrt INS paper
  dV(xi) << ((-2.0 * dt) * omega_cross_vel);
  if (secondOrder) {
    const Vector3 omega_cross2_t = omega.cross(omega.cross(n_t.vector()));
    dP(xi) -= (0.5 * dt2) * omega_cross2_t;
    dV(xi) -= dt * omega_cross2_t;
  }
  if (H) {
    H->setZero();
    const Matrix3 Omega = skewSymmetric(omega);
    const Matrix3 D_cross_state = Omega * R();
    H->setZero();
    D_R_R(H) << D_dP_R;
    D_t_v(H) << (-dt2) * D_cross_state;
    D_v_v(H) << (-2.0 * dt) * D_cross_state;
    if (secondOrder) {
      const Matrix3 D_cross2_state = Omega * D_cross_state;
      D_t_t(H) -= (0.5 * dt2) * D_cross2_state;
      D_v_t(H) -= dt * D_cross2_state;
    }
  }
  return xi;
}

//------------------------------------------------------------------------------
Vector9 NavState::correctPIM(const Vector9& pim, double dt,
    const Vector3& n_gravity, const boost::optional<Vector3>& omegaCoriolis,
    bool use2ndOrderCoriolis, OptionalJacobian<9, 9> H1,
    OptionalJacobian<9, 9> H2) const {
  const Rot3& nRb = R_;
  const Velocity3& n_v = v_; // derivative is Ri !
  const double dt22 = 0.5 * dt * dt;

  Vector9 xi;
  Matrix3 D_dP_Ri1, D_dP_Ri2, D_dP_nv, D_dV_Ri;
  dR(xi) = dR(pim);
  dP(xi) = dP(pim)
      + dt * nRb.unrotate(n_v, H1 ? &D_dP_Ri1 : 0, H2 ? &D_dP_nv : 0)
      + dt22 * nRb.unrotate(n_gravity, H1 ? &D_dP_Ri2 : 0);
  dV(xi) = dV(pim) + dt * nRb.unrotate(n_gravity, H1 ? &D_dV_Ri : 0);

  if (omegaCoriolis) {
    xi += coriolis(dt, *omegaCoriolis, use2ndOrderCoriolis, H1);
  }

  if (H1 || H2) {
    Matrix3 Ri = nRb.matrix();

    if (H1) {
      if (!omegaCoriolis)
        H1->setZero(); // if coriolis H1 is already initialized
      D_t_R(H1) += dt * D_dP_Ri1 + dt22 * D_dP_Ri2;
      D_t_v(H1) += dt * D_dP_nv * Ri;
      D_v_R(H1) += dt * D_dV_Ri;
    }
    if (H2) {
      H2->setIdentity();
    }
  }

  return xi;
}
//------------------------------------------------------------------------------

}/// namespace gtsam

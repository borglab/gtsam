/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    NavState.cpp
 * @brief   Navigation state composing of attitude, position, and velocity
 * @author  Frank Dellaert
 * @date    July 2015
 **/

#include <gtsam/navigation/NavState.h>
#include <gtsam/geometry/Kernel.h>

#include <string>

namespace gtsam {

//------------------------------------------------------------------------------
NavState NavState::Create(const Rot3& R, const Point3& t, const Velocity3& v,
    OptionalJacobian<9, 3> H1, OptionalJacobian<9, 3> H2,
    OptionalJacobian<9, 3> H3) {
  Matrix3 Rt;
  if (H2 || H3) Rt = R.transpose();
  if (H1) *H1 << I_3x3, Z_3x3, Z_3x3;
  if (H2) *H2 << Z_3x3, Rt, Z_3x3;
  if (H3) *H3 << Z_3x3, Z_3x3, Rt;
  return NavState(R, t, v);
}
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
  return Base::rotation(H);
}

//------------------------------------------------------------------------------
Point3 NavState::position(OptionalJacobian<3, 9> H) const {
  return Base::x(0, H);
}

//------------------------------------------------------------------------------
Vector3 NavState::velocity(OptionalJacobian<3, 9> H) const {
  return Base::x(1, H);
}

//------------------------------------------------------------------------------
Vector3 NavState::bodyVelocity(OptionalJacobian<3, 9> H) const {
  const Rot3& nRb = R_;
  const Vector3 n_v = t_.col(1);
  Matrix3 D_bv_nRb;
  Vector3 b_v = nRb.unrotate(n_v, H ? &D_bv_nRb : 0);
  if (H)
    *H << D_bv_nRb, Z_3x3, I_3x3;
  return b_v;
}

//------------------------------------------------------------------------------
double NavState::range(const Point3& point, OptionalJacobian<1, 9> Hself,
                       OptionalJacobian<1, 3> Hpoint) const {
  const Vector3 delta = point - t_.col(0);
  const double r = delta.norm();
  if (!Hself && !Hpoint) return r;

  const Vector3 u = delta / r;  // unit vector from position to point
  const Matrix13 D_r_point = u.transpose();

  if (Hpoint) *Hpoint = D_r_point;
  if (Hself) {
    Hself->setZero();
    // position() = t + R * dP, so d(range)/d(dP) = d(range)/dt * dt/d(dP)
    Hself->block<1, 3>(0, 3) = -D_r_point * R_.matrix();
  }
  return r;
}

//------------------------------------------------------------------------------
Unit3 NavState::bearing(const Point3& point, OptionalJacobian<2, 9> Hself,
                        OptionalJacobian<2, 3> Hpoint) const {
  Matrix26 Hpose;
  OptionalJacobian<2, 6> HposeOptional(Hself ? &Hpose : nullptr);
  const Unit3 b = pose().bearing(point, HposeOptional, Hpoint);

  if (Hself) {
    Hself->setZero();
    Hself->block<2, 6>(0, 0) = Hpose;
  }
  return b;
}

//------------------------------------------------------------------------------
std::ostream& operator<<(std::ostream& os, const NavState& state) {
  os << "R: " << state.attitude() << "\n";
  os << "p: " << state.position().transpose() << "\n";
  os << "v: " << state.velocity().transpose();
  return os;
}

//------------------------------------------------------------------------------
void NavState::print(const std::string& s) const {
  std::cout << (s.empty() ? s : s + " ") << *this << std::endl;
}

//------------------------------------------------------------------------------
bool NavState::equals(const NavState& other, double tol) const {
  return Base::equals(other, tol);
}

//------------------------------------------------------------------------------
NavState NavState::retract(const Vector9& xi, //
    OptionalJacobian<9, 9> H1, OptionalJacobian<9, 9> H2) const {
  // NOTE: This is an intentional custom chart for NavState manifold updates.
  // It differs from the default LieGroup chart based on full Expmap/Logmap.
  Rot3 nRb = R_;
  Point3 n_t = t_.col(0), n_v = t_.col(1);
  Matrix3 D_bRc_xi, D_R_nRb, D_t_nRb, D_v_nRb;
  const Rot3 bRc = Rot3::Expmap(dR(xi), H2 ? &D_bRc_xi : 0);
  const Rot3 nRc = nRb.compose(bRc, H1 ? &D_R_nRb : 0);
  const Point3 t = n_t + nRb.rotate(dP(xi), H1 ? &D_t_nRb : 0);
  const Point3 v = n_v + nRb.rotate(dV(xi), H1 ? &D_v_nRb : 0);
  if (H1) {
    *H1 << D_R_nRb, Z_3x3, Z_3x3, //
    // Note(frank): the derivative of n_t with respect to xi is nRb
    // We pre-multiply with nRc' to account for NavState::Create
    // Then we make use of the identity nRc' * nRb = bRc'
    nRc.transpose() * D_t_nRb, bRc.transpose(), Z_3x3,
    // Similar reasoning for v:
    nRc.transpose() * D_v_nRb, Z_3x3, bRc.transpose();
  }
  if (H2) {
    *H2 << D_bRc_xi, Z_3x3, Z_3x3, //
    Z_3x3, bRc.transpose(), Z_3x3, //
    Z_3x3, Z_3x3, bRc.transpose();
  }
  return NavState(nRc, t, v);
}

//------------------------------------------------------------------------------
Vector9 NavState::localCoordinates(const NavState& g, //
    OptionalJacobian<9, 9> H1, OptionalJacobian<9, 9> H2) const {
  // Inverse of the custom component-wise chart used in retract().
  Matrix3 D_dR_R, D_dt_R, D_dv_R;
  const Rot3 dR = R_.between(g.R_, H1 ? &D_dR_R : 0);
  const Point3 dP = R_.unrotate(g.t_.col(0) - t_.col(0), H1 ? &D_dt_R : 0);
  const Vector dV = R_.unrotate(g.t_.col(1) - t_.col(1), H1 ? &D_dv_R : 0);

  Vector9 xi;
  Matrix3 D_xi_R;
  xi << Rot3::Logmap(dR, (H1 || H2) ? &D_xi_R : 0), dP, dV;
  if (H1) {
    *H1 << D_xi_R * D_dR_R, Z_3x3, Z_3x3,  //
        D_dt_R, -I_3x3, Z_3x3,             //
        D_dv_R, Z_3x3, -I_3x3;
  }
  if (H2) {
    *H2 << D_xi_R, Z_3x3, Z_3x3,    //
        Z_3x3, dR.matrix(), Z_3x3,  //
        Z_3x3, Z_3x3, dR.matrix();
  }

  return xi;
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

// Because our navigation frames are placed on a spinning Earth, we experience two apparent forces on our inertials
// Let Omega be the Earth's rotation rate in the navigation frame
// Coriolis acceleration = -2 * (omega X n_v)
// Centrifugal acceleration (secondOrder) = -omega X (omega X n_t)
// We would also experience a rotation of (omega*dt) over time - so, counteract by compensating rotation by (-omega * dt)
// Integrate centrifugal & coriolis accelerations to yield position, velocity perturbations
Vector9 NavState::coriolis(double dt, const Vector3& omega, bool secondOrder,
    OptionalJacobian<9, 9> H) const {
  Rot3 nRb = R_;
  Point3 n_t = t_.col(0), n_v = t_.col(1);

  const double dt2 = dt * dt;
  const Vector3 omega_cross_vel = omega.cross(n_v);

  Vector9 n_xi;
  // Coriolis (first order) acceleration corrections
  dR(n_xi) << ((-dt) * omega);
  dP(n_xi) << ((-dt2) * omega_cross_vel); // NOTE(luca): we got rid of the 2 wrt INS paper
  dV(n_xi) << ((-2.0 * dt) * omega_cross_vel);

  // Centrifugal (second order) acceleration corrections
  Matrix3 D_c2_nt; // To store Jacobian (if needed/desired)
  if (secondOrder) {
    const Vector3 omega_cross2_t = doubleCross(omega, n_t, nullptr, H ? &D_c2_nt : nullptr);
    dP(n_xi) -= (0.5 * dt2) * omega_cross2_t;
    dV(n_xi) -= dt * omega_cross2_t;
  }

  // Transform correction from navigation frame -> body frame and get Jacobians
  Vector9 xi;
  Matrix3 D_dR_R, D_dP_R, D_dV_R;
  dR(xi) = nRb.unrotate(dR(n_xi), H ? &D_dR_R : 0);
  dP(xi) = nRb.unrotate(dP(n_xi), H ? &D_dP_R : 0);
  dV(xi) = nRb.unrotate(dV(n_xi), H ? &D_dV_R : 0);

  // Assemble Jacobians
  if (H) {
    H->setZero();
    const Vector3 omega_b = nRb.unrotate(omega);
    const Matrix3 D_c1_b = skewSymmetric(omega_b);
    H->setZero();
    D_R_R(H) << D_dR_R;
    D_t_v(H) << (-dt2) * D_c1_b;
    D_t_R(H) << D_dP_R;
    D_v_v(H) << (-2.0 * dt) * D_c1_b;
    D_v_R(H) << D_dV_R;
    if (secondOrder) {
      Matrix3 D_c2_b = D_c1_b * D_c1_b;
      D_t_t(H) -= (0.5 * dt2) * D_c2_b;
      D_v_t(H) -= dt * D_c2_b;
    }
  }
  return xi;
}

//------------------------------------------------------------------------------
Vector9 NavState::correctPIM(const Vector9& pim, double dt,
    const Vector3& n_gravity, const std::optional<Vector3>& omegaCoriolis,
    bool use2ndOrderCoriolis, OptionalJacobian<9, 9> H1,
    OptionalJacobian<9, 9> H2) const {
  const Rot3& nRb = R_;
  const Velocity3 n_v = t_.col(1); // derivative is Ri !
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

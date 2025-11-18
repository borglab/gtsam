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
Matrix5 NavState::matrix() const {
  Matrix3 R = this->R();

  Matrix5 T = Matrix5::Identity();
  T.block<3, 3>(0, 0) = R;
  T.block<3, 1>(0, 3) = t_;
  T.block<3, 1>(0, 4) = v_;
  return T;
}

//------------------------------------------------------------------------------
NavState::Vector25 NavState::vec(OptionalJacobian<25, 9> H) const {
  const Matrix5 T = this->matrix();
  if (H) {
    H->setZero();
    auto R = T.block<3, 3>(0, 0);
    H->block<3, 1>(0, 1) = -R.col(2);
    H->block<3, 1>(0, 2) = R.col(1);
    H->block<3, 1>(5, 0) = R.col(2);
    H->block<3, 1>(5, 2) = -R.col(0);
    H->block<3, 1>(10, 0) = -R.col(1);
    H->block<3, 1>(10, 1) = R.col(0);
    H->block<3, 3>(15, 3) = R;
    H->block<3, 3>(20, 6) = R;
  }
  return Eigen::Map<const Vector25>(T.data());
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
  return R_.equals(other.R_, tol) && traits<Point3>::Equals(t_, other.t_, tol)
      && equal_with_abs_tol(v_, other.v_, tol);
}

//------------------------------------------------------------------------------
NavState NavState::inverse() const {
  Rot3 Rt = R_.inverse();
  return NavState(Rt, Rt * (-t_), Rt * -(v_));
}

//------------------------------------------------------------------------------
// See [this document](doc/Jacobians.md) for details.
NavState NavState::Expmap(const Vector9& xi, OptionalJacobian<9, 9> Hxi) {
  // Get angular velocity w and components rho (for t) and nu (for v) from xi
  Vector3 w = xi.head<3>(), rho = xi.segment<3>(3), nu = xi.tail<3>();

  // Instantiate functor for Dexp-related operations:
  const so3::DexpFunctor local(w);

  // Compute rotation using Expmap
#ifdef GTSAM_USE_QUATERNIONS
  const Rot3 R = traits<gtsam::Quaternion>::Expmap(w);
#else
  const Rot3 R(local.expmap());
#endif

  // Compute translation and velocity. See Pose3::Expmap
  Matrix3 H_t_w, H_v_w;
  const Vector3 t = local.Jacobian().applyLeft(rho, Hxi ? &H_t_w : nullptr);
  const Vector3 v = local.Jacobian().applyLeft(nu, Hxi ? &H_v_w : nullptr);

  if (Hxi) {
    const Matrix3 Jr = local.Jacobian().right();
    // We are creating a NavState, so we still need to chain H_t_w and H_v_w
    // with R^T, the Jacobian of Navstate::Create with respect to both t and v.
    const Matrix3 Rt = R.transpose();
    *Hxi << Jr, Z_3x3, Z_3x3,   // Jr here *is* the Jacobian of expmap
        Rt * H_t_w, Jr, Z_3x3,  //
        Rt * H_v_w, Z_3x3, Jr;
    // In the last two rows, Jr = R^T * Jl, see Barfoot eq. (8.83).
    // Jl is the left Jacobian of SO(3) at w.
  }

  return NavState(R, t, v);
}

//------------------------------------------------------------------------------
Vector9 NavState::Logmap(const NavState& state, OptionalJacobian<9, 9> Hstate) {
  if (Hstate) *Hstate = LogmapDerivative(state);

  const Vector3 phi = Rot3::Logmap(state.rotation());
  const Vector3& p = state.position();
  const Vector3& v = state.velocity();
  const double t = phi.norm();
  if (t < 1e-8) {
    Vector9 log;
    log << phi, p, v;
    return log;

  } else {
    const Matrix3 W = skewSymmetric(phi / t);

    const double Tan = tan(0.5 * t);
    const Vector3 Wp = W * p;
    const Vector3 Wv = W * v;
    const Vector3 rho = p - (0.5 * t) * Wp + (1 - t / (2. * Tan)) * (W * Wp);
    const Vector3 nu = v - (0.5 * t) * Wv + (1 - t / (2. * Tan)) * (W * Wv);
    Vector9 log;
    // Order is ω, p, v
    log << phi, rho, nu;
    return log;
  }
}

//------------------------------------------------------------------------------
Matrix9 NavState::AdjointMap() const {
  const Matrix3 R = R_.matrix();
  Matrix3 A = skewSymmetric(t_) * R;
  Matrix3 B = skewSymmetric(v_) * R;
  // Eqn 2 in Barrau20icra
  Matrix9 adj;
  adj << R, Z_3x3, Z_3x3, A, R, Z_3x3, B, Z_3x3, R;
  return adj;
}

//------------------------------------------------------------------------------
Vector9 NavState::Adjoint(const Vector9& xi_b, OptionalJacobian<9, 9> H_state,
                          OptionalJacobian<9, 9> H_xib) const {
  const Matrix9 Ad = AdjointMap();

  // Jacobians
  if (H_state) *H_state = -Ad * adjointMap(xi_b);
  if (H_xib) *H_xib = Ad;

  return Ad * xi_b;
}

//------------------------------------------------------------------------------
Matrix9 NavState::adjointMap(const Vector9& xi) {
  Matrix3 w_hat = skewSymmetric(xi(0), xi(1), xi(2));
  Matrix3 v_hat = skewSymmetric(xi(3), xi(4), xi(5));
  Matrix3 a_hat = skewSymmetric(xi(6), xi(7), xi(8));
  Matrix9 adj;
  adj << w_hat, Z_3x3, Z_3x3, v_hat, w_hat, Z_3x3, a_hat, Z_3x3, w_hat;
  return adj;
}

//------------------------------------------------------------------------------
Vector9 NavState::adjoint(const Vector9& xi, const Vector9& y,
                          OptionalJacobian<9, 9> Hxi,
                          OptionalJacobian<9, 9> H_y) {
  if (Hxi) {
    Hxi->setZero();
    for (int i = 0; i < 9; ++i) {
      Vector9 dxi;
      dxi.setZero();
      dxi(i) = 1.0;
      Matrix9 Gi = adjointMap(dxi);
      Hxi->col(i) = Gi * y;
    }
  }

  const Matrix9& ad_xi = adjointMap(xi);
  if (H_y) *H_y = ad_xi;

  return ad_xi * y;
}

//------------------------------------------------------------------------------
Matrix9 NavState::ExpmapDerivative(const Vector9& xi) {
  Matrix9 J;
  Expmap(xi, J);
  return J;
}

//------------------------------------------------------------------------------
Matrix9 NavState::LogmapDerivative(const Vector9& xi) {
  const Vector3 w = xi.head<3>();
  Vector3 rho = xi.segment<3>(3);
  Vector3 nu = xi.tail<3>();

  // Instantiate functor for Dexp-related operations:
  const so3::DexpFunctor local(w);

  // Call Jacobian().applyLeft to get its Jacobians
  Matrix3 H_t_w, H_v_w;
  local.Jacobian().applyLeft(rho, H_t_w);
  local.Jacobian().applyLeft(nu, H_v_w);

  // Multiply with R^T to account for NavState::Create Jacobian.
  const Matrix3 Rt = local.expmap().transpose();
  const Matrix3 Qt = Rt * H_t_w;
  const Matrix3 Qv = Rt * H_v_w;

  // Now compute the blocks of the LogmapDerivative Jacobian
  const Matrix3 Jw = Rot3::LogmapDerivative(w);
  const Matrix3 Qt2 = -Jw * Qt * Jw;
  const Matrix3 Qv2 = -Jw * Qv * Jw;

  Matrix9 J;
  J <<  Jw, Z_3x3, Z_3x3, 
       Qt2,    Jw, Z_3x3,
       Qv2, Z_3x3,    Jw;
  return J;
}

//------------------------------------------------------------------------------
Matrix9 NavState::LogmapDerivative(const NavState& state) {
  const Vector9 xi = Logmap(state);
  return LogmapDerivative(xi);
}

//------------------------------------------------------------------------------
Matrix5 NavState::Hat(const Vector9& xi) {
  Matrix5 X;
  const double wx = xi(0), wy = xi(1), wz = xi(2);
  const double px = xi(3), py = xi(4), pz = xi(5);
  const double vx = xi(6), vy = xi(7), vz = xi(8);
  X << 0., -wz, wy, px, vx,
    wz, 0., -wx, py, vy,
    -wy, wx, 0., pz, vz,
    0., 0., 0., 0., 0.,
    0., 0., 0., 0., 0.;
  return X;
}

//------------------------------------------------------------------------------
Vector9 NavState::Vee(const Matrix5& Xi) {
  Vector9 xi;
  xi << Xi(2, 1), Xi(0, 2), Xi(1, 0),
    Xi(0, 3), Xi(1, 3), Xi(2, 3),
    Xi(0, 4), Xi(1, 4), Xi(2, 4);
  return xi;
}

//------------------------------------------------------------------------------
NavState NavState::ChartAtOrigin::Retract(const Vector9& xi,
                                          ChartJacobian Hxi) {
  return Expmap(xi, Hxi);
}

//------------------------------------------------------------------------------
Vector9 NavState::ChartAtOrigin::Local(const NavState& state,
                                       ChartJacobian Hstate) {
  return Logmap(state, Hstate);
}

//------------------------------------------------------------------------------
NavState NavState::retract(const Vector9& xi, //
    OptionalJacobian<9, 9> H1, OptionalJacobian<9, 9> H2) const {
  Rot3 nRb = R_;
  Point3 n_t = t_, n_v = v_;
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
  Matrix3 D_dR_R, D_dt_R, D_dv_R;
  const Rot3 dR = R_.between(g.R_, H1 ? &D_dR_R : 0);
  const Point3 dP = R_.unrotate(g.t_ - t_, H1 ? &D_dt_R : 0);
  const Vector dV = R_.unrotate(g.v_ - v_, H1 ? &D_dv_R : 0);

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
Vector9 NavState::coriolis(double dt, const Vector3& omega, bool secondOrder,
    OptionalJacobian<9, 9> H) const {
  Rot3 nRb = R_;
  Point3 n_t = t_, n_v = v_;

  const double dt2 = dt * dt;
  const Vector3 omega_cross_vel = omega.cross(n_v);

  // Get perturbations in nav frame
  Vector9 n_xi, xi;
  Matrix3 D_dR_R, D_dP_R, D_dV_R, D_body_nav;
  dR(n_xi) << ((-dt) * omega);
  dP(n_xi) << ((-dt2) * omega_cross_vel); // NOTE(luca): we got rid of the 2 wrt INS paper
  dV(n_xi) << ((-2.0 * dt) * omega_cross_vel);
  if (secondOrder) {
    const Vector3 omega_cross2_t = omega.cross(omega.cross(n_t));
    dP(n_xi) -= (0.5 * dt2) * omega_cross2_t;
    dV(n_xi) -= dt * omega_cross2_t;
  }

  // Transform n_xi into the body frame
  xi << nRb.unrotate(dR(n_xi), H ? &D_dR_R : 0, H ? &D_body_nav : 0), 
        nRb.unrotate(dP(n_xi), H ? &D_dP_R : 0),
        nRb.unrotate(dV(n_xi), H ? &D_dV_R : 0);

  if (H) {
    H->setZero();
    const Matrix3 Omega = skewSymmetric(omega);
    const Matrix3 D_cross_state = Omega * R();
    H->setZero();
    D_R_R(H) << D_dR_R;
    D_t_v(H) << D_body_nav * (-dt2) * D_cross_state;
    D_t_R(H) << D_dP_R;
    D_v_v(H) << D_body_nav * (-2.0 * dt) * D_cross_state;
    D_v_R(H) << D_dV_R;
    if (secondOrder) {
      const Matrix3 D_cross2_state = Omega * D_cross_state;
      D_t_t(H) -= D_body_nav * (0.5 * dt2) * D_cross2_state;
      D_v_t(H) -= D_body_nav * dt * D_cross2_state;
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

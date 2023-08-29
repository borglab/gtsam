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

#include <string>

namespace gtsam {

//------------------------------------------------------------------------------
NavState NavState::Create(const Rot3& R, const Point3& t, const Velocity3& v,
    OptionalJacobian<9, 3> H1, OptionalJacobian<9, 3> H2,
    OptionalJacobian<9, 3> H3) {
  if (H1)
    *H1 << I_3x3, Z_3x3, Z_3x3;
  if (H2)
    *H2 << Z_3x3, R.transpose(), Z_3x3;
  if (H3)
    *H3 << Z_3x3, Z_3x3, R.transpose();
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
  Matrix5 T;
  T << R, t(), v(), Z_3x1.transpose(), 1.0, 0.0, Z_3x1.transpose(), 0.0, 1.0;
  return T;
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
NavState NavState::Expmap(const Vector9& xi, OptionalJacobian<9, 9> Hxi) {
  if (Hxi) *Hxi = ExpmapDerivative(xi);

  // Order is ω, velocity, position which represent by φ,ρ,ν
  Vector3 phi(xi(0), xi(1), xi(2)), rho(xi(3), xi(4), xi(5)),
      nu(xi(6), xi(7), xi(8));

  Rot3 R = Rot3::Expmap(phi);

  double phi_norm = phi.norm();
  if (phi_norm < 1e-8)
    return NavState(Rot3(), Point3(rho), Point3(nu));
  else {
    Matrix W = skewSymmetric(phi / phi_norm);
    Matrix A = I_3x3 + ((1 - cos(phi_norm)) / phi_norm) * W +
               ((phi_norm - sin(phi_norm)) / phi_norm) * (W * W);
    return NavState(Rot3::Expmap(phi), Point3(A * rho), Point3(A * nu));
  }
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
  Matrix3 A = skewSymmetric(t_.x(), t_.y(), t_.z()) * R;
  Matrix3 B = skewSymmetric(v_.x(), v_.y(), v_.z()) * R;
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
/// The dual version of Adjoint
Vector9 NavState::AdjointTranspose(const Vector9& x, OptionalJacobian<9, 9> H_state,
                                OptionalJacobian<9, 9> H_x) const {
  const Matrix9 Ad = AdjointMap();
  const Vector9 AdTx = Ad.transpose() * x;

  // Jacobians
  if (H_state) {
    //TODO(Varun)
    // const auto w_T_hat = skewSymmetric(AdTx.head<3>()),
    //            v_T_hat = skewSymmetric(AdTx.segment<3>(3)),
    //            a_T_hat = skewSymmetric(AdTx.tail<3>());
    //   *H_state << w_T_hat, v_T_hat,  //
    //       /*  */ v_T_hat, Z_3x3;
    throw std::runtime_error(
        "AdjointTranpose H_state Jacobian not implemented.");
  }
  if (H_x) {
    *H_x = Ad.transpose();
  }

  return AdTx;
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
Vector9 NavState::adjointTranspose(const Vector9& xi, const Vector9& y,
                                   OptionalJacobian<9, 9> Hxi,
                                   OptionalJacobian<9, 9> H_y) {
  if (Hxi) {
    Hxi->setZero();
    for (int i = 0; i < 9; ++i) {
      Vector9 dxi;
      dxi.setZero();
      dxi(i) = 1.0;
      Matrix9 GTi = adjointMap(dxi).transpose();
      Hxi->col(i) = GTi * y;
    }
  }
  const Matrix9& adT_xi = adjointMap(xi).transpose();
  if (H_y) *H_y = adT_xi;
  return adT_xi * y;
}

/* ************************************************************************* */
Matrix63 NavState::ComputeQforExpmapDerivative(const Vector9& xi,
                                               double nearZeroThreshold) {
  const auto omega = xi.head<3>();
  const auto nu = xi.segment<3>(3);
  const auto rho = xi.tail<3>();
  const Matrix3 V = skewSymmetric(nu);
  const Matrix3 P = skewSymmetric(rho);
  const Matrix3 W = skewSymmetric(omega);

  Matrix3 Qv, Qp;
  Matrix63 Q;

  // The closed-form formula in Barfoot14tro eq. (102)
  double phi = omega.norm();
  if (std::abs(phi) > 1e-5) {
    const double sinPhi = sin(phi), cosPhi = cos(phi);
    const double phi2 = phi * phi, phi3 = phi2 * phi, phi4 = phi3 * phi,
                 phi5 = phi4 * phi;
    // Invert the sign of odd-order terms to have the right Jacobian
    Qv = -0.5 * V + (phi - sinPhi) / phi3 * (W * V + V * W - W * V * W) +
         (1 - phi2 / 2 - cosPhi) / phi4 *
             (W * W * V + V * W * W - 3 * W * V * W) -
         0.5 *
             ((1 - phi2 / 2 - cosPhi) / phi4 -
              3 * (phi - sinPhi - phi3 / 6.) / phi5) *
             (W * V * W * W + W * W * V * W);
    Qp = -0.5 * P + (phi - sinPhi) / phi3 * (W * P + P * W - W * P * W) +
         (1 - phi2 / 2 - cosPhi) / phi4 *
             (W * W * P + P * W * W - 3 * W * P * W) -
         0.5 *
             ((1 - phi2 / 2 - cosPhi) / phi4 -
              3 * (phi - sinPhi - phi3 / 6.) / phi5) *
             (W * P * W * W + W * W * P * W);
  } else {
    Qv = -0.5 * V + 1. / 6. * (W * V + V * W - W * V * W) +
         1. / 24. * (W * W * V + V * W * W - 3 * W * V * W) -
         0.5 * (1. / 24. + 3. / 120.) * (W * V * W * W + W * W * V * W);
    Qp = -0.5 * P + 1. / 6. * (W * P + P * W - W * P * W) +
         1. / 24. * (W * W * P + P * W * W - 3 * W * P * W) -
         0.5 * (1. / 24. + 3. / 120.) * (W * P * W * W + W * W * P * W);
  }

  Q << Qv, Qp;
  return Q;
}

//------------------------------------------------------------------------------
Matrix9 NavState::ExpmapDerivative(const Vector9& xi) {
  const Vector3 w = xi.head<3>();
  const Matrix3 Jw = Rot3::ExpmapDerivative(w);
  const Matrix63 Q = ComputeQforExpmapDerivative(xi);
  const Matrix3 Qv = Q.topRows<3>();
  const Matrix3 Qp = Q.bottomRows<3>();

  Matrix9 J;
  J << Jw, Z_3x3, Z_3x3, Qp, Jw, Z_3x3, Qv, Z_3x3, Jw;

  return J;
}

//------------------------------------------------------------------------------
Matrix9 NavState::LogmapDerivative(const NavState& state) {
  const Matrix3 R = state.rotation().matrix();

  const Vector9 xi = Logmap(state);
  const Vector3 w = xi.head<3>();
  const Matrix3 Jw = Rot3::LogmapDerivative(w);
  const Matrix63 Q = ComputeQforExpmapDerivative(xi);
  const Matrix3 Qv = Q.topRows<3>();
  const Matrix3 Qp = Q.bottomRows<3>();
  const Matrix3 Qv2 = -Jw * Qv * Jw;
  const Matrix3 Qp2 = -Jw * Qp * Jw;

  Matrix9 J;
  // TODO(Varun) If we remove the custom localCoordinates, first one mirrors
  // Pose3, but the second one gives the correct values as per unit tests
  J << Jw, Z_3x3, Z_3x3, Qp2, Jw, Z_3x3, Qv2, Z_3x3, Jw;
  // J << Jw, Z_3x3, Z_3x3, Z_3x3, R, Z_3x3, Z_3x3, Z_3x3, R;
  return J;
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
  auto [nRb, n_t, n_v] = (*this);
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
  auto [nRb, n_t, n_v] = (*this);

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

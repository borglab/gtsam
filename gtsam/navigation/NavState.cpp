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

#include <gtsam/base/MatrixConstants.h>
#include <gtsam/geometry/Kernel.h>
#include <gtsam/geometry/SO3.h>
#include <gtsam/navigation/NavState.h>

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
#ifdef GTSAM_NAVSTATE_EXPMAP
  return expmap(xi, H1, H2);
#else
  return internal::navStateComponentWiseRetract(*this, xi, H1, H2);
#endif
}

//------------------------------------------------------------------------------
Vector9 NavState::localCoordinates(const NavState& g, //
    OptionalJacobian<9, 9> H1, OptionalJacobian<9, 9> H2) const {
#ifdef GTSAM_NAVSTATE_EXPMAP
  return logmap(g, H1, H2);
#else
  return internal::navStateComponentWiseLocalCoordinates(*this, g, H1, H2);
#endif
}

//------------------------------------------------------------------------------
NavState internal::navStateComponentWiseRetract(
    const NavState& state, const Vector9& xi,
    OptionalJacobian<9, 9> H1, OptionalJacobian<9, 9> H2) {
  const Rot3& nRb = state.attitude();
  const Point3 n_t = state.position();
  const Vector3 n_v = state.velocity();
  Matrix3 D_bRc_xi, D_R_nRb, D_t_nRb, D_v_nRb;
  const Rot3 bRc = Rot3::Expmap(NavState::dR(xi), H2 ? &D_bRc_xi : 0);
  const Rot3 nRc = nRb.compose(bRc, H1 ? &D_R_nRb : 0);
  const Point3 t =
      n_t + nRb.rotate(NavState::dP(xi), H1 ? &D_t_nRb : 0);
  const Point3 v =
      n_v + nRb.rotate(NavState::dV(xi), H1 ? &D_v_nRb : 0);
  Matrix3 bRcTranspose;
  if (H1 || H2) bRcTranspose = bRc.transpose();
  if (H1) {
    const Matrix3 nRcTranspose = nRc.transpose();
    *H1 << D_R_nRb, Z_3x3, Z_3x3,  //
        // Note(frank): the derivative of n_t with respect to xi is nRb
        // We pre-multiply with nRc' to account for NavState::Create
        // Then we make use of the identity nRc' * nRb = bRc'
        nRcTranspose * D_t_nRb, bRcTranspose, Z_3x3,
        // Similar reasoning for v:
        nRcTranspose * D_v_nRb, Z_3x3, bRcTranspose;
  }
  if (H2) {
    *H2 << D_bRc_xi, Z_3x3, Z_3x3,   //
        Z_3x3, bRcTranspose, Z_3x3,  //
        Z_3x3, Z_3x3, bRcTranspose;
  }
  return NavState(nRc, t, v);
}

//------------------------------------------------------------------------------
Vector9 internal::navStateComponentWiseLocalCoordinates(
    const NavState& state, const NavState& other,
    OptionalJacobian<9, 9> H1, OptionalJacobian<9, 9> H2) {
  Matrix3 D_dR_R, D_dt_R, D_dv_R;
  const Rot3 dR = state.attitude().between(
      other.attitude(), H1 ? &D_dR_R : nullptr);
  const Point3 dP = state.attitude().unrotate(
      other.position() - state.position(), H1 ? &D_dt_R : nullptr);
  const Vector3 dV = state.attitude().unrotate(
      other.velocity() - state.velocity(), H1 ? &D_dv_R : nullptr);

  Vector9 xi;
  Matrix3 D_xi_R;
  xi << Rot3::Logmap(dR, (H1 || H2) ? &D_xi_R : nullptr), dP, dV;
  if (H1) {
    *H1 << D_xi_R * D_dR_R, Z_3x3, Z_3x3,  //
        D_dt_R, -I_3x3, Z_3x3,             //
        D_dv_R, Z_3x3, -I_3x3;
  }
  if (H2) {
    const Matrix3 dRMatrix = dR.matrix();
    *H2 << D_xi_R, Z_3x3, Z_3x3,  //
        Z_3x3, dRMatrix, Z_3x3,   //
        Z_3x3, Z_3x3, dRMatrix;
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
  NavState newState = internal::navStateComponentWiseRetract(
      *this, xi, F, G1 || G2 ? &D_newState_xi : nullptr);

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

#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V43
// Because our navigation frames are placed on a spinning Earth, we experience
// two apparent forces on our inertials. Let Omega be the Earth's rotation rate
// in the navigation frame.
// Coriolis acceleration = -2 * (omega X n_v)
// Centrifugal acceleration (secondOrder) = -omega X (omega X n_t)
// We would also experience a rotation of (omega*dt) over time - so, counteract
// by compensating rotation by (-omega * dt).
// Integrate centrifugal & coriolis accelerations to yield position and velocity
// perturbations.
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
#endif

namespace {

/** Common data and operations for inertial and rotating PIM prediction. */
struct PIMPrediction {
  const Vector9& pim;
  const double dt;
  const Vector3& gravity;

  /** Make the body-frame PIM increment U = (Delta R, Delta p, Delta v). */
  NavState makeU(OptionalJacobian<9, 9> D_U = {}) const {
    Matrix3 D_R;
    const Rot3 deltaR = Rot3::Expmap(NavState::dR(pim), D_U ? &D_R : nullptr);
    if (D_U) {
      const Matrix3 deltaRt = deltaR.transpose();
      *D_U << D_R, Z_3x3, Z_3x3,  //
          Z_3x3, deltaRt, Z_3x3,  //
          Z_3x3, Z_3x3, deltaRt;
    }
    return {deltaR, NavState::dP(pim), NavState::dV(pim)};
  }

  /** Evaluate W * phi_dt(X) * U and differentiate with respect to W, X, U. */
  NavState propagate(const NavState& W, const NavState& X, const NavState& U,
                     OptionalJacobian<9, 9> D_Y_W = {},
                     OptionalJacobian<9, 9> D_Y_X = {},
                     OptionalJacobian<9, 9> D_Y_U = {}) const {
    Matrix9 D_F_X, D_Z_F, D_Z_U, D_Y_Z;
    const NavState::AutonomousFlow phi{dt};
    if (D_Y_X) D_F_X = phi.dIdentity();
    const NavState F = phi(X);
    const NavState Z =
        F.compose(U, D_Y_X ? &D_Z_F : nullptr, D_Y_U ? &D_Z_U : nullptr);
    const NavState Y = W.compose(Z, D_Y_W, (D_Y_X || D_Y_U) ? &D_Y_Z : nullptr);

    if (D_Y_X) *D_Y_X = D_Y_Z * D_Z_F * D_F_X;
    if (D_Y_U) *D_Y_U = D_Y_Z * D_Z_U;
    return Y;
  }
};

/** PIM prediction in the ordinary inertial navigation frame. */
struct PIMPredictionInertial : PIMPrediction {
  /** Make the world increment W, which contains only gravity. */
  NavState makeW() const {
    const double dt2 = dt * dt;
    return {Rot3(), 0.5 * gravity * dt2, gravity * dt};
  }

  NavState predict(const NavState& X, OptionalJacobian<9, 9> H1 = {},
                   OptionalJacobian<9, 9> H2 = {},
                   OptionalJacobian<9, 3> H3 = {}) const {
    // The ordinary path is deliberately a literal reading of
    //
    //                 X_j = W * phi_dt(X_i) * U.
    //
    // W carries world-frame gravity, while U carries the body-frame PIM.
    const NavState W = makeW();
    const NavState U = makeU(H2);
    const Point3 X_p = X.position(), U_p = U.position();
    const Velocity3 X_v = X.velocity(), U_v = U.velocity();
    const Rot3 Y_R = X.attitude().compose(U.attitude());
    const Matrix3 X_R = X.R();
    const NavState Y{Y_R, W.position() + X_p + X_v * dt + X_R * U_p,
                     W.velocity() + X_v + X_R * U_v};

    if (H1) {
      const Matrix3 deltaRt = U.attitude().transpose();
      H1->setZero();
      H1->block<3, 3>(0, 0) = deltaRt;
      H1->block<3, 3>(3, 0) = -deltaRt * skewSymmetric(U_p);
      H1->block<3, 3>(3, 3) = deltaRt;
      H1->block<3, 3>(3, 6) = dt * deltaRt;
      H1->block<3, 3>(6, 0) = -deltaRt * skewSymmetric(U_v);
      H1->block<3, 3>(6, 6) = deltaRt;
    }
    // makeU has already written its own Jacobian directly into H2.
    if (H3) {
      const double dt2 = dt * dt;
      const Matrix3 Y_Rt = Y_R.transpose();
      *H3 << Z_3x3, 0.5 * dt2 * Y_Rt, dt * Y_Rt;
    }
    return Y;
  }
};

/** PIM prediction in a rotating navigation frame. */
struct PIMPredictionRotating : PIMPrediction {
  /** Make the rotating-frame world increment W, including gravity. */
  NavState makeW(const Vector3& omega, OptionalJacobian<9, 3> D_W = {}) const {
    const so3::DexpFunctor earthRotation(-omega * dt);
    const Rot3 A(earthRotation.Rodrigues().left());
    const Matrix3 gammaVelocity = earthRotation.Jacobian().left();

    // Brossard's Gamma^p is J_l(w) - Gamma_2,l(w) in GTSAM's kernel
    // convention. Its limit as omega approaches zero is 1/2 I.
    const Matrix3 gammaPosition = gammaVelocity - earthRotation.Gamma().left();

    const double dt2 = dt * dt;
    if (D_W) {
      // NavState uses right-local position and velocity coordinates, hence
      // the A^T factors in the differential of W.
      *D_W << Z_3x3,  // rotation does not depend on gravity
          A.transpose() * gammaPosition * dt2,
          A.transpose() * gammaVelocity * dt;
    }
    return {A, gammaPosition * gravity * dt2, gammaVelocity * gravity * dt};
  }

  /** Lift physical velocity v to transported velocity v_bar = v + Omega*p. */
  NavState lift(const NavState& X, const Matrix3& omegaCross,
                OptionalJacobian<9, 9> D_L_X = {}) const {
    const Point3 p = X.position();
    if (D_L_X) {
      const Matrix3 R = X.attitude().matrix();
      const Matrix3 omegaBody = R.transpose() * omegaCross * R;
      *D_L_X << I_3x3, Z_3x3, Z_3x3,  //
          Z_3x3, I_3x3, Z_3x3,        //
          Z_3x3, omegaBody, I_3x3;
    }
    return {X.attitude(), p, X.velocity() + omegaCross * p};
  }

  /** Project transported velocity back to physical velocity v. */
  NavState project(const NavState& Y, const Matrix3& omegaCross,
                   OptionalJacobian<9, 9> D_P_Y = {}) const {
    const Point3 p = Y.position();
    if (D_P_Y) {
      const Matrix3 R = Y.attitude().matrix();
      const Matrix3 omegaBody = R.transpose() * omegaCross * R;
      *D_P_Y << I_3x3, Z_3x3, Z_3x3,  //
          Z_3x3, I_3x3, Z_3x3,        //
          Z_3x3, -omegaBody, I_3x3;
    }
    return {Y.attitude(), p, Y.velocity() - omegaCross * p};
  }

  NavState predict(const NavState& X, const Vector3& omega,
                   OptionalJacobian<9, 9> H1 = {},
                   OptionalJacobian<9, 9> H2 = {},
                   OptionalJacobian<9, 3> H3 = {}) const {
    // The same W * phi(L) * U equation applies after lifting the initial state
    // to transported velocity. Projection is the only extra operation:
    //
    //             X_j = P(W * phi_dt(L(X_i)) * U).
    Matrix9 D_U, D_L_X, D_Y_W, D_Y_L, D_Y_U, D_P_Y;
    Matrix93 D_W;
    const Matrix3 omegaCross = skewSymmetric(omega);
    const NavState W = makeW(omega, H3 ? &D_W : nullptr);
    const NavState U = makeU(H2 ? &D_U : nullptr);
    const NavState L = lift(X, omegaCross, H1 ? &D_L_X : nullptr);
    const NavState Y = propagate(W, L, U, H3 ? &D_Y_W : nullptr,
                                 H1 ? &D_Y_L : nullptr, H2 ? &D_Y_U : nullptr);
    const NavState P =
        project(Y, omegaCross, (H1 || H2 || H3) ? &D_P_Y : nullptr);

    // The short D_* names keep the complete chain rule visible next to the
    // equally visible value computation above.
    if (H1) *H1 = D_P_Y * D_Y_L * D_L_X;
    if (H2) *H2 = D_P_Y * D_Y_U * D_U;
    if (H3) *H3 = D_P_Y * D_Y_W * D_W;
    return P;
  }
};

}  // namespace

//------------------------------------------------------------------------------
NavState NavState::predictPIM(const Vector9& pim, double dt,
                              const Vector3& n_gravity,
                              const std::optional<Vector3>& omegaCoriolis,
                              OptionalJacobian<9, 9> H1,
                              OptionalJacobian<9, 9> H2,
                              OptionalJacobian<9, 3> H3) const {
  if (omegaCoriolis && !omegaCoriolis->isZero(0.0)) {
    return PIMPredictionRotating{{pim, dt, n_gravity}}.predict(
        *this, *omegaCoriolis, H1, H2, H3);
  }
  return PIMPredictionInertial{{pim, dt, n_gravity}}.predict(*this, H1, H2, H3);
}

//------------------------------------------------------------------------------
#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V43
Vector9 NavState::correctPIM(const Vector9& pim, double dt,
                             const Vector3& n_gravity,
                             const std::optional<Vector3>& omegaCoriolis,
                             bool /*use2ndOrderCoriolis*/,
                             OptionalJacobian<9, 9> H1,
                             OptionalJacobian<9, 9> H2,
                             OptionalJacobian<9, 3> H3) const {
  if (omegaCoriolis && !omegaCoriolis->isZero(0.0)) {
    Matrix9 predicted_H_state, predicted_H_pim;
    Matrix93 predicted_H_gravity;
    const bool computeJacobians = H1 || H2 || H3;
    const NavState predicted = predictPIM(
        pim, dt, n_gravity, omegaCoriolis, H1 ? &predicted_H_state : nullptr,
        H2 ? &predicted_H_pim : nullptr, H3 ? &predicted_H_gravity : nullptr);

    Matrix9 chart_H_initial, chart_H_predicted;
    const Vector9 result =
        localCoordinates(predicted, H1 ? &chart_H_initial : nullptr,
                         computeJacobians ? &chart_H_predicted : nullptr);
    if (H1) *H1 = chart_H_initial + chart_H_predicted * predicted_H_state;
    if (H2) *H2 = chart_H_predicted * predicted_H_pim;
    if (H3) *H3 = chart_H_predicted * predicted_H_gravity;
    return result;
  }
  const Rot3& nRb = R_;
  const Velocity3 n_v = t_.col(1); // derivative is Ri !
  const double dt22 = 0.5 * dt * dt;

  Vector9 xi;
  Matrix3 D_dP_Ri1, D_dP_Ri2, D_dP_nv;
  // The gravity contributions to the position and velocity rows share both
  // the unrotated vector and the Jacobians wrt rotation and wrt gravity.
  Matrix3 D_bGravity_nGravity;
  const Vector3 b_gravity = nRb.unrotate(n_gravity, H1 ? &D_dP_Ri2 : 0,
                                         H3 ? &D_bGravity_nGravity : 0);
  dR(xi) = dR(pim);
  dP(xi) = dP(pim)
      + dt * nRb.unrotate(n_v, H1 ? &D_dP_Ri1 : 0, H2 ? &D_dP_nv : 0)
      + dt22 * b_gravity;
  dV(xi) = dV(pim) + dt * b_gravity;

  if (H1 || H2 || H3) {
    if (H1) {
      const Matrix3 Ri = nRb.matrix();
      H1->setZero();
      D_t_R(H1) += dt * D_dP_Ri1 + dt22 * D_dP_Ri2;
      D_t_v(H1) += dt * D_dP_nv * Ri;
      D_v_R(H1) += dt * D_dP_Ri2;
    }
    if (H2) {
      H2->setIdentity();
    }
    if (H3) {
      // The rotation rows do not depend on gravity:
      H3->block<3, 3>(0, 0).setZero();
      H3->block<3, 3>(3, 0) = dt22 * D_bGravity_nGravity;
      H3->block<3, 3>(6, 0) = dt * D_bGravity_nGravity;
    }
  }

  return xi;
}
#endif
//------------------------------------------------------------------------------

}/// namespace gtsam

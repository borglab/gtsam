/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  ManifoldPreintegration.cpp
 *  @author Luca Carlone
 *  @author Stephen Williams
 *  @author Richard Roberts
 *  @author Vadim Indelman
 *  @author David Jensen
 *  @author Frank Dellaert
 **/

#include "ManifoldPreintegration.h"

using namespace std;

namespace gtsam {

//------------------------------------------------------------------------------
ManifoldPreintegration::ManifoldPreintegration(
    const std::shared_ptr<Params>& p, const Bias& biasHat) :
    PreintegrationBase(p, biasHat) {
  resetIntegration();
}

//------------------------------------------------------------------------------
void ManifoldPreintegration::resetIntegration() {
  deltaTij_ = 0.0;
  deltaXij_ = NavState();
  delRdelBiasOmega_.setZero();
  delPdelBiasAcc_.setZero();
  delPdelBiasOmega_.setZero();
  delVdelBiasAcc_.setZero();
  delVdelBiasOmega_.setZero();
}

//------------------------------------------------------------------------------
bool ManifoldPreintegration::equals(const ManifoldPreintegration& other,
    double tol) const {
  return p_->equals(*other.p_, tol) && std::abs(deltaTij_ - other.deltaTij_) < tol
      && biasHat_.equals(other.biasHat_, tol)
      && deltaXij_.equals(other.deltaXij_, tol)
      && equal_with_abs_tol(delRdelBiasOmega_, other.delRdelBiasOmega_, tol)
      && equal_with_abs_tol(delPdelBiasAcc_, other.delPdelBiasAcc_, tol)
      && equal_with_abs_tol(delPdelBiasOmega_, other.delPdelBiasOmega_, tol)
      && equal_with_abs_tol(delVdelBiasAcc_, other.delVdelBiasAcc_, tol)
      && equal_with_abs_tol(delVdelBiasOmega_, other.delVdelBiasOmega_, tol);
}

NavState ManifoldPreintegration::UpdatePreintegrated(
    const Eigen::Vector3d& a_body,
    const Eigen::Vector3d& w_body,
    double dt,
    const NavState& X,                       // (R,p,v)
    gtsam::OptionalJacobian<9,9> A_t,        // dXt_{k+1} / dXt_k
    gtsam::OptionalJacobian<9,3> B_t,        // dXt_{k+1} / d a_body
    gtsam::OptionalJacobian<9,3> C_t) {      // dXt_{k+1} / d w_body

  // Let's denote the right perturbation dXn on the NavState X_ij's manifold, i.e.,
  // dXn = [\delta \phi_ij, \delta p_ij, \delta v_ij] where R_ij = \hat{R}_ij Exp(\delta \phi_ij)
  // p_ij = \hat{p}_ij + R_ij \delta p_{ij} and v_ij = \hat{v}_ij + R_ij \delta v_{ij}.
  // The error state dXt of the preint X_ij is actually defined as
  // dXt = [\delta \phi_ij, \delta p_ij, \delta v_ij] where R_ij = \hat{R}_ij Exp(\delta \phi_ij)
  // p_ij = \hat{p}_ij + \delta p_{ij} and v_ij = \hat{v}_ij + \delta v_{ij}.
  // So to propagate the X_ij's covariance, we need to transform the transition matrix A.
  // from NavState.update(), An = \frac{\delta X^n_j}{\delta X^n_{j-1}}, and transform it to
  // At = \frac{\delta X^t_j}{\delta X^t_{j-1}} = \frac{\delta X^t_j}{\delta X^n_j} * An * \frac{\delta X^n_{j-1}}{\delta X^t_{j-1}}
  
  const double dt2  = dt * dt;
  const double dt22 = 0.5 * dt2;

  const gtsam::Rot3& R = X.rotation();
  const Eigen::Matrix3d Rm = R.matrix();

  // Rotation increment
  const Eigen::Vector3d phi = w_body * dt;

  Eigen::Matrix3d Dexp;  // not used directly here, but cheap to compute
  const gtsam::Rot3 dR = gtsam::Rot3::Expmap(phi, (A_t || C_t) ? &Dexp : nullptr);
  const gtsam::Rot3 Rn = R.compose(dR);

  // a_nav uses OLD R (matching typical preintegration / NavState.update structure)
  const Eigen::Vector3d a_nav = R.rotate(a_body);

  const gtsam::Point3  pn = X.position() + X.v() * dt + a_nav * dt22;
  const Eigen::Vector3d vn = X.v() + a_nav * dt;

  NavState Xn(Rn, pn, vn);

  if (A_t) {
    A_t->setZero();

    // dphi+ = dR^T * dphi  (right-perturbation transport)
    A_t->block<3,3>(0,0) = dR.transpose().matrix();

    // dp+ = dp + dt dv + (d/dphi)(0.5*R*a*dt^2)*dphi
    // dv+ = dv            + (d/dphi)(R*a*dt)*dphi
    //
    // Right perturbation: R Exp(dphi) a ≈ R (I + [dphi]x) a
    // so δ(R a) = R (dphi x a) = - R [a]x dphi
    const Eigen::Matrix3d a_skew = skewSymmetric(a_body);
    A_t->block<3,3>(3,0) = -Rm * a_skew * dt22;
    A_t->block<3,3>(6,0) = -Rm * a_skew * dt;

    // world-additive p,v parts
    A_t->block<3,3>(3,3) = Eigen::Matrix3d::Identity();
    A_t->block<3,3>(3,6) = Eigen::Matrix3d::Identity() * dt;
    A_t->block<3,3>(6,6) = Eigen::Matrix3d::Identity();
  }

  if (B_t) {
    B_t->setZero();
    // dp+ / da = 0.5 * R * dt^2, dv+ / da = R * dt
    B_t->block<3,3>(3,0) = Rm * dt22;
    B_t->block<3,3>(6,0) = Rm * dt;
  }

  if (C_t) {
    C_t->setZero();
    // dphi+ / dw = invJr(phi) * dt
    so3::DexpFunctor local(phi);
    const Eigen::Matrix3d invJr = local.InvJacobian().right();
    C_t->block<3,3>(0,0) = invJr * dt;
  }

  return Xn;
}

//------------------------------------------------------------------------------
void ManifoldPreintegration::update(const Vector3& measuredAcc,
    const Vector3& measuredOmega, const double dt, Matrix9* A, Matrix93* B,
    Matrix93* C) {

  // Correct for bias in the sensor frame
  Vector3 acc = biasHat_.correctAccelerometer(measuredAcc);
  Vector3 omega = biasHat_.correctGyroscope(measuredOmega);

  // Possibly correct for sensor pose
  Matrix3 D_correctedAcc_acc, D_correctedAcc_omega, D_correctedOmega_omega;
  if (p().body_P_sensor) {
    std::tie(acc, omega) = correctMeasurementsBySensorPose(
        acc, omega, D_correctedAcc_acc, D_correctedAcc_omega,
        D_correctedOmega_omega);
  }

  // Save current rotation for updating Jacobians
  const Rot3 oldRij = deltaXij_.attitude();

  // Do update
  deltaTij_ += dt;
  deltaXij_ = UpdatePreintegrated(acc, omega, dt, deltaXij_, A, B, C);

  if (p().body_P_sensor) {
    // More complicated derivatives in case of non-trivial sensor pose
    *C *= D_correctedOmega_omega;
    if (!p().body_P_sensor->translation().isZero())
      *C += *B * D_correctedAcc_omega;
    *B *= D_correctedAcc_acc; // NOTE(frank): needs to be last
  }

  // Update Jacobians
  // TODO(frank): Try same simplification as in new approach
  Matrix3 D_acc_R;
  oldRij.rotate(acc, D_acc_R);
  const Matrix3 D_acc_biasOmega = D_acc_R * delRdelBiasOmega_;

  const Vector3 integratedOmega = omega * dt;
  Matrix3 D_incrR_integratedOmega;
  const Rot3 incrR = Rot3::Expmap(integratedOmega, D_incrR_integratedOmega); // expensive !!
  const Matrix3 incrRt = incrR.transpose();
  delRdelBiasOmega_ = incrRt * delRdelBiasOmega_ - D_incrR_integratedOmega * dt;

  double dt22 = 0.5 * dt * dt;
  const Matrix3 dRij = oldRij.matrix(); // expensive
  delPdelBiasAcc_ += delVdelBiasAcc_ * dt - dt22 * dRij;
  delPdelBiasOmega_ += dt * delVdelBiasOmega_ + dt22 * D_acc_biasOmega;
  delVdelBiasAcc_ += -dRij * dt;
  delVdelBiasOmega_ += D_acc_biasOmega * dt;
}

//------------------------------------------------------------------------------
Vector9 ManifoldPreintegration::biasCorrectedDelta(
    const imuBias::ConstantBias& bias_i, OptionalJacobian<9, 6> H) const {
  // Correct deltaRij, derivative is delRdelBiasOmega_
  const imuBias::ConstantBias biasIncr = bias_i - biasHat_;
  Matrix3 D_correctedRij_bias;
  const Vector3 biasInducedOmega = delRdelBiasOmega_ * biasIncr.gyroscope();
  const Rot3 correctedRij = deltaRij().expmap(biasInducedOmega, {},
      H ? &D_correctedRij_bias : 0);
  if (H)
    D_correctedRij_bias *= delRdelBiasOmega_;

  Vector9 xi;
  Matrix3 D_dR_correctedRij;
  // TODO(frank): could line below be simplified? It is equivalent to
  //   LogMap(deltaRij_.compose(Expmap(biasInducedOmega)))
  NavState::dR(xi) = Rot3::Logmap(correctedRij, H ? &D_dR_correctedRij : 0);
  NavState::dP(xi) = deltaPij() + delPdelBiasAcc_ * biasIncr.accelerometer()
      + delPdelBiasOmega_ * biasIncr.gyroscope();
  NavState::dV(xi) = deltaVij() + delVdelBiasAcc_ * biasIncr.accelerometer()
      + delVdelBiasOmega_ * biasIncr.gyroscope();

  if (H) {
    Matrix36 D_dR_bias, D_dP_bias, D_dV_bias;
    D_dR_bias << Z_3x3, D_dR_correctedRij * D_correctedRij_bias;
    D_dP_bias << delPdelBiasAcc_, delPdelBiasOmega_;
    D_dV_bias << delVdelBiasAcc_, delVdelBiasOmega_;
    (*H) << D_dR_bias, D_dP_bias, D_dV_bias;
  }
  return xi;
}

//------------------------------------------------------------------------------

}// namespace gtsam

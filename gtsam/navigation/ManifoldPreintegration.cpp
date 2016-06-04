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
    const boost::shared_ptr<Params>& p, const Bias& biasHat) :
    PreintegrationBase(p, biasHat) {
  resetIntegration();
}

//------------------------------------------------------------------------------
void ManifoldPreintegration::resetIntegration() {
  deltaTij_ = 0.0;
  deltaXij_ = NavState();
  preintegrated_H_biasAcc_.setZero();
  preintegrated_H_biasOmega_.setZero();
  delRdelBiasOmega_.setZero();
  delPdelBiasAcc_.setZero();
  delPdelBiasOmega_.setZero();
  delVdelBiasAcc_.setZero();
  delVdelBiasOmega_.setZero();
}

//------------------------------------------------------------------------------
bool ManifoldPreintegration::equals(const ManifoldPreintegration& other,
    double tol) const {
  return p_->equals(*other.p_, tol) && fabs(deltaTij_ - other.deltaTij_) < tol
      && biasHat_.equals(other.biasHat_, tol)
      && deltaXij_.equals(other.deltaXij_, tol)
      && equal_with_abs_tol(delRdelBiasOmega_, other.delRdelBiasOmega_, tol)
      && equal_with_abs_tol(delPdelBiasAcc_, other.delPdelBiasAcc_, tol)
      && equal_with_abs_tol(delPdelBiasOmega_, other.delPdelBiasOmega_, tol)
      && equal_with_abs_tol(delVdelBiasAcc_, other.delVdelBiasAcc_, tol)
      && equal_with_abs_tol(delVdelBiasOmega_, other.delVdelBiasOmega_, tol);
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
  if (p().body_P_sensor)
    boost::tie(acc, omega) = correctMeasurementsBySensorPose(acc, omega,
        D_correctedAcc_acc, D_correctedAcc_omega, D_correctedOmega_omega);

  // Save current rotation for updating Jacobians
  const Rot3 oldRij = deltaXij_.attitude();

  // Do update
  deltaTij_ += dt;
  deltaXij_ = deltaXij_.update(acc, omega, dt, A, B, C); // functional

  if (p().body_P_sensor) {
    // More complicated derivatives in case of non-trivial sensor pose
    *C *= D_correctedOmega_omega;
    if (!p().body_P_sensor->translation().isZero())
      *C += *B * D_correctedAcc_omega;
    *B *= D_correctedAcc_acc; // NOTE(frank): needs to be last
  }

  // new_H_biasAcc = new_H_old * old_H_biasAcc + new_H_acc * acc_H_biasAcc
  // where acc_H_biasAcc = -I_3x3, hence
  // new_H_biasAcc = new_H_old * old_H_biasAcc - new_H_acc
  preintegrated_H_biasAcc_ = (*A) * preintegrated_H_biasAcc_ - (*B);

  // new_H_biasOmega = new_H_old * old_H_biasOmega + new_H_omega * omega_H_biasOmega
  // where omega_H_biasOmega = -I_3x3, hence
  // new_H_biasOmega = new_H_old * old_H_biasOmega - new_H_omega
  preintegrated_H_biasOmega_ = (*A) * preintegrated_H_biasOmega_ - (*C);

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
  const Vector9 delta_xi =
        preintegrated_H_biasAcc_ * biasIncr.accelerometer()
      + preintegrated_H_biasOmega_ * biasIncr.gyroscope();

  Matrix3 D_correctedRij_deltaRxi;
  const Rot3 correctedRij = deltaRij().expmap(NavState::dR(delta_xi),
      boost::none, H ? &D_correctedRij_deltaRxi : 0);

//  TIE(nRb, n_t, n_v, *this);
//  const Rot3 bRc = Rot3::Expmap(dR(xi));
//  return NavState(nRb * bRc, n_t + nRb * dP(xi), n_v + nRb * dV(xi));

  Vector9 xi;
  Matrix3 D_dR_correctedRij;
  // TODO(frank): could line below be simplified? It is equivalent to
  //   LogMap(deltaRij_.compose(Expmap(biasInducedOmega)))
  NavState::dR(xi) = Rot3::Logmap(correctedRij, H ? &D_dR_correctedRij : 0);
  NavState::dP(xi) = deltaPij() + deltaRij() * NavState::dP(delta_xi);
  NavState::dV(xi) = deltaVij() + deltaRij() * NavState::dV(delta_xi);

  if (H) {
    (*H) << preintegrated_H_biasAcc_, preintegrated_H_biasOmega_;
    // NOTE(frank): correct for the Logmap and Expmap in rotational part
    H->block<3,3>(0,3) = D_dR_correctedRij * D_correctedRij_deltaRxi * H->block<3,3>(0,3);
  }
  return xi;
}

//------------------------------------------------------------------------------

}// namespace gtsam

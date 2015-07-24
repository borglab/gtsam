/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  PreintegrationBase.h
 *  @author Luca Carlone
 *  @author Stephen Williams
 *  @author Richard Roberts
 *  @author Vadim Indelman
 *  @author David Jensen
 *  @author Frank Dellaert
 **/

#include "PreintegrationBase.h"
#include <boost/make_shared.hpp>

using namespace std;

namespace gtsam {

/// Re-initialize PreintegratedMeasurements
void PreintegrationBase::resetIntegration() {
  PreintegratedRotation::resetIntegration();
  deltaPij_ = Vector3::Zero();
  deltaVij_ = Vector3::Zero();
  delPdelBiasAcc_ = Z_3x3;
  delPdelBiasOmega_ = Z_3x3;
  delVdelBiasAcc_ = Z_3x3;
  delVdelBiasOmega_ = Z_3x3;
}

/// Needed for testable
void PreintegrationBase::print(const string& s) const {
  PreintegratedRotation::print(s);
  cout << "    deltaPij [ " << deltaPij_.transpose() << " ]" << endl;
  cout << "    deltaVij [ " << deltaVij_.transpose() << " ]" << endl;
  biasHat_.print("    biasHat");
}

/// Needed for testable
bool PreintegrationBase::equals(const PreintegrationBase& other,
    double tol) const {
  return PreintegratedRotation::equals(other, tol)
      && biasHat_.equals(other.biasHat_, tol)
      && equal_with_abs_tol(deltaPij_, other.deltaPij_, tol)
      && equal_with_abs_tol(deltaVij_, other.deltaVij_, tol)
      && equal_with_abs_tol(delPdelBiasAcc_, other.delPdelBiasAcc_, tol)
      && equal_with_abs_tol(delPdelBiasOmega_, other.delPdelBiasOmega_, tol)
      && equal_with_abs_tol(delVdelBiasAcc_, other.delVdelBiasAcc_, tol)
      && equal_with_abs_tol(delVdelBiasOmega_, other.delVdelBiasOmega_, tol);
}

/// Update preintegrated measurements
void PreintegrationBase::updatePreintegratedMeasurements(
    const Vector3& correctedAcc, const Rot3& incrR, const double deltaT,
    OptionalJacobian<9, 9> F) {

  const Matrix3 dRij = deltaRij_.matrix(); // expensive
  const Vector3 j_acc = dRij * correctedAcc; // acceleration in current frame

  double dt22 = 0.5 * deltaT * deltaT;
  deltaPij_ += deltaVij_ * deltaT + dt22 * j_acc;
  deltaVij_ += deltaT * j_acc;

  Matrix3 R_i, F_angles_angles;
  if (F)
    R_i = dRij; // has to be executed before updateIntegratedRotationAndDeltaT as that updates deltaRij
  updateIntegratedRotationAndDeltaT(incrR, deltaT, F ? &F_angles_angles : 0);

  if (F) {
    const Matrix3 F_vel_angles = -R_i * skewSymmetric(correctedAcc) * deltaT;
    Matrix3 F_pos_angles;
    F_pos_angles = 0.5 * F_vel_angles * deltaT;

    //    pos  vel             angle
    *F << //
        I_3x3, I_3x3 * deltaT, F_pos_angles, // pos
    Z_3x3, I_3x3, F_vel_angles, // vel
    Z_3x3, Z_3x3, F_angles_angles; // angle
  }
}

/// Update Jacobians to be used during preintegration
void PreintegrationBase::updatePreintegratedJacobians(
    const Vector3& correctedAcc, const Matrix3& D_Rincr_integratedOmega,
    const Rot3& incrR, double deltaT) {
  const Matrix3 dRij = deltaRij_.matrix(); // expensive
  const Matrix3 temp = -dRij * skewSymmetric(correctedAcc) * deltaT
      * delRdelBiasOmega_;
  delPdelBiasAcc_ += delVdelBiasAcc_ * deltaT - 0.5 * dRij * deltaT * deltaT;
  delPdelBiasOmega_ += deltaT * (delVdelBiasOmega_ + temp * 0.5);
  delVdelBiasAcc_ += -dRij * deltaT;
  delVdelBiasOmega_ += temp;
  update_delRdelBiasOmega(D_Rincr_integratedOmega, incrR, deltaT);
}

void PreintegrationBase::correctMeasurementsByBiasAndSensorPose(
    const Vector3& measuredAcc, const Vector3& measuredOmega,
    Vector3* correctedAcc, Vector3* correctedOmega) {
  *correctedAcc = biasHat_.correctAccelerometer(measuredAcc);
  *correctedOmega = biasHat_.correctGyroscope(measuredOmega);

  // Then compensate for sensor-body displacement: we express the quantities
  // (originally in the IMU frame) into the body frame
  if (p().body_P_sensor) {
    Matrix3 body_R_sensor = p().body_P_sensor->rotation().matrix();
    *correctedOmega = body_R_sensor * (*correctedOmega); // rotation rate vector in the body frame
    Matrix3 body_omega_body__cross = skewSymmetric(*correctedOmega);
    *correctedAcc = body_R_sensor * (*correctedAcc)
        - body_omega_body__cross * body_omega_body__cross
            * p().body_P_sensor->translation().vector();
    // linear acceleration vector in the body frame
  }
}

//------------------------------------------------------------------------------
Vector9 PreintegrationBase::biasCorrectedDelta(
    const imuBias::ConstantBias& bias_i, OptionalJacobian<9, 6> H) const {
  // Correct deltaRij, derivative is delRdelBiasOmega_
  const imuBias::ConstantBias biasIncr = bias_i - biasHat_;
  Rot3 deltaRij = biascorrectedDeltaRij(biasIncr.gyroscope());

  Vector9 xi;
  Matrix3 D_dR_deltaRij;
  NavState::dR(xi) = Rot3::Logmap(deltaRij, H ? &D_dR_deltaRij : 0);
  NavState::dP(xi) = deltaPij_ + delPdelBiasAcc_ * biasIncr.accelerometer()
      + delPdelBiasOmega_ * biasIncr.gyroscope();
  NavState::dV(xi) = deltaVij_ + delVdelBiasAcc_ * biasIncr.accelerometer()
      + delVdelBiasOmega_ * biasIncr.gyroscope();

  if (H) {
    Matrix36 D_dR_bias, D_dP_bias, D_dV_bias;
    D_dR_bias << Z_3x3, D_dR_deltaRij * delRdelBiasOmega_;
    D_dP_bias << delPdelBiasAcc_, delPdelBiasOmega_;
    D_dV_bias << delVdelBiasAcc_, delVdelBiasOmega_;
    (*H) << D_dR_bias, D_dP_bias, D_dV_bias;
  }
  return xi;
}

//------------------------------------------------------------------------------
NavState PreintegrationBase::predict(const NavState& state_i,
    const imuBias::ConstantBias& bias_i, OptionalJacobian<9, 9> H1,
    OptionalJacobian<9, 6> H2) const {
  // correct for bias
  Matrix96 D_biasCorrected_bias;
  Vector9 biasCorrected = biasCorrectedDelta(bias_i,
      H2 ? &D_biasCorrected_bias : 0);

  // integrate on tangent space
  Matrix9 D_delta_state, D_delta_biasCorrected;
  Vector9 xi = state_i.correctPIM(biasCorrected, deltaTij_, p().n_gravity,
      p().omegaCoriolis, p().use2ndOrderCoriolis, H1 ? &D_delta_state : 0,
      H2 ? &D_delta_biasCorrected : 0);

  // Use retract to get back to NavState manifold
  Matrix9 D_predict_state, D_predict_delta;
  NavState state_j = state_i.retract(xi, D_predict_state, D_predict_delta);
  if (H1)
    *H1 = D_predict_state + D_predict_delta * D_delta_state;
  if (H2)
    *H2 = D_predict_delta * D_delta_biasCorrected * D_biasCorrected_bias;
  return state_j;
}

//------------------------------------------------------------------------------
// TODO(frank): this is *almost* state_j.localCoordinates(predict),
// except for the damn Ri.transpose. Ri is also the only way this depends on state_i.
// That is not an accident! Put R in computed covariances instead ?
static Vector9 computeError(const NavState& state_i, const NavState& state_j,
    const NavState& predictedState_j) {

  const Rot3& rot_i = state_i.attitude();
  const Matrix Ri = rot_i.matrix();

  // Residual rotation error
  // TODO: this also seems to be flipped from localCoordinates
  const Rot3 fRrot = predictedState_j.attitude().between(state_j.attitude());
  const Vector3 fR = Rot3::Logmap(fRrot);

  // Evaluate residual error, according to [3]
  // Ri.transpose() is important here to preserve a model with *additive* Gaussian noise of correct covariance
  const Vector3 fp = Ri.transpose()
      * (state_j.position() - predictedState_j.position()).vector();

  // Ri.transpose() is important here to preserve a model with *additive* Gaussian noise of correct covariance
  const Vector3 fv = Ri.transpose()
      * (state_j.velocity() - predictedState_j.velocity());

  Vector9 r;
  r << fR, fp, fv;
  return r;
  // return state_j.localCoordinates(predictedState_j);
}

//------------------------------------------------------------------------------
Vector9 PreintegrationBase::computeErrorAndJacobians(const Pose3& pose_i,
    const Vector3& vel_i, const Pose3& pose_j, const Vector3& vel_j,
    const imuBias::ConstantBias& bias_i, OptionalJacobian<9, 6> H1,
    OptionalJacobian<9, 3> H2, OptionalJacobian<9, 6> H3,
    OptionalJacobian<9, 3> H4, OptionalJacobian<9, 6> H5) const {

  // we give some shorter name to rotations and translations
  const Rot3& rot_i = pose_i.rotation();
  const Matrix Ri = rot_i.matrix();
  NavState state_i(pose_i, vel_i);

  const Rot3& rot_j = pose_j.rotation();
  const Vector3 pos_j = pose_j.translation().vector();
  NavState state_j(pose_j, vel_j);

  // Compute bias-corrected quantities
  // TODO(frank): now redundant with biasCorrected below
  Matrix96 D_biasCorrected_bias;
  Vector9 biasCorrected = biasCorrectedDelta(bias_i, D_biasCorrected_bias);

  /// Predict state at time j
  Matrix99 D_predict_state;
  Matrix96 D_predict_bias;
  NavState predictedState_j = predict(state_i, bias_i, D_predict_state,
      D_predict_bias);

  // Evaluate residual error, according to [3]
  // Ri.transpose() is important here to preserve a model with *additive* Gaussian noise of correct covariance
  const Vector3 fp = Ri.transpose()
      * (pos_j - predictedState_j.pose().translation().vector());

  // Ri.transpose() is important here to preserve a model with *additive* Gaussian noise of correct covariance
  const Vector3 fv = Ri.transpose() * (vel_j - predictedState_j.velocity());

  // fR will be computed later.
  // Note: it is the same as: fR = predictedState_j.pose.rotation().between(Rot_j)

  // Coriolis term, NOTE inconsistent with AHRS, where coriolisHat is *after* integration
  // TODO(frank): move derivatives to predict and do coriolis branching there
  const Vector3 coriolis = PreintegratedRotation::integrateCoriolis(rot_i);
  const Vector3 correctedOmega = NavState::dR(biasCorrected) - coriolis;

  // Residual rotation error
  Matrix3 D_cDeltaRij_cOmega;
  const Rot3 correctedDeltaRij = Rot3::Expmap(correctedOmega,
      H1 || H5 ? &D_cDeltaRij_cOmega : 0);
  const Rot3 RiBetweenRj = rot_i.between(rot_j);
  const Rot3 fRrot = correctedDeltaRij.between(RiBetweenRj);
  Matrix3 D_fR_fRrot;
  Rot3::Logmap(fRrot, H1 || H3 || H5 ? &D_fR_fRrot : 0);

  const double dt = deltaTij_, dt2 = dt * dt;
  Matrix3 RitOmegaCoriolisHat = Z_3x3;
  if ((H1 || H2) && p().omegaCoriolis)
    RitOmegaCoriolisHat = Ri.transpose() * skewSymmetric(*p().omegaCoriolis);

  if (H1) {
    const Matrix3 D_coriolis = -D_cDeltaRij_cOmega * skewSymmetric(coriolis);
    Matrix3 dfPdPi = -I_3x3, dfVdPi = Z_3x3;
    if (p().use2ndOrderCoriolis) {
      // this is the same as: Ri.transpose() * p().omegaCoriolisHat * p().omegaCoriolisHat * Ri
      const Matrix3 temp = RitOmegaCoriolisHat
          * (-RitOmegaCoriolisHat.transpose());
      dfPdPi += 0.5 * temp * dt2;
      dfVdPi += temp * dt;
    }
    (*H1)
        << D_fR_fRrot
            * (-rot_j.between(rot_i).matrix()
                - fRrot.inverse().matrix() * D_coriolis), // dfR/dRi
    Z_3x3, // dfR/dPi
    skewSymmetric(fp + NavState::dP(biasCorrected)), // dfP/dRi
    dfPdPi, // dfP/dPi
    skewSymmetric(fv + NavState::dV(biasCorrected)), // dfV/dRi
    dfVdPi; // dfV/dPi
  }
  if (H2) {
    (*H2) << Z_3x3, // dfR/dVi
    -Ri.transpose() * dt + RitOmegaCoriolisHat * dt2, // dfP/dVi
    -Ri.transpose() + 2 * RitOmegaCoriolisHat * dt; // dfV/dVi
  }
  if (H3) {
    (*H3) << D_fR_fRrot, Z_3x3, // dfR/dPosej
    Z_3x3, Ri.transpose() * rot_j.matrix(), // dfP/dPosej
    Matrix::Zero(3, 6); // dfV/dPosej
  }
  if (H4) {
    (*H4) << Z_3x3, // dfR/dVj
    Z_3x3, // dfP/dVj
    Ri.transpose(); // dfV/dVj
  }
  if (H5) {
    const Matrix36 JbiasOmega = D_cDeltaRij_cOmega
        * D_biasCorrected_bias.middleRows<3>(0);
    (*H5) << -D_fR_fRrot * fRrot.inverse().matrix() * JbiasOmega, // dfR/dBias
    -D_biasCorrected_bias.middleRows<3>(3), // dfP/dBias
    -D_biasCorrected_bias.middleRows<3>(6); // dfV/dBias
  }
  // TODO(frank): Do everything via derivatives of function below
  return computeError(state_i, state_j, predictedState_j);
}

//------------------------------------------------------------------------------
PoseVelocityBias PreintegrationBase::predict(const Pose3& pose_i,
    const Vector3& vel_i, const imuBias::ConstantBias& bias_i,
    const Vector3& n_gravity, const Vector3& omegaCoriolis,
    const bool use2ndOrderCoriolis) {
  // NOTE(frank): parameters are supposed to be constant, below is only provided for compatibility
  boost::shared_ptr<Params> q = boost::make_shared<Params>(p());
  q->n_gravity = n_gravity;
  q->omegaCoriolis = omegaCoriolis;
  q->use2ndOrderCoriolis = use2ndOrderCoriolis;
  p_ = q;
  return PoseVelocityBias(predict(NavState(pose_i, vel_i), bias_i), bias_i);
}
} /// namespace gtsam

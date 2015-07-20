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
bool PreintegrationBase::equals(const PreintegrationBase& other, double tol) const {
  return PreintegratedRotation::equals(other, tol) &&
         biasHat_.equals(other.biasHat_, tol) &&
         equal_with_abs_tol(deltaPij_, other.deltaPij_, tol) &&
         equal_with_abs_tol(deltaVij_, other.deltaVij_, tol) &&
         equal_with_abs_tol(delPdelBiasAcc_, other.delPdelBiasAcc_, tol) &&
         equal_with_abs_tol(delPdelBiasOmega_, other.delPdelBiasOmega_, tol) &&
         equal_with_abs_tol(delVdelBiasAcc_, other.delVdelBiasAcc_, tol) &&
         equal_with_abs_tol(delVdelBiasOmega_, other.delVdelBiasOmega_, tol);
}

/// Update preintegrated measurements
void PreintegrationBase::updatePreintegratedMeasurements(const Vector3& correctedAcc,
                                                         const Rot3& incrR, const double deltaT,
                                                         OptionalJacobian<9, 9> F) {

  const Matrix3 dRij = deltaRij().matrix();  // expensive
  const Vector3 temp = dRij * correctedAcc * deltaT;

  if (!p().use2ndOrderIntegration) {
    deltaPij_ += deltaVij_ * deltaT;
  } else {
    deltaPij_ += deltaVij_ * deltaT + 0.5 * temp * deltaT;
  }
  deltaVij_ += temp;

  Matrix3 R_i, F_angles_angles;
  if (F)
    R_i = dRij;  // has to be executed before updateIntegratedRotationAndDeltaT as that updates deltaRij
  updateIntegratedRotationAndDeltaT(incrR, deltaT, F ? &F_angles_angles : 0);

  if (F) {
    const Matrix3 F_vel_angles = -R_i * skewSymmetric(correctedAcc) * deltaT;
    Matrix3 F_pos_angles;
    if (p().use2ndOrderIntegration)
      F_pos_angles = 0.5 * F_vel_angles * deltaT;
    else
      F_pos_angles = Z_3x3;

    //    pos  vel             angle
    *F <<  //
        I_3x3, I_3x3 * deltaT, F_pos_angles,    // pos
        Z_3x3, I_3x3,          F_vel_angles,    // vel
        Z_3x3, Z_3x3,          F_angles_angles; // angle
  }
}

/// Update Jacobians to be used during preintegration
void PreintegrationBase::updatePreintegratedJacobians(const Vector3& correctedAcc,
                                                      const Matrix3& D_Rincr_integratedOmega,
                                                      const Rot3& incrR, double deltaT) {
  const Matrix3 dRij = deltaRij().matrix();  // expensive
  const Matrix3 temp = -dRij * skewSymmetric(correctedAcc) * deltaT * delRdelBiasOmega();
  if (!p().use2ndOrderIntegration) {
    delPdelBiasAcc_ += delVdelBiasAcc_ * deltaT;
    delPdelBiasOmega_ += delVdelBiasOmega_ * deltaT;
  } else {
    delPdelBiasAcc_ += delVdelBiasAcc_ * deltaT - 0.5 * dRij * deltaT * deltaT;
    delPdelBiasOmega_ += deltaT * (delVdelBiasOmega_ + temp * 0.5);
  }
  delVdelBiasAcc_ += -dRij * deltaT;
  delVdelBiasOmega_ += temp;
  update_delRdelBiasOmega(D_Rincr_integratedOmega, incrR, deltaT);
}

void PreintegrationBase::correctMeasurementsByBiasAndSensorPose(
    const Vector3& measuredAcc, const Vector3& measuredOmega, Vector3* correctedAcc,
    Vector3* correctedOmega) {
  *correctedAcc = biasHat_.correctAccelerometer(measuredAcc);
  *correctedOmega = biasHat_.correctGyroscope(measuredOmega);

  // Then compensate for sensor-body displacement: we express the quantities
  // (originally in the IMU frame) into the body frame
  if (p().body_P_sensor) {
    Matrix3 body_R_sensor = p().body_P_sensor->rotation().matrix();
    *correctedOmega = body_R_sensor * (*correctedOmega);  // rotation rate vector in the body frame
    Matrix3 body_omega_body__cross = skewSymmetric(*correctedOmega);
    *correctedAcc = body_R_sensor * (*correctedAcc)
        - body_omega_body__cross * body_omega_body__cross * p().body_P_sensor->translation().vector();
    // linear acceleration vector in the body frame
  }
}

//------------------------------------------------------------------------------
Vector9 PreintegrationBase::biasCorrectedDelta(
    const imuBias::ConstantBias& bias_i, OptionalJacobian<9, 6> H) const {
  const imuBias::ConstantBias biasIncr = bias_i - biasHat_;
  Matrix3 D_deltaRij_bias;
  Rot3 deltaRij = PreintegratedRotation::biascorrectedDeltaRij(
      biasIncr.gyroscope(), H ? &D_deltaRij_bias : 0);

  Vector9 delta;
  Matrix3 D_dR_deltaRij;
  NavState::dR(delta) = Rot3::Logmap(deltaRij, H ? &D_dR_deltaRij : 0);
  NavState::dP(delta) = deltaPij_ + delPdelBiasAcc_ * biasIncr.accelerometer()
      + delPdelBiasOmega_ * biasIncr.gyroscope();
  NavState::dV(delta) = deltaVij_ + delVdelBiasAcc_ * biasIncr.accelerometer()
      + delVdelBiasOmega_ * biasIncr.gyroscope();
  if (H) {
    Matrix36 D_dR_bias, D_dP_bias, D_dV_bias;
    D_dR_bias << Z_3x3, D_dR_deltaRij * D_deltaRij_bias;
    D_dP_bias << delPdelBiasAcc_, delPdelBiasOmega_;
    D_dV_bias << delVdelBiasAcc_, delVdelBiasOmega_;
    (*H) << D_dR_bias, D_dP_bias, D_dV_bias;
  }
  return delta;
}

//------------------------------------------------------------------------------
static Vector3 rotate(const Matrix3& R, const Vector3& p,
    OptionalJacobian<3, 3> H1 = boost::none, OptionalJacobian<3, 3> H2 = boost::none) {
  if (H1) *H1 = R * skewSymmetric(-p.x(), -p.y(), -p.z());
  if (H2) *H2 = R;
  return R * p;
}

//------------------------------------------------------------------------------
static Vector3 unrotate(const Matrix3& R, const Vector3& p,
    OptionalJacobian<3, 3> H1 = boost::none, OptionalJacobian<3, 3> H2 = boost::none) {
  const Matrix3 Rt = R.transpose();
  Vector3 q = Rt * p;
  const double wx = q.x(), wy = q.y(), wz = q.z();
  if (H1) *H1 << 0.0, -wz, +wy, +wz, 0.0, -wx, -wy, +wx, 0.0;
  if (H2) *H2 = Rt;
  return q;
}

//------------------------------------------------------------------------------
Vector9 PreintegrationBase::integrateCoriolis(const NavState& state_i,
    OptionalJacobian<9, 9> H) const {
  Vector9 result = Vector9::Zero();
  if (H) H->setZero();
  if (p().omegaCoriolis) {
    const Pose3& pose_i = state_i.pose();
    const Vector3& vel_i = state_i.velocity();
    const Matrix3 Ri = pose_i.rotation().matrix();
    const Vector3& t_i = state_i.translation().vector();
    const double dt = deltaTij(), dt2 = dt * dt;

    const Vector3& omegaCoriolis = *p().omegaCoriolis;
    Matrix3 D_dP_Ri;
    NavState::dR(result) -= unrotate(Ri, omegaCoriolis * dt, H ? &D_dP_Ri : 0);
    NavState::dP(result) -= omegaCoriolis.cross(vel_i) * dt2; // NOTE(luca): we got rid of the 2 wrt INS paper
    NavState::dV(result) -= 2.0 * omegaCoriolis.cross(vel_i) * dt;
    if (p().use2ndOrderCoriolis) {
      Vector3 temp = omegaCoriolis.cross(omegaCoriolis.cross(t_i));
      NavState::dP(result) -= 0.5 * temp * dt2;
      NavState::dV(result) -= temp * dt;
    }
    if (H) {
      const Matrix3 omegaCoriolisHat = skewSymmetric(omegaCoriolis);
      H->block<3,3>(0,0) -= D_dP_Ri;
      H->block<3,3>(3,6) -= omegaCoriolisHat * dt2;
      H->block<3,3>(6,6) -= 2.0 * omegaCoriolisHat * dt;
      if (p().use2ndOrderCoriolis) {
        const Matrix3 omegaCoriolisHat2 = omegaCoriolisHat * omegaCoriolisHat;
        H->block<3,3>(3,3) -= 0.5 * omegaCoriolisHat2 * dt2;
        H->block<3,3>(6,3) -= omegaCoriolisHat2 * dt;
      }
    }
  }
  return result;
}

//------------------------------------------------------------------------------
Vector9 PreintegrationBase::recombinedPrediction(const NavState& state_i,
    const Vector9& biasCorrectedDelta, OptionalJacobian<9, 9> H1,
    OptionalJacobian<9, 9> H2) const {

  const Pose3& pose_i = state_i.pose();
  const Vector3& vel_i = state_i.velocity();
  const Matrix3 Ri = pose_i.rotation().matrix();
  const double dt = deltaTij(), dt2 = dt * dt;

  // Rotation, translation, and velocity:
  Vector9 delta;
  Matrix3 D_dP_Ri, D_dP_bc, D_dV_Ri, D_dV_bc;
  NavState::dR(delta) = NavState::dR(biasCorrectedDelta);
  NavState::dP(delta) = rotate(Ri, NavState::dP(biasCorrectedDelta), D_dP_Ri, D_dP_bc) + vel_i * dt + 0.5 * p().gravity * dt2;
  NavState::dV(delta) = rotate(Ri, NavState::dV(biasCorrectedDelta), D_dV_Ri, D_dV_bc) + p().gravity * dt;

  Matrix9 Hcoriolis;
  if (p().omegaCoriolis) {
    delta += integrateCoriolis(state_i, H1 ? &Hcoriolis : 0);
  }
  if (H1) {
    H1->setZero();
    H1->block<3,3>(3,0) = D_dP_Ri;
    H1->block<3,3>(3,6) = I_3x3 * dt;
    H1->block<3,3>(6,0) = D_dV_Ri;
    if (p().omegaCoriolis) *H1 += Hcoriolis;
  }
  if (H2) {
    H2->setZero();
    H2->block<3,3>(0,0) = I_3x3;
    H2->block<3,3>(3,3) = Ri;
    H2->block<3,3>(6,6) = Ri;
  }

  return delta;
}

//------------------------------------------------------------------------------
NavState PreintegrationBase::predict(const NavState& state_i,
    const imuBias::ConstantBias& bias_i, OptionalJacobian<9, 9> H1,
    OptionalJacobian<9, 6> H2) const {
  Matrix96 D_biasCorrected_bias;
  Vector9 biasCorrected = biasCorrectedDelta(bias_i,
      H2 ? &D_biasCorrected_bias : 0);
  Matrix9 D_delta_state, D_delta_biasCorrected;
  Vector9 delta = recombinedPrediction(state_i, biasCorrected,
      H1 ? &D_delta_state : 0, H2 ? &D_delta_biasCorrected : 0);
  Matrix9 D_predict_state, D_predict_delta;
  NavState state_j = state_i.retract(delta, D_predict_state, D_predict_delta);
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

  const Rot3& rot_i = state_i.rotation();
  const Matrix Ri = rot_i.matrix();

  // Residual rotation error
  // TODO: this also seems to be flipped from localCoordinates
  const Rot3 fRrot = predictedState_j.rotation().between(state_j.rotation());
  const Vector3 fR = Rot3::Logmap(fRrot);

  // Evaluate residual error, according to [3]
  // Ri.transpose() is important here to preserve a model with *additive* Gaussian noise of correct covariance
  const Vector3 fp = Ri.transpose()
      * (state_j.translation() - predictedState_j.translation()).vector();

  // Ri.transpose() is important here to preserve a model with *additive* Gaussian noise of correct covariance
  const Vector3 fv = Ri.transpose()
      * (state_j.velocity() - predictedState_j.velocity());

  Vector9 r;
  r << fR, fp, fv;
  return r;
  // return state_j.localCoordinates(predictedState_j);
}

//------------------------------------------------------------------------------
Vector9 PreintegrationBase::computeErrorAndJacobians(const Pose3& pose_i, const Vector3& vel_i,
                                                     const Pose3& pose_j, const Vector3& vel_j,
                                                     const imuBias::ConstantBias& bias_i,
                                                     OptionalJacobian<9, 6> H1,
                                                     OptionalJacobian<9, 3> H2,
                                                     OptionalJacobian<9, 6> H3,
                                                     OptionalJacobian<9, 3> H4,
                                                     OptionalJacobian<9, 6> H5) const {

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
  NavState predictedState_j = predict(state_i, bias_i, D_predict_state, D_predict_bias);

  // Evaluate residual error, according to [3]
  // Ri.transpose() is important here to preserve a model with *additive* Gaussian noise of correct covariance
  const Vector3 fp = Ri.transpose() * (pos_j - predictedState_j.pose().translation().vector());

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
  const Rot3 correctedDeltaRij = Rot3::Expmap(correctedOmega, H1 || H5 ? &D_cDeltaRij_cOmega : 0);
  const Rot3 RiBetweenRj = rot_i.between(rot_j);
  const Rot3 fRrot = correctedDeltaRij.between(RiBetweenRj);
  Matrix3 D_fR_fRrot;
  Rot3::Logmap(fRrot, H1 || H3 || H5 ? &D_fR_fRrot : 0);

  const double dt = deltaTij(), dt2 = dt*dt;
  Matrix3 RitOmegaCoriolisHat = Z_3x3;
  if ((H1 || H2) && p().omegaCoriolis)
    RitOmegaCoriolisHat =  Ri.transpose() * skewSymmetric(*p().omegaCoriolis);

  if (H1) {
    const Matrix3 D_coriolis = -D_cDeltaRij_cOmega * skewSymmetric(coriolis);
    Matrix3 dfPdPi = -I_3x3, dfVdPi = Z_3x3;
    if (p().use2ndOrderCoriolis) {
      // this is the same as: Ri.transpose() * p().omegaCoriolisHat * p().omegaCoriolisHat * Ri
      const Matrix3 temp = RitOmegaCoriolisHat * (-RitOmegaCoriolisHat.transpose());
      dfPdPi += 0.5 * temp * dt2;
      dfVdPi += temp * dt;
    }
    (*H1) <<
    D_fR_fRrot * (-rot_j.between(rot_i).matrix() - fRrot.inverse().matrix() * D_coriolis),  // dfR/dRi
    Z_3x3,                                                                                  // dfR/dPi
    skewSymmetric(fp + NavState::dP(biasCorrected)),                                        // dfP/dRi
    dfPdPi,                                                                                 // dfP/dPi
    skewSymmetric(fv + NavState::dV(biasCorrected)),                                        // dfV/dRi
    dfVdPi;                                                                                 // dfV/dPi
  }
  if (H2) {
    (*H2) <<
    Z_3x3,                                                      // dfR/dVi
    -Ri.transpose() * dt + RitOmegaCoriolisHat * dt2,  // dfP/dVi
    -Ri.transpose() + 2 * RitOmegaCoriolisHat * dt;    // dfV/dVi
  }
  if (H3) {
    (*H3) <<
    D_fR_fRrot, Z_3x3,                      // dfR/dPosej
    Z_3x3, Ri.transpose() * rot_j.matrix(), // dfP/dPosej
    Matrix::Zero(3, 6);                     // dfV/dPosej
  }
  if (H4) {
    (*H4) <<
    Z_3x3,          // dfR/dVj
    Z_3x3,          // dfP/dVj
    Ri.transpose(); // dfV/dVj
  }
  if (H5) {
    const Matrix36 JbiasOmega = D_cDeltaRij_cOmega * D_biasCorrected_bias.middleRows<3>(0);
    (*H5) <<
    -D_fR_fRrot * fRrot.inverse().matrix() * JbiasOmega,  // dfR/dBias
    -D_biasCorrected_bias.middleRows<3>(3),               // dfP/dBias
    -D_biasCorrected_bias.middleRows<3>(6);               // dfV/dBias
  }
  // TODO(frank): Do everything via derivatives of function below
  return computeError(state_i, state_j, predictedState_j);
}

//------------------------------------------------------------------------------
PoseVelocityBias PreintegrationBase::predict(const Pose3& pose_i, const Vector3& vel_i,
    const imuBias::ConstantBias& bias_i, const Vector3& gravity, const Vector3& omegaCoriolis,
    const bool use2ndOrderCoriolis) {
  // NOTE(frank): parameters are supposed to be constant, below is only provided for compatibility
  boost::shared_ptr<Params> q = boost::make_shared<Params>(p());
  q->gravity = gravity;
  q->omegaCoriolis = omegaCoriolis;
  q->use2ndOrderCoriolis = use2ndOrderCoriolis;
  p_ = q;
  return PoseVelocityBias(predict(NavState(pose_i, vel_i), bias_i), bias_i);
}
}  /// namespace gtsam

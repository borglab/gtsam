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
NavState PreintegrationBase::predict(const NavState& state_i,
    const Rot3& deltaRij_biascorrected,
    const Vector3& deltaPij_biascorrected,
    const Vector3& deltaVij_biascorrected) const {

  const Pose3& pose_i = state_i.pose();
  const Vector3& vel_i = state_i.velocity();

  const double dt = deltaTij(), dt2 = dt * dt;
  const Matrix3 Ri = pose_i.rotation().matrix();

  // Rotation, translation, and velocity:
  Vector3 dR = Rot3::Logmap(deltaRij_biascorrected);
  Vector3 dP = Ri * deltaPij_biascorrected + vel_i * dt + 0.5 * p().gravity * dt2;
  Vector3 dV = Ri * deltaVij_biascorrected + p().gravity * dt;

  if (p().omegaCoriolis) {
    const Vector3& omegaCoriolis = *p().omegaCoriolis;
    dR -= Ri.transpose() * omegaCoriolis * dt;  // Coriolis term
    dP -= omegaCoriolis.cross(vel_i) * dt2;     // NOTE(luca): we got rid of the 2 wrt INS paper
    dV -= 2 * omegaCoriolis.cross(vel_i) * dt;
    if (p().use2ndOrderCoriolis) {
      Vector3 temp = omegaCoriolis.cross(omegaCoriolis.cross(pose_i.translation().vector()));
      dP -= 0.5 * temp * dt2;
      dV -= temp * dt;
    }
  }

  // TODO(frank): pose update below is separate expmap for R,t. Is that kosher?
  const Pose3 pose_j = Pose3(pose_i.rotation().expmap(dR), pose_i.translation() + Point3(dP));
  return NavState(pose_j, vel_i + dV);
}

//------------------------------------------------------------------------------
NavState PreintegrationBase::predict(const NavState& state_i,
    const imuBias::ConstantBias& bias_i) const {
  const imuBias::ConstantBias biasIncr = bias_i - biasHat_;
  return predict(state_i, biascorrectedDeltaRij(biasIncr.gyroscope()),
      biascorrectedDeltaPij(biasIncr), biascorrectedDeltaVij(biasIncr));
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

  const Rot3& rot_j = pose_j.rotation();
  const Vector3 pos_j = pose_j.translation().vector();

  /// Compute bias-corrected quantities
  const imuBias::ConstantBias biasIncr = bias_i - biasHat_;
  Matrix3 D_biascorrected_biasIncr;
  const Rot3 deltaRij_biascorrected = biascorrectedDeltaRij(
      biasIncr.gyroscope(), H5 ? &D_biascorrected_biasIncr : 0);
  const Vector3 deltaPij_biascorrected = biascorrectedDeltaPij(biasIncr);
  const Vector3 deltaVij_biascorrected = biascorrectedDeltaVij(biasIncr);

  /// Predict state at time j
  NavState predictedState_j =
      predict(NavState(pose_i, vel_i), deltaRij_biascorrected,
              deltaPij_biascorrected, deltaVij_biascorrected);

  // Evaluate residual error, according to [3]
  // Ri.transpose() is important here to preserve a model with *additive* Gaussian noise of correct covariance
  const Vector3 fp = Ri.transpose() * (pos_j - predictedState_j.pose().translation().vector());

  // Ri.transpose() is important here to preserve a model with *additive* Gaussian noise of correct covariance
  const Vector3 fv = Ri.transpose() * (vel_j - predictedState_j.velocity());

  // fR will be computed later.
  // Note: it is the same as: fR = predictedState_j.pose.rotation().between(Rot_j)

  /* ---------------------------------------------------------------------------------------------------- */
  // Get Get so<3> version of bias corrected rotation
  // If H5 is asked for, we will need the Jacobian, which we store in H5
  // H5 will then be corrected below to take into account the Coriolis effect
  Matrix3 D_omega_biascorrected;
  const Vector3 biascorrectedOmega = Rot3::Logmap(deltaRij_biascorrected, H5 ? &D_omega_biascorrected : 0);

  // Coriolis term, NOTE inconsistent with AHRS, where coriolisHat is *after* integration
  // TODO(frank): move derivatives to predict and do coriolis branching there
  const Vector3 coriolis = integrateCoriolis(rot_i);
  const Vector3 correctedOmega = biascorrectedOmega - coriolis;

  // Residual rotation error
  Matrix3 D_cDeltaRij_cOmega;
  const Rot3 correctedDeltaRij = Rot3::Expmap(correctedOmega, H1 || H5 ? &D_cDeltaRij_cOmega : 0);
  const Rot3 RiBetweenRj = rot_i.between(rot_j);
  const Rot3 fRrot = correctedDeltaRij.between(RiBetweenRj);
  Matrix3 D_fR_fRrot;
  const Vector3 fR = Rot3::Logmap(fRrot, H1 || H3 || H5 ? &D_fR_fRrot : 0);

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
    skewSymmetric(fp + deltaPij_biascorrected),                                             // dfP/dRi
    dfPdPi,                                                                                 // dfP/dPi
    skewSymmetric(fv + deltaVij_biascorrected),                                             // dfV/dRi
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
    // H5 by this point already contains 3*3 biascorrectedThetaRij derivative
    const Matrix3 JbiasOmega = D_cDeltaRij_cOmega * D_omega_biascorrected * D_biascorrected_biasIncr;
    (*H5) <<
    Z_3x3, D_fR_fRrot * (-fRrot.inverse().matrix() * JbiasOmega),   // dfR/dBias
    -delPdelBiasAcc(), -delPdelBiasOmega(),                         // dfP/dBias
    -delVdelBiasAcc(), -delVdelBiasOmega();                         // dfV/dBias
  }
  Vector9 r;
  r << fR, fp, fv;
  return r;
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

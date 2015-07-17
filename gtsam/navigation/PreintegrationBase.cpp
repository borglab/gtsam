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

namespace gtsam {

PreintegrationBase::PreintegrationBase(const imuBias::ConstantBias& bias,
                                       const Matrix3& measuredAccCovariance,
                                       const Matrix3& measuredOmegaCovariance,
                                       const Matrix3&integrationErrorCovariance,
                                       const bool use2ndOrderIntegration)
    : PreintegratedRotation(measuredOmegaCovariance),
      biasHat_(bias),
      use2ndOrderIntegration_(use2ndOrderIntegration),
      deltaPij_(Vector3::Zero()),
      deltaVij_(Vector3::Zero()),
      delPdelBiasAcc_(Z_3x3),
      delPdelBiasOmega_(Z_3x3),
      delVdelBiasAcc_(Z_3x3),
      delVdelBiasOmega_(Z_3x3),
      accelerometerCovariance_(measuredAccCovariance),
      integrationCovariance_(integrationErrorCovariance) {
}

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
void PreintegrationBase::print(const std::string& s) const {
  PreintegratedRotation::print(s);
  std::cout << "    deltaPij [ " << deltaPij_.transpose() << " ]" << std::endl;
  std::cout << "    deltaVij [ " << deltaVij_.transpose() << " ]" << std::endl;
  biasHat_.print("    biasHat");
}

/// Needed for testable
bool PreintegrationBase::equals(const PreintegrationBase& other, double tol) const {
  return PreintegratedRotation::equals(other, tol) && biasHat_.equals(other.biasHat_, tol)
      && equal_with_abs_tol(deltaPij_, other.deltaPij_, tol)
      && equal_with_abs_tol(deltaVij_, other.deltaVij_, tol)
      && equal_with_abs_tol(delPdelBiasAcc_, other.delPdelBiasAcc_, tol)
      && equal_with_abs_tol(delPdelBiasOmega_, other.delPdelBiasOmega_, tol)
      && equal_with_abs_tol(delVdelBiasAcc_, other.delVdelBiasAcc_, tol)
      && equal_with_abs_tol(delVdelBiasOmega_, other.delVdelBiasOmega_, tol)
      && equal_with_abs_tol(accelerometerCovariance_, other.accelerometerCovariance_, tol)
      && equal_with_abs_tol(integrationCovariance_, other.integrationCovariance_, tol);
}

/// Update preintegrated measurements
void PreintegrationBase::updatePreintegratedMeasurements(const Vector3& correctedAcc,
                                                         const Rot3& incrR, const double deltaT,
                                                         OptionalJacobian<9, 9> F) {

  const Matrix3 dRij = deltaRij();  // expensive
  const Vector3 temp = dRij * correctedAcc * deltaT;
  if (!use2ndOrderIntegration_) {
    deltaPij_ += deltaVij_ * deltaT;
  } else {
    deltaPij_ += deltaVij_ * deltaT + 0.5 * temp * deltaT;
  }
  deltaVij_ += temp;

  Matrix3 R_i, F_angles_angles;
  if (F)
    R_i = deltaRij();  // has to be executed before updateIntegratedRotationAndDeltaT as that updates deltaRij
  updateIntegratedRotationAndDeltaT(incrR, deltaT, F ? &F_angles_angles : 0);

  if (F) {
    const Matrix3 F_vel_angles = -R_i * skewSymmetric(correctedAcc) * deltaT;
    Matrix3 F_pos_angles;
    if (use2ndOrderIntegration_)
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
  const Matrix3 dRij = deltaRij();  // expensive
  const Matrix3 temp = -dRij * skewSymmetric(correctedAcc) * deltaT * delRdelBiasOmega();
  if (!use2ndOrderIntegration_) {
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
    const Vector3& measuredAcc, const Vector3& measuredOmega, Vector3& correctedAcc,
    Vector3& correctedOmega, boost::optional<const Pose3&> body_P_sensor) {
  correctedAcc = biasHat_.correctAccelerometer(measuredAcc);
  correctedOmega = biasHat_.correctGyroscope(measuredOmega);

  // Then compensate for sensor-body displacement: we express the quantities
  // (originally in the IMU frame) into the body frame
  if (body_P_sensor) {
    Matrix3 body_R_sensor = body_P_sensor->rotation().matrix();
    correctedOmega = body_R_sensor * correctedOmega;  // rotation rate vector in the body frame
    Matrix3 body_omega_body__cross = skewSymmetric(correctedOmega);
    correctedAcc = body_R_sensor * correctedAcc
        - body_omega_body__cross * body_omega_body__cross * body_P_sensor->translation().vector();
    // linear acceleration vector in the body frame
  }
}

/// Predict state at time j
//------------------------------------------------------------------------------
PoseVelocityBias PreintegrationBase::predict(const Pose3& pose_i,
    const Vector3& vel_i, const imuBias::ConstantBias& bias_i,
    const Vector3& gravity, const Vector3& omegaCoriolis,
    const Rot3& deltaRij_biascorrected, const Vector3& deltaPij_biascorrected,
    const Vector3& deltaVij_biascorrected,
    const bool use2ndOrderCoriolis) const {

  const double dt = deltaTij(), dt2 = dt * dt;

  // Rotation
  const Matrix3 Ri = pose_i.rotation().matrix();
  const Vector3 biascorrectedOmega = Rot3::Logmap(deltaRij_biascorrected);
  const Vector3 dR = biascorrectedOmega
      - Ri.transpose() * omegaCoriolis * dt; // Coriolis term

  // Translation
  Vector3 dP = Ri * deltaPij_biascorrected + vel_i * dt + 0.5 * gravity * dt2
      - omegaCoriolis.cross(vel_i) * dt2; // Coriolis term - we got rid of the 2 wrt INS paper

  // Velocity
  Vector3 dV = Ri * deltaVij_biascorrected + gravity * dt
      - 2 * omegaCoriolis.cross(vel_i) * dt; // Coriolis term

  if (use2ndOrderCoriolis) {
    Vector3 temp = omegaCoriolis.cross(omegaCoriolis.cross(pose_i.translation().vector()));
    dP -= 0.5 * temp * dt2;
    dV -= temp * dt;
  }

  // TODO(frank): pose update below is separate expmap for R,t. Is that kosher?
  const Pose3 pose_j = Pose3(pose_i.rotation().expmap(dR), pose_i.translation() + Point3(dP));
  return PoseVelocityBias(pose_j, vel_i + dV, bias_i); // bias is predicted as a constant
}

/// Predict state at time j
//------------------------------------------------------------------------------
PoseVelocityBias PreintegrationBase::predict(
    const Pose3& pose_i, const Vector3& vel_i, const imuBias::ConstantBias& bias_i,
    const Vector3& gravity, const Vector3& omegaCoriolis, const bool use2ndOrderCoriolis) const {
  const imuBias::ConstantBias biasIncr = bias_i - biasHat_;
  return predict(pose_i, vel_i, bias_i, gravity, omegaCoriolis,
      biascorrectedDeltaRij(biasIncr.gyroscope()),
      biascorrectedDeltaPij(biasIncr), biascorrectedDeltaVij(biasIncr),
      use2ndOrderCoriolis);
}

/// Compute errors w.r.t. preintegrated measurements and Jacobians wrpt pose_i, vel_i, bias_i, pose_j, bias_j
//------------------------------------------------------------------------------
Vector9 PreintegrationBase::computeErrorAndJacobians(const Pose3& pose_i, const Vector3& vel_i,
                                                     const Pose3& pose_j, const Vector3& vel_j,
                                                     const imuBias::ConstantBias& bias_i,
                                                     const Vector3& gravity,
                                                     const Vector3& omegaCoriolis,
                                                     const bool use2ndOrderCoriolis,
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

  // Evaluate residual error, according to [3]
  /* ---------------------------------------------------------------------------------------------------- */
  const imuBias::ConstantBias biasIncr = bias_i - biasHat_;
  const Rot3 deltaRij_biascorrected = biascorrectedDeltaRij(biasIncr.gyroscope());
  const Vector3 deltaPij_biascorrected = biascorrectedDeltaPij(biasIncr);
  const Vector3 deltaVij_biascorrected = biascorrectedDeltaVij(biasIncr);
  PoseVelocityBias predictedState_j = predict(pose_i, vel_i, bias_i, gravity,
      omegaCoriolis, deltaRij_biascorrected, deltaPij_biascorrected,
      deltaVij_biascorrected, use2ndOrderCoriolis);

  // Ri.transpose() is important here to preserve a model with *additive* Gaussian noise of correct covariance
  const Vector3 fp = Ri.transpose() * (pos_j - predictedState_j.pose.translation().vector());

  // Ri.transpose() is important here to preserve a model with *additive* Gaussian noise of correct covariance
  const Vector3 fv = Ri.transpose() * (vel_j - predictedState_j.velocity);

  // fR will be computed later. Note: it is the same as: fR = (predictedState_j.pose.translation()).between(Rot_j)

  /* ---------------------------------------------------------------------------------------------------- */
  // Get Get so<3> version of bias corrected rotation
  // If H5 is asked for, we will need the Jacobian, which we store in H5
  // H5 will then be corrected below to take into account the Coriolis effect
  // TODO(frank): biascorrectedThetaRij calculates deltaRij_biascorrected, which we already have
  Matrix3 D_cThetaRij_biasOmegaIncr;
  const Vector3 biascorrectedOmega = biascorrectedThetaRij(biasIncr.gyroscope(),
                                                           H5 ? &D_cThetaRij_biasOmegaIncr : 0);

  // Coriolis term, NOTE inconsistent with AHRS, where coriolisHat is *after* integration
  const Vector3 coriolis = integrateCoriolis(rot_i, omegaCoriolis);
  const Vector3 correctedOmega = biascorrectedOmega - coriolis;

  // Residual rotation error
  Matrix3 D_cDeltaRij_cOmega;
  const Rot3 correctedDeltaRij = Rot3::Expmap(correctedOmega, H1 || H5 ? &D_cDeltaRij_cOmega : 0);
  const Rot3 RiBetweenRj = rot_i.between(rot_j);
  const Rot3 fRrot = correctedDeltaRij.between(RiBetweenRj);
  Matrix3 D_fR_fRrot;
  const Vector3 fR = Rot3::Logmap(fRrot, H1 || H3 || H5 ? &D_fR_fRrot : 0);

  const double dt = deltaTij(), dt2 = dt*dt;
  Matrix3 Ritranspose_omegaCoriolisHat;
  if (H1 || H2)
    Ritranspose_omegaCoriolisHat = Ri.transpose() * skewSymmetric(omegaCoriolis);

  if (H1) {
    const Matrix3 D_coriolis = -D_cDeltaRij_cOmega * skewSymmetric(coriolis);
    Matrix3 dfPdPi = -I_3x3, dfVdPi = Z_3x3;
    if (use2ndOrderCoriolis) {
      // this is the same as: Ri.transpose() * omegaCoriolisHat * omegaCoriolisHat * Ri
      const Matrix3 temp = Ritranspose_omegaCoriolisHat
          * (-Ritranspose_omegaCoriolisHat.transpose());
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
    -Ri.transpose() * dt + Ritranspose_omegaCoriolisHat * dt2,  // dfP/dVi
    -Ri.transpose() + 2 * Ritranspose_omegaCoriolisHat * dt;    // dfV/dVi
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
    const Matrix3 JbiasOmega = D_cDeltaRij_cOmega * D_cThetaRij_biasOmegaIncr;
    (*H5) <<
    Z_3x3, D_fR_fRrot * (-fRrot.inverse().matrix() * JbiasOmega),   // dfR/dBias
    -delPdelBiasAcc(), -delPdelBiasOmega(),                         // dfP/dBias
    -delVdelBiasAcc(), -delVdelBiasOmega();                         // dfV/dBias
  }
  Vector9 r;
  r << fR, fp, fv;
  return r;
}

ImuBase::ImuBase()
    : gravity_(Vector3(0.0, 0.0, 9.81)),
      omegaCoriolis_(Vector3(0.0, 0.0, 0.0)),
      body_P_sensor_(boost::none),
      use2ndOrderCoriolis_(false) {
}

ImuBase::ImuBase(const Vector3& gravity, const Vector3& omegaCoriolis,
                 boost::optional<const Pose3&> body_P_sensor, const bool use2ndOrderCoriolis)
    : gravity_(gravity),
      omegaCoriolis_(omegaCoriolis),
      body_P_sensor_(body_P_sensor),
      use2ndOrderCoriolis_(use2ndOrderCoriolis) {
}

}  /// namespace gtsam

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

std::pair<Vector3, Vector3> PreintegrationBase::correctMeasurementsByBiasAndSensorPose(
    const Vector3& measuredAcc, const Vector3& measuredOmega) const {
  // Correct for bias in the sensor frame
  Vector3 s_correctedAcc, s_correctedOmega;
  s_correctedAcc = biasHat_.correctAccelerometer(measuredAcc);
  s_correctedOmega = biasHat_.correctGyroscope(measuredOmega);

  // Compensate for sensor-body displacement if needed: we express the quantities
  // (originally in the IMU frame) into the body frame
  // Equations below assume the "body" frame is the CG
  if (p().body_P_sensor) {
    Matrix3 bRs = p().body_P_sensor->rotation().matrix();
    Vector3 b_arm = p().body_P_sensor->translation().vector();
    Vector3 b_correctedOmega = bRs * s_correctedOmega; // rotation rate vector in the body frame
    Vector3 b_velocity_bs = b_correctedOmega.cross(b_arm); // magnitude: omega * arm
    // Subtract out the the centripetal acceleration from the measured one
    // to get linear acceleration vector in the body frame:
    Vector3 b_correctedAcc = bRs * s_correctedAcc
        - b_correctedOmega.cross(b_velocity_bs);
    return std::make_pair(b_correctedAcc, b_correctedOmega);
  } else
    return std::make_pair(s_correctedAcc, s_correctedOmega);
}

//------------------------------------------------------------------------------
Vector9 PreintegrationBase::biasCorrectedDelta(
    const imuBias::ConstantBias& bias_i, OptionalJacobian<9, 6> H) const {
  // Correct deltaRij, derivative is delRdelBiasOmega_
  const imuBias::ConstantBias biasIncr = bias_i - biasHat_;
  Matrix3 D_deltaRij_bias;
  Rot3 deltaRij = biascorrectedDeltaRij(biasIncr.gyroscope(), H ? &D_deltaRij_bias : 0);

  Vector9 xi;
  Matrix3 D_dR_deltaRij;
  // TODO(frank): could line below be simplified? It is equivalent to
  //   LogMap(deltaRij_.compose(Expmap(delRdelBiasOmega_ * biasIncr.gyroscope())))
  NavState::dR(xi) = Rot3::Logmap(deltaRij, H ? &D_dR_deltaRij : 0);
  NavState::dP(xi) = deltaPij_ + delPdelBiasAcc_ * biasIncr.accelerometer()
      + delPdelBiasOmega_ * biasIncr.gyroscope();
  NavState::dV(xi) = deltaVij_ + delVdelBiasAcc_ * biasIncr.accelerometer()
      + delVdelBiasOmega_ * biasIncr.gyroscope();

  if (H) {
    Matrix36 D_dR_bias, D_dP_bias, D_dV_bias;
    D_dR_bias << Z_3x3, D_dR_deltaRij * D_deltaRij_bias;
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
Vector9 PreintegrationBase::computeErrorAndJacobians(const Pose3& pose_i,
    const Vector3& vel_i, const Pose3& pose_j, const Vector3& vel_j,
    const imuBias::ConstantBias& bias_i, OptionalJacobian<9, 6> H1,
    OptionalJacobian<9, 3> H2, OptionalJacobian<9, 6> H3,
    OptionalJacobian<9, 3> H4, OptionalJacobian<9, 6> H5) const {

  NavState state_i(pose_i, vel_i);
  NavState state_j(pose_j, vel_j);

  /// Predict state at time j
  Matrix99 D_predict_state_i;
  Matrix96 D_predict_bias_i;
  NavState predictedState_j = predict(state_i, bias_i,
      H1 || H2 ? &D_predict_state_i : 0, H5 ? &D_predict_bias_i : 0);

  Matrix9 D_error_state_j, D_error_predict;
  Vector9 error = state_j.localCoordinates(predictedState_j,
      H3 || H4 ? &D_error_state_j : 0, H1 || H2 || H5 ? &D_error_predict : 0);

  // Separate out derivatives in terms of 5 arguments
  // Note that doing so requires special treatment of velocities, as when treated as
  // separate variables the retract applied will not be the semi-direct product in NavState
  // Instead, the velocities in nav are updated using a straight addition
  // This is difference is accounted for by the R().transpose calls below
  if (H1)
    *H1 << D_error_predict * D_predict_state_i.leftCols<6>();
  if (H2)
    *H2 << D_error_predict * D_predict_state_i.rightCols<3>() * state_i.R().transpose();
  if (H3)
    *H3 << D_error_state_j.leftCols<6>();
  if (H4)
    *H4 << D_error_state_j.rightCols<3>() * state_j.R().transpose();
  if (H5)
    *H5 << D_error_predict * D_predict_bias_i;

  return error;
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

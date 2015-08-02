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
  deltaTij_ = 0.0;
  deltaXij_ = NavState();
  delRdelBiasOmega_ = Z_3x3;
  delPdelBiasAcc_ = Z_3x3;
  delPdelBiasOmega_ = Z_3x3;
  delVdelBiasAcc_ = Z_3x3;
  delVdelBiasOmega_ = Z_3x3;
}

/// Needed for testable
void PreintegrationBase::print(const string& s) const {
  cout << s << endl;
  cout << "    deltaTij [" << deltaTij_ << "]" << endl;
  cout << "    deltaRij.ypr = (" << deltaRij().ypr().transpose() << ")" << endl;
  cout << "    deltaPij [ " << deltaPij().transpose() << " ]" << endl;
  cout << "    deltaVij [ " << deltaVij().transpose() << " ]" << endl;
  biasHat_.print("    biasHat");
}

/// Needed for testable
bool PreintegrationBase::equals(const PreintegrationBase& other,
    double tol) const {
  return fabs(deltaTij_ - other.deltaTij_) < tol
      && deltaXij_.equals(other.deltaXij_, tol)
      && biasHat_.equals(other.biasHat_, tol)
      && equal_with_abs_tol(delRdelBiasOmega_, other.delRdelBiasOmega_, tol)
      && equal_with_abs_tol(delPdelBiasAcc_, other.delPdelBiasAcc_, tol)
      && equal_with_abs_tol(delPdelBiasOmega_, other.delPdelBiasOmega_, tol)
      && equal_with_abs_tol(delVdelBiasAcc_, other.delVdelBiasAcc_, tol)
      && equal_with_abs_tol(delVdelBiasOmega_, other.delVdelBiasOmega_, tol);
}

//------------------------------------------------------------------------------
NavState PreintegrationBase::updatedDeltaXij(const Vector3& measuredAcc,
    const Vector3& measuredOmega, const double dt, OptionalJacobian<9, 9> F,
    OptionalJacobian<9, 3> G1, OptionalJacobian<9, 3> G2) const {

  // Correct for bias in the sensor frame
  Vector3 correctedAcc = biasHat_.correctAccelerometer(measuredAcc);
  Vector3 correctedOmega = biasHat_.correctGyroscope(measuredOmega);

  // Compensate for sensor-body displacement if needed: we express the quantities
  // (originally in the IMU frame) into the body frame
  // Equations below assume the "body" frame is the CG
  if (p().body_P_sensor) {
    // Correct omega: slight duplication as this is also done in integrateMeasurement below
    Matrix3 bRs = p().body_P_sensor->rotation().matrix();
    correctedOmega = bRs * correctedOmega; // rotation rate vector in the body frame

    // Correct acceleration
    Vector3 b_arm = p().body_P_sensor->translation().vector();
    Vector3 b_velocity_bs = correctedOmega.cross(b_arm); // magnitude: omega * arm
    // Subtract out the the centripetal acceleration from the measured one
    // to get linear acceleration vector in the body frame:
    correctedAcc = bRs * correctedAcc - correctedOmega.cross(b_velocity_bs);
  }

  // Do update in one fell swoop
  return deltaXij_.update(correctedAcc, correctedOmega, dt, F, G1, G2);
}

//------------------------------------------------------------------------------
void PreintegrationBase::update(
    const Vector3& measuredAcc, const Vector3& measuredOmega, const double dt,
    Matrix3* D_incrR_integratedOmega, Matrix9* F, Matrix93* G1, Matrix93* G2) {

  // Save current rotation for updating Jacobians
  const Rot3 oldRij = deltaXij_.attitude();

  // Do update
  deltaTij_ += dt;
  deltaXij_ = updatedDeltaXij(measuredAcc, measuredOmega, dt, F, G1, G2); // functional

  // Update Jacobians
  // TODO(frank): we are repeating some computation here: accessible in F ?
  // Correct for bias in the sensor frame
  Vector3 correctedAcc = biasHat_.correctAccelerometer(measuredAcc);
  Vector3 correctedOmega = biasHat_.correctGyroscope(measuredOmega);

  Matrix3 D_acc_R;
  oldRij.rotate(correctedAcc, D_acc_R);
  const Matrix3 D_acc_biasOmega = D_acc_R * delRdelBiasOmega_;

  const Vector3 integratedOmega = correctedOmega * dt;
  const Rot3 incrR = Rot3::Expmap(integratedOmega, D_incrR_integratedOmega); // expensive !!
  const Matrix3 incrRt = incrR.transpose();
  delRdelBiasOmega_ = incrRt * delRdelBiasOmega_ - *D_incrR_integratedOmega * dt;

  double dt22 = 0.5 * dt * dt;
  const Matrix3 dRij = oldRij.matrix(); // expensive
  delPdelBiasAcc_ += delVdelBiasAcc_ * dt - dt22 * dRij;
  delPdelBiasOmega_ += dt * delVdelBiasOmega_ + dt22 * D_acc_biasOmega;
  delVdelBiasAcc_ += -dRij * dt;
  delVdelBiasOmega_ += D_acc_biasOmega * dt;
}

//------------------------------------------------------------------------------
Vector9 PreintegrationBase::biasCorrectedDelta(
    const imuBias::ConstantBias& bias_i, OptionalJacobian<9, 6> H) const {
  // Correct deltaRij, derivative is delRdelBiasOmega_
  const imuBias::ConstantBias biasIncr = bias_i - biasHat_;
  Matrix3 D_correctedRij_bias;
  const Vector3 biasInducedOmega = delRdelBiasOmega_ * biasIncr.gyroscope();
  const Rot3 correctedRij = deltaRij().expmap(biasInducedOmega, boost::none,
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

  // Note that derivative of constructors below is not identity for velocity, but
  // a 9*3 matrix == Z_3x3, Z_3x3, state.R().transpose()
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

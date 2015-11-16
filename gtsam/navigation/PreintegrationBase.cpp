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

//------------------------------------------------------------------------------
void PreintegrationBase::Params::print(const string& s) const {
  PreintegratedRotation::Params::print(s);
  cout << "accelerometerCovariance:\n[\n" << accelerometerCovariance << "\n]"
      << endl;
  cout << "integrationCovariance:\n[\n" << accelerometerCovariance << "\n]"
      << endl;
  if (omegaCoriolis && use2ndOrderCoriolis)
    cout << "Using 2nd-order Coriolis" << endl;
  if (body_P_sensor)
    body_P_sensor->print("    ");
  cout << "n_gravity = (" << n_gravity.transpose() << ")" << endl;
}

//------------------------------------------------------------------------------
void PreintegrationBase::resetIntegration() {
  deltaTij_ = 0.0;
  deltaXij_ = NavState();
  delRdelBiasOmega_ = Z_3x3;
  delPdelBiasAcc_ = Z_3x3;
  delPdelBiasOmega_ = Z_3x3;
  delVdelBiasAcc_ = Z_3x3;
  delVdelBiasOmega_ = Z_3x3;
}

//------------------------------------------------------------------------------
void PreintegrationBase::print(const string& s) const {
  cout << s << endl;
  cout << "    deltaTij [" << deltaTij_ << "]" << endl;
  cout << "    deltaRij.ypr = (" << deltaRij().ypr().transpose() << ")" << endl;
  cout << "    deltaPij [ " << deltaPij().transpose() << " ]" << endl;
  cout << "    deltaVij [ " << deltaVij().transpose() << " ]" << endl;
  biasHat_.print("    biasHat");
}

//------------------------------------------------------------------------------
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
pair<Vector3, Vector3> PreintegrationBase::correctMeasurementsByBiasAndSensorPose(
    const Vector3& j_measuredAcc, const Vector3& j_measuredOmega,
    OptionalJacobian<3, 3> D_correctedAcc_measuredAcc,
    OptionalJacobian<3, 3> D_correctedAcc_measuredOmega,
    OptionalJacobian<3, 3> D_correctedOmega_measuredOmega) const {

  // Correct for bias in the sensor frame
  Vector3 j_correctedAcc = biasHat_.correctAccelerometer(j_measuredAcc);
  Vector3 j_correctedOmega = biasHat_.correctGyroscope(j_measuredOmega);

  // Compensate for sensor-body displacement if needed: we express the quantities
  // (originally in the IMU frame) into the body frame
  // Equations below assume the "body" frame is the CG
  if (p().body_P_sensor) {
    // Correct omega to rotation rate vector in the body frame
    const Matrix3 bRs = p().body_P_sensor->rotation().matrix();
    j_correctedOmega = bRs * j_correctedOmega;

    // Correct acceleration
    j_correctedAcc = bRs * j_correctedAcc;

    // Jacobians
    if (D_correctedAcc_measuredAcc) *D_correctedAcc_measuredAcc = bRs;
    if (D_correctedAcc_measuredOmega) *D_correctedAcc_measuredOmega = Matrix3::Zero();
    if (D_correctedOmega_measuredOmega) *D_correctedOmega_measuredOmega = bRs;

    // Centrifugal acceleration
    const Vector3 b_arm = p().body_P_sensor->translation().vector();
    if (!b_arm.isZero()) {
      // Subtract out the the centripetal acceleration from the measured one
      // to get linear acceleration vector in the body frame:
      const Matrix3 body_Omega_body = skewSymmetric(j_correctedOmega);
      const Vector3 b_velocity_bs = body_Omega_body * b_arm; // magnitude: omega * arm
      j_correctedAcc -= body_Omega_body * b_velocity_bs;
      // Update derivative: centrifugal causes the correlation between acc and omega!!!
      if (D_correctedAcc_measuredOmega) {
        double wdp = j_correctedOmega.dot(b_arm);
        *D_correctedAcc_measuredOmega = -(diag(Vector3::Constant(wdp))
            + j_correctedOmega * b_arm.transpose()) * bRs.matrix()
            + 2 * b_arm * j_measuredOmega.transpose();
      }
    }
  }

  // Do update in one fell swoop
  return make_pair(j_correctedAcc, j_correctedOmega);
}

//------------------------------------------------------------------------------
NavState PreintegrationBase::updatedDeltaXij(const Vector3& j_measuredAcc,
    const Vector3& j_measuredOmega, const double dt,
    OptionalJacobian<9, 9> D_updated_current,
    OptionalJacobian<9, 3> D_updated_measuredAcc,
    OptionalJacobian<9, 3> D_updated_measuredOmega) const {

  Vector3 j_correctedAcc, j_correctedOmega;
  Matrix3 D_correctedAcc_measuredAcc, //
      D_correctedAcc_measuredOmega, //
      D_correctedOmega_measuredOmega;
  bool needDerivs = D_updated_measuredAcc && D_updated_measuredOmega && p().body_P_sensor;
  boost::tie(j_correctedAcc, j_correctedOmega) =
      correctMeasurementsByBiasAndSensorPose(j_measuredAcc, j_measuredOmega,
          (needDerivs ? &D_correctedAcc_measuredAcc : 0),
          (needDerivs ? &D_correctedAcc_measuredOmega : 0),
          (needDerivs ? &D_correctedOmega_measuredOmega : 0));
  // Do update in one fell swoop
  Matrix93 D_updated_correctedAcc, D_updated_correctedOmega;
  NavState updated = deltaXij_.update(j_correctedAcc, j_correctedOmega, dt, D_updated_current,
              (needDerivs ? D_updated_correctedAcc : D_updated_measuredAcc),
              (needDerivs ? D_updated_correctedOmega : D_updated_measuredOmega));
  if (needDerivs) {
    *D_updated_measuredAcc = D_updated_correctedAcc * D_correctedAcc_measuredAcc;
    *D_updated_measuredOmega = D_updated_correctedOmega * D_correctedOmega_measuredOmega;
    if (!p().body_P_sensor->translation().vector().isZero()) {
      *D_updated_measuredOmega += D_updated_correctedAcc * D_correctedAcc_measuredOmega;
    }
  }
  return updated;
}

//------------------------------------------------------------------------------
void PreintegrationBase::update(const Vector3& j_measuredAcc,
    const Vector3& j_measuredOmega, const double dt,
    Matrix3* D_incrR_integratedOmega, Matrix9* D_updated_current,
    Matrix93* D_updated_measuredAcc, Matrix93* D_updated_measuredOmega) {

  // Save current rotation for updating Jacobians
  const Rot3 oldRij = deltaXij_.attitude();

  // Do update
  deltaTij_ += dt;
  deltaXij_ = updatedDeltaXij(j_measuredAcc, j_measuredOmega, dt,
      D_updated_current, D_updated_measuredAcc, D_updated_measuredOmega); // functional

  // Update Jacobians
  // TODO(frank): we are repeating some computation here: accessible in F ?
  Vector3 j_correctedAcc, j_correctedOmega;
  boost::tie(j_correctedAcc, j_correctedOmega) =
      correctMeasurementsByBiasAndSensorPose(j_measuredAcc, j_measuredOmega);

  Matrix3 D_acc_R;
  oldRij.rotate(j_correctedAcc, D_acc_R);
  const Matrix3 D_acc_biasOmega = D_acc_R * delRdelBiasOmega_;

  const Vector3 integratedOmega = j_correctedOmega * dt;
  const Rot3 incrR = Rot3::Expmap(integratedOmega, D_incrR_integratedOmega); // expensive !!
  const Matrix3 incrRt = incrR.transpose();
  delRdelBiasOmega_ = incrRt * delRdelBiasOmega_
      - *D_incrR_integratedOmega * dt;

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
    *H2
        << D_error_predict * D_predict_state_i.rightCols<3>()
            * state_i.R().transpose();
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

//------------------------------------------------------------------------------

}/// namespace gtsam

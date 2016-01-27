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
#ifdef ALLOW_DEPRECATED_IN_GTSAM4
#include <boost/make_shared.hpp>
#endif

using namespace std;

namespace gtsam {

//------------------------------------------------------------------------------
PreintegrationBase::PreintegrationBase(const boost::shared_ptr<Params>& p,
                                       const Bias& biasHat)
    : p_(p), biasHat_(biasHat), deltaTij_(0.0) {
  resetIntegration();
}

//------------------------------------------------------------------------------
void PreintegrationBase::resetIntegration() {
  deltaTij_ = 0.0;
  deltaXij_ = TangentVector();
  delRdelBiasOmega_ = Z_3x3;
  delPdelBiasAcc_ = Z_3x3;
  delPdelBiasOmega_ = Z_3x3;
  delVdelBiasAcc_ = Z_3x3;
  delVdelBiasOmega_ = Z_3x3;
}

//------------------------------------------------------------------------------
ostream& operator<<(ostream& os, const PreintegrationBase& pim) {
  os << "    deltaTij " << pim.deltaTij_ << endl;
  os << "    deltaRij " << Point3(pim.theta()) << endl;
  os << "    deltaPij " << Point3(pim.deltaPij()) << endl;
  os << "    deltaVij " << Point3(pim.deltaVij()) << endl;
  os << "    gyrobias " << Point3(pim.biasHat_.gyroscope()) << endl;
  os << "    acc_bias " << Point3(pim.biasHat_.accelerometer()) << endl;
  return os;
}

//------------------------------------------------------------------------------
void PreintegrationBase::print(const string& s) const {
  cout << s << *this << endl;
}

//------------------------------------------------------------------------------
bool PreintegrationBase::equals(const PreintegrationBase& other,
    double tol) const {
  const bool params_match = p_->equals(*other.p_, tol);
  return params_match && fabs(deltaTij_ - other.deltaTij_) < tol
      && biasHat_.equals(other.biasHat_, tol)
      && equal_with_abs_tol(deltaXij_.vector(), other.deltaXij_.vector(), tol)
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
    // Get sensor to body rotation matrix
    const Matrix3 bRs = p().body_P_sensor->rotation().matrix();

    // Convert angular velocity and acceleration from sensor to body frame
    j_correctedOmega = bRs * j_correctedOmega;
    j_correctedAcc = bRs * j_correctedAcc;

    // Jacobians
    if (D_correctedAcc_measuredAcc) *D_correctedAcc_measuredAcc = bRs;
    if (D_correctedAcc_measuredOmega) *D_correctedAcc_measuredOmega = Z_3x3;
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

  return make_pair(j_correctedAcc, j_correctedOmega);
}

//------------------------------------------------------------------------------
// See extensive discussion in ImuFactor.lyx
PreintegrationBase::TangentVector PreintegrationBase::UpdateEstimate(
    const Vector3& a_body, const Vector3& w_body, double dt,
    const TangentVector& zeta, OptionalJacobian<9, 9> A,
    OptionalJacobian<9, 3> B, OptionalJacobian<9, 3> C) {
  // Calculate exact mean propagation
  Matrix3 H;
  const Matrix3 R = Rot3::Expmap(zeta.theta(), H).matrix();
  const Matrix3 invH = H.inverse();
  const Vector3 a_nav = R * a_body;
  const double dt22 = 0.5 * dt * dt;

  TangentVector zetaPlus(zeta.theta() + invH * w_body * dt,
                         zeta.position() + zeta.velocity() * dt + a_nav * dt22,
                         zeta.velocity() + a_nav * dt);

  if (A) {
    // First order (small angle) approximation of derivative of invH*w:
    const Matrix3 invHw_H_theta = skewSymmetric(-0.5 * w_body);

    // Exact derivative of R*a with respect to theta:
    const Matrix3 a_nav_H_theta = R * skewSymmetric(-a_body) * H;

    A->setIdentity();
    A->block<3, 3>(0, 0).noalias() += invHw_H_theta * dt;
    A->block<3, 3>(3, 0) = a_nav_H_theta * dt22;
    A->block<3, 3>(3, 6) = I_3x3 * dt;
    A->block<3, 3>(6, 0) = a_nav_H_theta * dt;
  }
  if (B) {
    B->block<3, 3>(0, 0) = Z_3x3;
    B->block<3, 3>(3, 0) = R * dt22;
    B->block<3, 3>(6, 0) = R * dt;
  }
  if (C) {
    C->block<3, 3>(0, 0) = invH * dt;
    C->block<3, 3>(3, 0) = Z_3x3;
    C->block<3, 3>(6, 0) = Z_3x3;
  }

  return zetaPlus;
}

//------------------------------------------------------------------------------
PreintegrationBase::TangentVector PreintegrationBase::updatedDeltaXij(
    const Vector3& measuredAcc, const Vector3& measuredOmega, double dt,
    OptionalJacobian<9, 9> A, OptionalJacobian<9, 3> B,
    OptionalJacobian<9, 3> C) const {
  if (!p().body_P_sensor) {
    // Correct for bias in the sensor frame
    Vector3 correctedAcc = biasHat_.correctAccelerometer(measuredAcc);
    Vector3 correctedOmega = biasHat_.correctGyroscope(measuredOmega);

    // Do update in one fell swoop
    return UpdateEstimate(correctedAcc, correctedOmega, dt, deltaXij_, A, B, C);
  } else {
    // More complicated derivatives in case of sensor displacement
    Vector3 correctedAcc, correctedOmega;
    Matrix3 D_correctedAcc_measuredAcc, D_correctedAcc_measuredOmega,
        D_correctedOmega_measuredOmega;
    boost::tie(correctedAcc, correctedOmega) =
        correctMeasurementsByBiasAndSensorPose(
            measuredAcc, measuredOmega, (B ? &D_correctedAcc_measuredAcc : 0),
            (C ? &D_correctedAcc_measuredOmega : 0),
            (C ? &D_correctedOmega_measuredOmega : 0));

    // Do update in one fell swoop
    Matrix93 D_updated_correctedAcc, D_updated_correctedOmega;
    const PreintegrationBase::TangentVector updated =
        UpdateEstimate(correctedAcc, correctedOmega, dt, deltaXij_, A,
                       ((B || C) ? &D_updated_correctedAcc : 0),
                       (C ? &D_updated_correctedOmega : 0));
    if (B) *B = D_updated_correctedAcc* D_correctedAcc_measuredAcc;
    if (C) {
      *C = D_updated_correctedOmega* D_correctedOmega_measuredOmega;
      if (!p().body_P_sensor->translation().vector().isZero())
        *C += D_updated_correctedAcc* D_correctedAcc_measuredOmega;
    }
    return updated;
  }
}

//------------------------------------------------------------------------------
void PreintegrationBase::update(const Vector3& j_measuredAcc,
                                const Vector3& j_measuredOmega, double dt,
                                Matrix3* D_incrR_integratedOmega, Matrix9* A,
                                Matrix93* B, Matrix93* C) {
  // Save current rotation for updating Jacobians
  const Rot3 oldRij = deltaRij();

  // Do update
  deltaTij_ += dt;
  deltaXij_ = updatedDeltaXij(j_measuredAcc, j_measuredOmega, dt, A, B, C);

  // Update Jacobians
  // TODO(frank): we are repeating some computation here: accessible in A ?
  // Possibly: derivatives are just -B and -C ??
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
                                     const imuBias::ConstantBias& bias_i,
                                     OptionalJacobian<9, 9> H1,
                                     OptionalJacobian<9, 6> H2) const {
  // TODO(frank): make sure this stuff is still correct
  Matrix96 D_biasCorrected_bias;
  Vector9 biasCorrected =
      biasCorrectedDelta(bias_i, H2 ? &D_biasCorrected_bias : 0);

  // Correct for initial velocity and gravity
  Matrix9 D_delta_state, D_delta_biasCorrected;
  Vector9 xi = state_i.correctPIM(biasCorrected, deltaTij_, p().n_gravity,
                                  p().omegaCoriolis, p().use2ndOrderCoriolis,
                                  H1 ? &D_delta_state : 0,
                                  H2 ? &D_delta_biasCorrected : 0);

  // Use retract to get back to NavState manifold
  Matrix9 D_predict_state, D_predict_delta;
  NavState state_j = state_i.retract(xi, D_predict_state, D_predict_delta);
  if (H1) *H1 = D_predict_state + D_predict_delta* D_delta_state;
  if (H2) *H2 = D_predict_delta* D_delta_biasCorrected * D_biasCorrected_bias;
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
  if (H1) *H1 << D_error_predict* D_predict_state_i.leftCols<6>();
  if (H2) *H2 << D_error_predict* D_predict_state_i.rightCols<3>() * state_i.R().transpose();
  if (H3) *H3 << D_error_state_j.leftCols<6>();
  if (H4) *H4 << D_error_state_j.rightCols<3>() * state_j.R().transpose();
  if (H5) *H5 << D_error_predict* D_predict_bias_i;

  return error;
}

//------------------------------------------------------------------------------
#ifdef ALLOW_DEPRECATED_IN_GTSAM4
PoseVelocityBias PreintegrationBase::predict(const Pose3& pose_i,
    const Vector3& vel_i, const imuBias::ConstantBias& bias_i,
    const Vector3& n_gravity, const Vector3& omegaCoriolis,
    const bool use2ndOrderCoriolis) const {
// NOTE(frank): parameters are supposed to be constant, below is only provided for compatibility
  boost::shared_ptr<Params> q = boost::make_shared<Params>(p());
  q->n_gravity = n_gravity;
  q->omegaCoriolis = omegaCoriolis;
  q->use2ndOrderCoriolis = use2ndOrderCoriolis;
  p_ = q;
  return PoseVelocityBias(predict(NavState(pose_i, vel_i), bias_i), bias_i);
}
#endif
//------------------------------------------------------------------------------

}
 /// namespace gtsam

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
#include <gtsam/base/numericalDerivative.h>
#include <boost/make_shared.hpp>

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
  preintegrated_ = Vector9();
  preintegrated_H_biasAcc_.setZero();
  preintegrated_H_biasOmega_.setZero();
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
      && equal_with_abs_tol(preintegrated_, other.preintegrated_, tol)
      && equal_with_abs_tol(preintegrated_H_biasAcc_, other.preintegrated_H_biasAcc_, tol)
      && equal_with_abs_tol(preintegrated_H_biasOmega_, other.preintegrated_H_biasOmega_, tol);
}

//------------------------------------------------------------------------------
pair<Vector3, Vector3> PreintegrationBase::correctMeasurementsBySensorPose(
    const Vector3& unbiasedAcc, const Vector3& unbiasedOmega,
    OptionalJacobian<3, 3> D_correctedAcc_unbiasedAcc,
    OptionalJacobian<3, 3> D_correctedAcc_unbiasedOmega,
    OptionalJacobian<3, 3> D_correctedOmega_unbiasedOmega) const {
  assert(p().body_P_sensor);

  // Compensate for sensor-body displacement if needed: we express the quantities
  // (originally in the IMU frame) into the body frame
  // Equations below assume the "body" frame is the CG

  // Get sensor to body rotation matrix
  const Matrix3 bRs = p().body_P_sensor->rotation().matrix();

  // Convert angular velocity and acceleration from sensor to body frame
  Vector3 correctedAcc = bRs * unbiasedAcc;
  const Vector3 correctedOmega = bRs * unbiasedOmega;

  // Jacobians
  if (D_correctedAcc_unbiasedAcc) *D_correctedAcc_unbiasedAcc = bRs;
  if (D_correctedAcc_unbiasedOmega) *D_correctedAcc_unbiasedOmega = Z_3x3;
  if (D_correctedOmega_unbiasedOmega) *D_correctedOmega_unbiasedOmega = bRs;

  // Centrifugal acceleration
  const Vector3 b_arm = p().body_P_sensor->translation().vector();
  if (!b_arm.isZero()) {
    // Subtract out the the centripetal acceleration from the unbiased one
    // to get linear acceleration vector in the body frame:
    const Matrix3 body_Omega_body = skewSymmetric(correctedOmega);
    const Vector3 b_velocity_bs = body_Omega_body * b_arm; // magnitude: omega * arm
    correctedAcc -= body_Omega_body * b_velocity_bs;

    // Update derivative: centrifugal causes the correlation between acc and omega!!!
    if (D_correctedAcc_unbiasedOmega) {
      double wdp = correctedOmega.dot(b_arm);
      *D_correctedAcc_unbiasedOmega = -(diag(Vector3::Constant(wdp))
          + correctedOmega * b_arm.transpose()) * bRs.matrix()
          + 2 * b_arm * unbiasedOmega.transpose();
    }
  }

  return make_pair(correctedAcc, correctedOmega);
}

//------------------------------------------------------------------------------
// See extensive discussion in ImuFactor.lyx
Vector9 PreintegrationBase::UpdateEstimate(const Vector3& a_body,
                                           const Vector3& w_body, double dt,
                                           const Vector9& preintegrated,
                                           OptionalJacobian<9, 9> A,
                                           OptionalJacobian<9, 3> B,
                                           OptionalJacobian<9, 3> C) {
  const auto theta = preintegrated.segment<3>(0);
  const auto position = preintegrated.segment<3>(3);
  const auto velocity = preintegrated.segment<3>(6);

  // Calculate exact mean propagation
  Matrix3 H;
  const Matrix3 R = Rot3::Expmap(theta, H).matrix();
  const Matrix3 invH = H.inverse();
  const Vector3 a_nav = R * a_body;
  const double dt22 = 0.5 * dt * dt;

  Vector9 preintegratedPlus;
  preintegratedPlus <<                                 //
      theta + invH* w_body* dt,               // theta
      position + velocity* dt + a_nav* dt22,  // position
      velocity + a_nav* dt;                   // velocity

  if (A) {
#ifdef USE_NUMERICAL_DERIVATIVE
    // The use of this yields much more accurate derivatives, but it's slow!
    // TODO(frank): find a cheap closed form solution (look at Iserles)
    auto f = [w_body](const Vector3& theta) {
      return Rot3::ExpmapDerivative(theta).inverse() * w_body;
    };
    const Matrix3 invHw_H_theta =
        numericalDerivative11<Vector3, Vector3>(f, theta);
#else
    // First order (small angle) approximation of derivative of invH*w:
    // NOTE(frank): Rot3::ExpmapDerivative(w_body) is a less accurate approximation
    const Matrix3 invHw_H_theta = skewSymmetric(-0.5 * w_body);
#endif

    // Exact derivative of R*a with respect to theta:
    const Matrix3 a_nav_H_theta = R * skewSymmetric(-a_body) * H;

    A->setIdentity();
    A->block<3, 3>(0, 0).noalias() += invHw_H_theta * dt;  // theta
    A->block<3, 3>(3, 0) = a_nav_H_theta * dt22;  // position wrpt theta...
    A->block<3, 3>(3, 6) = I_3x3 * dt;            // .. and velocity
    A->block<3, 3>(6, 0) = a_nav_H_theta * dt;    // velocity wrpt theta
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

  return preintegratedPlus;
}

//------------------------------------------------------------------------------
Vector9 PreintegrationBase::updatedPreintegrated(const Vector3& measuredAcc,
                                                 const Vector3& measuredOmega,
                                                 double dt, Matrix9* A,
                                                 Matrix93* B, Matrix93* C) const {
  // Correct for bias in the sensor frame
  Vector3 unbiasedAcc = biasHat_.correctAccelerometer(measuredAcc);
  Vector3 unbiasedOmega = biasHat_.correctGyroscope(measuredOmega);

  if (!p().body_P_sensor) {
    return UpdateEstimate(unbiasedAcc, unbiasedOmega, dt, preintegrated_, A, B,
                          C);
  } else {
    // More complicated derivatives in case of sensor displacement
    Matrix3 D_correctedAcc_unbiasedAcc, D_correctedAcc_unbiasedOmega,
        D_correctedOmega_unbiasedOmega;
    auto corrected = correctMeasurementsBySensorPose(
        unbiasedAcc, unbiasedOmega, D_correctedAcc_unbiasedAcc,
        D_correctedAcc_unbiasedOmega, D_correctedOmega_unbiasedOmega);

    const Vector9 updated = UpdateEstimate(corrected.first, corrected.second, dt,
                                           preintegrated_, A, B, C);

    *C *= D_correctedOmega_unbiasedOmega;
    if (!p().body_P_sensor->translation().vector().isZero())
      *C += *B* D_correctedAcc_unbiasedOmega;
    *B *= D_correctedAcc_unbiasedAcc;  // NOTE(frank): needs to be last
    return updated;
  }
}

//------------------------------------------------------------------------------
void PreintegrationBase::update(const Vector3& measuredAcc,
                                const Vector3& measuredOmega, double dt,
                                Matrix3* D_incrR_integratedOmega, Matrix9* A,
                                Matrix93* B, Matrix93* C) {
  // Do update
  deltaTij_ += dt;
  preintegrated_ = updatedPreintegrated(measuredAcc, measuredOmega, dt, A, B, C);

  // D_plus_abias = D_plus_preintegrated * D_preintegrated_abias + D_plus_a * D_a_abias
  preintegrated_H_biasAcc_ = (*A) * preintegrated_H_biasAcc_ - (*B);

  // D_plus_wbias = D_plus_preintegrated * D_preintegrated_wbias + D_plus_w * D_w_wbias
  preintegrated_H_biasOmega_ = (*A) * preintegrated_H_biasOmega_ - (*C);
}

//------------------------------------------------------------------------------
Vector9 PreintegrationBase::biasCorrectedDelta(
    const imuBias::ConstantBias& bias_i, OptionalJacobian<9, 6> H) const {
  // We correct for a change between bias_i and the biasHat_ used to integrate
  // This is a simple linear correction with obvious derivatives
  const imuBias::ConstantBias biasIncr = bias_i - biasHat_;
  const Vector9 biasCorrected =
      preintegrated() + preintegrated_H_biasAcc_ * biasIncr.accelerometer() +
      preintegrated_H_biasOmega_ * biasIncr.gyroscope();

  if (H) {
    (*H) << preintegrated_H_biasAcc_, preintegrated_H_biasOmega_;
  }
  return biasCorrected;
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
  if (H2) *H2 = D_predict_delta* D_delta_biasCorrected* D_biasCorrected_bias;
  return state_j;
}

//------------------------------------------------------------------------------
Vector9 PreintegrationBase::computeError(const NavState& state_i,
                                         const NavState& state_j,
                                         const imuBias::ConstantBias& bias_i,
                                         OptionalJacobian<9, 9> H1,
                                         OptionalJacobian<9, 9> H2,
                                         OptionalJacobian<9, 6> H3) const {
  // Predict state at time j
  Matrix9 D_predict_state_i;
  Matrix96 D_predict_bias_i;
  NavState predictedState_j = predict(
      state_i, bias_i, H1 ? &D_predict_state_i : 0, H3 ? &D_predict_bias_i : 0);

  // Calculate error
  Matrix9 D_error_state_j, D_error_predict;
  Vector9 error =
      state_j.localCoordinates(predictedState_j, H2 ? &D_error_state_j : 0,
                               H1 || H3 ? &D_error_predict : 0);

  if (H1) *H1 << D_error_predict* D_predict_state_i;
  if (H2) *H2 << D_error_state_j;
  if (H3) *H3 << D_error_predict* D_predict_bias_i;

  return error;
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

  // Predict state at time j
  Matrix9 D_error_state_i, D_error_state_j;
  Vector9 error = computeError(state_i, state_j, bias_i,
                         H1 || H2 ? &D_error_state_i : 0, H3 || H4 ? &D_error_state_j : 0, H5);

  // Separate out derivatives in terms of 5 arguments
  // Note that doing so requires special treatment of velocities, as when treated as
  // separate variables the retract applied will not be the semi-direct product in NavState
  // Instead, the velocities in nav are updated using a straight addition
  // This is difference is accounted for by the R().transpose calls below
  if (H1) *H1 << D_error_state_i.leftCols<6>();
  if (H2) *H2 << D_error_state_i.rightCols<3>() * state_i.R().transpose();
  if (H3) *H3 << D_error_state_j.leftCols<6>();
  if (H4) *H4 << D_error_state_j.rightCols<3>() * state_j.R().transpose();

  return error;
}

//------------------------------------------------------------------------------
#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V4
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

}  // namespace gtsam

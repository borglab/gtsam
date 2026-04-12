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
 *  @author Varun Agrawal
 **/

#include <gtsam/navigation/PreintegrationBase.h>
#include <gtsam/base/numericalDerivative.h>

#include <cassert>

using namespace std;

namespace gtsam {

//------------------------------------------------------------------------------
PreintegrationBase::PreintegrationBase(const std::shared_ptr<Params>& p,
                                       const Bias& biasHat)
    : p_(p), biasHat_(biasHat), deltaTij_(0.0) {
}

//------------------------------------------------------------------------------
ostream& operator<<(ostream& os, const PreintegrationBase& pim) {
  os << "    deltaTij = " << pim.deltaTij_ << endl;
  os << "    deltaRij.ypr = (" << pim.deltaRij().ypr().transpose() << ")" << endl;
  os << "    deltaPij = " << pim.deltaPij().transpose() << endl;
  os << "    deltaVij = " << pim.deltaVij().transpose() << endl;
  os << "    gyrobias = " << pim.biasHat_.gyroscope().transpose() << endl;
  os << "    acc_bias = " << pim.biasHat_.accelerometer().transpose() << endl;
  return os;
}

//------------------------------------------------------------------------------
void PreintegrationBase::print(const string& s) const {
  cout << (s.empty() ? s : s + "\n") << *this << endl;
}

//------------------------------------------------------------------------------
void PreintegrationBase::resetIntegrationAndSetBias(const Bias& biasHat) {
	biasHat_ = biasHat;
	resetIntegration();
}

//------------------------------------------------------------------------------
pair<Vector3, Vector3> PreintegrationBase::correctMeasurementsBySensorPose(
    const Vector3& unbiasedAcc, const Vector3& unbiasedOmega,
    OptionalJacobian<3, 3> correctedAcc_H_unbiasedAcc,
    OptionalJacobian<3, 3> correctedAcc_H_unbiasedOmega,
    OptionalJacobian<3, 3> correctedOmega_H_unbiasedOmega) const {
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
  if (correctedAcc_H_unbiasedAcc) *correctedAcc_H_unbiasedAcc = bRs;
  if (correctedAcc_H_unbiasedOmega) *correctedAcc_H_unbiasedOmega = Z_3x3;
  if (correctedOmega_H_unbiasedOmega) *correctedOmega_H_unbiasedOmega = bRs;

  // Centrifugal acceleration
  const Vector3 b_arm = p().body_P_sensor->translation();
  if (!b_arm.isZero()) {
    // Subtract out the the centripetal acceleration from the unbiased one
    // to get linear acceleration vector in the body frame:
    const Matrix3 body_Omega_body = skewSymmetric(correctedOmega);
    const Vector3 b_velocity_bs = body_Omega_body * b_arm; // magnitude: omega * arm
    correctedAcc -= body_Omega_body * b_velocity_bs;

    // Update derivative: centrifugal causes the correlation between acc and omega!!!
    if (correctedAcc_H_unbiasedOmega) {
      double wdp = correctedOmega.dot(b_arm);
      const Matrix3 diag_wdp = Vector3::Constant(wdp).asDiagonal();
      *correctedAcc_H_unbiasedOmega = -( diag_wdp
          + correctedOmega * b_arm.transpose()) * bRs.matrix()
          + 2 * b_arm * unbiasedOmega.transpose();
    }
  }

  return make_pair(correctedAcc, correctedOmega);
}

//------------------------------------------------------------------------------
void PreintegrationBase::integrateMeasurement(const Vector3& measuredAcc,
    const Vector3& measuredOmega, double dt) {
  // NOTE(frank): integrateMeasurement always needs to compute the derivatives,
  // even when not of interest to the caller. Provide scratch space here.
  Matrix9 A;
  Matrix93 B, C;
  update(measuredAcc, measuredOmega, dt, &A, &B, &C);
}

//------------------------------------------------------------------------------
NavState PreintegrationBase::predict(const NavState& state_i,
    const imuBias::ConstantBias& bias_i, OptionalJacobian<9, 9> H1,
    OptionalJacobian<9, 6> H2) const {
  Matrix96 D_biasCorrected_bias;
  Vector9 biasCorrected = biasCorrectedDelta(bias_i,
                                             H2 ? &D_biasCorrected_bias : nullptr);

  // Correct for initial velocity and gravity
  Matrix9 D_delta_state, D_delta_biasCorrected;
  Vector9 xi = state_i.correctPIM(biasCorrected, deltaTij_, p().n_gravity,
                                  p().omegaCoriolis, p().use2ndOrderCoriolis, H1 ? &D_delta_state : nullptr,
                                  H2 ? &D_delta_biasCorrected : nullptr);

  // Use retract to get back to NavState manifold
  Matrix9 D_predict_state, D_predict_delta;
  NavState state_j = state_i.retract(xi,
                                     H1 ? &D_predict_state : nullptr,
                                     H1 || H2 ? &D_predict_delta : nullptr);
  if (H1)
    *H1 = D_predict_state + D_predict_delta * D_delta_state;
  if (H2)
    *H2 = D_predict_delta * D_delta_biasCorrected * D_biasCorrected_bias;
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

  // Rotation residual: r_R = Log(R_j^{-1} R_pred).
  Matrix3 D_Rrel_Rj, D_Rrel_Rpred, D_rR_Rrel;
  const Rot3 Rrel = state_j.attitude().between(predictedState_j.attitude(),
      H2 ? &D_Rrel_Rj : nullptr, (H1 || H3) ? &D_Rrel_Rpred : nullptr);
  const Vector3 r_R = Rot3::Logmap(Rrel, (H1 || H2 || H3) ? &D_rR_Rrel : nullptr);

  // Position and velocity residuals in body frame of state_i.
  // Using R_i^T makes the Jacobian of [r_p; r_v] w.r.t. preint error state dXt
  // equal to identity, so preintMeasCov_ applies directly as residual covariance.
  const Matrix3 Ri_T = state_i.R().transpose();
  const Vector3 dp = predictedState_j.t() - state_j.t();
  const Vector3 dv = predictedState_j.v() - state_j.v();
  const Vector3 r_p = Ri_T * dp;
  const Vector3 r_v = Ri_T * dv;

  Vector9 error;
  error << r_R, r_p, r_v;

  // NavState retract uses the body-frame (dXn) perturbation convention:
  //   dXn = [delta_phi, delta_p, delta_v] where
  //     R_i'  = R_i * Exp(delta_phi)
  //     p_i'  = p_i + R_i * delta_p     (body-frame additive)
  //     v_i'  = v_i + R_i * delta_v     (body-frame additive)
  // Therefore D_predict_state_i and D_predict_bias_i have their position/velocity
  // rows expressed in the body frame of predictedState_j.
  if (H1) {
    // r_R depends on R_pred only through D_predict_state_i.topRows<3>().
    // D_predict_state_i's position/velocity rows are in the body frame of
    // predictedState_j; convert to navigation frame via R_i^T * R_pred.
    const Matrix3 D_rR_Rpred = D_rR_Rrel * D_Rrel_Rpred;
    const Matrix3 RiT_Rpred = Ri_T * predictedState_j.R();
    const Eigen::Matrix<double,3,9> H1_rot = D_rR_Rpred * D_predict_state_i.topRows<3>();
    Eigen::Matrix<double,3,9> H1_pos, H1_vel;
    H1_pos << skewSymmetric(r_p) + RiT_Rpred * D_predict_state_i.block<3,3>(3,0),
               RiT_Rpred * D_predict_state_i.block<3,3>(3,3),
               RiT_Rpred * D_predict_state_i.block<3,3>(3,6);
    H1_vel << skewSymmetric(r_v) + RiT_Rpred * D_predict_state_i.block<3,3>(6,0),
               RiT_Rpred * D_predict_state_i.block<3,3>(6,3),
               RiT_Rpred * D_predict_state_i.block<3,3>(6,6);
    *H1 << H1_rot, H1_pos, H1_vel;
  }

  if (H2) {
    // r_R = Log(R_j^{-1} R_pred) depends on R_j; r_p, r_v depend on p_j, v_j via -R_i^T R_j.
    const Matrix3 D_rR_Rj = D_rR_Rrel * D_Rrel_Rj;
    const Matrix3 neg_RiT_Rj = -(Ri_T * state_j.R());
    *H2 << D_rR_Rj,      Z_3x3,       Z_3x3,
           Z_3x3,        neg_RiT_Rj,  Z_3x3,
           Z_3x3,        Z_3x3,       neg_RiT_Rj;
  }

  if (H3) {
    const Matrix3 D_rR_Rpred = D_rR_Rrel * D_Rrel_Rpred;
    const Matrix3 RiT_Rpred = Ri_T * predictedState_j.R();
    *H3 << D_rR_Rpred * D_predict_bias_i.topRows<3>(),
           RiT_Rpred * D_predict_bias_i.middleRows<3>(3),
           RiT_Rpred * D_predict_bias_i.middleRows<3>(6);
  }

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

}  // namespace gtsam

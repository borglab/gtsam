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

#include <gtsam/base/MatrixConstants.h>
#include <gtsam/base/VectorConstants.h>
#include <gtsam/navigation/PreintegrationBase.h>

#include <cassert>
#include <stdexcept>

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
Vector3 PreintegrationBase::so3TangentAt(double t) const {
  if (t < 0.0 || t > deltaTij_) {
    throw std::out_of_range("t must be in [0, deltaTij]");
  }
  if (deltaTij_ == 0.0) return Z_3x1;
  return (t / deltaTij_) * Rot3::Logmap(deltaRij());
}

//------------------------------------------------------------------------------
Matrix PreintegrationBase::deskewPoints(
    ConstMatrixView points, const Vector3& velocity_i) const {
  if (points.rows() % 3 != 0) {
    throw std::invalid_argument(
        "points must have shape 3m x n with rows divisible by 3");
  }
  const Eigen::Index batchCount = points.cols();
  if (batchCount == 0) return Matrix(points.rows(), 0);

  const Vector3 endpointTangent = so3TangentAt(deltaTij_);
  const Rot3 rotationStep =
      Rot3::Expmap(endpointTangent / static_cast<double>(batchCount));
  Rot3 rotation;
  Matrix deskewed(points.rows(), batchCount);
  for (Eigen::Index batch = 0; batch < batchCount; ++batch) {
    const double t = deltaTij_ * static_cast<double>(batch) /
                     static_cast<double>(batchCount);
    const Vector3 translation = velocity_i * t;
    const Matrix3 rotationMatrix = rotation.matrix();
    for (Eigen::Index pointRow = 0; pointRow < points.rows(); pointRow += 3) {
      deskewed.block<3, 1>(pointRow, batch) =
          rotationMatrix * points.block<3, 1>(pointRow, batch) + translation;
    }
    rotation = rotation.compose(rotationStep);
  }
  return deskewed;
}

//------------------------------------------------------------------------------
Matrix PreintegrationBase::deskewPointsAtTimes(
    ConstMatrixView points, const Vector& times,
    const Vector3& velocity_i) const {
  if (points.rows() % 3 != 0) {
    throw std::invalid_argument(
        "points must have shape 3m x n with rows divisible by 3");
  }
  const Eigen::Index batchCount = points.cols();
  if (times.size() != batchCount) {
    throw std::invalid_argument("times size must match the point batch count");
  }
  if (batchCount == 0) return Matrix(points.rows(), 0);

  const Vector3 endpointTangent = so3TangentAt(deltaTij_);
  Matrix deskewed(points.rows(), batchCount);
  for (Eigen::Index batch = 0; batch < batchCount; ++batch) {
    const double t = times(batch);
    if (t < 0.0 || t > deltaTij_) {
      throw std::out_of_range("t must be in [0, deltaTij]");
    }
    const double fraction = deltaTij_ == 0.0 ? 0.0 : t / deltaTij_;
    const Matrix3 rotationMatrix =
        Rot3::Expmap(fraction * endpointTangent).matrix();
    const Vector3 translation = velocity_i * t;
    for (Eigen::Index pointRow = 0; pointRow < points.rows(); pointRow += 3) {
      deskewed.block<3, 1>(pointRow, batch) =
          rotationMatrix * points.block<3, 1>(pointRow, batch) + translation;
    }
  }
  return deskewed;
}

//------------------------------------------------------------------------------
NavState PreintegrationBase::predict(const NavState& state_i,
    const imuBias::ConstantBias& bias_i, const Vector3& n_gravity,
    OptionalJacobian<9, 9> H1, OptionalJacobian<9, 6> H2,
    OptionalJacobian<9, 3> H3) const {
  Matrix96 D_biasCorrected_bias;
  Vector9 biasCorrected = biasCorrectedDelta(bias_i,
                                             H2 ? &D_biasCorrected_bias : nullptr);

  // Correct for initial velocity and gravity
  Matrix9 D_delta_state, D_delta_biasCorrected;
  Matrix93 D_delta_gravity;
  Vector9 xi = state_i.correctPIM(biasCorrected, deltaTij_, n_gravity,
                                  p().omegaCoriolis, p().use2ndOrderCoriolis, H1 ? &D_delta_state : nullptr,
                                  H2 ? &D_delta_biasCorrected : nullptr,
                                  H3 ? &D_delta_gravity : nullptr);

  // Use retract to get back to NavState manifold
  Matrix9 D_predict_state, D_predict_delta;
  NavState state_j = state_i.retract(xi,
                                     H1 ? &D_predict_state : nullptr,
                                     H1 || H2 || H3 ? &D_predict_delta : nullptr);
  if (H1)
    *H1 = D_predict_state + D_predict_delta * D_delta_state;
  if (H2)
    *H2 = D_predict_delta * D_delta_biasCorrected * D_biasCorrected_bias;
  if (H3)
    *H3 = D_predict_delta * D_delta_gravity;
  return state_j;
}

//------------------------------------------------------------------------------
NavState PreintegrationBase::predict(const NavState& state_i,
    const imuBias::ConstantBias& bias_i, OptionalJacobian<9, 9> H1,
    OptionalJacobian<9, 6> H2) const {
  return predict(state_i, bias_i, p().n_gravity, H1, H2, nullptr);
}

}  // namespace gtsam

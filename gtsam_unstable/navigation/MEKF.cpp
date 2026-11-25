/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file MEKF.cpp
 * @brief MEKF strapdown propagation, measurement outputs and updates.
 * @author Arihant Lunawat
 *
 * Strapdown mean (forward Euler):
 *   R+ = R * Exp((omega - b_gyro) dt)
 *   v+ = v + (R (accel - b_accel) + g) dt   (g = u.g_vec, caller's frame)
 *   p+ = p + v dt
 * F: finite-diff of discrete map in local coords, re-evaluated each step.
 *
 * Attitude correction via Retract, followed by the posterior-mean covariance
 * reset P <- B P B^T, B = resetJacobian(eta).
 */

#include <gtsam/base/Matrix.h>
#include <gtsam_unstable/navigation/MEKF.h>

namespace gtsam {
namespace mekf {

MEKFState propagateMean(const MEKFState& X, const ImuInput& u, double dt) {
  const Vector3 gyro_corr = u.omega - X.b_gyro;
  const Vector3 acc_corr = u.accel - X.b_accel;

  MEKFState Xn;
  Xn.R = X.R * gtsam::Rot3::Expmap(gyro_corr * dt);
  Xn.v = X.v + (X.R.rotate(acc_corr) + u.g_vec) * dt;
  Xn.p = X.p + X.v * dt;
  Xn.b_gyro = X.b_gyro;
  Xn.b_accel = X.b_accel;
  return Xn;
}

MEKFState::Jacobian transitionJacobian(const MEKFState& X, const ImuInput& u,
                                       double dt) {
  constexpr double h = 1e-7;

  const MEKFState Xn_ref = propagateMean(X, u, dt);

  MEKFState::Jacobian F;
  for (int j = 0; j < 15; ++j) {
    MEKFState::TangentVector e = MEKFState::TangentVector::Zero();
    e(j) = h;
    const MEKFState Xp = gtsam::traits<MEKFState>::Retract(X, e);
    const MEKFState Xpn = propagateMean(Xp, u, dt);
    F.col(j) = gtsam::traits<MEKFState>::Local(Xn_ref, Xpn) / h;
  }
  return F;
}

Vector3 BodyVelocityOutput::predict(const MEKFState& X) {
  return X.R.unrotate(X.v);
}

BodyVelocityOutput::Jacobian BodyVelocityOutput::jacobian(const MEKFState& X) {
  const Vector3 body_v = predict(X);

  Jacobian H = Jacobian::Zero();
  H.block<3, 3>(0, 0) = gtsam::skewSymmetric(body_v);
  H.block<3, 3>(0, 3) = X.R.matrix().transpose();
  return H;
}

Vector3 PositionOutput::predict(const MEKFState& X) { return X.p; }

PositionOutput::Jacobian PositionOutput::jacobian() {
  Jacobian H = Jacobian::Zero();
  H.block<3, 3>(0, 6) = gtsam::Matrix3::Identity();
  return H;
}

double DepthOutput::predict(const MEKFState& X) { return X.p.z(); }

DepthOutput::Jacobian DepthOutput::jacobian() {
  Jacobian H = Jacobian::Zero();
  H(0, 8) = 1.0;
  return H;
}

MEKF::MEKF(const MEKFState& X0, const Covariance& P0) : Base(X0, P0) {}

void MEKF::propagate(const Vector3& omega, const Vector3& accel,
                     const Vector3& g_vec, const Covariance& Qc, double dt) {
  ImuInput u;
  u.omega = omega;
  u.accel = accel;
  u.g_vec = g_vec;

  const MEKFState X_next = propagateMean(this->state(), u, dt);
  const Jacobian F = transitionJacobian(this->state(), u, dt);
  this->predict(X_next, F, Qc * dt);
}

void MEKF::propagate(const Vector3& omega, const Vector3& accel,
                     const Vector3& g_vec, const ImuNoise& noise, double dt) {
  const gtsam::Matrix3 R = this->state().R.matrix();
  Covariance Qc = Covariance::Zero();
  Qc.block<3, 3>(0, 0) = noise.gyro.asDiagonal();
  Qc.block<3, 3>(3, 3) = R * noise.accel.asDiagonal() * R.transpose();
  Qc.block<3, 3>(9, 9) = noise.gyro_rw.asDiagonal();
  Qc.block<3, 3>(12, 12) = noise.accel_rw.asDiagonal();
  propagate(omega, accel, g_vec, Qc, dt);
}

void MEKF::updatePosition(const Vector3& pi, const Covariance3& R_pos) {
  const Vector3 prediction = PositionOutput::predict(this->state());
  const PositionOutput::Jacobian H = PositionOutput::jacobian();
  updateWithReset(prediction, H, pi, R_pos);
}

void MEKF::updateDvl(const Vector3& z_dvl, const Covariance3& R_dvl) {
  const Vector3 prediction = BodyVelocityOutput::predict(this->state());
  const BodyVelocityOutput::Jacobian H =
      BodyVelocityOutput::jacobian(this->state());
  updateWithReset(prediction, H, z_dvl, R_dvl);
}

void MEKF::updateDepth(double z_depth, const Covariance1& R_depth,
                       double horizontal_variance) {
  Vector3 pseudo_position = position();
  pseudo_position.z() = z_depth;

  Covariance3 R_pseudo = Covariance3::Zero();
  R_pseudo(0, 0) = horizontal_variance;
  R_pseudo(1, 1) = horizontal_variance;
  R_pseudo(2, 2) = R_depth(0, 0);

  updatePosition(pseudo_position, R_pseudo);
}

void MEKF::updateDepthDirect(double z_depth, const Covariance1& R_depth) {
  const Vector1 prediction{DepthOutput::predict(this->state())};
  const Vector1 z{z_depth};
  updateWithReset(prediction, DepthOutput::jacobian(), z, R_depth);
}

MEKF::Jacobian MEKF::resetJacobian(const TangentVector& eta) {
  Jacobian B = Jacobian::Identity();
  B.block<3, 3>(0, 0) = gtsam::Rot3::ExpmapDerivative(eta.head<3>());
  return B;
}

void MEKF::updateWithReset(const gtsam::Vector& prediction,
                           const gtsam::Matrix& H, const gtsam::Vector& z,
                           const gtsam::Matrix& R) {
  const MEKFState prior = this->state();
  this->updateWithVector(prediction, H, z, R, /*performReset=*/false);
  const TangentVector eta =
      gtsam::traits<MEKFState>::Local(prior, this->state());
  const Jacobian B = resetJacobian(eta);
  this->P_ = B * this->P_ * B.transpose();
}

MEKF::TangentVector MEKF::errorStateVector(const MEKFState& X_true) const {
  return gtsam::traits<MEKFState>::Local(this->state(), X_true);
}

gtsam::Rot3 MEKF::attitude() const { return this->state().R; }
Vector3 MEKF::velocity() const { return this->state().v; }
Vector3 MEKF::position() const { return this->state().p; }
Vector3 MEKF::biasGyro() const { return this->state().b_gyro; }
Vector3 MEKF::biasAccel() const { return this->state().b_accel; }

}  // namespace mekf
}  // namespace gtsam

/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file TFGInEKF.cpp
 * @brief TFG-InEKF input lift, transition matrix, measurement outputs and
 *        updates.
 * @author Arihant Lunawat
 */

#include <gtsam/base/Matrix.h>
#include <gtsam_unstable/navigation/TFGInEKF.h>

namespace tfg {

using gtsam::Matrix3;

static inline Matrix3 skew(const Vector3& x) { return gtsam::skewSymmetric(x); }

Tangent lift(const TwoFrameGroup& X, const ImuInput& u) {
  const Vector3 b_omega = X.bias_omega();
  const Vector3 b_accel = X.bias_accel();
  const Vector3 gyro_err = u.omega - b_omega;

  Tangent xi;
  xi.segment<3>(0) = gyro_err;
  xi.segment<3>(3) = (u.accel - b_accel) + X.R.unrotate(u.g_vec);
  xi.segment<3>(6) = X.R.unrotate(X.v);
  xi.segment<3>(9) = b_omega.cross(gyro_err) - u.tau_omega;
  xi.segment<3>(12) = b_accel.cross(gyro_err) - u.tau_accel;
  return xi;
}

// Df = d lift(X * Exp(eps), u) / d eps at eps = 0. Non-zero blocks only:
//   Df[theta , theta ] = -b_w^                  Df[theta , g_w] =  I
//   Df[v     , theta ] = (R^T g)^ - b_a^         Df[v     , g_a] =  I
//   Df[p     , theta ] = (R^T v)^               Df[p     , v  ] =  I
//   Df[g_w   , theta ] = -omega^ b_w^            Df[g_w   , g_w] =  omega^
//   Df[g_a   , theta ] = -w_hat^ b_a^ - b_a^ b_w^
//                        Df[g_a, g_w] = b_a^     Df[g_a   , g_a] =  w_hat^
TwoFrameGroup::Jacobian liftJacobian(const TwoFrameGroup& X,
                                     const ImuInput& u) {
  const Vector3 b_omega = X.bias_omega();
  const Vector3 b_accel = X.bias_accel();
  const Vector3 w_hat = u.omega - b_omega;
  const Vector3 Rt_g = X.R.unrotate(u.g_vec);
  const Vector3 Rt_v = X.R.unrotate(X.v);
  const Matrix3 I3 = Matrix3::Identity();

  TwoFrameGroup::Jacobian Df = TwoFrameGroup::Jacobian::Zero();
  Df.block<3, 3>(0, 0) = -skew(b_omega);
  Df.block<3, 3>(0, 9) = I3;
  Df.block<3, 3>(3, 0) = skew(Rt_g) - skew(b_accel);
  Df.block<3, 3>(3, 12) = I3;
  Df.block<3, 3>(6, 0) = skew(Rt_v);
  Df.block<3, 3>(6, 3) = I3;
  Df.block<3, 3>(9, 0) = -skew(u.omega) * skew(b_omega);
  Df.block<3, 3>(9, 9) = skew(u.omega);
  Df.block<3, 3>(12, 0) =
      -skew(w_hat) * skew(b_accel) - skew(b_accel) * skew(b_omega);
  Df.block<3, 3>(12, 9) = skew(b_accel);
  Df.block<3, 3>(12, 12) = skew(w_hat);
  return Df;
}

// Qd = B diag(sigma^2) B^T dt. Input order: (n_omega, n_accel, tau_omega,
// tau_accel).
TwoFrameGroup::Jacobian inputNoiseCov(const TwoFrameGroup& X,
                                      const ImuNoise& noise, double dt) {
  using InputNoiseJacobian = Eigen::Matrix<double, 15, 12>;

  const Vector3 b_omega = X.bias_omega();
  const Vector3 b_accel = X.bias_accel();
  const Matrix3 I3 = Matrix3::Identity();

  InputNoiseJacobian B = InputNoiseJacobian::Zero();
  B.block<3, 3>(0, 0) = I3;
  B.block<3, 3>(3, 3) = I3;
  B.block<3, 3>(9, 0) = skew(b_omega);
  B.block<3, 3>(9, 6) = -I3;
  B.block<3, 3>(12, 0) = skew(b_accel);
  B.block<3, 3>(12, 9) = -I3;

  gtsam::Vector12 sig;
  sig.segment<3>(0) = noise.gyro;
  sig.segment<3>(3) = noise.accel;
  sig.segment<3>(6) = noise.gyro_rw;
  sig.segment<3>(9) = noise.accel_rw;

  return (B * sig.asDiagonal() * B.transpose() * dt).eval();
}

TwoFrameGroup increment(const TwoFrameGroup& X, const ImuInput& u, double dt) {
  return TwoFrameGroup::Expmap(lift(X, u) * dt);
}

TwoFrameGroup::Jacobian transitionJacobian(const TwoFrameGroup& X,
                                           const ImuInput& u, double dt) {
  TwoFrameGroup::Jacobian Jr;
  const TwoFrameGroup U =
      gtsam::traits<TwoFrameGroup>::Expmap(lift(X, u) * dt, Jr);
  return U.inverse().AdjointMap() + Jr * liftJacobian(X, u) * dt;
}

Vector3 BodyVelocityOutput::predict(const TwoFrameGroup& xi) {
  return xi.R.unrotate(xi.v);
}

Vector3 BodyVelocityOutput::output_action(const TwoFrameGroup& X,
                                          const Vector3& y) {
  return X.R.unrotate(y) + X.R.unrotate(X.v);
}

BodyVelocityOutput::Jacobian BodyVelocityOutput::jacobian(
    const TwoFrameGroup& xi_hat) {
  const Vector3 y_hat = predict(xi_hat);

  Jacobian H = Jacobian::Zero();
  H.block<3, 3>(0, 0) = gtsam::skewSymmetric(y_hat);
  H.block<3, 3>(0, 3) = Matrix3::Identity();
  return H;
}

Vector3 PositionOutput::predict(const TwoFrameGroup& xi, const Vector3& pi) {
  return xi.R.unrotate(pi - xi.p);
}

Vector3 PositionOutput::output_action(const TwoFrameGroup& X,
                                      const Vector3& y) {
  return X.R.unrotate(y - X.p);
}

PositionOutput::Jacobian PositionOutput::jacobian(const TwoFrameGroup& xi_hat,
                                                  const Vector3& pi,
                                                  Variant variant) {
  const Vector3 y_hat = predict(xi_hat, pi);
  const Vector3 coupling =
      (variant == Variant::Cstar) ? (0.5 * y_hat).eval() : y_hat;

  Jacobian C = Jacobian::Zero();
  C.block<3, 3>(0, 0) = gtsam::skewSymmetric(coupling);
  C.block<3, 3>(0, 6) = -Matrix3::Identity();
  return C;
}

TfgInEKF::TfgInEKF(const TwoFrameGroup& X0, const Covariance& P0)
    : Base(X0, P0) {}

namespace {
ImuInput makeInput(const Vector3& omega, const Vector3& accel,
                   const Vector3& g_vec) {
  ImuInput u;
  u.omega = omega;
  u.accel = accel;
  u.g_vec = g_vec;
  return u;
}

// X_next = X * U with the exact discrete-step Jacobian F.
void transition(const TwoFrameGroup& X, const ImuInput& u, double dt,
                TwoFrameGroup& X_next, TfgInEKF::Covariance& F) {
  X_next = X * increment(X, u, dt);
  F = transitionJacobian(X, u, dt);
}
}  // namespace

void TfgInEKF::propagate(const Vector3& omega, const Vector3& accel,
                         const Vector3& g_vec, const Covariance& Qc,
                         double dt) {
  const ImuInput u = makeInput(omega, accel, g_vec);
  TwoFrameGroup X_next;
  Covariance F;
  transition(this->state(), u, dt, X_next, F);
  this->gtsam::ManifoldEKF<TwoFrameGroup>::predict(X_next, F, Qc * dt);
}

void TfgInEKF::propagate(const Vector3& omega, const Vector3& accel,
                         const Vector3& g_vec, const ImuNoise& noise,
                         double dt) {
  const ImuInput u = makeInput(omega, accel, g_vec);
  const Covariance Qd = inputNoiseCov(this->state(), noise, dt);
  TwoFrameGroup X_next;
  Covariance F;
  transition(this->state(), u, dt, X_next, F);
  this->gtsam::ManifoldEKF<TwoFrameGroup>::predict(X_next, F, Qd);
}

void TfgInEKF::update_position(const Vector3& pi, const Covariance3& R_pos,
                               bool use_cstar) {
  const TwoFrameGroup& X = this->state();

  const auto variant =
      use_cstar ? PositionOutput::Variant::Cstar : PositionOutput::Variant::C0;
  const Vector3 prediction = PositionOutput::predict(X, pi);
  const PositionOutput::Jacobian H = PositionOutput::jacobian(X, pi, variant);
  const Vector3 z = Vector3::Zero();

  const Matrix3 Rm = X.R.matrix();
  const Covariance3 R_meas = Rm.transpose() * R_pos * Rm;

  updateWithReset(prediction, H, z, R_meas);
}

void TfgInEKF::update_dvl(const Vector3& z_dvl, const Covariance3& R_dvl) {
  const TwoFrameGroup& X = this->state();

  // Body-frame residual; R_dvl used directly (no R^T(.)R rotation).
  const Vector3 prediction = BodyVelocityOutput::predict(X);
  const BodyVelocityOutput::Jacobian H = BodyVelocityOutput::jacobian(X);

  updateWithReset(prediction, H, z_dvl, R_dvl);
}

void TfgInEKF::update_depth(double z_depth, const Covariance1& R_depth,
                            double horizontal_variance, bool use_cstar) {
  Vector3 pseudo_position = position();
  pseudo_position.z() = z_depth;

  Covariance3 R_pseudo = Covariance3::Zero();
  R_pseudo(0, 0) = horizontal_variance;
  R_pseudo(1, 1) = horizontal_variance;
  R_pseudo(2, 2) = R_depth(0, 0);

  update_position(pseudo_position, R_pseudo, use_cstar);
}

TfgInEKF::Jacobian TfgInEKF::resetJacobian(const TangentVector& eta) {
  Jacobian B;
  gtsam::traits<TwoFrameGroup>::Expmap(eta, B);
  return B;
}

void TfgInEKF::updateWithReset(const gtsam::Vector& prediction,
                               const gtsam::Matrix& H, const gtsam::Vector& z,
                               const gtsam::Matrix& R) {
  const TwoFrameGroup prior = this->state();
  this->updateWithVector(prediction, H, z, R, /*performReset=*/false);
  const TangentVector eta =
      gtsam::traits<TwoFrameGroup>::Local(prior, this->state());
  const Jacobian B = resetJacobian(eta);
  this->P_ = B * this->P_ * B.transpose();
}

TfgInEKF::TangentVector TfgInEKF::errorStateVector(
    const TwoFrameGroup& X_true) const {
  return gtsam::traits<TwoFrameGroup>::Local(this->state(), X_true);
}

gtsam::Rot3 TfgInEKF::attitude() const { return this->state().R; }
Vector3 TfgInEKF::velocity() const { return this->state().v; }
Vector3 TfgInEKF::position() const { return this->state().p; }
Vector3 TfgInEKF::bias_gyro() const { return this->state().bias_omega(); }
Vector3 TfgInEKF::bias_accel() const { return this->state().bias_accel(); }

}  // namespace tfg

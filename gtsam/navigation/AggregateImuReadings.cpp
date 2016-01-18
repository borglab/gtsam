/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    AggregateImuReadings.cpp
 * @brief   Integrates IMU readings on the NavState tangent space
 * @author  Frank Dellaert
 */

#include <gtsam/navigation/AggregateImuReadings.h>
#include <gtsam/base/numericalDerivative.h>

#include <cmath>

using namespace std;

namespace gtsam {

AggregateImuReadings::AggregateImuReadings(const boost::shared_ptr<Params>& p,
                                           const Bias& estimatedBias)
    : p_(p), estimatedBias_(estimatedBias), deltaTij_(0.0) {
  zeta_.setZero();
  cov_.setZero();
}

// Tangent space sugar.
namespace sugar {
static Eigen::Block<Vector9, 3, 1> dR(Vector9& v) { return v.segment<3>(0); }
static Eigen::Block<Vector9, 3, 1> dP(Vector9& v) { return v.segment<3>(3); }
static Eigen::Block<Vector9, 3, 1> dV(Vector9& v) { return v.segment<3>(6); }
}  // namespace sugar

// See extensive discussion in ImuFactor.lyx
void AggregateImuReadings::UpdateEstimate(const Vector3& a_body,
                                          const Vector3& w_body, double dt,
                                          Vector9* zeta,
                                          OptionalJacobian<9, 9> A,
                                          OptionalJacobian<9, 3> B,
                                          OptionalJacobian<9, 3> C) {
  using namespace sugar;

  const Vector3 theta = dR(*zeta);
  const Vector3 v = dV(*zeta);

  // Calculate exact mean propagation
  Matrix3 H;
  const Matrix3 R = Rot3::Expmap(theta, H).matrix();
  const Matrix3 invH = H.inverse();
  const Vector3 a_nav = R * a_body;
  const double dt22 = 0.5 * dt * dt;

  dR(*zeta) += invH * w_body * dt;     // theta
  dP(*zeta) += v * dt + a_nav * dt22;  // position
  dV(*zeta) += a_nav * dt;             // velocity

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
}

void AggregateImuReadings::integrateMeasurement(const Vector3& measuredAcc,
                                                const Vector3& measuredOmega,
                                                double dt) {
  // Correct measurements
  const Vector3 a_body = measuredAcc - estimatedBias_.accelerometer();
  const Vector3 w_body = measuredOmega - estimatedBias_.gyroscope();

  // Do exact mean propagation
  Matrix9 A;
  Matrix93 B, C;
  UpdateEstimate(a_body, w_body, dt, &zeta_, A, B, C);

  // propagate uncertainty
  // TODO(frank): use noiseModel routine so we can have arbitrary noise models.
  const Matrix3& aCov = p_->accelerometerCovariance;
  const Matrix3& wCov = p_->gyroscopeCovariance;

  // TODO(frank): use Eigen-tricks for symmetric matrices
  cov_ = A * cov_ * A.transpose();
  cov_.noalias() += B * (aCov / dt) * B.transpose();
  cov_.noalias() += C * (wCov / dt) * C.transpose();

  deltaTij_ += dt;
}

NavState AggregateImuReadings::predict(const NavState& state_i,
                                       const Bias& bias_i,
                                       OptionalJacobian<9, 9> H1,
                                       OptionalJacobian<9, 6> H2) const {
  using namespace sugar;
  Vector9 zeta = zeta_;

// Correct for initial velocity and gravity
#if 1
  Rot3 Ri = state_i.attitude();
  Matrix3 Rit = Ri.transpose();
  Vector3 gt = deltaTij_ * p_->n_gravity;
  dP(zeta) += Rit * (state_i.velocity() * deltaTij_ + 0.5 * deltaTij_ * gt);
  dV(zeta) += Rit * gt;
#endif

  return state_i.retract(zeta);
}

SharedGaussian AggregateImuReadings::noiseModel() const {
#ifndef LOCALCOORDINATES_ONLY
  // Correct for application of retract, by calculating the retract derivative H
  // From NavState::retract:
  Matrix3 D_R_theta;
  const Matrix3 iRj = Rot3::Expmap(theta(), D_R_theta).matrix();
  Matrix9 H;
  H << D_R_theta, Z_3x3, Z_3x3,       //
      Z_3x3, iRj.transpose(), Z_3x3,  //
      Z_3x3, Z_3x3, iRj.transpose();

  // TODO(frank): theta() itself is noisy, so should we not correct for that?
  Matrix9 HcH = H * cov_ * H.transpose();
  return noiseModel::Gaussian::Covariance(HcH, false);
#else
  return noiseModel::Gaussian::Covariance(cov_, false);
#endif
}

Matrix9 AggregateImuReadings::preintMeasCov() const {
  return noiseModel()->covariance();
}

}  // namespace gtsam

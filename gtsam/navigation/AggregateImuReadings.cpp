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
#include <gtsam/navigation/functors.h>
#include <cmath>

using namespace std;

namespace gtsam {

AggregateImuReadings::AggregateImuReadings(const boost::shared_ptr<Params>& p,
                                           const Bias& estimatedBias)
    : p_(p),
      accelerometerNoiseModel_(
          noiseModel::Gaussian::Covariance(p->accelerometerCovariance, true)),
      gyroscopeNoiseModel_(
          noiseModel::Gaussian::Covariance(p->gyroscopeCovariance, true)),
      estimatedBias_(estimatedBias),
      k_(0),
      deltaTij_(0.0) {
  zeta_.setZero();
  cov_.setZero();
}

// Tangent space sugar.
namespace sugar {

static Eigen::Block<Vector9, 3, 1> dR(Vector9& v) { return v.segment<3>(0); }
static Eigen::Block<Vector9, 3, 1> dP(Vector9& v) { return v.segment<3>(3); }
static Eigen::Block<Vector9, 3, 1> dV(Vector9& v) { return v.segment<3>(6); }

typedef const Vector9 constV9;
static Eigen::Block<constV9, 3, 1> dR(constV9& v) { return v.segment<3>(0); }
static Eigen::Block<constV9, 3, 1> dP(constV9& v) { return v.segment<3>(3); }
static Eigen::Block<constV9, 3, 1> dV(constV9& v) { return v.segment<3>(6); }

}  // namespace sugar

Vector9 AggregateImuReadings::UpdateEstimate(
    const Vector9& zeta, const Vector3& correctedAcc,
    const Vector3& correctedOmega, double dt, bool useExactDexpDerivative,
    OptionalJacobian<9, 9> A, OptionalJacobian<9, 3> Ba,
    OptionalJacobian<9, 3> Bw) {
  using namespace sugar;

  const Vector3 a_dt = correctedAcc * dt;
  const Vector3 w_dt = correctedOmega * dt;

  // Calculate exact mean propagation
  Matrix3 D_R_theta;
  const auto theta = dR(zeta);
  const Rot3 R = Rot3::Expmap(theta, D_R_theta).matrix();

  Matrix3 D_invHwdt_theta, D_invHwdt_wdt;
  Vector3 invHwdt;
  if (useExactDexpDerivative) {
    invHwdt = correctWithExpmapDerivative(
        -theta, w_dt, A ? &D_invHwdt_theta : 0, A ? &D_invHwdt_wdt : 0);
    if (A) D_invHwdt_theta *= -1;
  } else {
    const Matrix3 invH = D_R_theta.inverse();
    invHwdt = invH * w_dt;
    // First order (small angle) approximation of derivative of invH*w*dt:
    if (A) D_invHwdt_theta = skewSymmetric(-0.5 * w_dt);
    if (A) D_invHwdt_wdt = invH;
  }

  Matrix3 D_Radt_R, D_Radt_adt;
  const Vector3 Radt = R.rotate(a_dt, A ? &D_Radt_R : 0, A ? &D_Radt_adt : 0);

  Vector9 zeta_plus;
  const double dt2 = 0.5 * dt;
  dR(zeta_plus) = dR(zeta) + invHwdt;                     // theta
  dP(zeta_plus) = dP(zeta) + dV(zeta) * dt + Radt * dt2;  // position
  dV(zeta_plus) = dV(zeta) + Radt;                        // velocity

  if (A) {
    // Exact derivative of R*a*dt with respect to theta:
    const Matrix3 D_Radt_theta = D_Radt_R * D_R_theta;

    A->setIdentity();
    A->block<3, 3>(0, 0) += D_invHwdt_theta;
    A->block<3, 3>(3, 0) = D_Radt_theta * dt2;
    A->block<3, 3>(3, 6) = I_3x3 * dt;
    A->block<3, 3>(6, 0) = D_Radt_theta;
  }
  if (Ba) *Ba << Z_3x3, D_Radt_adt* dt* dt2, D_Radt_adt* dt;
  if (Bw) *Bw << D_invHwdt_wdt* dt, Z_3x3, Z_3x3;

  return zeta_plus;
}

void AggregateImuReadings::integrateMeasurement(const Vector3& measuredAcc,
                                                const Vector3& measuredOmega,
                                                double dt,
                                                bool useExactDexpDerivative) {
  // Correct measurements
  const Vector3 correctedAcc = measuredAcc - estimatedBias_.accelerometer();
  const Vector3 correctedOmega = measuredOmega - estimatedBias_.gyroscope();

  // Do exact mean propagation
  Matrix9 A;
  Matrix93 Ba, Bw;
  zeta_ = UpdateEstimate(zeta_, correctedAcc, correctedOmega, dt,
                         useExactDexpDerivative, A, Ba, Bw);

  // propagate uncertainty
  // TODO(frank): use noiseModel power: covariance is very expensive !
  const Matrix3 w = gyroscopeNoiseModel_->covariance() / dt;
  const Matrix3 a = accelerometerNoiseModel_->covariance() / dt;
  cov_ = A * cov_ * A.transpose() + Bw * w * Bw.transpose() +
         Ba * a * Ba.transpose();

  // increment counter and time
  k_ += 1;
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
}

Matrix9 AggregateImuReadings::preintMeasCov() const {
  return noiseModel()->covariance();
}

}  // namespace gtsam

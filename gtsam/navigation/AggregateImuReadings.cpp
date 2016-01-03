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
#include <cmath>

using namespace std;

namespace gtsam {

AggregateImuReadings::AggregateImuReadings(const boost::shared_ptr<Params>& p,
                                           const Bias& estimatedBias)
    : p_(p),
      accelerometerNoiseModel_(Diagonal(p->accelerometerCovariance)),
      gyroscopeNoiseModel_(Diagonal(p->gyroscopeCovariance)),
      estimatedBias_(estimatedBias),
      k_(0),
      deltaTij_(0.0) {
  zeta_.setZero();
  cov_.setZero();
}

SharedDiagonal AggregateImuReadings::discreteAccelerometerNoiseModel(
    double dt) const {
  return noiseModel::Diagonal::Sigmas(accelerometerNoiseModel_->sigmas() /
                                      std::sqrt(dt));
}

SharedDiagonal AggregateImuReadings::discreteGyroscopeNoiseModel(
    double dt) const {
  return noiseModel::Diagonal::Sigmas(gyroscopeNoiseModel_->sigmas() /
                                      std::sqrt(dt));
}

// Tangent space sugar.
namespace sugar {
typedef const Vector9 constV9;
static Eigen::Block<Vector9, 3, 1> dR(Vector9& v) { return v.segment<3>(0); }
static Eigen::Block<Vector9, 3, 1> dP(Vector9& v) { return v.segment<3>(3); }
static Eigen::Block<Vector9, 3, 1> dV(Vector9& v) { return v.segment<3>(6); }
static Eigen::Block<constV9, 3, 1> dR(constV9& v) { return v.segment<3>(0); }
static Eigen::Block<constV9, 3, 1> dP(constV9& v) { return v.segment<3>(3); }
static Eigen::Block<constV9, 3, 1> dV(constV9& v) { return v.segment<3>(6); }
}

Vector9 AggregateImuReadings::UpdateEstimate(
    const Vector9& zeta, const Vector3& correctedAcc,
    const Vector3& correctedOmega, double dt, OptionalJacobian<9, 9> A,
    OptionalJacobian<9, 3> Ba, OptionalJacobian<9, 3> Bw) {
  using namespace sugar;

  // Calculate exact mean propagation
  Matrix3 dexp;
  const Rot3 R = Rot3::Expmap(dR(zeta), dexp);
  const Matrix3 F = dexp.inverse() * dt, H = R.matrix() * dt;

  Vector9 zeta_plus;
  dR(zeta_plus) = dR(zeta) + F * correctedOmega;
  dP(zeta_plus) = dP(zeta) + dV(zeta) * dt + H * (0.5 * dt) * correctedAcc;
  dV(zeta_plus) = dV(zeta) + H * correctedAcc;

  if (A) {
    const Matrix3 Avt = skewSymmetric(-correctedAcc * dt);
    A->setIdentity();
    A->block<3, 3>(6, 0) = Avt;
    A->block<3, 3>(3, 6) = I_3x3 * dt;
  }
  if (Ba) *Ba << Z_3x3, H*(dt * 0.5), H;
  if (Bw) *Bw << F, Z_3x3, Z_3x3;

  return zeta_plus;
}

void AggregateImuReadings::integrateMeasurement(const Vector3& measuredAcc,
                                                const Vector3& measuredOmega,
                                                double dt) {
  // Correct measurements
  const Vector3 correctedAcc = measuredAcc - estimatedBias_.accelerometer();
  const Vector3 correctedOmega = measuredOmega - estimatedBias_.gyroscope();

  // Do exact mean propagation
  Matrix9 A;
  Matrix93 Ba, Bw;
  zeta_ = UpdateEstimate(zeta_, correctedAcc, correctedOmega, dt, A, Ba, Bw);

  // propagate uncertainty
  // TODO(frank): specialize to diagonal and upper triangular views
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
  // We have inv(Rp'Rp) = H inv(Rz'Rz) H' => Rp = Rz * inv(H)
  // From NavState::retract:
  Matrix3 D_R_theta;
  const Matrix3 iRj = Rot3::Expmap(theta(), D_R_theta).matrix();
  Matrix9 H;
  H << D_R_theta, Z_3x3, Z_3x3,       //
      Z_3x3, iRj.transpose(), Z_3x3,  //
      Z_3x3, Z_3x3, iRj.transpose();

  Matrix9 HcH = H * cov_ * H.transpose();
  return noiseModel::Gaussian::Covariance(cov_, false);

  //  // Get covariance on zeta from Bayes Net, which stores P(zeta|bias) as a
  //  // quadratic |R*zeta + S*bias -d|^2
  //  Matrix RS;
  //  Vector d;
  //  boost::tie(RS, d) = posterior_k_->matrix();
  //  // NOTEfrank): R'*R = inv(zetaCov)
  //
  //  Matrix9 R = RS.block<9, 9>(0, 0);
  //  cout << "R'R" << endl;
  //  cout << (R.transpose() * R).inverse() << endl;
  //  cout << "cov" << endl;
  //  cout << cov << endl;

  //  // Rp = R * H.inverse(), implemented blockwise in-place below
  //  // TODO(frank): yet another application of expmap and expmap derivative
  //  // NOTE(frank): makes sense: a change in the j-frame has to be converted
  //  // to a change in the i-frame, byy rotating with iRj. Similarly, a change
  //  // in rotation nRj is mapped to a change in theta via the inverse dexp.
  //  R.block<9, 3>(0, 0) *= D_R_theta.inverse();
  //  R.block<9, 3>(0, 3) *= iRj;
  //  R.block<9, 3>(0, 6) *= iRj;
  //
  //  // TODO(frank): think of a faster way - implement in noiseModel
  //  return noiseModel::Gaussian::SqrtInformation(R, false);
}

Matrix9 AggregateImuReadings::preintMeasCov() const {
  return noiseModel()->covariance();
}

}  // namespace gtsam

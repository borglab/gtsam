/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  AHRSFactor.cpp
 *  @author Krunal Chande
 *  @author Luca Carlone
 *  @date   July 2014
 **/

#include <gtsam/navigation/AHRSFactor.h>

/* GTSAM includes */
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/base/debug.h>

/* External or standard includes */
#include <ostream>

namespace gtsam {

//------------------------------------------------------------------------------
// Inner class PreintegratedMeasurements
//------------------------------------------------------------------------------
AHRSFactor::PreintegratedMeasurements::PreintegratedMeasurements(
    const imuBias::ConstantBias& bias, const Matrix3& measuredOmegaCovariance) :
    biasHat_(bias), deltaTij_(0.0) {
  measurementCovariance_ << measuredOmegaCovariance;
  delRdelBiasOmega_.setZero();
  PreintMeasCov_.setZero();
}

//------------------------------------------------------------------------------
AHRSFactor::PreintegratedMeasurements::PreintegratedMeasurements() :
    biasHat_(imuBias::ConstantBias()), deltaTij_(0.0) {
  measurementCovariance_.setZero();
  delRdelBiasOmega_.setZero();
  delRdelBiasOmega_.setZero();
  PreintMeasCov_.setZero();
}

//------------------------------------------------------------------------------
void AHRSFactor::PreintegratedMeasurements::print(const std::string& s) const {
  std::cout << s << std::endl;
  biasHat_.print(" biasHat");
  deltaRij_.print(" deltaRij ");
  std::cout << " measurementCovariance [" << measurementCovariance_ << " ]"
      << std::endl;
  std::cout << " PreintMeasCov [ " << PreintMeasCov_ << " ]" << std::endl;
}

//------------------------------------------------------------------------------
bool AHRSFactor::PreintegratedMeasurements::equals(
    const PreintegratedMeasurements& expected, double tol) const {
  return biasHat_.equals(expected.biasHat_, tol)
      && equal_with_abs_tol(measurementCovariance_,
          expected.measurementCovariance_, tol)
      && deltaRij_.equals(expected.deltaRij_, tol)
      && std::fabs(deltaTij_ - expected.deltaTij_) < tol
      && equal_with_abs_tol(delRdelBiasOmega_, expected.delRdelBiasOmega_, tol);
}

//------------------------------------------------------------------------------
void AHRSFactor::PreintegratedMeasurements::resetIntegration() {
  deltaRij_ = Rot3();
  deltaTij_ = 0.0;
  delRdelBiasOmega_.setZero();
  PreintMeasCov_.setZero();
}

//------------------------------------------------------------------------------
void AHRSFactor::PreintegratedMeasurements::integrateMeasurement(
    const Vector3& measuredOmega, double deltaT,
    boost::optional<const Pose3&> body_P_sensor) {

  // NOTE: order is important here because each update uses old values.
  // First we compensate the measurements for the bias
  Vector3 correctedOmega = biasHat_.correctGyroscope(measuredOmega);

  // Then compensate for sensor-body displacement: we express the quantities
  // (originally in the IMU frame) into the body frame
  if (body_P_sensor) {
    Matrix3 body_R_sensor = body_P_sensor->rotation().matrix();
    // rotation rate vector in the body frame
    correctedOmega = body_R_sensor * correctedOmega;
  }

  // rotation vector describing rotation increment computed from the
  // current rotation rate measurement
  const Vector3 theta_incr = correctedOmega * deltaT;

  // rotation increment computed from the current rotation rate measurement
  const Rot3 incrR = Rot3::Expmap(theta_incr);
  const Matrix3 incrRt = incrR.transpose();

  // Right Jacobian computed at theta_incr
  const Matrix3 Jr_theta_incr = Rot3::rightJacobianExpMapSO3(theta_incr);

  // Update Jacobians
  // ---------------------------------------------------------------------------
  delRdelBiasOmega_ = incrRt * delRdelBiasOmega_ - Jr_theta_incr * deltaT;

  // Update preintegrated measurements covariance
  // ---------------------------------------------------------------------------
  const Vector3 theta_i = Rot3::Logmap(deltaRij_); // Parameterization of so(3)
  const Matrix3 Jr_theta_i = Rot3::rightJacobianExpMapSO3inverse(theta_i);

  Rot3 Rot_j = deltaRij_ * incrR;
  const Vector3 theta_j = Rot3::Logmap(Rot_j); // Parameterization of so(3)
  const Matrix3 Jrinv_theta_j = Rot3::rightJacobianExpMapSO3inverse(theta_j);

  // Update preintegrated measurements covariance: as in [2] we consider a first
  // order propagation that can be seen as a prediction phase in an EKF framework
  Matrix3 H_angles_angles = Jrinv_theta_j * incrRt * Jr_theta_i;
  // analytic expression corresponding to the following numerical derivative
  // Matrix H_angles_angles = numericalDerivative11<LieVector, LieVector>
  // (boost::bind(&PreIntegrateIMUObservations_delta_angles, correctedOmega, deltaT, _1), thetaij);

  // overall Jacobian wrpt preintegrated measurements (df/dx)
  const Matrix3& F = H_angles_angles;

  // first order uncertainty propagation
  // the deltaT allows to pass from continuous time noise to discrete time noise
  PreintMeasCov_ = F * PreintMeasCov_ * F.transpose()
      + measurementCovariance_ * deltaT;

  // Update preintegrated measurements
  // ---------------------------------------------------------------------------
  deltaRij_ = deltaRij_ * incrR;
  deltaTij_ += deltaT;
}

//------------------------------------------------------------------------------
Vector AHRSFactor::PreintegratedMeasurements::PreIntegrateIMUObservations_delta_angles(
    const Vector& msr_gyro_t, const double msr_dt,
    const Vector3& delta_angles) {

  // Note: all delta terms refer to an IMU\sensor system at t0

  // Calculate the corrected measurements using the Bias object
  Vector body_t_omega_body = msr_gyro_t;

  Rot3 R_t_to_t0 = Rot3::Expmap(delta_angles);

  R_t_to_t0 = R_t_to_t0 * Rot3::Expmap(body_t_omega_body * msr_dt);
  return Rot3::Logmap(R_t_to_t0);
}

} //namespace gtsam

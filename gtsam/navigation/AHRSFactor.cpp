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
 *  @author Frank Dellaert
 *  @date   July 2014
 **/

#include <gtsam/navigation/AHRSFactor.h>

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
  preintMeasCov_.setZero();
}

//------------------------------------------------------------------------------
AHRSFactor::PreintegratedMeasurements::PreintegratedMeasurements() :
    biasHat_(imuBias::ConstantBias()), deltaTij_(0.0) {
  measurementCovariance_.setZero();
  delRdelBiasOmega_.setZero();
  delRdelBiasOmega_.setZero();
  preintMeasCov_.setZero();
}

//------------------------------------------------------------------------------
void AHRSFactor::PreintegratedMeasurements::print(const std::string& s) const {
  std::cout << s << std::endl;
  biasHat_.print(" biasHat");
  deltaRij_.print(" deltaRij ");
  std::cout << " measurementCovariance [" << measurementCovariance_ << " ]"
      << std::endl;
  std::cout << " PreintMeasCov [ " << preintMeasCov_ << " ]" << std::endl;
}

//------------------------------------------------------------------------------
bool AHRSFactor::PreintegratedMeasurements::equals(
    const PreintegratedMeasurements& other, double tol) const {
  return biasHat_.equals(other.biasHat_, tol)
      && equal_with_abs_tol(measurementCovariance_,
          other.measurementCovariance_, tol)
      && deltaRij_.equals(other.deltaRij_, tol)
      && std::fabs(deltaTij_ - other.deltaTij_) < tol
      && equal_with_abs_tol(delRdelBiasOmega_, other.delRdelBiasOmega_, tol);
}

//------------------------------------------------------------------------------
void AHRSFactor::PreintegratedMeasurements::resetIntegration() {
  deltaRij_ = Rot3();
  deltaTij_ = 0.0;
  delRdelBiasOmega_.setZero();
  preintMeasCov_.setZero();
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
  preintMeasCov_ = F * preintMeasCov_ * F.transpose()
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

//------------------------------------------------------------------------------
// AHRSFactor methods
//------------------------------------------------------------------------------
AHRSFactor::AHRSFactor() :
    preintegratedMeasurements_(imuBias::ConstantBias(), Matrix3::Zero()) {
}

AHRSFactor::AHRSFactor(Key rot_i, Key rot_j, Key bias,
    const PreintegratedMeasurements& preintegratedMeasurements,
    const Vector3& omegaCoriolis, boost::optional<const Pose3&> body_P_sensor) :
    Base(
        noiseModel::Gaussian::Covariance(
            preintegratedMeasurements.preintMeasCov_), rot_i, rot_j, bias), preintegratedMeasurements_(
        preintegratedMeasurements), omegaCoriolis_(omegaCoriolis), body_P_sensor_(
        body_P_sensor) {
}

/// @return a deep copy of this factor
gtsam::NonlinearFactor::shared_ptr AHRSFactor::clone() const {
  return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new This(*this)));
}

//------------------------------------------------------------------------------
void AHRSFactor::print(const std::string& s,
    const KeyFormatter& keyFormatter) const {
  std::cout << s << "AHRSFactor(" << keyFormatter(this->key1()) << ","
      << keyFormatter(this->key2()) << "," << keyFormatter(this->key3()) << ",";
  preintegratedMeasurements_.print("  preintegrated measurements:");
  std::cout << "  omegaCoriolis: [ " << omegaCoriolis_.transpose() << " ]"
      << std::endl;
  this->noiseModel_->print("  noise model: ");
  if (this->body_P_sensor_)
    this->body_P_sensor_->print("  sensor pose in body frame: ");
}

//------------------------------------------------------------------------------
bool AHRSFactor::equals(const NonlinearFactor& other, double tol) const {
  const This *e = dynamic_cast<const This*>(&other);
  return e != NULL && Base::equals(*e, tol)
      && preintegratedMeasurements_.equals(e->preintegratedMeasurements_, tol)
      && equal_with_abs_tol(omegaCoriolis_, e->omegaCoriolis_, tol)
      && ((!body_P_sensor_ && !e->body_P_sensor_)
          || (body_P_sensor_ && e->body_P_sensor_
              && body_P_sensor_->equals(*e->body_P_sensor_)));
}

//------------------------------------------------------------------------------
Vector AHRSFactor::evaluateError(const Rot3& rot_i, const Rot3& rot_j,
    const imuBias::ConstantBias& bias, boost::optional<Matrix&> H1,
    boost::optional<Matrix&> H2, boost::optional<Matrix&> H3) const {

  double deltaTij = preintegratedMeasurements_.deltaTij_;

  Vector3 biasOmegaIncr = bias.gyroscope()
      - preintegratedMeasurements_.biasHat_.gyroscope();

  // We compute factor's Jacobians
  /* ---------------------------------------------------------------------------------------------------- */
  Rot3 deltaRij_biascorrected = preintegratedMeasurements_.deltaRij_.retract(
      preintegratedMeasurements_.delRdelBiasOmega_ * biasOmegaIncr,
      Rot3::EXPMAP);

  Vector3 theta_biascorrected = Rot3::Logmap(deltaRij_biascorrected);

  Vector3 theta_biascorrected_corioliscorrected = theta_biascorrected
      - rot_i.inverse().matrix() * omegaCoriolis_ * deltaTij; // Coriolis term

  Rot3 deltaRij_biascorrected_corioliscorrected = Rot3::Expmap(
      theta_biascorrected_corioliscorrected);

  Rot3 fRhat = deltaRij_biascorrected_corioliscorrected.between(
      rot_i.between(rot_j));

  Matrix3 Jr_theta_bcc = Rot3::rightJacobianExpMapSO3(
      theta_biascorrected_corioliscorrected);

  Matrix3 Jtheta = -Jr_theta_bcc
      * skewSymmetric(rot_i.inverse().matrix() * omegaCoriolis_ * deltaTij);

  Matrix3 Jrinv_fRhat = Rot3::rightJacobianExpMapSO3inverse(
      Rot3::Logmap(fRhat));

  if (H1) {
    H1->resize(3, 3);
    (*H1) << // dfR/dRi
        Jrinv_fRhat
            * (-rot_j.between(rot_i).matrix()
                - fRhat.inverse().matrix() * Jtheta);
  }
  if (H2) {

    H2->resize(3, 3);
    (*H2) <<
    // dfR/dPosej
        Jrinv_fRhat * (Matrix3::Identity());
  }

  if (H3) {

    Matrix3 Jrinv_theta_bc = Rot3::rightJacobianExpMapSO3inverse(
        theta_biascorrected);
    Matrix3 Jr_JbiasOmegaIncr = Rot3::rightJacobianExpMapSO3(
        preintegratedMeasurements_.delRdelBiasOmega_ * biasOmegaIncr);
    Matrix3 JbiasOmega = Jr_theta_bcc * Jrinv_theta_bc * Jr_JbiasOmegaIncr
        * preintegratedMeasurements_.delRdelBiasOmega_;

    H3->resize(3, 6);
    (*H3) <<
    // dfR/dBias
        Matrix::Zero(3, 3), Jrinv_fRhat
        * (-fRhat.inverse().matrix() * JbiasOmega);
  }

  Vector3 fR = Rot3::Logmap(fRhat);

  Vector r(3);
  r << fR;
  return r;
}

//------------------------------------------------------------------------------
Rot3 AHRSFactor::predict(const Rot3& rot_i, const imuBias::ConstantBias& bias,
    const PreintegratedMeasurements preintegratedMeasurements,
    const Vector3& omegaCoriolis, boost::optional<const Pose3&> body_P_sensor) {

  const double& deltaTij = preintegratedMeasurements.deltaTij_;
//    const Vector3 biasAccIncr = bias.accelerometer()
  -preintegratedMeasurements.biasHat_.accelerometer();
  const Vector3 biasOmegaIncr = bias.gyroscope()
      - preintegratedMeasurements.biasHat_.gyroscope();

  const Rot3 Rot_i = rot_i;

  // Predict state at time j
  /* ---------------------------------------------------------------------------------------------------- */
  const Rot3 deltaRij_biascorrected =
      preintegratedMeasurements.deltaRij_.retract(
          preintegratedMeasurements.delRdelBiasOmega_ * biasOmegaIncr,
          Rot3::EXPMAP);
  // deltaRij_biascorrected is expmap(deltaRij) * expmap(delRdelBiasOmega * biasOmegaIncr)
  Vector3 theta_biascorrected = Rot3::Logmap(deltaRij_biascorrected);
  Vector3 theta_biascorrected_corioliscorrected = theta_biascorrected
      - Rot_i.inverse().matrix() * omegaCoriolis * deltaTij; // Coriolis term
  const Rot3 deltaRij_biascorrected_corioliscorrected = Rot3::Expmap(
      theta_biascorrected_corioliscorrected);
//    const Rot3 Rot_j =
  return (Rot_i.compose(deltaRij_biascorrected_corioliscorrected));

}

} //namespace gtsam

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
    const Vector3& bias, const Matrix3& measuredOmegaCovariance) :
    biasHat_(bias), deltaTij_(0.0) {
  measurementCovariance_ << measuredOmegaCovariance;
  delRdelBiasOmega_.setZero();
  preintMeasCov_.setZero();
}

//------------------------------------------------------------------------------
AHRSFactor::PreintegratedMeasurements::PreintegratedMeasurements() :
    biasHat_(Vector3()), deltaTij_(0.0) {
  measurementCovariance_.setZero();
  delRdelBiasOmega_.setZero();
  delRdelBiasOmega_.setZero();
  preintMeasCov_.setZero();
}

//------------------------------------------------------------------------------
void AHRSFactor::PreintegratedMeasurements::print(const std::string& s) const {
  std::cout << s << std::endl;
  std::cout << "biasHat [" << biasHat_.transpose() << "]" << std::endl;
  deltaRij_.print(" deltaRij ");
  std::cout << " measurementCovariance [" << measurementCovariance_ << " ]"
      << std::endl;
  std::cout << " PreintMeasCov [ " << preintMeasCov_ << " ]" << std::endl;
}

//------------------------------------------------------------------------------
bool AHRSFactor::PreintegratedMeasurements::equals(
    const PreintegratedMeasurements& other, double tol) const {
  return equal_with_abs_tol(biasHat_, other.biasHat_, tol)
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
  Vector3 correctedOmega = measuredOmega - biasHat_;

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
  // (boost::bind(&DeltaAngles, correctedOmega, deltaT, _1), thetaij);

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
Vector3 AHRSFactor::PreintegratedMeasurements::predict(const Vector3& bias,
    boost::optional<Matrix&> H) const {
  const Vector3 biasOmegaIncr = bias - biasHat_;
  Vector3 delRdelBiasOmega_biasOmegaIncr = delRdelBiasOmega_ * biasOmegaIncr;
  const Rot3 deltaRij_biascorrected = deltaRij_.retract(
      delRdelBiasOmega_biasOmegaIncr, Rot3::EXPMAP);
  const Vector3 theta_biascorrected = Rot3::Logmap(deltaRij_biascorrected);
  if (H) {
    const Matrix3 Jrinv_theta_bc = //
        Rot3::rightJacobianExpMapSO3inverse(theta_biascorrected);
    const Matrix3 Jr_JbiasOmegaIncr = //
        Rot3::rightJacobianExpMapSO3(delRdelBiasOmega_biasOmegaIncr);
    (*H) = Jrinv_theta_bc * Jr_JbiasOmegaIncr * delRdelBiasOmega_;
  }
  return theta_biascorrected;
}
//------------------------------------------------------------------------------
Vector AHRSFactor::PreintegratedMeasurements::DeltaAngles(
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
    preintegratedMeasurements_(Vector3(), Matrix3::Zero()) {
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

//------------------------------------------------------------------------------
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
  noiseModel_->print("  noise model: ");
  if (body_P_sensor_)
    body_P_sensor_->print("  sensor pose in body frame: ");
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
    const Vector3& bias, boost::optional<Matrix&> H1,
    boost::optional<Matrix&> H2, boost::optional<Matrix&> H3) const {

  // Do bias correction, if (H3) will contain 3*3 derivative used below
  const Vector3 theta_biascorrected = //
      preintegratedMeasurements_.predict(bias, H3);

  // Coriolis term
  const Vector3 coriolis = //
      preintegratedMeasurements_.integrateCoriolis(rot_i, omegaCoriolis_);
  const Vector3 theta_corrected = theta_biascorrected - coriolis;

  // Prediction
  const Rot3 deltaRij_corrected = Rot3::Expmap(theta_corrected);

  // Get error between actual and prediction
  const Rot3 actualRij = rot_i.between(rot_j);
  const Rot3 fRhat = deltaRij_corrected.between(actualRij);
  Vector3 fR = Rot3::Logmap(fRhat);

  // Terms common to derivatives
  const Matrix3 Jr_theta_bcc = Rot3::rightJacobianExpMapSO3(theta_corrected);
  const Matrix3 Jrinv_fRhat = Rot3::rightJacobianExpMapSO3inverse(fR);

  if (H1) {
    // dfR/dRi
    H1->resize(3, 3);
    Matrix3 Jtheta = -Jr_theta_bcc * skewSymmetric(coriolis);
    (*H1)
        << Jrinv_fRhat * (-actualRij.transpose() - fRhat.transpose() * Jtheta);
  }

  if (H2) {
    // dfR/dPosej
    H2->resize(3, 3);
    (*H2) << Jrinv_fRhat * Matrix3::Identity();
  }

  if (H3) {
    // dfR/dBias, note H3 contains derivative of predict
    const Matrix3 JbiasOmega = Jr_theta_bcc * (*H3);
    H3->resize(3, 3);
    (*H3) << Jrinv_fRhat * (-fRhat.transpose() * JbiasOmega);
  }

  Vector error(3);
  error << fR;
  return error;
}

//------------------------------------------------------------------------------
Rot3 AHRSFactor::predict(const Rot3& rot_i, const Vector3& bias,
    const PreintegratedMeasurements preintegratedMeasurements,
    const Vector3& omegaCoriolis, boost::optional<const Pose3&> body_P_sensor) {

  const Vector3 theta_biascorrected = preintegratedMeasurements.predict(bias);

  // Coriolis term
  const Vector3 coriolis = //
      preintegratedMeasurements.integrateCoriolis(rot_i, omegaCoriolis);

  const Vector3 theta_corrected = theta_biascorrected - coriolis;
  const Rot3 deltaRij_corrected = Rot3::Expmap(theta_corrected);

  return rot_i.compose(deltaRij_corrected);
}

} //namespace gtsam

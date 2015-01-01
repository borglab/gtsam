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
#include <iostream>

using namespace std;

namespace gtsam {

//------------------------------------------------------------------------------
// Inner class PreintegratedMeasurements
//------------------------------------------------------------------------------
AHRSFactor::PreintegratedMeasurements::PreintegratedMeasurements(
    const Vector3& bias, const Matrix3& measuredOmegaCovariance) :
    PreintegratedRotation(measuredOmegaCovariance), biasHat_(bias)
{
  preintMeasCov_.setZero();
}

//------------------------------------------------------------------------------
AHRSFactor::PreintegratedMeasurements::PreintegratedMeasurements() :
    PreintegratedRotation(I_3x3), biasHat_(Vector3())
{
  preintMeasCov_.setZero();
}

//------------------------------------------------------------------------------
void AHRSFactor::PreintegratedMeasurements::print(const string& s) const {
  PreintegratedRotation::print(s);
  cout << "biasHat [" << biasHat_.transpose() << "]" << endl;
  cout << " PreintMeasCov [ " << preintMeasCov_ << " ]" << endl;
}

//------------------------------------------------------------------------------
bool AHRSFactor::PreintegratedMeasurements::equals(
    const PreintegratedMeasurements& other, double tol) const {
  return PreintegratedRotation::equals(other, tol)
      && equal_with_abs_tol(biasHat_, other.biasHat_, tol);
}

//------------------------------------------------------------------------------
void AHRSFactor::PreintegratedMeasurements::resetIntegration() {
  PreintegratedRotation::resetIntegration();
  preintMeasCov_.setZero();
}

//------------------------------------------------------------------------------
void AHRSFactor::PreintegratedMeasurements::integrateMeasurement(
    const Vector3& measuredOmega, double deltaT,
    boost::optional<const Pose3&> body_P_sensor) {

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
  Matrix3 D_Rincr_integratedOmega;
  const Rot3 incrR = Rot3::Expmap(theta_incr, D_Rincr_integratedOmega); // expensive !!

  // Update Jacobian
  update_delRdelBiasOmega(D_Rincr_integratedOmega, incrR, deltaT);

  // Update rotation and deltaTij.
  Matrix3 Fr; // Jacobian of the update
  updateIntegratedRotationAndDeltaT(incrR, deltaT, Fr);

  // first order uncertainty propagation
  // the deltaT allows to pass from continuous time noise to discrete time noise
  preintMeasCov_ = Fr * preintMeasCov_ * Fr.transpose()
      + gyroscopeCovariance() * deltaT;
}

//------------------------------------------------------------------------------
Vector3 AHRSFactor::PreintegratedMeasurements::predict(const Vector3& bias,
    boost::optional<Matrix&> H) const {
  const Vector3 biasOmegaIncr = bias - biasHat_;
  return biascorrectedThetaRij(biasOmegaIncr, H);
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
    _PIM_(Vector3(), Z_3x3) {
}

AHRSFactor::AHRSFactor(Key rot_i, Key rot_j, Key bias,
    const PreintegratedMeasurements& preintegratedMeasurements,
    const Vector3& omegaCoriolis, boost::optional<const Pose3&> body_P_sensor) :
    Base(
        noiseModel::Gaussian::Covariance(
            preintegratedMeasurements.preintMeasCov_), rot_i, rot_j, bias), _PIM_(
        preintegratedMeasurements), omegaCoriolis_(omegaCoriolis), body_P_sensor_(
        body_P_sensor) {
}

//------------------------------------------------------------------------------
gtsam::NonlinearFactor::shared_ptr AHRSFactor::clone() const {
  return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new This(*this)));
}

//------------------------------------------------------------------------------
void AHRSFactor::print(const string& s,
    const KeyFormatter& keyFormatter) const {
  cout << s << "AHRSFactor(" << keyFormatter(this->key1()) << ","
      << keyFormatter(this->key2()) << "," << keyFormatter(this->key3()) << ",";
  _PIM_.print("  preintegrated measurements:");
  cout << "  omegaCoriolis: [ " << omegaCoriolis_.transpose() << " ]" << endl;
  noiseModel_->print("  noise model: ");
  if (body_P_sensor_)
    body_P_sensor_->print("  sensor pose in body frame: ");
}

//------------------------------------------------------------------------------
bool AHRSFactor::equals(const NonlinearFactor& other, double tol) const {
  const This *e = dynamic_cast<const This*>(&other);
  return e != NULL && Base::equals(*e, tol) && _PIM_.equals(e->_PIM_, tol)
      && equal_with_abs_tol(omegaCoriolis_, e->omegaCoriolis_, tol)
      && ((!body_P_sensor_ && !e->body_P_sensor_)
          || (body_P_sensor_ && e->body_P_sensor_
              && body_P_sensor_->equals(*e->body_P_sensor_)));
}

//------------------------------------------------------------------------------
Vector AHRSFactor::evaluateError(const Rot3& Ri, const Rot3& Rj,
    const Vector3& bias, boost::optional<Matrix&> H1,
    boost::optional<Matrix&> H2, boost::optional<Matrix&> H3) const {

  // Do bias correction, if (H3) will contain 3*3 derivative used below
  const Vector3 biascorrectedOmega = _PIM_.predict(bias, H3);

  // Coriolis term
  const Vector3 coriolis = _PIM_.integrateCoriolis(Ri, omegaCoriolis_);
  const Matrix3 coriolisHat = skewSymmetric(coriolis);
  const Vector3 correctedOmega = biascorrectedOmega - coriolis;

  // Prediction
  const Rot3 correctedDeltaRij = Rot3::Expmap(correctedOmega);

  // Get error between actual and prediction
  const Rot3 actualRij = Ri.between(Rj);
  const Rot3 fRrot = correctedDeltaRij.between(actualRij);
  Vector3 fR = Rot3::Logmap(fRrot);

  // Terms common to derivatives
  const Matrix3 D_cDeltaRij_cOmega = Rot3::ExpmapDerivative(correctedOmega);
  const Matrix3 D_fR_fRrot = Rot3::LogmapDerivative(fR);

  if (H1) {
    // dfR/dRi
    H1->resize(3, 3);
    Matrix3 D_coriolis = -D_cDeltaRij_cOmega * coriolisHat;
    (*H1)
        << D_fR_fRrot * (-actualRij.transpose() - fRrot.transpose() * D_coriolis);
  }

  if (H2) {
    // dfR/dPosej
    H2->resize(3, 3);
    (*H2) << D_fR_fRrot * Matrix3::Identity();
  }

  if (H3) {
    // dfR/dBias, note H3 contains derivative of predict
    const Matrix3 JbiasOmega = D_cDeltaRij_cOmega * (*H3);
    H3->resize(3, 3);
    (*H3) << D_fR_fRrot * (-fRrot.transpose() * JbiasOmega);
  }

  Vector error(3);
  error << fR;
  return error;
}

//------------------------------------------------------------------------------
Rot3 AHRSFactor::predict(const Rot3& rot_i, const Vector3& bias,
    const PreintegratedMeasurements preintegratedMeasurements,
    const Vector3& omegaCoriolis, boost::optional<const Pose3&> body_P_sensor) {

  const Vector3 biascorrectedOmega = preintegratedMeasurements.predict(bias);

  // Coriolis term
  const Vector3 coriolis = //
      preintegratedMeasurements.integrateCoriolis(rot_i, omegaCoriolis);

  const Vector3 correctedOmega = biascorrectedOmega - coriolis;
  const Rot3 correctedDeltaRij = Rot3::Expmap(correctedOmega);

  return rot_i.compose(correctedDeltaRij);
}

} //namespace gtsam

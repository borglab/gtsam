/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  PreintegratedRotation.cpp
 *  @author Luca Carlone
 *  @author Stephen Williams
 *  @author Richard Roberts
 *  @author Vadim Indelman
 *  @author David Jensen
 *  @author Frank Dellaert
 **/

#include "PreintegratedRotation.h"

using namespace std;

namespace gtsam {

void PreintegratedRotationParams::print(const string& s) const {
  cout << s << endl;
  cout << "gyroscopeCovariance:\n[\n" << gyroscopeCovariance << "\n]" << endl;
  if (omegaCoriolis)
    cout << "omegaCoriolis = (" << omegaCoriolis->transpose() << ")" << endl;
  if (body_P_sensor)
    body_P_sensor->print("body_P_sensor");
}

bool PreintegratedRotationParams::equals(
    const PreintegratedRotationParams& other, double tol) const {
  if (body_P_sensor) {
    if (!other.body_P_sensor
        || !assert_equal(*body_P_sensor, *other.body_P_sensor, tol))
      return false;
  }
  if (omegaCoriolis) {
    if (!other.omegaCoriolis
        || !equal_with_abs_tol(*omegaCoriolis, *other.omegaCoriolis, tol))
      return false;
  }
  return equal_with_abs_tol(gyroscopeCovariance, other.gyroscopeCovariance, tol);
}

void PreintegratedRotation::resetIntegration() {
  deltaTij_ = 0.0;
  deltaRij_ = Rot3();
  delRdelBiasOmega_ = Z_3x3;
}

void PreintegratedRotation::print(const string& s) const {
  cout << s;
  cout << "    deltaTij [" << deltaTij_ << "]" << endl;
  cout << "    deltaRij.ypr = (" << deltaRij_.ypr().transpose() << ")" << endl;
}

bool PreintegratedRotation::equals(const PreintegratedRotation& other,
    double tol) const {
  return this->matchesParamsWith(other)
      && deltaRij_.equals(other.deltaRij_, tol)
      && std::abs(deltaTij_ - other.deltaTij_) < tol
      && equal_with_abs_tol(delRdelBiasOmega_, other.delRdelBiasOmega_, tol);
}

Rot3 PreintegratedRotation::incrementalRotation(const Vector3& measuredOmega,
    const Vector3& biasHat, double deltaT,
    OptionalJacobian<3, 3> D_incrR_integratedOmega) const {

  // First we compensate the measurements for the bias
  Vector3 correctedOmega = measuredOmega - biasHat;

  // Then compensate for sensor-body displacement: we express the quantities
  // (originally in the IMU frame) into the body frame
  if (p_->body_P_sensor) {
    Matrix3 body_R_sensor = p_->body_P_sensor->rotation().matrix();
    // rotation rate vector in the body frame
    correctedOmega = body_R_sensor * correctedOmega;
  }

  // rotation vector describing rotation increment computed from the
  // current rotation rate measurement
  const Vector3 integratedOmega = correctedOmega * deltaT;
  return Rot3::Expmap(integratedOmega, D_incrR_integratedOmega); // expensive !!
}

void PreintegratedRotation::integrateMeasurement(const Vector3& measuredOmega,
    const Vector3& biasHat, double deltaT,
    OptionalJacobian<3, 3> optional_D_incrR_integratedOmega,
    OptionalJacobian<3, 3> F) {
  Matrix3 D_incrR_integratedOmega;
  const Rot3 incrR = incrementalRotation(measuredOmega, biasHat, deltaT,
      D_incrR_integratedOmega);

  // If asked, pass first derivative as well
  if (optional_D_incrR_integratedOmega) {
    *optional_D_incrR_integratedOmega << D_incrR_integratedOmega;
  }

  // Update deltaTij and rotation
  deltaTij_ += deltaT;
  deltaRij_ = deltaRij_.compose(incrR, F);

  // Update Jacobian
  const Matrix3 incrRt = incrR.transpose();
  delRdelBiasOmega_ = incrRt * delRdelBiasOmega_
      - D_incrR_integratedOmega * deltaT;
}

Rot3 PreintegratedRotation::biascorrectedDeltaRij(const Vector3& biasOmegaIncr,
    OptionalJacobian<3, 3> H) const {
  const Vector3 biasInducedOmega = delRdelBiasOmega_ * biasOmegaIncr;
  const Rot3 deltaRij_biascorrected = deltaRij_.expmap(biasInducedOmega,
      boost::none, H);
  if (H)
    (*H) *= delRdelBiasOmega_;
  return deltaRij_biascorrected;
}

Vector3 PreintegratedRotation::integrateCoriolis(const Rot3& rot_i) const {
  if (!p_->omegaCoriolis)
    return Vector3::Zero();
  return rot_i.transpose() * (*p_->omegaCoriolis) * deltaTij_;
}

} // namespace gtsam

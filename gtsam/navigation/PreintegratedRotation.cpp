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

#include <gtsam/base/MatrixConstants.h>

#include <stdexcept>

using namespace std;

namespace gtsam {

void PreintegratedRotationParams::print(const string& s) const {
  cout << (s.empty() ? s : s + "\n") << endl;
  cout << "gyroscopeCovariance:\n" << gyroscopeCovariance << endl;
  if (omegaCoriolis)
    cout << "omegaCoriolis = (" << omegaCoriolis->transpose() << ")" << endl;
  if (body_P_sensor) body_P_sensor->print("body_P_sensor");
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

namespace internal {
Rot3 IncrementalRotation::operator()(
    const Vector3& bias, OptionalJacobian<3, 3> H_bias) const {
  // First we compensate the measurements for the bias
  Vector3 correctedOmega = measuredOmega - bias;

  // Then compensate for sensor-body displacement: we express the quantities
  // (originally in the IMU frame) into the body frame.
  // Note that the rotate Jacobian is just body_P_sensor->rotation().matrix().
  if (body_P_sensor) {
    // rotation rate vector in the body frame
    correctedOmega = body_P_sensor->rotation() * correctedOmega;
  }

  // rotation vector describing rotation increment computed from the
  // current rotation rate measurement
  const Vector3 integratedOmega = correctedOmega * deltaT;
  Rot3 incrR = Rot3::Expmap(integratedOmega, H_bias);  // expensive !!
  if (H_bias) {
    *H_bias *= -deltaT;  // Correct so accurately reflects bias derivative
    if (body_P_sensor) *H_bias *= body_P_sensor->rotation().matrix();
  }
  return incrR;
}
}  // namespace internal

void PreintegratedRotation::integrateGyroMeasurement(
    const Vector3& measuredOmega, const Vector3& biasHat, double deltaT,
    OptionalJacobian<3, 3> F) {
  Matrix3 H_bias;
  internal::IncrementalRotation f{measuredOmega, deltaT, p_->body_P_sensor};
  const Rot3 incrR = f(biasHat, H_bias);

  // Update deltaTij and rotation
  deltaTij_ += deltaT;
  deltaRij_ = deltaRij_.compose(incrR, F);

  // Update Jacobian
  const Matrix3 incrRt = incrR.transpose();
  delRdelBiasOmega_ = incrRt * delRdelBiasOmega_ + H_bias;
}

namespace {

void validateGyroIntegrationInputs(const Vector& times,
                                   ConstMatrixView measuredOmegas) {
  if (times.size() < 2) {
    throw invalid_argument("Gyro integration requires at least two samples");
  }
  if (measuredOmegas.rows() != times.size() || measuredOmegas.cols() != 3) {
    throw invalid_argument(
        "Gyro integration requires an Mx3 omega matrix with one row per "
        "sample time");
  }
  for (Eigen::Index index = 0; index + 1 < times.size(); ++index) {
    if (times(index + 1) <= times(index)) {
      throw invalid_argument(
          "Gyro integration requires strictly increasing sample times");
    }
  }
}

Vector3 correctedOmega(ConstMatrixView measuredOmegas, Eigen::Index row,
                       const Vector3& biasHat, const Rot3& body_R_sensor) {
  const Vector3 measuredOmega = measuredOmegas.row(row).transpose();
  return body_R_sensor * (measuredOmega - biasHat);
}

}  // namespace

Rot3 integrateSequentialRotations(const Vector& times,
                                  ConstMatrixView measuredOmegas,
                                  const Vector3& biasHat,
                                  const Rot3& body_R_sensor) {
  validateGyroIntegrationInputs(times, measuredOmegas);

  Rot3 rotation;
  for (Eigen::Index index = 0; index + 1 < times.size(); ++index) {
    const double deltaTime = times(index + 1) - times(index);
    const Vector3 omega0 =
        correctedOmega(measuredOmegas, index, biasHat, body_R_sensor);
    const Vector3 omega1 =
        correctedOmega(measuredOmegas, index + 1, biasHat, body_R_sensor);
    const Vector3 theta = 0.5 * deltaTime * (omega0 + omega1);
    rotation = rotation.compose(Rot3::Expmap(theta));
  }
  return rotation;
}

Rot3 integrateSingleSpeedConing(const Vector& times,
                                ConstMatrixView measuredOmegas,
                                const Vector3& biasHat,
                                const Rot3& body_R_sensor) {
  validateGyroIntegrationInputs(times, measuredOmegas);

  Rot3 rotation;
  Vector3 previousTheta = Vector3::Zero();
  for (Eigen::Index index = 0; index + 1 < times.size(); ++index) {
    const double deltaTime = times(index + 1) - times(index);
    const Vector3 omega0 =
        correctedOmega(measuredOmegas, index, biasHat, body_R_sensor);
    const Vector3 omega1 =
        correctedOmega(measuredOmegas, index + 1, biasHat, body_R_sensor);
    const Vector3 theta = 0.5 * deltaTime * (omega0 + omega1);
    const Vector3 correctedTheta =
        index == 0 ? theta
                   : theta + (1.0 / 12.0) * previousTheta.cross(theta);
    rotation = rotation.compose(Rot3::Expmap(correctedTheta));
    previousTheta = theta;
  }
  return rotation;
}

#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V43
void PreintegratedRotation::integrateMeasurement(
    const Vector3& measuredOmega, const Vector3& biasHat, double deltaT,
    OptionalJacobian<3, 3> optional_D_incrR_integratedOmega,
    OptionalJacobian<3, 3> F) {
  integrateGyroMeasurement(measuredOmega, biasHat, deltaT, F);

  // If asked, pass obsolete Jacobians as well
  if (optional_D_incrR_integratedOmega) {
    Matrix3 H_bias;
    internal::IncrementalRotation f{measuredOmega, deltaT, p_->body_P_sensor};
    const Rot3 incrR = f(biasHat, H_bias);
    *optional_D_incrR_integratedOmega << H_bias / -deltaT;
  }
}
#endif

Rot3 PreintegratedRotation::biascorrectedDeltaRij(const Vector3& biasOmegaIncr,
    OptionalJacobian<3, 3> H) const {
  const Vector3 biasInducedOmega = delRdelBiasOmega_ * biasOmegaIncr;
  const Rot3 deltaRij_biascorrected = deltaRij_.expmap(biasInducedOmega,{}, H);
  if (H)
    (*H) *= delRdelBiasOmega_;
  return deltaRij_biascorrected;
}

Vector3 PreintegratedRotation::integrateCoriolis(
    const Rot3& Ri, OptionalJacobian<3, 3> H) const {
  if (!p_->omegaCoriolis) return Vector3::Zero();
  return Ri.unrotate(*p_->omegaCoriolis * deltaTij_, H);
}

}  // namespace gtsam

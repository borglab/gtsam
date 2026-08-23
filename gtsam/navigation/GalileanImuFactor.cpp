/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GalileanImuFactor.cpp
 * @brief   Left-invariant Galilean IMU preintegration
 * @author  Frank Dellaert
 * @author  Giulio Delama
 */

#include <gtsam/base/MatrixConstants.h>
#include <gtsam/navigation/GalileanImuFactor.h>

#include <stdexcept>

namespace gtsam {

//------------------------------------------------------------------------------
PreintegratedImuMeasurementsG::Matrix910
PreintegratedImuMeasurementsG::NavStateProjector() {
  Matrix910 P = Matrix910::Zero();
  P.block<3, 3>(0, 0) = I_3x3;
  P.block<3, 3>(3, 6) = I_3x3;
  P.block<3, 3>(6, 3) = I_3x3;
  return P;
}

//------------------------------------------------------------------------------
PreintegratedImuMeasurementsG::PreintegratedImuMeasurementsG(
    const std::shared_ptr<Params>& p, const imuBias::ConstantBias& biasHat)
    : Base(p, biasHat) {
  resetIntegration();
}

//------------------------------------------------------------------------------
void PreintegratedImuMeasurementsG::resetIntegration() {
  deltaTij_ = 0.0;
  preintMatrix_ = Gal3::Identity();
  preintMeasCov_.setZero();
  biasJacobian_.setZero();
}

//------------------------------------------------------------------------------
void PreintegratedImuMeasurementsG::updateGal3(const Vector3& measuredAcc,
                                               const Vector3& measuredOmega,
                                               double dt, Matrix10* A,
                                               Matrix103* B, Matrix103* C) {
  Vector3 acc = biasHat_.correctAccelerometer(measuredAcc);
  Vector3 omega = biasHat_.correctGyroscope(measuredOmega);

  Matrix3 correctedAcc_H_acc = I_3x3;
  Matrix3 correctedAcc_H_omega = Z_3x3;
  Matrix3 correctedOmega_H_omega = I_3x3;
  if (p().body_P_sensor) {
    std::tie(acc, omega) = correctMeasurementsBySensorPose(
        acc, omega, correctedAcc_H_acc, correctedAcc_H_omega,
        correctedOmega_H_omega);
  }

  Vector10 integratedInput = Vector10::Zero();
  integratedInput.head<3>() = omega * dt;
  integratedInput.segment<3>(3) = acc * dt;
  integratedInput(9) = dt;

  Matrix10 rightJacobian;
  const Gal3 increment = Gal3::Expmap(integratedInput, rightJacobian);
  const Matrix10 transition = increment.inverse().AdjointMap();

  Matrix103 accelerationJacobian = rightJacobian.block<10, 3>(0, 3) * dt;
  Matrix103 omegaJacobian = rightJacobian.block<10, 3>(0, 0) * dt;
  omegaJacobian = accelerationJacobian * correctedAcc_H_omega +
                  omegaJacobian * correctedOmega_H_omega;
  accelerationJacobian *= correctedAcc_H_acc;

  preintMatrix_ = preintMatrix_.compose(increment);
  deltaTij_ += dt;

  Matrix106 inputJacobian;
  inputJacobian << accelerationJacobian, omegaJacobian;
  biasJacobian_ = transition * biasJacobian_ - inputJacobian;

  if (A) *A = transition;
  if (B) *B = accelerationJacobian;
  if (C) *C = omegaJacobian;
}

//------------------------------------------------------------------------------
void PreintegratedImuMeasurementsG::update(const Vector3& measuredAcc,
                                           const Vector3& measuredOmega,
                                           double dt, Matrix9* A, Matrix93* B,
                                           Matrix93* C) {
  Matrix10 galTransition;
  Matrix103 galAcceleration, galOmega;
  updateGal3(measuredAcc, measuredOmega, dt, &galTransition, &galAcceleration,
             &galOmega);

  const Matrix910 P = NavStateProjector();
  if (A) *A = P * galTransition * P.transpose();
  if (B) *B = P * galAcceleration;
  if (C) *C = P * galOmega;
}

//------------------------------------------------------------------------------
void PreintegratedImuMeasurementsG::integrateMeasurement(
    const Vector3& measuredAcc, const Vector3& measuredOmega, double dt) {
  if (dt <= 0) {
    throw std::runtime_error(
        "PreintegratedImuMeasurementsG::integrateMeasurement: dt <=0");
  }

  Matrix10 transition;
  Matrix103 accelerationJacobian, omegaJacobian;
  updateGal3(measuredAcc, measuredOmega, dt, &transition, &accelerationJacobian,
             &omegaJacobian);

  preintMeasCov_ = transition * preintMeasCov_ * transition.transpose();
  preintMeasCov_.noalias() += accelerationJacobian *
                              (p().accelerometerCovariance / dt) *
                              accelerationJacobian.transpose();
  preintMeasCov_.noalias() += omegaJacobian * (p().gyroscopeCovariance / dt) *
                              omegaJacobian.transpose();
  preintMeasCov_.block<3, 3>(6, 6).noalias() += p().integrationCovariance * dt;
}

//------------------------------------------------------------------------------
Matrix9 PreintegratedImuMeasurementsG::preintMeasCov() const {
  const Matrix910 P = NavStateProjector();
  return P * preintMeasCov_ * P.transpose();
}

//------------------------------------------------------------------------------
Vector9 PreintegratedImuMeasurementsG::biasCorrectedDelta(
    const imuBias::ConstantBias& bias_i, OptionalJacobian<9, 6> H) const {
  const Vector6 biasIncrement = (bias_i - biasHat_).vector();
  const Vector10 correction = biasJacobian_ * biasIncrement;

  Matrix10 correction_H_vector;
  const Gal3 correctionGroup =
      Gal3::Expmap(correction, H ? &correction_H_vector : nullptr);
  Matrix10 corrected_H_correctionGroup;
  const Gal3 corrected = preintMatrix_.compose(
      correctionGroup, {}, H ? &corrected_H_correctionGroup : nullptr);

  Matrix3 logRotation_H_rotation;
  Eigen::Matrix<double, 3, 10> rotation_H_corrected, position_H_corrected,
      velocity_H_corrected;

  Vector9 result;
  NavState::dR(result) =
      Rot3::Logmap(corrected.rotation(H ? &rotation_H_corrected : nullptr),
                   H ? &logRotation_H_rotation : nullptr);
  NavState::dP(result) =
      corrected.translation(H ? &position_H_corrected : nullptr);
  NavState::dV(result) =
      corrected.velocity(H ? &velocity_H_corrected : nullptr);

  if (H) {
    Matrix910 result_H_corrected = Matrix910::Zero();
    result_H_corrected.block<3, 10>(0, 0) =
        logRotation_H_rotation * rotation_H_corrected;
    result_H_corrected.block<3, 10>(3, 0) = position_H_corrected;
    result_H_corrected.block<3, 10>(6, 0) = velocity_H_corrected;
    *H = result_H_corrected * corrected_H_correctionGroup *
         correction_H_vector * biasJacobian_;
  }
  return result;
}

//------------------------------------------------------------------------------
void PreintegratedImuMeasurementsG::print(const std::string& s) const {
  Base::print(s);
  std::cout << "    preintMatrix = " << preintMatrix_ << '\n'
            << "    preintMeasCov (Gal3) =\n[" << preintMeasCov_ << "]\n"
            << "    biasJacobian =\n[" << biasJacobian_ << "]" << std::endl;
}

//------------------------------------------------------------------------------
bool PreintegratedImuMeasurementsG::equals(
    const PreintegratedImuMeasurementsG& other, double tol) const {
  return p().equals(other.p(), tol) && biasHat_.equals(other.biasHat_, tol) &&
         std::abs(deltaTij_ - other.deltaTij_) <= tol &&
         preintMatrix_.equals(other.preintMatrix_, tol) &&
         equal_with_abs_tol(preintMeasCov_, other.preintMeasCov_, tol) &&
         equal_with_abs_tol(biasJacobian_, other.biasJacobian_, tol);
}

}  // namespace gtsam

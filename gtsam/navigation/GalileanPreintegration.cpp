/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GalileanPreintegration.cpp
 * @brief   Left-invariant Galilean IMU preintegration
 * @author  Frank Dellaert
 * @author  Giulio Delama
 */

#include <gtsam/base/MatrixConstants.h>
#include <gtsam/navigation/GalileanPreintegration.h>

namespace gtsam {

//------------------------------------------------------------------------------
GalileanPreintegration::Matrix910 GalileanPreintegration::NavStateProjector() {
  Matrix910 P = Matrix910::Zero();
  P.block<3, 3>(0, 0) = I_3x3;
  P.block<3, 3>(3, 6) = I_3x3;
  P.block<3, 3>(6, 3) = I_3x3;
  return P;
}

//------------------------------------------------------------------------------
GalileanPreintegration::Matrix106 GalileanPreintegration::LiftJacobian() {
  Matrix106 E = Matrix106::Zero();
  E.block<3, 3>(0, 3) = I_3x3;
  E.block<3, 3>(3, 0) = I_3x3;
  return E;
}

//------------------------------------------------------------------------------
GalileanPreintegration::GalileanPreintegration(
    const std::shared_ptr<Params>& p, const imuBias::ConstantBias& biasHat)
    : Base(p, biasHat) {
  resetIntegration();
}

//------------------------------------------------------------------------------
void GalileanPreintegration::resetIntegration() {
  deltaTij_ = 0.0;
  preintMatrix_ = Gal3::Identity();
  biasJacobian_.setZero();
}

//------------------------------------------------------------------------------
void GalileanPreintegration::updateGal3(const Vector3& measuredAcc,
                                        const Vector3& measuredOmega, double dt,
                                        Matrix10* A, Matrix106* B) {
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

  Matrix6 correctedInput_H_input = Matrix6::Zero();
  correctedInput_H_input.block<3, 3>(0, 0) = correctedAcc_H_acc;
  correctedInput_H_input.block<3, 3>(0, 3) = correctedAcc_H_omega;
  correctedInput_H_input.block<3, 3>(3, 3) = correctedOmega_H_omega;
  const Matrix106 inputJacobian =
      rightJacobian * LiftJacobian() * correctedInput_H_input * dt;

  preintMatrix_ = preintMatrix_.compose(increment);
  deltaTij_ += dt;

  biasJacobian_ = transition * biasJacobian_ - inputJacobian;

  if (A) *A = transition;
  if (B) *B = inputJacobian;
}

//------------------------------------------------------------------------------
void GalileanPreintegration::update(const Vector3& measuredAcc,
                                    const Vector3& measuredOmega, double dt,
                                    Matrix9* A, Matrix93* B, Matrix93* C) {
  Matrix10 galTransition;
  Matrix106 galInput;
  updateGal3(measuredAcc, measuredOmega, dt, &galTransition, &galInput);

  const Matrix910 P = NavStateProjector();
  if (A) *A = P * galTransition * P.transpose();
  if (B) *B = P * galInput.leftCols<3>();
  if (C) *C = P * galInput.rightCols<3>();
}

//------------------------------------------------------------------------------
Vector9 GalileanPreintegration::biasCorrectedDelta(
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
void GalileanPreintegration::print(const std::string& s) const {
  Base::print(s);
  std::cout << "    preintMatrix = " << preintMatrix_ << '\n'
            << "    biasJacobian =\n[" << biasJacobian_ << "]" << std::endl;
}

//------------------------------------------------------------------------------
bool GalileanPreintegration::equals(const GalileanPreintegration& other,
                                    double tol) const {
  return p().equals(other.p(), tol) && biasHat_.equals(other.biasHat_, tol) &&
         std::abs(deltaTij_ - other.deltaTij_) <= tol &&
         preintMatrix_.equals(other.preintMatrix_, tol) &&
         equal_with_abs_tol(biasJacobian_, other.biasJacobian_, tol);
}

}  // namespace gtsam

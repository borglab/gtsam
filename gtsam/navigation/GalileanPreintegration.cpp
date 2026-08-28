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

  Matrix6 correctedInput_H_input;
  if (p().body_P_sensor) {
    Matrix3 correctedAcc_H_acc, correctedAcc_H_omega, correctedOmega_H_omega;
    std::tie(acc, omega) = correctMeasurementsBySensorPose(
        acc, omega, correctedAcc_H_acc, correctedAcc_H_omega,
        correctedOmega_H_omega);
    correctedInput_H_input.setZero();
    correctedInput_H_input.block<3, 3>(0, 0) = correctedAcc_H_acc;
    correctedInput_H_input.block<3, 3>(0, 3) = correctedAcc_H_omega;
    correctedInput_H_input.block<3, 3>(3, 3) = correctedOmega_H_omega;
  }

  Vector10 integratedInput = Vector10::Zero();
  integratedInput.head<3>() = omega * dt;
  integratedInput.segment<3>(3) = acc * dt;
  integratedInput(9) = dt;

  Matrix10 rightJacobian;
  const Gal3 increment = Gal3::Expmap(integratedInput, rightJacobian);
  const Matrix10 transition = increment.inverse().AdjointMap();

  Matrix106 liftedInputJacobian;
  liftedInputJacobian.leftCols<3>() = rightJacobian.middleCols<3>(3);
  liftedInputJacobian.rightCols<3>() = rightJacobian.leftCols<3>();
  Matrix106 inputJacobian;
  if (p().body_P_sensor) {
    inputJacobian.noalias() = liftedInputJacobian * correctedInput_H_input * dt;
  } else {
    inputJacobian = liftedInputJacobian * dt;
  }

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

  constexpr int kGal3BlockForNavState[3] = {0, 6, 3};
  for (int row = 0; row < 3; ++row) {
    const int galRow = kGal3BlockForNavState[row];
    if (B) {
      B->block<3, 3>(3 * row, 0) = galInput.block<3, 3>(galRow, 0);
    }
    if (C) {
      C->block<3, 3>(3 * row, 0) = galInput.block<3, 3>(galRow, 3);
    }
    if (A) {
      for (int column = 0; column < 3; ++column) {
        const int galColumn = kGal3BlockForNavState[column];
        A->block<3, 3>(3 * row, 3 * column) =
            galTransition.block<3, 3>(galRow, galColumn);
      }
    }
  }
}

//------------------------------------------------------------------------------
Vector9 GalileanPreintegration::NavStateTangent(const Gal3& galilean,
                                                OptionalJacobian<9, 10> H) {
  Matrix3 logRotation_H_rotation;
  Eigen::Matrix<double, 3, 10> rotation_H_galilean, position_H_galilean,
      velocity_H_galilean;

  Vector9 result;
  NavState::dR(result) =
      Rot3::Logmap(galilean.rotation(H ? &rotation_H_galilean : nullptr),
                   H ? &logRotation_H_rotation : nullptr);
  NavState::dP(result) =
      galilean.translation(H ? &position_H_galilean : nullptr);
  NavState::dV(result) = galilean.velocity(H ? &velocity_H_galilean : nullptr);

  if (H) {
    H->setZero();
    H->block<3, 10>(0, 0) = logRotation_H_rotation * rotation_H_galilean;
    H->block<3, 10>(3, 0) = position_H_galilean;
    H->block<3, 10>(6, 0) = velocity_H_galilean;
  }
  return result;
}

//------------------------------------------------------------------------------
Vector9 GalileanPreintegration::biasCorrectedDelta(
    const imuBias::ConstantBias& bias_i, OptionalJacobian<9, 6> H) const {
  const Vector6 biasIncrement = (bias_i - biasHat_).vector();
  if (biasIncrement.isZero(0.0)) {
    Matrix910 result_H_preintegrated;
    const Vector9 result =
        NavStateTangent(preintMatrix_, H ? &result_H_preintegrated : nullptr);
    if (H) *H = result_H_preintegrated * biasJacobian_;
    return result;
  }

  const Vector10 correction = biasJacobian_ * biasIncrement;

  Matrix10 correction_H_vector;
  const Gal3 correctionGroup =
      Gal3::Expmap(correction, H ? &correction_H_vector : nullptr);
  Matrix10 corrected_H_correctionGroup;
  const Gal3 corrected = preintMatrix_.compose(
      correctionGroup, {}, H ? &corrected_H_correctionGroup : nullptr);

  Matrix910 result_H_corrected;
  const Vector9 result =
      NavStateTangent(corrected, H ? &result_H_corrected : nullptr);

  if (H) {
    const Matrix106 correction_H_bias = correction_H_vector * biasJacobian_;
    const Matrix106 corrected_H_bias =
        corrected_H_correctionGroup * correction_H_bias;
    *H = result_H_corrected * corrected_H_bias;
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

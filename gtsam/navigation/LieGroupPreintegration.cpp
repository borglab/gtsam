/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved.
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file LieGroupPreintegration.cpp
 * @brief IMU preintegration using the SE_2(3) group structure of NavState.
 */

#include <gtsam/base/MatrixConstants.h>
#include <gtsam/navigation/LieGroupPreintegration.h>

namespace gtsam {

/* ************************************************************************* */
LieGroupPreintegration::LieGroupPreintegration(
    const std::shared_ptr<Params>& params, const imuBias::ConstantBias& biasHat)
    : ManifoldPreintegration(params, biasHat) {}

/* ************************************************************************* */
void LieGroupPreintegration::updateBiasJacobians(
    const Rot3& oldRotation, const Vector3& /*bodyAcceleration*/,
    const Vector3& /*bodyOmega*/, double /*dt*/, const Matrix9& stateTransition,
    const Matrix93& accelerationJacobian, const Matrix93& omegaJacobian) {
  const Matrix3 oldRotationTranspose = oldRotation.transpose();
  Matrix93 oldState_H_biasAcc, oldState_H_biasOmega;
  oldState_H_biasAcc << Z_3x3, oldRotationTranspose * delPdelBiasAcc_,
      oldRotationTranspose * delVdelBiasAcc_;
  oldState_H_biasOmega << delRdelBiasOmega_,
      oldRotationTranspose * delPdelBiasOmega_,
      oldRotationTranspose * delVdelBiasOmega_;

  const Matrix93 newState_H_biasAcc =
      stateTransition * oldState_H_biasAcc - accelerationJacobian;
  const Matrix93 newState_H_biasOmega =
      stateTransition * oldState_H_biasOmega - omegaJacobian;
  const Matrix3 newRotation = deltaXij_.R();
  delRdelBiasOmega_ = newState_H_biasOmega.topRows<3>();
  delPdelBiasAcc_ = newRotation * newState_H_biasAcc.middleRows<3>(3);
  delPdelBiasOmega_ = newRotation * newState_H_biasOmega.middleRows<3>(3);
  delVdelBiasAcc_ = newRotation * newState_H_biasAcc.bottomRows<3>();
  delVdelBiasOmega_ = newRotation * newState_H_biasOmega.bottomRows<3>();
}

/* ************************************************************************* */
void LieGroupPreintegration::updateFactor(const Vector3& bodyAcceleration,
                                          const Vector3& bodyOmega, double dt,
                                          OptionalJacobian<9, 9> F,
                                          OptionalJacobian<9, 3> G1,
                                          OptionalJacobian<9, 3> G2) {
  const bool computeJacobians = F || G1 || G2;
  Matrix39 bodyVelocity_H_state;
  const Vector3 bodyVelocity =
      deltaXij_.bodyVelocity(F ? &bodyVelocity_H_state : nullptr);
  const double halfDtSquared = 0.5 * dt * dt;

  Matrix3 incrementRotation_H_integratedOmega;
  const Rot3 incrementRotation = Rot3::Expmap(
      dt * bodyOmega, G2 ? &incrementRotation_H_integratedOmega : nullptr);
  Matrix93 increment_H_rotation, increment_H_position, increment_H_velocity;
  const NavState incrementState = NavState::Create(
      incrementRotation, dt * bodyVelocity + halfDtSquared * bodyAcceleration,
      dt * bodyAcceleration, computeJacobians ? &increment_H_rotation : nullptr,
      computeJacobians ? &increment_H_position : nullptr,
      computeJacobians ? &increment_H_velocity : nullptr);

  Matrix9 tangent_H_increment;
  const Vector9 increment = NavState::Logmap(
      incrementState, computeJacobians ? &tangent_H_increment : nullptr);

  Matrix9 newState_H_increment;
  deltaXij_ = deltaXij_.expmap(
      increment, F, computeJacobians ? &newState_H_increment : nullptr);

  if (F) {
    *F += newState_H_increment * tangent_H_increment * increment_H_position *
          dt * bodyVelocity_H_state;
  }
  if (G1) {
    *G1 = newState_H_increment * tangent_H_increment *
          (increment_H_position * halfDtSquared + increment_H_velocity * dt);
  }
  if (G2) {
    *G2 = newState_H_increment * tangent_H_increment * increment_H_rotation *
          incrementRotation_H_integratedOmega * dt;
  }
}

/* ************************************************************************* */
Vector9 LieGroupPreintegration::biasCorrectedDelta(
    const imuBias::ConstantBias& bias, OptionalJacobian<9, 6> H) const {
  const imuBias::ConstantBias biasIncrement = bias - biasHat_;
  const Vector3 rotationCorrection =
      delRdelBiasOmega_ * biasIncrement.gyroscope();

  Matrix3 correctedRotation_H_correction;
  const Rot3 correctedRotation = deltaRij().expmap(
      rotationCorrection, {}, H ? &correctedRotation_H_correction : nullptr);
  if (H) correctedRotation_H_correction *= delRdelBiasOmega_;

  Vector9 correctionTangent;
  NavState::dR(correctionTangent) = rotationCorrection;
  NavState::dP(correctionTangent) =
      delPdelBiasAcc_ * biasIncrement.accelerometer() +
      delPdelBiasOmega_ * biasIncrement.gyroscope();
  NavState::dV(correctionTangent) =
      delVdelBiasAcc_ * biasIncrement.accelerometer() +
      delVdelBiasOmega_ * biasIncrement.gyroscope();

  Matrix9 correction_H_tangent;
  const NavState correction =
      NavState::Expmap(correctionTangent, H ? &correction_H_tangent : nullptr);

  Vector9 corrected;
  Matrix3 logRotation_H_rotation;
  NavState::dR(corrected) =
      Rot3::Logmap(correctedRotation, H ? &logRotation_H_rotation : nullptr);
  NavState::dP(corrected) = deltaPij() + correction.position();
  NavState::dV(corrected) = deltaVij() + correction.velocity();

  if (H) {
    const Matrix3 J = correction_H_tangent.block<3, 3>(0, 0);
    const Matrix3 Qp = correction_H_tangent.block<3, 3>(3, 0);
    const Matrix3 Qv = correction_H_tangent.block<3, 3>(6, 0);

    Matrix36 rotation_H_bias, position_H_bias, velocity_H_bias;
    rotation_H_bias << Z_3x3,
        logRotation_H_rotation * correctedRotation_H_correction;
    position_H_bias << J * delPdelBiasAcc_,
        J * delPdelBiasOmega_ + Qp * delRdelBiasOmega_;
    velocity_H_bias << J * delVdelBiasAcc_,
        J * delVdelBiasOmega_ + Qv * delRdelBiasOmega_;

    const Matrix3 correctionRotation = correction.attitude().matrix();
    *H << rotation_H_bias, correctionRotation * position_H_bias,
        correctionRotation * velocity_H_bias;
  }
  return corrected;
}

}  // namespace gtsam

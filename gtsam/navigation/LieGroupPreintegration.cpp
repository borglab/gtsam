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
  const Matrix3 oldPosition_H_biasAcc = oldRotationTranspose * delPdelBiasAcc_;
  const Matrix3 oldVelocity_H_biasAcc = oldRotationTranspose * delVdelBiasAcc_;
  const Matrix3 oldPosition_H_biasOmega =
      oldRotationTranspose * delPdelBiasOmega_;
  const Matrix3 oldVelocity_H_biasOmega =
      oldRotationTranspose * delVdelBiasOmega_;

  // The NavState transition has the exact block structure
  //
  //                       [ A00   0    0  ]
  //                   A = [ A10  A11  A12 ] .
  //                       [ A20   0   A22 ]
  //
  // Apply only its nonzero 3x3 blocks to the two bias Jacobians.
  const auto A00 = stateTransition.block<3, 3>(0, 0);
  const auto A10 = stateTransition.block<3, 3>(3, 0);
  const auto A11 = stateTransition.block<3, 3>(3, 3);
  const auto A12 = stateTransition.block<3, 3>(3, 6);
  const auto A20 = stateTransition.block<3, 3>(6, 0);
  const auto A22 = stateTransition.block<3, 3>(6, 6);
  const Matrix3 newRotation_H_biasOmega =
      A00 * delRdelBiasOmega_ - omegaJacobian.topRows<3>();
  const Matrix3 newPosition_H_biasAcc = A11 * oldPosition_H_biasAcc +
                                        A12 * oldVelocity_H_biasAcc -
                                        accelerationJacobian.middleRows<3>(3);
  const Matrix3 newVelocity_H_biasAcc =
      A22 * oldVelocity_H_biasAcc - accelerationJacobian.bottomRows<3>();
  const Matrix3 newPosition_H_biasOmega =
      A10 * delRdelBiasOmega_ + A11 * oldPosition_H_biasOmega +
      A12 * oldVelocity_H_biasOmega - omegaJacobian.middleRows<3>(3);
  const Matrix3 newVelocity_H_biasOmega = A20 * delRdelBiasOmega_ +
                                          A22 * oldVelocity_H_biasOmega -
                                          omegaJacobian.bottomRows<3>();

  const Matrix3 newRotation = deltaXij_.R();
  delRdelBiasOmega_ = newRotation_H_biasOmega;
  delPdelBiasAcc_ = newRotation * newPosition_H_biasAcc;
  delPdelBiasOmega_ = newRotation * newPosition_H_biasOmega;
  delVdelBiasAcc_ = newRotation * newVelocity_H_biasAcc;
  delVdelBiasOmega_ = newRotation * newVelocity_H_biasOmega;
}

/* ************************************************************************* */
void LieGroupPreintegration::updateFactor(const Vector3& bodyAcceleration,
                                          const Vector3& bodyOmega, double dt,
                                          OptionalJacobian<9, 9> F,
                                          OptionalJacobian<9, 3> G1,
                                          OptionalJacobian<9, 3> G2) {
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
      dt * bodyAcceleration, G2 ? &increment_H_rotation : nullptr,
      (F || G1) ? &increment_H_position : nullptr,
      G1 ? &increment_H_velocity : nullptr);

  // Applying this increment through the Lie exponential would compute
  //
  //   deltaXij_ * Expmap(Logmap(incrementState)),
  //
  // which is exactly this composition. Its Jacobian with respect to the
  // increment is the identity in right-local coordinates, so direct
  // composition also cancels the corresponding Expmap and Logmap Jacobians.
  deltaXij_ = deltaXij_.compose(incrementState, F);

  if (F) {
    *F += increment_H_position * dt * bodyVelocity_H_state;
  }
  if (G1) {
    *G1 = increment_H_position * halfDtSquared + increment_H_velocity * dt;
  }
  if (G2) {
    *G2 = increment_H_rotation * incrementRotation_H_integratedOmega * dt;
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

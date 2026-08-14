/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  ManifoldPreintegration.cpp
 *  @author Luca Carlone
 *  @author Stephen Williams
 *  @author Richard Roberts
 *  @author Vadim Indelman
 *  @author David Jensen
 *  @author Frank Dellaert
 **/

#include "ManifoldPreintegration.h"

#include <gtsam/base/MatrixConstants.h>

using namespace std;

namespace gtsam {

//------------------------------------------------------------------------------
ManifoldPreintegration::ManifoldPreintegration(
    const std::shared_ptr<Params>& p, const Bias& biasHat) :
    PreintegrationBase(p, biasHat) {
  resetIntegration();
}

//------------------------------------------------------------------------------
void ManifoldPreintegration::resetIntegration() {
  deltaTij_ = 0.0;
  deltaXij_ = NavState();
  delRdelBiasOmega_.setZero();
  delPdelBiasAcc_.setZero();
  delPdelBiasOmega_.setZero();
  delVdelBiasAcc_.setZero();
  delVdelBiasOmega_.setZero();
}

//------------------------------------------------------------------------------
bool ManifoldPreintegration::equals(const ManifoldPreintegration& other,
    double tol) const {
  return p_->equals(*other.p_, tol) && std::abs(deltaTij_ - other.deltaTij_) < tol
      && biasHat_.equals(other.biasHat_, tol)
      && deltaXij_.equals(other.deltaXij_, tol)
      && equal_with_abs_tol(delRdelBiasOmega_, other.delRdelBiasOmega_, tol)
      && equal_with_abs_tol(delPdelBiasAcc_, other.delPdelBiasAcc_, tol)
      && equal_with_abs_tol(delPdelBiasOmega_, other.delPdelBiasOmega_, tol)
      && equal_with_abs_tol(delVdelBiasAcc_, other.delVdelBiasAcc_, tol)
      && equal_with_abs_tol(delVdelBiasOmega_, other.delVdelBiasOmega_, tol);
}

//------------------------------------------------------------------------------
void ManifoldPreintegration::update(const Vector3& measuredAcc,
                                    const Vector3& measuredOmega,
                                    const double dt, Matrix9* A, Matrix93* B,
                                    Matrix93* C) {
  // Correct for bias in the sensor frame
  Vector3 acc = biasHat_.correctAccelerometer(measuredAcc);
  Vector3 omega = biasHat_.correctGyroscope(measuredOmega);

  // Possibly correct for sensor pose
  Matrix3 D_correctedAcc_acc, D_correctedAcc_omega, D_correctedOmega_omega;
  if (p().body_P_sensor) {
    std::tie(acc, omega) = correctMeasurementsBySensorPose(
        acc, omega, D_correctedAcc_acc, D_correctedAcc_omega,
        D_correctedOmega_omega);
  }

  const Rot3 oldRotation = deltaXij_.attitude();

  // Do update. The transition and measurement Jacobians are also needed to
  // propagate the accumulated bias Jacobians, even if the caller omits them.
  Matrix9 stateTransition;
  Matrix93 accelerationJacobian, omegaJacobian;
  Matrix9* stateTransitionOutput = A ? A : &stateTransition;
  Matrix93* accelerationOutput = B ? B : &accelerationJacobian;
  Matrix93* omegaOutput = C ? C : &omegaJacobian;
  deltaTij_ += dt;
  updateFactor(acc, omega, dt, stateTransitionOutput, accelerationOutput,
               omegaOutput);

  if (p().body_P_sensor) {
    // More complicated derivatives in case of non-trivial sensor pose
    *omegaOutput *= D_correctedOmega_omega;
    if (!p().body_P_sensor->translation().isZero())
      *omegaOutput += *accelerationOutput * D_correctedAcc_omega;
    *accelerationOutput *= D_correctedAcc_acc;  // Must be last.
  }

  updateBiasJacobians(oldRotation, acc, omega, dt, *stateTransitionOutput,
                      *accelerationOutput, *omegaOutput);
}

//------------------------------------------------------------------------------
void ManifoldPreintegration::updateFactor(const Vector3& bodyAcceleration,
                                          const Vector3& bodyOmega, double dt,
                                          OptionalJacobian<9, 9> F,
                                          OptionalJacobian<9, 3> G1,
                                          OptionalJacobian<9, 3> G2) {
  deltaXij_ = deltaXij_.update(bodyAcceleration, bodyOmega, dt, F, G1, G2);
}

//------------------------------------------------------------------------------
void ManifoldPreintegration::updateBiasJacobians(
    const Rot3& oldRotation, const Vector3& bodyAcceleration,
    const Vector3& bodyOmega, double dt, const Matrix9& /*stateTransition*/,
    const Matrix93& /*accelerationJacobian*/,
    const Matrix93& /*omegaJacobian*/) {
  Matrix3 acceleration_H_rotation;
  oldRotation.rotate(bodyAcceleration, acceleration_H_rotation);
  const Matrix3 acceleration_H_biasOmega =
      acceleration_H_rotation * delRdelBiasOmega_;

  const Vector3 integratedOmega = bodyOmega * dt;
  Matrix3 incrementRotation_H_integratedOmega;
  const Rot3 incrementRotation =
      Rot3::Expmap(integratedOmega, incrementRotation_H_integratedOmega);
  delRdelBiasOmega_ = incrementRotation.transpose() * delRdelBiasOmega_ -
                      incrementRotation_H_integratedOmega * dt;

  const double halfDtSquared = 0.5 * dt * dt;
  const Matrix3 oldR = oldRotation.matrix();
  delPdelBiasAcc_ += delVdelBiasAcc_ * dt - halfDtSquared * oldR;
  delPdelBiasOmega_ +=
      dt * delVdelBiasOmega_ + halfDtSquared * acceleration_H_biasOmega;
  delVdelBiasAcc_ -= oldR * dt;
  delVdelBiasOmega_ += acceleration_H_biasOmega * dt;
}

//------------------------------------------------------------------------------
Vector9 ManifoldPreintegration::biasCorrectedDelta(
    const imuBias::ConstantBias& bias_i, OptionalJacobian<9, 6> H) const {
  // Correct deltaRij, derivative is delRdelBiasOmega_
  const imuBias::ConstantBias biasIncr = bias_i - biasHat_;
  Matrix3 D_correctedRij_bias;
  const Vector3 biasInducedOmega = delRdelBiasOmega_ * biasIncr.gyroscope();
  const Rot3 correctedRij = deltaRij().expmap(biasInducedOmega, {},
      H ? &D_correctedRij_bias : 0);
  if (H)
    D_correctedRij_bias *= delRdelBiasOmega_;

  Vector9 xi;
  Matrix3 D_dR_correctedRij;
  // TODO(frank): could line below be simplified? It is equivalent to
  //   LogMap(deltaRij_.compose(Expmap(biasInducedOmega)))
  NavState::dR(xi) = Rot3::Logmap(correctedRij, H ? &D_dR_correctedRij : 0);
  NavState::dP(xi) = deltaPij() + delPdelBiasAcc_ * biasIncr.accelerometer()
      + delPdelBiasOmega_ * biasIncr.gyroscope();
  NavState::dV(xi) = deltaVij() + delVdelBiasAcc_ * biasIncr.accelerometer()
      + delVdelBiasOmega_ * biasIncr.gyroscope();

  if (H) {
    Matrix36 D_dR_bias, D_dP_bias, D_dV_bias;
    D_dR_bias << Z_3x3, D_dR_correctedRij * D_correctedRij_bias;
    D_dP_bias << delPdelBiasAcc_, delPdelBiasOmega_;
    D_dV_bias << delVdelBiasAcc_, delVdelBiasOmega_;
    (*H) << D_dR_bias, D_dP_bias, D_dV_bias;
  }
  return xi;
}

//------------------------------------------------------------------------------

}// namespace gtsam

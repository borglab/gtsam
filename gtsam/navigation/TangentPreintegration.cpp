/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   TangentPreintegration.cpp
 *  @author Frank Dellaert
 *  @author Adam Bry
 **/

#include <gtsam/base/MatrixConstants.h>
#include <gtsam/base/VectorConstants.h>
#include <gtsam/navigation/TangentPreintegration.h>

#include <stdexcept>

using namespace std;

namespace gtsam {

//------------------------------------------------------------------------------
TangentPreintegration::TangentPreintegration(const std::shared_ptr<Params>& p,
    const Bias& biasHat) :
    PreintegrationBase(p, biasHat) {
  resetIntegration();
}

//------------------------------------------------------------------------------
void TangentPreintegration::resetIntegration() {
  deltaTij_ = 0.0;
  preintegrated_.setZero();
  preintegrated_H_biasAcc_.setZero();
  preintegrated_H_biasOmega_.setZero();
}

//------------------------------------------------------------------------------
bool TangentPreintegration::equals(const TangentPreintegration& other,
    double tol) const {
  return p_->equals(*other.p_, tol) && std::abs(deltaTij_ - other.deltaTij_) < tol
      && biasHat_.equals(other.biasHat_, tol)
      && equal_with_abs_tol(preintegrated_, other.preintegrated_, tol)
      && equal_with_abs_tol(preintegrated_H_biasAcc_,
          other.preintegrated_H_biasAcc_, tol)
      && equal_with_abs_tol(preintegrated_H_biasOmega_,
          other.preintegrated_H_biasOmega_, tol);
}

//------------------------------------------------------------------------------
Vector3 TangentPreintegration::so3TangentAt(double t) const {
  if (t < 0.0 || t > deltaTij_) {
    throw std::out_of_range("t must be in [0, deltaTij]");
  }
  if (deltaTij_ == 0.0) return Z_3x1;
  return (t / deltaTij_) * theta();
}

//------------------------------------------------------------------------------
// See extensive discussion in ImuFactor.lyx
Vector9 TangentPreintegration::UpdatePreintegrated(const Vector3& a_body,
    const Vector3& w_body, double dt, const Vector9& preintegrated,
    OptionalJacobian<9, 9> A, OptionalJacobian<9, 3> B,
    OptionalJacobian<9, 3> C) {
  const auto theta = preintegrated.segment<3>(0);
  const auto position = preintegrated.segment<3>(3);
  const auto velocity = preintegrated.segment<3>(6);

  // This functor allows for saving computation when exponential map and its
  // derivatives are needed at the same location in so<3>
  so3::DexpFunctor local(theta);

  // Calculate exact mean propagation
  Matrix3 w_tangent_H_theta, invH;
  const Vector3 w_tangent = // angular velocity mapped back to tangent space
      local.InvJacobian().applyRight(w_body, A ? &w_tangent_H_theta : 0, C ? &invH : 0);
  const Matrix3 nRb = local.expmap();  // nRb: rotation of body in nav frame
  const Vector3 a_nav = nRb * a_body;
  const double dt22 = 0.5 * dt * dt;

  Vector9 preintegratedPlus;
  preintegratedPlus <<                          // new preintegrated vector:
      theta + w_tangent * dt,                   // theta
      position + velocity * dt + a_nav * dt22,  // position
      velocity + a_nav * dt;                    // velocity

  if (A) {
    // Exact derivative of nRb*a with respect to theta:
    const Matrix3 a_nav_H_theta = nRb * skewSymmetric(-a_body) * local.rightJacobian();

    A->setIdentity();
    A->block<3, 3>(0, 0).noalias() += w_tangent_H_theta * dt;  // theta
    A->block<3, 3>(3, 0) = a_nav_H_theta * dt22;  // position wrpt theta...
    A->block<3, 3>(3, 6) = I_3x3 * dt;            // .. and velocity
    A->block<3, 3>(6, 0) = a_nav_H_theta * dt;    // velocity wrpt theta
  }
  if (B) {
    B->block<3, 3>(0, 0) = Z_3x3;
    B->block<3, 3>(3, 0) = nRb * dt22;
    B->block<3, 3>(6, 0) = nRb * dt;
  }
  if (C) {
    C->block<3, 3>(0, 0) = invH * dt;
    C->block<3, 3>(3, 0) = Z_3x3;
    C->block<3, 3>(6, 0) = Z_3x3;
  }

  return preintegratedPlus;
}

//------------------------------------------------------------------------------
void TangentPreintegration::update(const Vector3& measuredAcc,
    const Vector3& measuredOmega, const double dt, Matrix9* A, Matrix93* B,
    Matrix93* C) {
  // Correct for bias in the sensor frame
  Vector3 acc = biasHat_.correctAccelerometer(measuredAcc);
  Vector3 omega = biasHat_.correctGyroscope(measuredOmega);

  // Possibly correct for sensor pose by converting to body frame
  Matrix3 D_correctedAcc_acc, D_correctedAcc_omega, D_correctedOmega_omega;
  if (p().body_P_sensor) {
    std::tie(acc, omega) = correctMeasurementsBySensorPose(
        acc, omega, D_correctedAcc_acc, D_correctedAcc_omega,
        D_correctedOmega_omega);
  }

  // Do update
  deltaTij_ += dt;
  preintegrated_ = UpdatePreintegrated(acc, omega, dt, preintegrated_, A, B, C);

  if (p().body_P_sensor) {
    // More complicated derivatives in case of non-trivial sensor pose
    *C *= D_correctedOmega_omega;
    if (!p().body_P_sensor->translation().isZero())
      *C += *B * D_correctedAcc_omega;
    *B *= D_correctedAcc_acc; // NOTE(frank): needs to be last
  }

  // Propagate the two accumulated bias Jacobians using the exact Tangent
  // transition structure instead of two dense 9x9-by-9x3 products:
  //
  //                       [ A00   0    0  ]
  //                   A = [ A10   I   dtI ] .
  //                       [ A20   0    I  ]
  //
  // Acceleration bias never affects rotation. Its top block therefore remains
  // zero, while position and velocity need only additions and scaling by dt.
  const Matrix3 oldAccelerationVelocity =
      preintegrated_H_biasAcc_.bottomRows<3>();
  preintegrated_H_biasAcc_.middleRows<3>(3) +=
      oldAccelerationVelocity * dt - B->middleRows<3>(3);
  preintegrated_H_biasAcc_.bottomRows<3>() -= B->bottomRows<3>();

  // Gyroscope bias affects all three blocks. Save the old rotation block since
  // the position and velocity updates both use it after rotation is replaced.
  const Matrix3 oldOmegaRotation = preintegrated_H_biasOmega_.topRows<3>();
  preintegrated_H_biasOmega_.topRows<3>().noalias() =
      A->block<3, 3>(0, 0) * oldOmegaRotation - C->topRows<3>();
  preintegrated_H_biasOmega_.middleRows<3>(3).noalias() +=
      A->block<3, 3>(3, 0) * oldOmegaRotation +
      dt * preintegrated_H_biasOmega_.bottomRows<3>() - C->middleRows<3>(3);
  preintegrated_H_biasOmega_.bottomRows<3>().noalias() +=
      A->block<3, 3>(6, 0) * oldOmegaRotation - C->bottomRows<3>();
}

//------------------------------------------------------------------------------
Vector9 TangentPreintegration::biasCorrectedDelta(
    const imuBias::ConstantBias& bias_i, OptionalJacobian<9, 6> H) const {
  // We correct for a change between bias_i and the biasHat_ used to integrate
  // This is a simple linear correction with obvious derivatives
  const imuBias::ConstantBias biasIncr = bias_i - biasHat_;
  const Vector9 biasCorrected = preintegrated()
      + preintegrated_H_biasAcc_ * biasIncr.accelerometer()
      + preintegrated_H_biasOmega_ * biasIncr.gyroscope();

  if (H) {
    (*H) << preintegrated_H_biasAcc_, preintegrated_H_biasOmega_;
  }
  return biasCorrected;
}

//------------------------------------------------------------------------------
// sugar for derivative blocks
#define D_R_R(H) (H)->block<3,3>(0,0)
#define D_R_t(H) (H)->block<3,3>(0,3)
#define D_R_v(H) (H)->block<3,3>(0,6)
#define D_t_R(H) (H)->block<3,3>(3,0)
#define D_t_t(H) (H)->block<3,3>(3,3)
#define D_t_v(H) (H)->block<3,3>(3,6)
#define D_v_R(H) (H)->block<3,3>(6,0)
#define D_v_t(H) (H)->block<3,3>(6,3)
#define D_v_v(H) (H)->block<3,3>(6,6)

//------------------------------------------------------------------------------
Vector9 TangentPreintegration::Compose(const Vector9& zeta01,
    const Vector9& zeta12, double deltaT12, OptionalJacobian<9, 9> H1,
    OptionalJacobian<9, 9> H2) {
  const auto t01 = zeta01.segment<3>(0);
  const auto p01 = zeta01.segment<3>(3);
  const auto v01 = zeta01.segment<3>(6);

  const auto t12 = zeta12.segment<3>(0);
  const auto p12 = zeta12.segment<3>(3);
  const auto v12 = zeta12.segment<3>(6);

  Matrix3 R01_H_t01, R12_H_t12;
  const Rot3 R01 = Rot3::Expmap(t01, R01_H_t01);
  const Rot3 R12 = Rot3::Expmap(t12, R12_H_t12);

  Matrix3 R02_H_R01, R02_H_R12; // NOTE(frank): R02_H_R12 == Identity
  const Rot3 R02 = R01.compose(R12, R02_H_R01, R02_H_R12);

  Matrix3 t02_H_R02;
  Vector9 zeta02;
  const Matrix3 R = R01.matrix();
  zeta02 << Rot3::Logmap(R02, t02_H_R02), // theta
  p01 + v01 * deltaT12 + R * p12, // position
  v01 + R * v12; // velocity

  if (H1) {
    H1->setIdentity();
    D_R_R(H1) = t02_H_R02 * R02_H_R01 * R01_H_t01;
    D_t_R(H1) = R * skewSymmetric(-p12) * R01_H_t01;
    D_t_v(H1) = I_3x3 * deltaT12;
    D_v_R(H1) = R * skewSymmetric(-v12) * R01_H_t01;
  }

  if (H2) {
    H2->setZero();
    D_R_R(H2) = t02_H_R02 * R02_H_R12 * R12_H_t12;
    D_t_t(H2) = R;
    D_v_v(H2) = R;
  }

  return zeta02;
}

//------------------------------------------------------------------------------
void TangentPreintegration::mergeWith(const TangentPreintegration& pim12,
    Matrix9* H1, Matrix9* H2) {
  if (!matchesParamsWith(pim12)) {
    throw std::domain_error(
        "Cannot merge pre-integrated measurements with different params");
  }

  if (params()->body_P_sensor) {
    throw std::domain_error(
        "Cannot merge pre-integrated measurements with sensor pose yet");
  }

  const double t01 = deltaTij();
  const double t12 = pim12.deltaTij();
  deltaTij_ = t01 + t12;

  const Vector9 zeta01 = preintegrated();
  Vector9 zeta12 = pim12.preintegrated(); // will be modified.

  const imuBias::ConstantBias bias_incr_for_12 = biasHat() - pim12.biasHat();
  zeta12 += pim12.preintegrated_H_biasOmega_ * bias_incr_for_12.gyroscope()
      + pim12.preintegrated_H_biasAcc_ * bias_incr_for_12.accelerometer();

  preintegrated_ = TangentPreintegration::Compose(zeta01, zeta12, t12, H1, H2);

  preintegrated_H_biasAcc_ = (*H1) * preintegrated_H_biasAcc_
      + (*H2) * pim12.preintegrated_H_biasAcc_;

  preintegrated_H_biasOmega_ = (*H1) * preintegrated_H_biasOmega_
      + (*H2) * pim12.preintegrated_H_biasOmega_;
}

//------------------------------------------------------------------------------

}// namespace gtsam

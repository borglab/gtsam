/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GalileanImuFactor.cpp
 * @brief   Implementation of Delama et al. IMU Factor
 * @author  Frank Dellaert
 * @author  Giulio Delama
 */

#include <gtsam/navigation/GalileanImuFactor.h>

namespace gtsam {

// -- Constructors ------------------------------------------------------------
PreintegratedImuMeasurementsG::PreintegratedImuMeasurementsG(
    const std::shared_ptr<Params>& p, const imuBias::ConstantBias& biasHat)
    : Base(p, biasHat), p_(p) {}

// -- Integration API ---------------------------------------------------------
void PreintegratedImuMeasurementsG::resetIntegration() {
  preintMatrix_ = Gal3().Identity();
  preintMeasCov_.setZero();
  preintBiasJacobian_.setZero();
}

void PreintegratedImuMeasurementsG::integrateMeasurement(
    const Vector3& measuredAcc, const Vector3& measuredOmega, double dt) {
  // Correct measurements for bias and construct input
  Vector3 acc = biasHat_.correctAccelerometer(measuredAcc);
  Vector3 omega = biasHat_.correctGyroscope(measuredOmega);
  if (p_->body_P_sensor) {
    const auto& bRs = p_->body_P_sensor->rotation();
    acc = bRs * acc;
    omega = bRs * omega;
  }
  Vector10 wInput = Vector10::Zero();
  wInput.head<3>() = omega;
  wInput.segment<3>(3) = acc;
  wInput(9) = 1.0;  // time scale
  Matrix10 Jl;
  Gal3 wExp = Gal3::Expmap(-wInput * dt, Jl).inverse();
  Matrix10 preintMatrixAdjoint = preintMatrix_.AdjointMap();

  // Propagate mean - from eq. (22) in the paper
  preintMatrix_ = preintMatrix_.compose(wExp);

  // Propagate covariance - from eq. (35) in the paper
  // IMPORTANT: the covariance propagation is based on the Right-Invariant (RI)
  // error since the equivariant error (34) is by definition RI. If we want to
  // use the Left-Invariant (LI) error to use NavState in the error formulation
  // for the factor, we need to change the covariance propagation accordingly
  // with the Adjoint. For RI error, the A_ matrix is identity.
  Matrix10 B_ = preintMatrixAdjoint * Jl * dt;
  Matrix10 Qd = Matrix10::Zero();
  Qd.block<3, 3>(0, 0) = p_->gyroscopeCovariance / dt;
  Qd.block<3, 3>(3, 3) = p_->accelerometerCovariance / dt;
  preintMeasCov_.noalias() += B_ * Qd * B_.transpose();

  // Propagate bias Jacobian - from eq. (38) in the paper
  Matrix20 phiBiasJacobian = Matrix20::Identity();
  phiBiasJacobian.block<10, 10>(9, 9) = -preintMatrixAdjoint * Jl * dt;
  preintBiasJacobian_ = phiBiasJacobian * preintBiasJacobian_;
}

// -- print / equals ----------------------------------------------------------
void PreintegratedImuMeasurementsG::print(const std::string& s) const {
  std::cout << (s.empty() ? s : s + "\n")
            << "PreintegratedImuMeasurementsG:" << std::endl;
  std::cout << "  preintMatrix_: " << preintMatrix_ << std::endl;
  std::cout << "  preintMeasCov_: \n" << preintMeasCov_ << std::endl;
  std::cout << "  preintBiasJacobian_: \n" << preintBiasJacobian_ << std::endl;
  std::cout << "  biasHat_: " << biasHat_.vector().transpose() << std::endl;
}
bool PreintegratedImuMeasurementsG::equals(
    const PreintegratedImuMeasurementsG& o, double tol) const {
  return p_->equals(*o.p_, tol) && biasHat_.equals(o.biasHat_, tol) &&
         preintMatrix_.equals(o.preintMatrix_, tol) &&
         equal_with_abs_tol(preintMeasCov_, o.preintMeasCov_, tol) &&
         equal_with_abs_tol(preintBiasJacobian_, o.preintBiasJacobian_, tol);
}

}  // namespace gtsam

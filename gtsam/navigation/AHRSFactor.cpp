/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  AHRSFactor.cpp
 *  @author Krunal Chande
 *  @author Luca Carlone
 *  @author Frank Dellaert
 *  @date   July 2014
 **/

#include <gtsam/navigation/AHRSFactor.h>

#include <iostream>

using namespace std;

namespace gtsam {

//------------------------------------------------------------------------------
// Inner class PreintegratedMeasurements
//------------------------------------------------------------------------------
void PreintegratedAhrsMeasurements::print(const string& s) const {
  PreintegratedRotation::print(s);
  cout << "biasHat [" << biasHat_.transpose() << "]" << endl;
  cout << " PreintMeasCov [ " << preintMeasCov_ << " ]" << endl;
}

//------------------------------------------------------------------------------
bool PreintegratedAhrsMeasurements::equals(
    const PreintegratedAhrsMeasurements& other, double tol) const {
  return PreintegratedRotation::equals(other, tol) &&
         equal_with_abs_tol(biasHat_, other.biasHat_, tol);
}

//------------------------------------------------------------------------------
void PreintegratedAhrsMeasurements::resetIntegration() {
  PreintegratedRotation::resetIntegration();
  preintMeasCov_.setZero();
}

//------------------------------------------------------------------------------
void PreintegratedAhrsMeasurements::integrateMeasurement(
    const Vector3& measuredOmega, double deltaT) {
  // 1. integrate (handles bias + body_P_sensor rotation internally)
  // Fr is the Jacobian of the new preintegrated rotation w.r.t. the previous one.
  Matrix3 Fr;
  PreintegratedRotation::integrateGyroMeasurement(measuredOmega, biasHat_,
                                                  deltaT, &Fr);

  // 2. Calculate noise in the body frame
  Matrix3 SigmaBody = p().gyroscopeCovariance;
  if (p().body_P_sensor) {
    const Matrix3& bRs = p().body_P_sensor->rotation().matrix();  // body←sensor
    SigmaBody = bRs * SigmaBody * bRs.transpose();
  }

  // First order uncertainty propagation:
  //   new_cov = Fr * old_cov * Fr.transpose() + new_noise
  // The deltaT allows to pass from continuous time noise to discrete time
  // noise. Comparing with the IMUFactor.cpp implementation, the latter is an
  // approximation for C * (wCov / dt) * C.transpose(), with C \approx I * dt.
  preintMeasCov_ = Fr * preintMeasCov_ * Fr.transpose() + SigmaBody * deltaT;
}

//------------------------------------------------------------------------------
Rot3 PreintegratedAhrsMeasurements::predict(
    const Rot3& Ri, const Vector3& bias, gtsam::OptionalJacobian<3, 3> H1,
    gtsam::OptionalJacobian<3, 3> H2) const {
  // Use H2 as an in/out parameter to hold the Jacobian of the bias-corrected
  // rotation w.r.t. the bias increment. This is an efficient C++ pattern.
  const Vector3 biasOmegaIncr = bias - biasHat_;
  const Rot3 biascorrected = this->biascorrectedDeltaRij(biasOmegaIncr, H2);

  // We handle the common case of no Coriolis correction first.
  if (!p().omegaCoriolis) {
    // ---- FAST PATH ----
    // In this path, the Jacobian of compose wrt its second argument is
    // identity, so H2 already holds the final Jacobian wrt bias.
    return Ri.compose(biascorrected, H1);

  } else {
    // ---- SLOW PATH ----
    // Calculate Coriolis effects and its derivative w.r.t. Ri.
    Matrix3 D_coriolis_Ri;
    const Vector3 coriolis =
        integrateCoriolis(Ri, H1 ? &D_coriolis_Ri : nullptr);

    // Compose bias and Coriolis corrections.
    Matrix3 D_finalDelta_biasDelta, D_finalDelta_coriolis;
    const Rot3 finalDeltaRij =
        biascorrected.expmap(-coriolis, H2 ? &D_finalDelta_biasDelta : nullptr,
                             H1 ? &D_finalDelta_coriolis : nullptr);

    // Predict final orientation, getting the direct Jacobian wrt Ri.
    const Rot3 predicted_Rj = Ri.compose(finalDeltaRij, H1);

    // Augment Jacobians with the indirect paths.
    if (H1) {
      // Add in indirect path: Ri -> coriolis -> finalDeltaRij
      // H1 is now D_predicted_Rj_wrt_Ri
      *H1 -= D_finalDelta_coriolis * D_coriolis_Ri;
    }

    if (H2) {
      // Apply the chain rule for the bias Jacobian, updating H2 in-place.
      // H2 was D_biascorrected_wrt_bias, now it is D_predicted_Rj_wrt_bias
      *H2 = D_finalDelta_biasDelta * (*H2);
    }

    return predicted_Rj;
  }
}

//------------------------------------------------------------------------------
Vector3 PreintegratedAhrsMeasurements::computeError(
    const Rot3& Ri, const Rot3& Rj, const Vector3& bias,
    gtsam::OptionalJacobian<3, 3> H1, gtsam::OptionalJacobian<3, 3> H2,
    gtsam::OptionalJacobian<3, 3> H3) const {
  // Predict orientation at time j
  Matrix3 D_predict_Ri, D_predict_bias;
  Rot3 predicted_Rj = predict(Ri, bias, H1 ? &D_predict_Ri : nullptr,
                              H3 ? &D_predict_bias : nullptr);

  // Compute the error vector: log(Rj.inverse() * predicted_Rj)
  Matrix3 D_error_Rj, D_error_predict;
  Vector3 error = Rj.logmap(predicted_Rj, H2 ? &D_error_Rj : nullptr,
                            H1 || H3 ? &D_error_predict : nullptr);

  // Jacobians using the chain rule
  if (H1) *H1 = D_error_predict * D_predict_Ri;
  if (H2) *H2 = D_error_Rj;
  if (H3) *H3 = D_error_predict * D_predict_bias;

  return error;
}

//------------------------------------------------------------------------------
// AHRSFactor methods
//------------------------------------------------------------------------------
AHRSFactor::AHRSFactor(Key rot_i, Key rot_j, Key bias,
                       const PreintegratedAhrsMeasurements& pim)
    : Base(noiseModel::Gaussian::Covariance(pim.preintMeasCov_), rot_i, rot_j,
           bias),
      _PIM_(pim) {}

gtsam::NonlinearFactor::shared_ptr AHRSFactor::clone() const {
  //------------------------------------------------------------------------------
  return std::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new This(*this)));
}

//------------------------------------------------------------------------------
void AHRSFactor::print(const string& s,
                       const KeyFormatter& keyFormatter) const {
  cout << s << "AHRSFactor(" << keyFormatter(this->key<1>()) << ","
       << keyFormatter(this->key<2>()) << "," << keyFormatter(this->key<3>())
       << ",";
  _PIM_.print("  preintegrated measurements:");
  noiseModel_->print("  noise model: ");
}

//------------------------------------------------------------------------------
bool AHRSFactor::equals(const NonlinearFactor& other, double tol) const {
  const This* e = dynamic_cast<const This*>(&other);
  return e != nullptr && Base::equals(*e, tol) && _PIM_.equals(e->_PIM_, tol);
}

//------------------------------------------------------------------------------
Vector AHRSFactor::evaluateError(const Rot3& Ri, const Rot3& Rj,
                                 const Vector3& bias, OptionalMatrixType H1,
                                 OptionalMatrixType H2,
                                 OptionalMatrixType H3) const {
  return _PIM_.computeError(Ri, Rj, bias, H1, H2, H3);
}

//------------------------------------------------------------------------------
AHRSFactor::AHRSFactor(Key rot_i, Key rot_j, Key bias,
                       const PreintegratedAhrsMeasurements& pim,
                       const Vector3& omegaCoriolis,
                       const std::optional<Pose3>& body_P_sensor)
    : Base(noiseModel::Gaussian::Covariance(pim.preintMeasCov_), rot_i, rot_j,
           bias),
      _PIM_(pim) {
  auto p = std::make_shared<PreintegratedAhrsMeasurements::Params>(pim.p());
  p->body_P_sensor = body_P_sensor;
  _PIM_.p_ = p;
}

//------------------------------------------------------------------------------
Rot3 AHRSFactor::predict(const Rot3& Ri, const Vector3& bias,
                         const PreintegratedAhrsMeasurements& pim,
                         const Vector3& omegaCoriolis,
                         const std::optional<Pose3>& body_P_sensor) {
  auto p = std::make_shared<PreintegratedAhrsMeasurements::Params>(pim.p());
  p->omegaCoriolis = omegaCoriolis;
  p->body_P_sensor = body_P_sensor;
  PreintegratedAhrsMeasurements newPim = pim;
  newPim.p_ = p;
  return newPim.predict(Ri, bias);
}

}  // namespace gtsam

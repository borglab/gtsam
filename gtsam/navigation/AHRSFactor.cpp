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
  return p_->equals(*other.p_, tol) &&
         deltaRij_.equals(other.deltaRij_, tol) &&
         std::abs(deltaTij_ - other.deltaTij_) < tol &&
         equal_with_abs_tol(delRdelBiasOmega_, other.delRdelBiasOmega_, tol) &&
         equal_with_abs_tol(biasHat_, other.biasHat_, tol) &&
         equal_with_abs_tol(preintMeasCov_, other.preintMeasCov_, tol);
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

// Explicit template instantiation for the classic public factor.
template class GTSAM_EXPORT AHRSFactorT<PreintegratedAhrsMeasurements>;

}  // namespace gtsam

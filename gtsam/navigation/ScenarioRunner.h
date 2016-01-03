/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    ScenarioRunner.h
 * @brief   Simple class to test navigation scenarios
 * @author  Frank Dellaert
 */

#pragma once
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/Scenario.h>
#include <gtsam/linear/Sampler.h>

namespace gtsam {

/*
 *  Simple class to test navigation scenarios.
 *  Takes a trajectory scenario as input, and can generate IMU measurements
 */
class ScenarioRunner {
 public:
  typedef boost::shared_ptr<ImuFactor::PreintegratedMeasurements::Params>
      SharedParams;
  ScenarioRunner(const Scenario* scenario, const SharedParams& p,
                 double imuSampleTime = 1.0 / 100.0,
                 const imuBias::ConstantBias& bias = imuBias::ConstantBias())
      : scenario_(scenario),
        p_(p),
        imuSampleTime_(imuSampleTime),
        bias_(bias),
        // NOTE(duy): random seeds that work well:
        gyroSampler_(Diagonal(p->gyroscopeCovariance), 10),
        accSampler_(Diagonal(p->accelerometerCovariance), 29284) {}

  // NOTE(frank): hardcoded for now with Z up (gravity points in negative Z)
  // also, uses g=10 for easy debugging
  const Vector3& gravity_n() const { return p_->n_gravity; }

  // A gyro simply measures angular velocity in body frame
  Vector3 actualAngularVelocity(double t) const {
    return scenario_->omega_b(t);
  }

  // An accelerometer measures acceleration in body, but not gravity
  Vector3 actualSpecificForce(double t) const {
    Rot3 bRn = scenario_->rotation(t).transpose();
    return scenario_->acceleration_b(t) - bRn * gravity_n();
  }

  // versions corrupted by bias and noise
  Vector3 measuredAngularVelocity(double t) const {
    return actualAngularVelocity(t) + bias_.gyroscope() +
           gyroSampler_.sample() / std::sqrt(imuSampleTime_);
  }
  Vector3 measuredSpecificForce(double t) const {
    return actualSpecificForce(t) + bias_.accelerometer() +
           accSampler_.sample() / std::sqrt(imuSampleTime_);
  }

  const double& imuSampleTime() const { return imuSampleTime_; }

  /// Integrate measurements for T seconds into a PIM
  ImuFactor::PreintegratedMeasurements integrate(
      double T,
      const imuBias::ConstantBias& estimatedBias = imuBias::ConstantBias(),
      bool corrupted = false) const;

  /// Predict predict given a PIM
  NavState predict(const ImuFactor::PreintegratedMeasurements& pim,
                   const imuBias::ConstantBias& estimatedBias =
                       imuBias::ConstantBias()) const;

  /// Return pose covariance by re-arranging pim.preintMeasCov() appropriately
  Matrix6 poseCovariance(
      const ImuFactor::PreintegratedMeasurements& pim) const {
    Matrix9 cov = pim.preintMeasCov();
    Matrix6 poseCov;
    poseCov << cov.block<3, 3>(0, 0), cov.block<3, 3>(0, 3),  //
        cov.block<3, 3>(3, 0), cov.block<3, 3>(3, 3);
    return poseCov;
  }

  /// Compute a Monte Carlo estimate of the PIM pose covariance using N samples
  Matrix6 estimatePoseCovariance(double T, size_t N = 1000,
                                 const imuBias::ConstantBias& estimatedBias =
                                     imuBias::ConstantBias()) const;

 private:
  // Convert covariance to diagonal noise model, if possible, otherwise throw
  static noiseModel::Diagonal::shared_ptr Diagonal(const Matrix& covariance) {
    bool smart = true;
    auto model = noiseModel::Gaussian::Covariance(covariance, smart);
    auto diagonal = boost::dynamic_pointer_cast<noiseModel::Diagonal>(model);
    if (!diagonal)
      throw std::invalid_argument("ScenarioRunner::Diagonal: not a diagonal");
    return diagonal;
  }

  const Scenario* scenario_;
  const SharedParams p_;
  const double imuSampleTime_;
  const imuBias::ConstantBias bias_;

  // Create two samplers for acceleration and omega noise
  mutable Sampler gyroSampler_, accSampler_;
};

}  // namespace gtsam

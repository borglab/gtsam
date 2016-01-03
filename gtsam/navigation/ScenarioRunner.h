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
  ScenarioRunner(const Scenario* scenario, double imuSampleTime = 1.0 / 100.0,
                 double gyroSigma = 0.17, double accSigma = 0.01,
                 const imuBias::ConstantBias& bias = imuBias::ConstantBias())
      : scenario_(scenario),
        imuSampleTime_(imuSampleTime),
        gyroNoiseModel_(noiseModel::Isotropic::Sigma(3, gyroSigma)),
        accNoiseModel_(noiseModel::Isotropic::Sigma(3, accSigma)),
        bias_(bias),
        // NOTE(duy): random seeds that work well:
        gyroSampler_(gyroNoiseModel_, 10),
        accSampler_(accNoiseModel_, 29284)

  {}

  // NOTE(frank): hardcoded for now with Z up (gravity points in negative Z)
  // also, uses g=10 for easy debugging
  static Vector3 gravity_n() { return Vector3(0, 0, -10.0); }

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

  const noiseModel::Diagonal::shared_ptr& gyroNoiseModel() const {
    return gyroNoiseModel_;
  }

  const noiseModel::Diagonal::shared_ptr& accNoiseModel() const {
    return accNoiseModel_;
  }

  Matrix3 gyroCovariance() const { return gyroNoiseModel_->covariance(); }
  Matrix3 accCovariance() const { return accNoiseModel_->covariance(); }

  /// Integrate measurements for T seconds into a PIM
  ImuFactor::PreintegratedMeasurements integrate(
      double T,
      const imuBias::ConstantBias& estimatedBias = imuBias::ConstantBias(),
      bool corrupted = false) const;

  /// Predict predict given a PIM
  PoseVelocityBias predict(const ImuFactor::PreintegratedMeasurements& pim,
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
  const Scenario* scenario_;
  const double imuSampleTime_;
  const noiseModel::Diagonal::shared_ptr gyroNoiseModel_, accNoiseModel_;
  const imuBias::ConstantBias bias_;

  // Create two samplers for acceleration and omega noise
  mutable Sampler gyroSampler_, accSampler_;
};

}  // namespace gtsam

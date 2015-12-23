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
#include <gtsam/linear/Sampler.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/Scenario.h>

#include <cmath>

namespace gtsam {

static double intNoiseVar = 0.0000001;
static const Matrix3 kIntegrationErrorCovariance = intNoiseVar * I_3x3;

/// Simple class to test navigation scenarios
class ScenarioRunner {
 public:
  ScenarioRunner(const Scenario* scenario, double imuSampleTime = 1.0 / 100.0,
                 double gyroSigma = 0.17, double accSigma = 0.01)
      : scenario_(scenario),
        imuSampleTime_(imuSampleTime),
        gyroNoiseModel_(noiseModel::Isotropic::Sigma(3, gyroSigma)),
        accNoiseModel_(noiseModel::Isotropic::Sigma(3, accSigma)) {}

  const double& imuSampleTime() const { return imuSampleTime_; }

  // NOTE(frank): hardcoded for now with Z up (gravity points in negative Z)
  // also, uses g=10 for easy debugging
  Vector3 gravity_n() const { return Vector3(0, 0, -10.0); }

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
      double T, Sampler* gyroSampler = 0, Sampler* accSampler = 0) const {
    // TODO(frank): allow non-zero
    const imuBias::ConstantBias zeroBias;
    const bool use2ndOrderIntegration = true;

    ImuFactor::PreintegratedMeasurements pim(
        zeroBias, accCovariance(), gyroCovariance(),
        kIntegrationErrorCovariance, use2ndOrderIntegration);

    const double dt = imuSampleTime();
    const double sqrt_dt = std::sqrt(dt);
    const size_t nrSteps = T / dt;
    double t = 0;
    for (size_t k = 0; k < nrSteps; k++, t += dt) {
      Rot3 bRn = scenario_->rotation(t).transpose();
      Vector3 measuredOmega = scenario_->omega_b(t);
      if (gyroSampler) measuredOmega += gyroSampler->sample() / sqrt_dt;
      Vector3 measuredAcc = scenario_->acceleration_b(t) - bRn * gravity_n();
      if (accSampler) measuredAcc += accSampler->sample() / sqrt_dt;
      pim.integrateMeasurement(measuredAcc, measuredOmega, dt);
    }

    return pim;
  }

  /// Predict predict given a PIM
  PoseVelocityBias predict(
      const ImuFactor::PreintegratedMeasurements& pim) const {
    // TODO(frank): allow non-zero bias, omegaCoriolis
    const imuBias::ConstantBias zeroBias;
    const Vector3 omegaCoriolis = Vector3::Zero();
    const bool use2ndOrderCoriolis = true;
    return pim.predict(scenario_->pose(0), scenario_->velocity_n(0), zeroBias,
                       gravity_n(), omegaCoriolis, use2ndOrderCoriolis);
  }

  /// Return pose covariance by re-arranging pim.preintMeasCov() appropriately
  Matrix6 poseCovariance(
      const ImuFactor::PreintegratedMeasurements& pim) const {
    Matrix9 cov = pim.preintMeasCov();
    Matrix6 poseCov;
    poseCov << cov.block<3, 3>(6, 6), cov.block<3, 3>(6, 0),  //
        cov.block<3, 3>(0, 6), cov.block<3, 3>(0, 0);
    return poseCov;
  }

  /// Compute a Monte Carlo estimate of the PIM pose covariance using N samples
  Matrix6 estimatePoseCovariance(double T, size_t N = 1000) const {
    // Get predict prediction from ground truth measurements
    Pose3 prediction = predict(integrate(T)).pose;

    // Create two samplers for acceleration and omega noise
    Sampler gyroSampler(gyroNoiseModel(), 10);
    Sampler accSampler(accNoiseModel(), 29284);

    // Sample !
    Matrix samples(9, N);
    Vector6 sum = Vector6::Zero();
    for (size_t i = 0; i < N; i++) {
      Pose3 sampled = predict(integrate(T, &gyroSampler, &accSampler)).pose;
      Vector6 xi = sampled.localCoordinates(prediction);
      samples.col(i) = xi;
      sum += xi;
    }

    // Compute MC covariance
    Vector6 sampleMean = sum / N;
    Matrix6 Q;
    Q.setZero();
    for (size_t i = 0; i < N; i++) {
      Vector6 xi = samples.col(i);
      xi -= sampleMean;
      Q += xi * xi.transpose();
    }

    return Q / (N - 1);
  }

 private:
  const Scenario* scenario_;
  double imuSampleTime_;
  noiseModel::Diagonal::shared_ptr gyroNoiseModel_, accNoiseModel_;
};

}  // namespace gtsam

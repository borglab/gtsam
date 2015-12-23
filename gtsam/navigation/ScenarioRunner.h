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
  ScenarioRunner(const Scenario& scenario) : scenario_(scenario) {}

  /// Integrate measurements for T seconds into a PIM
  ImuFactor::PreintegratedMeasurements integrate(double T,
                                                 Sampler* gyroSampler = 0,
                                                 Sampler* accSampler = 0) {
    // TODO(frank): allow non-zero
    const imuBias::ConstantBias zeroBias;
    const bool use2ndOrderIntegration = true;

    ImuFactor::PreintegratedMeasurements pim(
        zeroBias, scenario_.accCovariance(), scenario_.gyroCovariance(),
        kIntegrationErrorCovariance, use2ndOrderIntegration);

    const double dt = scenario_.imuSampleTime();
    const double sqrt_dt = std::sqrt(dt);
    const size_t nrSteps = T / dt;
    double t = 0;
    for (size_t k = 0; k < nrSteps; k++, t += dt) {
      Vector3 measuredOmega = scenario_.angularVelocityInBody();
      if (gyroSampler) measuredOmega += gyroSampler->sample() / sqrt_dt;
      Vector3 measuredAcc = scenario_.accelerationInBody(t);
      if (accSampler) measuredAcc += accSampler->sample() / sqrt_dt;
      pim.integrateMeasurement(measuredAcc, measuredOmega, dt);
    }

    return pim;
  }

  /// Predict predict given a PIM
  PoseVelocityBias predict(const ImuFactor::PreintegratedMeasurements& pim) {
    // TODO(frank): allow non-zero bias, omegaCoriolis
    const imuBias::ConstantBias zeroBias;
    const Pose3 pose_i = Pose3::identity();
    const Vector3 vel_i = scenario_.velocity(0);
    const Vector3 omegaCoriolis = Vector3::Zero();
    const bool use2ndOrderCoriolis = true;
    return pim.predict(pose_i, vel_i, zeroBias, scenario_.gravity(),
                       omegaCoriolis, use2ndOrderCoriolis);
  }

  /// Return pose covariance by re-arranging pim.preintMeasCov() appropriately
  Matrix6 poseCovariance(const ImuFactor::PreintegratedMeasurements& pim) {
    Matrix9 cov = pim.preintMeasCov();  // _ position rotation
    Matrix6 poseCov;
    poseCov << cov.block<3, 3>(6, 6), cov.block<3, 3>(6, 3),  //
        cov.block<3, 3>(3, 6), cov.block<3, 3>(3, 3);
    return poseCov;
  }

  /// Compute a Monte Carlo estimate of the PIM pose covariance using N samples
  Matrix6 estimatePoseCovariance(double T, size_t N = 1000) {
    // Get predict prediction from ground truth measurements
    Pose3 prediction = predict(integrate(T)).pose;

    // Create two samplers for acceleration and omega noise
    Sampler gyroSampler(scenario_.gyroNoiseModel(), 10);
    Sampler accSampler(scenario_.accNoiseModel(), 29284);

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
  Scenario scenario_;
};

}  // namespace gtsam

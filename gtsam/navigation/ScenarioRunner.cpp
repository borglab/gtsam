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

#include <gtsam/navigation/ScenarioRunner.h>
#include <gtsam/linear/Sampler.h>

#include <cmath>

namespace gtsam {

static double intNoiseVar = 0.0000001;
static const Matrix3 kIntegrationErrorCovariance = intNoiseVar * I_3x3;

ImuFactor::PreintegratedMeasurements ScenarioRunner::integrate(
    double T, Sampler* gyroSampler, Sampler* accSampler) const {
  // TODO(frank): allow non-zero
  const imuBias::ConstantBias zeroBias;
  const bool use2ndOrderIntegration = true;

  ImuFactor::PreintegratedMeasurements pim(
      zeroBias, accCovariance(), gyroCovariance(), kIntegrationErrorCovariance,
      use2ndOrderIntegration);

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

PoseVelocityBias ScenarioRunner::predict(
    const ImuFactor::PreintegratedMeasurements& pim) const {
  // TODO(frank): allow non-zero bias, omegaCoriolis
  const imuBias::ConstantBias zeroBias;
  const Vector3 omegaCoriolis = Vector3::Zero();
  const bool use2ndOrderCoriolis = true;
  return pim.predict(scenario_->pose(0), scenario_->velocity_n(0), zeroBias,
                     gravity_n(), omegaCoriolis, use2ndOrderCoriolis);
}

Matrix6 ScenarioRunner::estimatePoseCovariance(double T, size_t N) const {
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

}  // namespace gtsam

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

#include <gtsam/base/timing.h>
#include <gtsam/navigation/ScenarioRunner.h>

#include <cmath>

using namespace std;

namespace gtsam {

static double intNoiseVar = 0.0000001;
static const Matrix3 kIntegrationErrorCovariance = intNoiseVar * I_3x3;

PreintegratedImuMeasurements ScenarioRunner::integrate(
    double T, const Bias& estimatedBias, bool corrupted) const {
  gttic_(integrate);
  PreintegratedImuMeasurements pim(p_, estimatedBias);

  const double dt = imuSampleTime();
  const size_t nrSteps = T / dt;
  double t = 0;
  for (size_t k = 0; k < nrSteps; k++, t += dt) {
    Vector3 measuredOmega =
        corrupted ? measuredAngularVelocity(t) : actualAngularVelocity(t);
    Vector3 measuredAcc =
        corrupted ? measuredSpecificForce(t) : actualSpecificForce(t);
    pim.integrateMeasurement(measuredAcc, measuredOmega, dt);
  }

  return pim;
}

NavState ScenarioRunner::predict(const PreintegratedImuMeasurements& pim,
                                 const Bias& estimatedBias) const {
  const NavState state_i(scenario_.pose(0), scenario_.velocity_n(0));
  return pim.predict(state_i, estimatedBias);
}

Matrix9 ScenarioRunner::estimateCovariance(double T, size_t N,
                                           const Bias& estimatedBias) const {
  gttic_(estimateCovariance);

  // Get predict prediction from ground truth measurements
  NavState prediction = predict(integrate(T));

  // Sample !
  Matrix samples(9, N);
  Vector9 sum = Vector9::Zero();
  for (size_t i = 0; i < N; i++) {
    auto pim = integrate(T, estimatedBias, true);
    NavState sampled = predict(pim);
    Vector9 xi = sampled.localCoordinates(prediction);
    samples.col(i) = xi;
    sum += xi;
  }

  // Compute MC covariance
  Vector9 sampleMean = sum / N;
  Matrix9 Q;
  Q.setZero();
  for (size_t i = 0; i < N; i++) {
    Vector9 xi = samples.col(i) - sampleMean;
    Q += xi * xi.transpose();
  }

  return Q / (N - 1);
}

Matrix6 ScenarioRunner::estimateNoiseCovariance(size_t N) const {
  Matrix samples(6, N);
  Vector6 sum = Vector6::Zero();
  for (size_t i = 0; i < N; i++) {
    samples.col(i) << accSampler_.sample() / sqrt_dt_,
        gyroSampler_.sample() / sqrt_dt_;
    sum += samples.col(i);
  }

  // Compute MC covariance
  Vector6 sampleMean = sum / N;
  Matrix6 Q;
  Q.setZero();
  for (size_t i = 0; i < N; i++) {
    Vector6 xi = samples.col(i) - sampleMean;
    Q += xi * xi.transpose();
  }

  return Q / (N - 1);
}

PreintegratedCombinedMeasurements CombinedScenarioRunner::integrate(
    double T, const Bias& estimatedBias, bool corrupted) const {
  gttic_(integrate);
  PreintegratedCombinedMeasurements pim(p_, estimatedBias, preintMeasCov_);

  const double dt = imuSampleTime();
  const size_t nrSteps = T / dt;
  double t = 0;
  for (size_t k = 0; k < nrSteps; k++, t += dt) {
    Vector3 measuredOmega =
        corrupted ? measuredAngularVelocity(t) : actualAngularVelocity(t);
    Vector3 measuredAcc =
        corrupted ? measuredSpecificForce(t) : actualSpecificForce(t);
    pim.integrateMeasurement(measuredAcc, measuredOmega, dt);
  }

  return pim;
}

NavState CombinedScenarioRunner::predict(
    const PreintegratedCombinedMeasurements& pim,
    const Bias& estimatedBias) const {
  const NavState state_i(scenario().pose(0), scenario().velocity_n(0));
  return pim.predict(state_i, estimatedBias);
}

Eigen::Matrix<double, 15, 15> CombinedScenarioRunner::estimateCovariance(
    double T, size_t N, const Bias& estimatedBias) const {
  gttic_(estimateCovariance);

  // Get predict prediction from ground truth measurements
  NavState prediction = predict(integrate(T));

  // Sample !
  Matrix samples(15, N);
  Vector15 sum = Vector15::Zero();
  for (size_t i = 0; i < N; i++) {
    auto pim = integrate(T, estimatedBias, true);
    NavState sampled = predict(pim);
    Vector15 xi = Vector15::Zero();
    xi << sampled.localCoordinates(prediction),
        (estimatedBias_.vector() - estimatedBias.vector());
    samples.col(i) = xi;
    sum += xi;
  }

  // Compute MC covariance
  Vector15 sampleMean = sum / N;
  Eigen::Matrix<double, 15, 15> Q;
  Q.setZero();
  for (size_t i = 0; i < N; i++) {
    Vector15 xi = samples.col(i) - sampleMean;
    Q += xi * xi.transpose();
  }

  return Q / (N - 1);
}

}  // namespace gtsam

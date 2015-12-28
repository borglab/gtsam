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

#include <cmath>

namespace gtsam {

////////////////////////////////////////////////////////////////////////////////////////////

void PreintegratedMeasurements2::integrateMeasurement(
    const Vector3& measuredAcc, const Vector3& measuredOmega, double dt) {}

NavState PreintegratedMeasurements2::predict(
    const NavState& state_i, const imuBias::ConstantBias& bias_i,
    OptionalJacobian<9, 9> H1, OptionalJacobian<9, 6> H2) const {
  return NavState();
}

////////////////////////////////////////////////////////////////////////////////////////////

static double intNoiseVar = 0.0000001;
static const Matrix3 kIntegrationErrorCovariance = intNoiseVar * I_3x3;

PreintegratedMeasurements2 ScenarioRunner::integrate(
    double T, const imuBias::ConstantBias& estimatedBias,
    bool corrupted) const {
  PreintegratedMeasurements2 pim(p_, estimatedBias);

  const double dt = imuSampleTime();
  const size_t nrSteps = T / dt;
  double t = 0;
  for (size_t k = 0; k < nrSteps; k++, t += dt) {
    Vector3 measuredOmega = corrupted ? measured_omega_b(t) : actual_omega_b(t);
    Vector3 measuredAcc =
        corrupted ? measured_specific_force_b(t) : actual_specific_force_b(t);
    pim.integrateMeasurement(measuredAcc, measuredOmega, dt);
  }

  return pim;
}

NavState ScenarioRunner::predict(
    const PreintegratedMeasurements2& pim,
    const imuBias::ConstantBias& estimatedBias) const {
  const NavState state_i(scenario_->pose(0), scenario_->velocity_n(0));
  return pim.predict(state_i, estimatedBias);
}

Matrix6 ScenarioRunner::estimatePoseCovariance(
    double T, size_t N, const imuBias::ConstantBias& estimatedBias) const {
  // Get predict prediction from ground truth measurements
  Pose3 prediction = predict(integrate(T)).pose();

  // Sample !
  Matrix samples(9, N);
  Vector6 sum = Vector6::Zero();
  for (size_t i = 0; i < N; i++) {
    Pose3 sampled = predict(integrate(T, estimatedBias, true)).pose();
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

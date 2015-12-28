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
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/inference/Symbol.h>

#include <boost/assign/std/list.hpp>

#include <cmath>

using namespace std;
using namespace boost::assign;

namespace gtsam {

using symbol_shorthand::T;
using symbol_shorthand::P;
using symbol_shorthand::V;

static const Symbol kBiasKey('B', 0);

void PreintegratedMeasurements2::integrateMeasurement(
    const Vector3& measuredAcc, const Vector3& measuredOmega, double dt) {
  typedef map<Key, Matrix> Terms;
  static const Matrix36 omega_D_bias = (Matrix36() << Z_3x3, I_3x3).finished();

  // Correct measurements by subtractig bias
  Vector3 correctedOmega = measuredOmega - estimatedBias_.gyroscope();

  GaussianFactorGraph graph;
  boost::shared_ptr<GaussianBayesNet> bayesNet;
  GaussianFactorGraph::shared_ptr shouldBeEmpty;

  // Handle first time differently
  if (k_ == 0) {
    // theta(1) = measuredOmega - (bias + bias_delta)
    graph.add<Terms>({{T(k_ + 1), I_3x3}, {kBiasKey, omega_D_bias}},
                     correctedOmega, gyroscopeNoiseModel_);
    GTSAM_PRINT(graph);

    // eliminate all but biases
    Ordering keys = list_of(T(k_ + 1));
    boost::tie(bayesNet, shouldBeEmpty) =
        graph.eliminatePartialSequential(keys, EliminateQR);
  } else {
    // add previous posterior
    graph.add(posterior_k_);

    // theta(k+1) = theta(k) + inverse(H)*(measuredOmega - (bias + bias_delta))
    // =>  H*theta(k+1) - H*theta(k) + bias_delta = measuredOmega - bias
    Matrix3 H = Rot3::ExpmapDerivative(theta_);
    graph.add<Terms>({{T(k_ + 1), H}, {T(k_), -H}, {kBiasKey, omega_D_bias}},
                     correctedOmega, gyroscopeNoiseModel_);
    GTSAM_PRINT(graph);

    // eliminate all but biases
    Ordering keys = list_of(T(k_))(T(k_ + 1));
    boost::tie(bayesNet, shouldBeEmpty) =
        graph.eliminatePartialSequential(keys, EliminateQR);
  }

  GTSAM_PRINT(*bayesNet);
  GTSAM_PRINT(*shouldBeEmpty);

  // The bayesNet now contains P(zeta(k)|zeta(k+1),bias) P(zeta(k+1)|bias)
  // We marginalize zeta(k) by dropping the first factor
  posterior_k_ = bayesNet->back();
  k_ += 1;
}

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

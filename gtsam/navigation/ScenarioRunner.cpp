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

using symbol_shorthand::T;  // for theta
using symbol_shorthand::P;  // for position
using symbol_shorthand::V;  // for velocity

static const Symbol kBiasKey('B', 0);
static const noiseModel::Constrained::shared_ptr kAllConstrained =
    noiseModel::Constrained::All(3);
static const Matrix36 acc_H_bias = (Matrix36() << I_3x3, Z_3x3).finished();
static const Matrix36 omega_H_bias = (Matrix36() << Z_3x3, I_3x3).finished();

Vector9 PreintegratedMeasurements2::currentEstimate() const {
  // TODO(frank): make faster version just for theta
  VectorValues biasValues;
  biasValues.insert(kBiasKey, estimatedBias_.vector());
  VectorValues zetaValues = posterior_k_->optimize(biasValues);
  Vector9 zeta;
  zeta << zetaValues.at(T(k_)), zetaValues.at(P(k_)), zetaValues.at(V(k_));
  return zeta;
}

void PreintegratedMeasurements2::initPosterior(const Vector3& correctedAcc,
                                               const Vector3& correctedOmega,
                                               double dt) {
  typedef map<Key, Matrix> Terms;

  GaussianFactorGraph graph;

  // theta(1) = (measuredOmega - (bias + bias_delta)) * dt
  graph.add<Terms>({{T(k_ + 1), I_3x3}, {kBiasKey, omega_H_bias}},
                   dt * correctedOmega, gyroscopeNoiseModel_);

  // pos(1) = 0
  graph.add<Terms>({{P(k_ + 1), I_3x3}}, Vector3::Zero(), kAllConstrained);

  // vel(1) = (measuredAcc - (bias + bias_delta)) * dt
  graph.add<Terms>({{V(k_ + 1), I_3x3}, {kBiasKey, acc_H_bias}},
                   dt * correctedAcc, accelerometerNoiseModel_);

  // eliminate all but biases
  // NOTE(frank): After this, posterior_k_ contains P(zeta(1)|bias)
  Ordering keys = list_of(P(k_ + 1))(V(k_ + 1))(T(k_ + 1));
  posterior_k_ = graph.eliminatePartialSequential(keys, EliminateQR).first;

  k_ += 1;
}

void PreintegratedMeasurements2::integrateMeasurement(
    const Vector3& measuredAcc, const Vector3& measuredOmega, double dt) {
  typedef map<Key, Matrix> Terms;

  // Correct measurements by subtracting bias
  Vector3 correctedAcc = measuredAcc - estimatedBias_.accelerometer();
  Vector3 correctedOmega = measuredOmega - estimatedBias_.gyroscope();

  // increment time
  deltaTij_ += dt;

  // Handle first time differently
  if (k_ == 0) {
    initPosterior(correctedAcc, correctedOmega, dt);
    return;
  }

  GaussianFactorGraph graph;

  // estimate current estimate from posterior
  // TODO(frank): maybe we should store this
  Vector9 zeta = currentEstimate();
  Vector3 theta_k = zeta.tail<3>();

  // add previous posterior
  for (const auto& conditional : *posterior_k_)
    graph.add(boost::static_pointer_cast<GaussianFactor>(conditional));

  // theta(k+1) = theta(k) + inverse(H)*(measuredOmega - bias - bias_delta) dt
  // =>  H*theta(k+1) - H*theta(k) + bias_delta dt = (measuredOmega - bias) dt
  Matrix3 H = Rot3::ExpmapDerivative(theta_k);
  graph.add<Terms>({{T(k_ + 1), H}, {T(k_), -H}, {kBiasKey, omega_H_bias * dt}},
                   dt * correctedOmega, gyroscopeNoiseModel_);

  // pos(k+1) = pos(k) + vel(k) dt
  graph.add<Terms>({{P(k_ + 1), I_3x3}, {P(k_), -I_3x3}, {V(k_), -I_3x3 * dt}},
                   Vector3::Zero(), kAllConstrained);

  // vel(k+1) = vel(k) + Rk*(measuredAcc - bias - bias_delta) dt
  // =>  Rkt*vel(k+1) - Rkt*vel(k) + bias_delta dt = (measuredAcc - bias) dt
  Rot3 Rk = Rot3::Expmap(theta_k);
  Matrix3 Rkt = Rk.transpose();
  graph.add<Terms>(
      {{V(k_ + 1), Rkt}, {V(k_), -Rkt}, {kBiasKey, acc_H_bias * dt}},
      dt * correctedAcc, accelerometerNoiseModel_);

  // eliminate all but biases
  Ordering keys = list_of(P(k_))(V(k_))(T(k_))(P(k_ + 1))(V(k_ + 1))(T(k_ + 1));
  boost::shared_ptr<GaussianBayesNet> bayesNet =
      graph.eliminatePartialSequential(keys, EliminateQR).first;

  // The bayesNet now contains P(zeta(k)|zeta(k+1),bias) P(zeta(k+1)|bias)
  // We marginalize zeta(k) by only saving the conditionals of
  // P(zeta(k+1)|bias):
  posterior_k_ = boost::make_shared<GaussianBayesNet>();
  for (const auto& conditional : *bayesNet) {
    Symbol symbol(conditional->front());
    if (symbol.index() == k_ + 1) posterior_k_->push_back(conditional);
  }

  k_ += 1;
}

NavState PreintegratedMeasurements2::predict(
    const NavState& state_i, const imuBias::ConstantBias& bias_i,
    OptionalJacobian<9, 9> H1, OptionalJacobian<9, 6> H2) const {
  // TODO(frank): handle bias
  Vector9 zeta = currentEstimate();
  cout << "zeta: " << zeta << endl;
  Rot3 Ri = state_i.attitude();
  Matrix3 Rit = Ri.transpose();
  Vector3 gt = deltaTij_ * p_->n_gravity;
  zeta.segment<3>(3) +=
      Rit * (state_i.velocity() * deltaTij_ + 0.5 * deltaTij_ * gt);
  zeta.segment<3>(6) += Rit * gt;
  cout << "zeta: " << zeta << endl;
  cout << "tij: " << deltaTij_ << endl;
  cout << "gt: " << gt.transpose() << endl;
  cout << "gt^2/2: " << 0.5 * deltaTij_ * gt.transpose() << endl;
  return state_i.expmap(zeta);
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

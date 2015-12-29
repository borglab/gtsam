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
static const Matrix36 acc_H_bias = (Matrix36() << I_3x3, Z_3x3).finished();
static const Matrix36 omega_H_bias = (Matrix36() << Z_3x3, I_3x3).finished();

Vector9 PreintegratedMeasurements2::currentEstimate() const {
  VectorValues biasValues;
  biasValues.insert(kBiasKey, estimatedBias_.vector());
  VectorValues zetaValues = posterior_k_->optimize(biasValues);
  Vector9 zeta;
  zeta << zetaValues.at(T(k_)), zetaValues.at(P(k_)), zetaValues.at(V(k_));
  return zeta;
}

Vector3 PreintegratedMeasurements2::currentTheta() const {
  // TODO(frank): make faster version theta = inv(R)*d
  VectorValues biasValues;
  biasValues.insert(kBiasKey, estimatedBias_.vector());
  VectorValues zetaValues = posterior_k_->optimize(biasValues);
  return zetaValues.at(T(k_));
}

SharedDiagonal PreintegratedMeasurements2::discreteAccelerometerNoiseModel(
    double dt) const {
  return noiseModel::Diagonal::Sigmas(accelerometerNoiseModel_->sigmas() /
                                      std::sqrt(dt));
}

SharedDiagonal PreintegratedMeasurements2::discreteGyroscopeNoiseModel(
    double dt) const {
  return noiseModel::Diagonal::Sigmas(gyroscopeNoiseModel_->sigmas() /
                                      std::sqrt(dt));
}

PreintegratedMeasurements2::SharedBayesNet
PreintegratedMeasurements2::initPosterior(const Vector3& correctedAcc,
                                          const Vector3& correctedOmega,
                                          double dt) const {
  typedef map<Key, Matrix> Terms;

  // We create a factor graph and then compute P(zeta|bias)
  GaussianFactorGraph graph;

  // theta(1) = (correctedOmega - bias_delta) * dt
  // => theta(1) + bias_delta * dt = correctedOmega * dt
  graph.add<Terms>({{T(k_ + 1), I_3x3}, {kBiasKey, omega_H_bias * dt}},
                   correctedOmega * dt, discreteGyroscopeNoiseModel(dt));

  // pose(1) = (correctedAcc - bias_delta) * dt^2/2
  // => pose(1) + bias_delta * dt^2/2 = correctedAcc * dt^2/2
  double dt22 = 0.5 * dt * dt;
  auto accModel = discreteAccelerometerNoiseModel(dt);
  graph.add<Terms>({{P(k_ + 1), I_3x3}, {kBiasKey, acc_H_bias * dt22}},
                   correctedAcc * dt22, accModel);

  // vel(1) = (correctedAcc - bias_delta) * dt
  // => vel(1) + bias_delta * dt = correctedAcc * dt
  graph.add<Terms>({{V(k_ + 1), I_3x3}, {kBiasKey, acc_H_bias * dt}},
                   correctedAcc * dt, accModel);

  // eliminate all but biases
  // NOTE(frank): After this, posterior_k_ contains P(zeta(1)|bias)
  Ordering keys = list_of(T(k_ + 1))(P(k_ + 1))(V(k_ + 1));
  return graph.eliminatePartialSequential(keys, EliminateQR).first;
}

PreintegratedMeasurements2::SharedBayesNet
PreintegratedMeasurements2::integrateCorrected(const Vector3& correctedAcc,
                                               const Vector3& correctedOmega,
                                               double dt) const {
  typedef map<Key, Matrix> Terms;

  GaussianFactorGraph graph;

  // estimate current theta from posterior
  Vector3 theta_k = currentTheta();
  Rot3 Rk = Rot3::Expmap(theta_k);
  Matrix3 Rkt = Rk.transpose();

  // add previous posterior
  for (const auto& conditional : *posterior_k_)
    graph.add(boost::static_pointer_cast<GaussianFactor>(conditional));

  // theta(k+1) = theta(k) + inverse(H)*(correctedOmega - bias_delta) dt
  // =>  H*theta(k+1) - H*theta(k) + bias_delta dt = (measuredOmega - bias) dt
  Matrix3 H = Rot3::ExpmapDerivative(theta_k);
  graph.add<Terms>({{T(k_ + 1), H}, {T(k_), -H}, {kBiasKey, omega_H_bias * dt}},
                   correctedOmega * dt, discreteGyroscopeNoiseModel(dt));

  // pos(k+1) = pos(k) + vel(k) dt + Rk*(correctedAcc - bias_delta) dt^2/2
  //  => Rkt*pos(k+1) - Rkt*pos(k) - Rkt*vel(k) dt + bias_delta dt^2/2
  //     = correctedAcc dt^2/2
  double dt22 = 0.5 * dt * dt;
  auto accModel = discreteAccelerometerNoiseModel(dt);
  graph.add<Terms>({{P(k_ + 1), Rkt},
                    {P(k_), -Rkt},
                    {V(k_), -Rkt * dt},
                    {kBiasKey, acc_H_bias * dt22}},
                   correctedAcc * dt22, accModel);

  // vel(k+1) = vel(k) + Rk*(correctedAcc - bias_delta) dt
  // =>  Rkt*vel(k+1) - Rkt*vel(k) + bias_delta dt = correctedAcc * dt
  graph.add<Terms>(
      {{V(k_ + 1), Rkt}, {V(k_), -Rkt}, {kBiasKey, acc_H_bias * dt}},
      correctedAcc * dt, accModel);

  // eliminate all but biases
  // TODO(frank): does not seem to eliminate in order I want. What gives?
  Ordering keys = list_of(T(k_))(P(k_))(V(k_))(T(k_ + 1))(P(k_ + 1))(V(k_ + 1));
  SharedBayesNet bayesNet =
      graph.eliminatePartialSequential(keys, EliminateQR).first;

  // The Bayes net now contains P(zeta(k)|zeta(k+1),bias) P(zeta(k+1)|bias)
  // We marginalize zeta(k) by removing the conditionals on zeta(k)
  // TODO(frank): could use erase(begin, begin+3) if order above was correct
  SharedBayesNet marginal = boost::make_shared<GaussianBayesNet>();
  for (const auto& conditional : *bayesNet) {
    Symbol symbol(conditional->front());
    if (symbol.index() > k_) marginal->push_back(conditional);
  }

  return marginal;
}

void PreintegratedMeasurements2::integrateMeasurement(
    const Vector3& measuredAcc, const Vector3& measuredOmega, double dt) {
  typedef map<Key, Matrix> Terms;

  // Correct measurements by subtracting bias
  Vector3 correctedAcc = measuredAcc - estimatedBias_.accelerometer();
  Vector3 correctedOmega = measuredOmega - estimatedBias_.gyroscope();

  // Handle first time differently
  if (k_ == 0)
    posterior_k_ = initPosterior(correctedAcc, correctedOmega, dt);
  else
    posterior_k_ = integrateCorrected(correctedAcc, correctedOmega, dt);

  // increment counter and time
  k_ += 1;
  deltaTij_ += dt;
}

NavState PreintegratedMeasurements2::predict(
    const NavState& state_i, const imuBias::ConstantBias& bias_i,
    OptionalJacobian<9, 9> H1, OptionalJacobian<9, 6> H2) const {
  // Get mean of current posterior on zeta
  // TODO(frank): handle bias
  Vector9 zeta = currentEstimate();

  // Correct for initial velocity and gravity
  Rot3 Ri = state_i.attitude();
  Matrix3 Rit = Ri.transpose();
  Vector3 gt = deltaTij_ * p_->n_gravity;
  zeta.segment<3>(3) +=
      Rit * (state_i.velocity() * deltaTij_ + 0.5 * deltaTij_ * gt);
  zeta.segment<3>(6) += Rit * gt;

  // Convert local coordinates to manifold near state_i
  return state_i.retract(zeta);
}

SharedGaussian PreintegratedMeasurements2::noiseModel() const {
  Matrix RS;
  Vector d;
  boost::tie(RS, d) = posterior_k_->matrix();

  // R'*R = A'*A = inv(Cov)
  // TODO(frank): think of a faster way - implement in noiseModel
  return noiseModel::Gaussian::SqrtInformation(RS.block<9, 9>(0, 0), false);
}

Matrix9 PreintegratedMeasurements2::preintMeasCov() const {
  return noiseModel()->covariance();
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

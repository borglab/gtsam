/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    AggregateImuReadings.cpp
 * @brief   Integrates IMU readings on the NavState tangent space
 * @author  Frank Dellaert
 */

#include <gtsam/navigation/AggregateImuReadings.h>
#include <gtsam/navigation/functors.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
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

AggregateImuReadings::AggregateImuReadings(const boost::shared_ptr<Params>& p,
                                           const Bias& estimatedBias)
    : p_(p),
      accelerometerNoiseModel_(Diagonal(p->accelerometerCovariance)),
      gyroscopeNoiseModel_(Diagonal(p->gyroscopeCovariance)),
      estimatedBias_(estimatedBias),
      k_(0),
      deltaTij_(0.0) {
  // Initialize values with zeros
  static const Vector3 kZero(Vector3::Zero());
  values.insert<Vector3>(T(0), kZero);
  values.insert<Vector3>(P(0), kZero);
  values.insert<Vector3>(V(0), kZero);
  values.insert<Bias>(kBiasKey, estimatedBias_);
}

SharedDiagonal AggregateImuReadings::discreteAccelerometerNoiseModel(
    double dt) const {
  return noiseModel::Diagonal::Sigmas(accelerometerNoiseModel_->sigmas() /
                                      std::sqrt(dt));
}

SharedDiagonal AggregateImuReadings::discreteGyroscopeNoiseModel(
    double dt) const {
  return noiseModel::Diagonal::Sigmas(gyroscopeNoiseModel_->sigmas() /
                                      std::sqrt(dt));
}

void AggregateImuReadings::updateEstimate(const Vector3& measuredAcc,
                                          const Vector3& measuredOmega,
                                          double dt) {
  // Get current estimates
  const Vector3 theta = values.at<Vector3>(T(k_));
  const Vector3 pos = values.at<Vector3>(P(k_));
  const Vector3 vel = values.at<Vector3>(V(k_));

  // Correct measurements
  const Vector3 correctedAcc = measuredAcc - estimatedBias_.accelerometer();
  const Vector3 correctedOmega = measuredOmega - estimatedBias_.gyroscope();

  // Calculate exact mean propagation
  Matrix3 H;
  const Rot3 R = Rot3::Expmap(theta, H);
  const Vector3 theta_plus = theta + H.inverse() * correctedOmega * dt;
  const Vector3 vel_plus = vel + R.rotate(correctedAcc) * dt;
  const Vector3 vel_avg = 0.5 * (vel + vel_plus);
  const Vector3 pos_plus = pos + vel_avg * dt;

  // Add those values to estimate and linearize around them
  values.insert<Vector3>(T(k_ + 1), theta_plus);
  values.insert<Vector3>(P(k_ + 1), pos_plus);
  values.insert<Vector3>(V(k_ + 1), vel_plus);
}

NonlinearFactorGraph AggregateImuReadings::createGraph(
    const Vector3_& theta_, const Vector3_& pos_, const Vector3_& vel_,
    const Vector3& measuredAcc, const Vector3& measuredOmega, double dt) const {
  NonlinearFactorGraph graph;
  Expression<Bias> bias_(kBiasKey);
  Vector3_ theta_plus_(T(k_ + 1)), pos_plus_(P(k_ + 1)), vel_plus_(V(k_ + 1));

  Vector3_ omega_(PredictAngularVelocity(dt), theta_, theta_plus_);
  Vector3_ measuredOmega_(boost::bind(&Bias::correctGyroscope, _1, _2, _3, _4),
                          bias_, omega_);
  auto gyroModel = discreteGyroscopeNoiseModel(dt);
  graph.addExpressionFactor(gyroModel, measuredOmega, measuredOmega_);

  Vector3_ averageVelocity_(averageVelocity, vel_, vel_plus_);
  Vector3_ defect_(PositionDefect(dt), pos_, pos_plus_, averageVelocity_);
  static const auto constrModel = noiseModel::Constrained::All(3);
  static const Vector3 kZero(Vector3::Zero());
  graph.addExpressionFactor(constrModel, kZero, defect_);

  Vector3_ acc_(PredictAcceleration(dt), vel_, vel_plus_, theta_);
  Vector3_ measuredAcc_(
      boost::bind(&Bias::correctAccelerometer, _1, _2, _3, _4), bias_, acc_);
  auto accModel = discreteAccelerometerNoiseModel(dt);
  graph.addExpressionFactor(accModel, measuredAcc, measuredAcc_);

  return graph;
}

AggregateImuReadings::SharedBayesNet AggregateImuReadings::initPosterior(
    const Vector3& measuredAcc, const Vector3& measuredOmega, double dt) {
  static const Vector3 kZero(Vector3::Zero());
  static const Vector3_ zero_(kZero);

  // We create a factor graph and then compute P(zeta|bias)
  auto graph = createGraph(zero_, zero_, zero_, measuredAcc, measuredOmega, dt);

  // Linearize using updated values (updateEstimate must have been called)
  auto linear_graph = graph.linearize(values);

  // eliminate all but biases
  // NOTE(frank): After this, posterior_k_ contains P(zeta(1)|bias)
  Ordering keys = list_of(T(k_ + 1))(P(k_ + 1))(V(k_ + 1));
  return linear_graph->eliminatePartialSequential(keys, EliminateQR).first;
}

AggregateImuReadings::SharedBayesNet AggregateImuReadings::updatePosterior(
    const Vector3& measuredAcc, const Vector3& measuredOmega, double dt) {
  static const Vector3 kZero(Vector3::Zero());
  static const auto constrModel = noiseModel::Constrained::All(3);

  // We create a factor graph and then compute P(zeta|bias)
  // TODO(frank): Expmap and ExpmapDerivative are called again :-(
  auto graph = createGraph(Vector3_(T(k_)), Vector3_(P(k_)), Vector3_(V(k_)),
                           measuredAcc, measuredOmega, dt);

  // Linearize using updated values (updateEstimate must have been called)
  auto linear_graph = graph.linearize(values);

  // add previous posterior
  for (const auto& conditional : *posterior_k_)
    linear_graph->add(boost::static_pointer_cast<GaussianFactor>(conditional));

  // eliminate all but biases
  // TODO(frank): does not seem to eliminate in order I want. What gives?
  Ordering keys = list_of(T(k_))(P(k_))(V(k_))(T(k_ + 1))(P(k_ + 1))(V(k_ + 1));
  SharedBayesNet bayesNet =
      linear_graph->eliminatePartialSequential(keys, EliminateQR).first;

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

void AggregateImuReadings::integrateMeasurement(const Vector3& measuredAcc,
                                                const Vector3& measuredOmega,
                                                double dt) {
  typedef map<Key, Matrix> Terms;

  // Do exact mean propagation
  updateEstimate(measuredAcc, measuredOmega, dt);

  // Use factor graph machinery to linearize aroud exact propagation and
  // calculate posterior density on the prediction
  if (k_ == 0)
    posterior_k_ = initPosterior(measuredAcc, measuredOmega, dt);
  else
    posterior_k_ = updatePosterior(measuredAcc, measuredOmega, dt);

  // increment counter and time
  k_ += 1;
  deltaTij_ += dt;
}

Vector9 AggregateImuReadings::zeta() const {
  Vector9 zeta;
  zeta << values.at<Vector3>(T(k_)), values.at<Vector3>(P(k_)),
      values.at<Vector3>(V(k_));
  return zeta;
}

NavState AggregateImuReadings::predict(const NavState& state_i,
                                       const Bias& bias_i,
                                       OptionalJacobian<9, 9> H1,
                                       OptionalJacobian<9, 6> H2) const {
  // TODO(frank): handle bias

  // Get current estimates
  Vector3 theta = values.at<Vector3>(T(k_));
  Vector3 pos = values.at<Vector3>(P(k_));
  Vector3 vel = values.at<Vector3>(V(k_));

// Correct for initial velocity and gravity
#if 1
  Rot3 Ri = state_i.attitude();
  Matrix3 Rit = Ri.transpose();
  Vector3 gt = deltaTij_ * p_->n_gravity;
  pos += Rit * (state_i.velocity() * deltaTij_ + 0.5 * deltaTij_ * gt);
  vel += Rit * gt;
#endif

  // Convert local coordinates to manifold near state_i
  Vector9 zeta;
  zeta << theta, pos, vel;
  return state_i.retract(zeta);
}

SharedGaussian AggregateImuReadings::noiseModel() const {
  // Get covariance on zeta from Bayes Net, which stores P(zeta|bias) as a
  // quadratic |R*zeta + S*bias -d|^2
  Matrix RS;
  Vector d;
  boost::tie(RS, d) = posterior_k_->matrix();
  // NOTEfrank): R'*R = inv(zetaCov)
  Matrix9 R = RS.block<9, 9>(0, 0);

  // Correct for application of retract, by calculating the retract derivative H
  // We have inv(Rp'Rp) = H inv(Rz'Rz) H' => Rp = Rz * inv(H)
  // From NavState::retract:
  //  H << D_R_theta, Z_3x3, Z_3x3,
  //       Z_3x3, iRj.transpose(), Z_3x3,
  //       Z_3x3, Z_3x3, iRj.transpose();
  Matrix3 D_R_theta;
  Vector3 theta = values.at<Vector3>(T(k_));
  // TODO(frank): yet another application of expmap and expmap derivative
  const Matrix3 iRj = Rot3::Expmap(theta, D_R_theta).matrix();

  // Rp = R * H.inverse(), implemented blockwise in-place below
  // NOTE(frank): makes sense: a change in the j-frame has to be converted to a
  // change in the i-frame, byy rotating with iRj. Similarly, a change in
  // rotation nRj is mapped to a change in theta via the inverse dexp.
  R.block<9, 3>(0, 0) *= D_R_theta.inverse();
  R.block<9, 3>(0, 3) *= iRj;
  R.block<9, 3>(0, 6) *= iRj;

  // TODO(frank): think of a faster way - implement in noiseModel
  return noiseModel::Gaussian::SqrtInformation(R, false);
}

Matrix9 AggregateImuReadings::preintMeasCov() const {
  return noiseModel()->covariance();
}

}  // namespace gtsam

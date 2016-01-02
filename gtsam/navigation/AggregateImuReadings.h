/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    AggregateImuReadings.h
 * @brief   Integrates IMU readings on the NavState tangent space
 * @author  Frank Dellaert
 */

#pragma once

#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/linear/NoiseModel.h>

namespace gtsam {

class NonlinearFactorGraph;
template <typename T>
class Expression;
typedef Expression<Vector3> Vector3_;

// Convert covariance to diagonal noise model, if possible, otherwise throw
static noiseModel::Diagonal::shared_ptr Diagonal(const Matrix& covariance) {
  bool smart = true;
  auto model = noiseModel::Gaussian::Covariance(covariance, smart);
  auto diagonal = boost::dynamic_pointer_cast<noiseModel::Diagonal>(model);
  if (!diagonal)
    throw std::invalid_argument("ScenarioRunner::Diagonal: not a diagonal");
  return diagonal;
}

class GaussianBayesNet;

/**
 * Class that integrates state estimate on the manifold.
 * We integrate zeta = [theta, position, velocity]
 * See ImuFactor.lyx for the relevant math.
 */
class AggregateImuReadings {
 public:
  typedef imuBias::ConstantBias Bias;
  typedef ImuFactor::PreintegratedMeasurements::Params Params;
  typedef boost::shared_ptr<GaussianBayesNet> SharedBayesNet;

 private:
  const boost::shared_ptr<Params> p_;
  const SharedDiagonal accelerometerNoiseModel_, gyroscopeNoiseModel_;
  const Bias estimatedBias_;

  size_t k_;         ///< index/count of measurements integrated
  double deltaTij_;  ///< sum of time increments

  /// posterior on current iterate, stored as a Bayes net
  /// P(delta_zeta|estimatedBias_delta):
  SharedBayesNet posterior_k_;

  /// Current estimate of zeta_k
  Values values;

 public:
  AggregateImuReadings(const boost::shared_ptr<Params>& p,
                       const Bias& estimatedBias = Bias())
      : p_(p),
        accelerometerNoiseModel_(Diagonal(p->accelerometerCovariance)),
        gyroscopeNoiseModel_(Diagonal(p->gyroscopeCovariance)),
        estimatedBias_(estimatedBias),
        k_(0),
        deltaTij_(0.0) {}

  // We obtain discrete-time noise models by dividing the continuous-time
  // covariances by dt:

  SharedDiagonal discreteAccelerometerNoiseModel(double dt) const;
  SharedDiagonal discreteGyroscopeNoiseModel(double dt) const;

  /**
   * Add a single IMU measurement to the preintegration.
   * @param measuredAcc Measured acceleration (in body frame)
   * @param measuredOmega Measured angular velocity (in body frame)
   * @param dt Time interval between this and the last IMU measurement
   */
  void integrateMeasurement(const Vector3& measuredAcc,
                            const Vector3& measuredOmega, double dt);

  Vector9 zeta() const;

  /// Predict state at time j
  NavState predict(const NavState& state_i, const Bias& estimatedBias_i,
                   OptionalJacobian<9, 9> H1 = boost::none,
                   OptionalJacobian<9, 6> H2 = boost::none) const;

  /// Return Gaussian noise model on prediction
  SharedGaussian noiseModel() const;

  /// @deprecated: Explicitly calculate covariance
  Matrix9 preintMeasCov() const;

 private:
  NonlinearFactorGraph createGraph(const Vector3_& theta_,
                                   const Vector3_& pose_, const Vector3_& vel_,
                                   const Vector3& measuredAcc,
                                   const Vector3& measuredOmega,
                                   double dt) const;

  // initialize posterior with first (corrected) IMU measurement
  SharedBayesNet initPosterior(const Vector3& measuredAcc,
                               const Vector3& measuredOmega, double dt);

  // integrate
  SharedBayesNet integrateCorrected(const Vector3& measuredAcc,
                                    const Vector3& measuredOmega, double dt);
};

}  // namespace gtsam

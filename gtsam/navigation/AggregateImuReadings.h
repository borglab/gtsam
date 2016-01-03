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

/**
 * Class that integrates state estimate on the manifold.
 * We integrate zeta = [theta, position, velocity]
 * See ImuFactor.lyx for the relevant math.
 */
class AggregateImuReadings {
 public:
  typedef imuBias::ConstantBias Bias;
  typedef ImuFactor::PreintegratedMeasurements::Params Params;

 private:
  const boost::shared_ptr<Params> p_;
  const SharedGaussian accelerometerNoiseModel_, gyroscopeNoiseModel_;
  const Bias estimatedBias_;

  size_t k_;         ///< index/count of measurements integrated
  double deltaTij_;  ///< sum of time increments

  /// Current estimate of zeta_k
  Vector9 zeta_;
  Matrix9 cov_;

 public:
  AggregateImuReadings(const boost::shared_ptr<Params>& p,
                       const Bias& estimatedBias = Bias());

  const Vector9& zeta() const { return zeta_; }
  const Matrix9& zetaCov() const { return cov_; }

  /**
   * Add a single IMU measurement to the preintegration.
   * @param measuredAcc Measured acceleration (in body frame)
   * @param measuredOmega Measured angular velocity (in body frame)
   * @param dt Time interval between this and the last IMU measurement
   * TODO(frank): put useExactDexpDerivative in params
   */
  void integrateMeasurement(const Vector3& measuredAcc,
                            const Vector3& measuredOmega, double dt,
                            bool useExactDexpDerivative = false);

  /// Predict state at time j
  NavState predict(const NavState& state_i, const Bias& estimatedBias_i,
                   OptionalJacobian<9, 9> H1 = boost::none,
                   OptionalJacobian<9, 6> H2 = boost::none) const;

  /// Return Gaussian noise model on prediction
  SharedGaussian noiseModel() const;

  /// @deprecated: Explicitly calculate covariance
  Matrix9 preintMeasCov() const;

  Vector3 theta() const { return zeta_.head<3>(); }

  static Vector9 UpdateEstimate(const Vector9& zeta,
                                const Vector3& correctedAcc,
                                const Vector3& correctedOmega, double dt,
                                bool useExactDexpDerivative = false,
                                OptionalJacobian<9, 9> A = boost::none,
                                OptionalJacobian<9, 3> Ba = boost::none,
                                OptionalJacobian<9, 3> Bw = boost::none);
};

}  // namespace gtsam

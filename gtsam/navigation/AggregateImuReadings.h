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

#include <Eigen/Dense>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/PreintegrationParams.h>
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
  typedef PreintegrationParams Params;

  /// The IMU is integrated in the tangent space, represented by a Vector9
  /// This small inner class provides some convenient constructors and efficient
  /// access to the orientation, position, and velocity components
  class TangentVector {
    Vector9 v_;
    typedef const Vector9 constV9;

   public:
    TangentVector() { v_.setZero(); }
    TangentVector(const Vector9& v) : v_(v) {}
    TangentVector(const Vector3& theta, const Vector3& pos,
                  const Vector3& vel) {
      this->theta() = theta;
      this->position() = pos;
      this->velocity() = vel;
    }

    const Vector9& vector() const { return v_; }

    Eigen::Block<Vector9, 3, 1> theta() { return v_.segment<3>(0); }
    Eigen::Block<constV9, 3, 1> theta() const { return v_.segment<3>(0); }

    Eigen::Block<Vector9, 3, 1> position() { return v_.segment<3>(3); }
    Eigen::Block<constV9, 3, 1> position() const { return v_.segment<3>(3); }

    Eigen::Block<Vector9, 3, 1> velocity() { return v_.segment<3>(6); }
    Eigen::Block<constV9, 3, 1> velocity() const { return v_.segment<3>(6); }
  };

 private:
  const boost::shared_ptr<Params> p_;
  const Bias biasHat_;

  double deltaTij_;  ///< sum of time increments

  /// Current estimate of zeta_k
  TangentVector zeta_;
  Matrix9 cov_;

 public:
  AggregateImuReadings(const boost::shared_ptr<Params>& p,
                       const Bias& estimatedBias = Bias());

  Vector3 theta() const { return zeta_.theta(); }
  const Vector9& zeta() const { return zeta_.vector(); }
  const Matrix9& zetaCov() const { return cov_; }

  /**
   * Add a single IMU measurement to the preintegration.
   * @param measuredAcc Measured acceleration (in body frame)
   * @param measuredOmega Measured angular velocity (in body frame)
   * @param dt Time interval between this and the last IMU measurement
   * TODO(frank): put useExactDexpDerivative in params
   */
  void integrateMeasurement(const Vector3& measuredAcc,
                            const Vector3& measuredOmega, double dt);

  /// Predict state at time j
  NavState predict(const NavState& state_i, const Bias& estimatedBias_i,
                   OptionalJacobian<9, 9> H1 = boost::none,
                   OptionalJacobian<9, 6> H2 = boost::none) const;

  /// Return Gaussian noise model on prediction
  SharedGaussian noiseModel() const;

  /// @deprecated: Explicitly calculate covariance
  Matrix9 preintMeasCov() const;

  // Update integrated vector on tangent manifold zeta with acceleration
  // readings a_body and gyro readings w_body, bias-corrected in body frame.
  static TangentVector UpdateEstimate(const Vector3& a_body,
                                      const Vector3& w_body, double dt,
                                      const TangentVector& zeta,
                                      OptionalJacobian<9, 9> A = boost::none,
                                      OptionalJacobian<9, 3> B = boost::none,
                                      OptionalJacobian<9, 3> C = boost::none);
};

}  // namespace gtsam

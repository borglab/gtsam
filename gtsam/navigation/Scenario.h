/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Scenario.h
 * @brief   Simple class to test navigation scenarios
 * @author  Frank Dellaert
 */

#pragma once
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/geometry/Pose3.h>

namespace gtsam {

/**
 * Simple IMU simulator.
 * It is also assumed that gravity is magically counteracted and has no effect
 * on trajectory. Hence, a simulated IMU yields the actual body angular
 * velocity, and negative G acceleration plus the acceleration created by the
 * rotating body frame.
 */
class Scenario {
 public:
  /// Construct scenario with constant twist [w,v]
  Scenario(double imuSampleTime = 1.0 / 100.0, double gyroSigma = 0.17,
           double accSigma = 0.01)
      : imuSampleTime_(imuSampleTime),
        gyroNoiseModel_(noiseModel::Isotropic::Sigma(3, gyroSigma)),
        accNoiseModel_(noiseModel::Isotropic::Sigma(3, accSigma)) {}

  const double& imuSampleTime() const { return imuSampleTime_; }

  // NOTE(frank): hardcoded for now with Z up (gravity points in negative Z)
  // also, uses g=10 for easy debugging
  Vector3 gravity() const { return Vector3(0, 0, -10.0); }

  const noiseModel::Diagonal::shared_ptr& gyroNoiseModel() const {
    return gyroNoiseModel_;
  }

  const noiseModel::Diagonal::shared_ptr& accNoiseModel() const {
    return accNoiseModel_;
  }

  Matrix3 gyroCovariance() const { return gyroNoiseModel_->covariance(); }
  Matrix3 accCovariance() const { return accNoiseModel_->covariance(); }

  /// Pose of body in nav frame at time t
  virtual Pose3 pose(double t) const = 0;

  /// Rotation of body in nav frame at time t
  virtual Rot3 rotation(double t) const { return pose(t).rotation(); }

  virtual Vector3 angularVelocityInBody(double t) const = 0;
  virtual Vector3 linearVelocityInBody(double t) const = 0;
  virtual Vector3 accelerationInBody(double t) const = 0;

  /// Velocity in nav frame at time t
  Vector3 velocity(double t) const {
    return rotation(t) * linearVelocityInBody(t);
  }

  // acceleration in nav frame
  // TODO(frank): correct for rotating frames?
  Vector3 acceleration(double t) const {
    return rotation(t) * accelerationInBody(t);
  }

 private:
  double imuSampleTime_;
  noiseModel::Diagonal::shared_ptr gyroNoiseModel_, accNoiseModel_;
};

/**
 * Simple IMU simulator with constant twist 3D trajectory.
 */
class ExpmapScenario : public Scenario {
 public:
  /// Construct scenario with constant twist [w,v]
  ExpmapScenario(const Vector3& w, const Vector3& v,
                 double imuSampleTime = 1.0 / 100.0, double gyroSigma = 0.17,
                 double accSigma = 0.01)
      : Scenario(imuSampleTime, gyroSigma, accSigma),
        twist_((Vector6() << w, v).finished()),
        centripetalAcceleration_(twist_.head<3>().cross(twist_.tail<3>())) {}

  Pose3 pose(double t) const { return Pose3::Expmap(twist_ * t); }

  Vector3 angularVelocityInBody(double t) const { return twist_.head<3>(); }
  Vector3 linearVelocityInBody(double t) const { return twist_.tail<3>(); }

  Vector3 accelerationInBody(double t) const {
    const Rot3 nRb = rotation(t);
    return centripetalAcceleration_ - nRb.transpose() * gravity();
  }

 private:
  const Vector6 twist_;
  const Vector3 centripetalAcceleration_;
};

}  // namespace gtsam

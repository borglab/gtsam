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
 * Simple IMU simulator with constant twist 3D trajectory.
 * It is also assumed that gravity is magically counteracted and has no effect
 * on trajectory. Hence, a simulated IMU yields the actual body angular
 * velocity, and negative G acceleration plus the acceleration created by the
 * rotating body frame.
 */
class Scenario {
 public:
  /// Construct scenario with constant twist [w,v]
  Scenario(const Vector3& w, const Vector3& v,
           double imuSampleTime = 1.0 / 100.0, double gyroSigma = 0.17,
           double accSigma = 0.01)
      : twist_((Vector6() << w, v).finished()),
        imuSampleTime_(imuSampleTime),
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

  Vector3 angularVelocityInBody() const { return twist_.head<3>(); }
  Vector3 linearVelocityInBody() const { return twist_.tail<3>(); }

  /// Rotation of body in nav frame at time t
  Rot3 rotAtTime(double t) const {
    return Rot3::Expmap(angularVelocityInBody() * t);
  }

  /// Pose of body in nav frame at time t
  Pose3 pose(double t) const { return Pose3::Expmap(twist_ * t); }

  /// Velocity in nav frame at time t
  Vector3 velocity(double t) const {
    const Rot3 nRb = rotAtTime(t);
    return nRb * linearVelocityInBody();
  }

  // acceleration in nav frame
  Vector3 acceleration(double t) const {
    const Vector3 centripetalAcceleration =
        angularVelocityInBody().cross(linearVelocityInBody());
    const Rot3 nRb = rotAtTime(t);
    return nRb * centripetalAcceleration - gravity();
  }

  // acceleration in body frame frame
  Vector3 accelerationInBody(double t) const {
    const Vector3 centripetalAcceleration =
        angularVelocityInBody().cross(linearVelocityInBody());
    const Rot3 nRb = rotAtTime(t);
    return centripetalAcceleration - nRb.transpose() * gravity();
  }

 private:
  Vector6 twist_;
  double imuSampleTime_;
  noiseModel::Diagonal::shared_ptr gyroNoiseModel_, accNoiseModel_;
};

}  // namespace gtsam

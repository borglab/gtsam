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
#include <gtsam/geometry/Pose3.h>

namespace gtsam {

/**
 * Simple class with constant twist 3D trajectory.
 * It is also assumed that gravity is magically counteracted and has no effect
 * on trajectory. Hence, a simulated IMU yields the actual body angular
 * velocity, and negative G acceleration plus the acceleration created by the
 * rotating body frame.
 */
class Scenario {
 public:
  /// Construct scenario with constant twist [w,v]
  Scenario(const Vector3& w, const Vector3& v,
           double imuSampleTime = 1.0 / 100.0)
      : twist_((Vector6() << w, v).finished()), imuSampleTime_(imuSampleTime) {}

  const double& imuSampleTime() const { return imuSampleTime_; }

  // NOTE(frank): hardcoded for now with Z up (gravity points in negative Z)
  // also, uses g=10 for easy debugging
  Vector3 gravity() const { return Vector3(0, 0, -10.0); }

  Vector3 angularVelocityInBody() const { return twist_.head<3>(); }
  Vector3 linearVelocityInBody() const { return twist_.tail<3>(); }

  /// Rotation of body in nav frame at time t
  Rot3 rotAtTime(double t) const {
    return Rot3::Expmap(angularVelocityInBody() * t);
  }

  /// Pose of body in nav frame at time t
  Pose3 poseAtTime(double t) const { return Pose3::Expmap(twist_ * t); }

  /// Velocity in nav frame at time t
  Vector3 velocityAtTime(double t) {
    const Rot3 nRb = rotAtTime(t);
    return nRb * linearVelocityInBody();
  }

  // acceleration in nav frame
  Vector3 accelerationAtTime(double t) const {
    const Rot3 nRb = rotAtTime(t);
    const Vector3 centripetalAcceleration =
        angularVelocityInBody().cross(linearVelocityInBody());
    return nRb * centripetalAcceleration - gravity();
  }

 private:
  Vector6 twist_;
  double imuSampleTime_;
};

}  // namespace gtsam

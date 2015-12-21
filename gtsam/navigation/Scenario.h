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

/// Simple class with constant twist 3D trajectory
class Scenario {
 public:
  /// Construct scenario with constant twist [w,v]
  Scenario(const Vector3& w, const Vector3& v, double imuSampleTime = 1e-2)
      : twist_((Vector6() << w, v).finished()), imuSampleTime_(imuSampleTime) {}

  // NOTE(frank): hardcoded for now with Z up (gravity points in negative Z)
  // also, uses g=10 for easy debugging
  Vector3 gravity() const { return Vector3(0, 0, -10.0); }

  Vector3 groundTruthGyroInBody() const { return twist_.head<3>(); }
  Vector3 groundTruthVelocityInBody() const { return twist_.tail<3>(); }

  // All constant twist scenarios have zero acceleration
  Vector3 groundTruthAccInBody() const { return Vector3::Zero(); }

  const double& imuSampleTime() const { return imuSampleTime_; }

  /// Pose of body in nav frame at time t
  Pose3 poseAtTime(double t) { return Pose3::Expmap(twist_ * t); }

  /// Velocity in nav frame at time t
  Vector3 velocityAtTime(double t) {
    const Pose3 pose = poseAtTime(t);
    const Rot3& nRb = pose.rotation();
    return nRb * groundTruthVelocityInBody();
  }

 private:
  Vector6 twist_;
  double imuSampleTime_;
};

}  // namespace gtsam

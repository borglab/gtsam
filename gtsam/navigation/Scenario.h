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

  Vector3 groundTruthAcc() const { return twist_.tail<3>(); }
  Vector3 groundTruthGyro() const { return twist_.head<3>(); }
  const double& imuSampleTime() const { return imuSampleTime_; }

  Pose3 poseAtTime(double t) { return Pose3::Expmap(twist_ * t); }

 private:
  Vector6 twist_;
  double imuSampleTime_;
};

}  // namespace gtsam

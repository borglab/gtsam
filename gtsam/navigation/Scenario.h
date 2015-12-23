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

/// Simple trajectory simulator.
class Scenario {
 public:
  // Quantities a Scenario needs to specify:

  virtual Pose3 pose(double t) const = 0;
  virtual Vector3 omega_b(double t) const = 0;
  virtual Vector3 velocity_n(double t) const = 0;
  virtual Vector3 acceleration_n(double t) const = 0;

  // Derived quantities:

  virtual Rot3 rotation(double t) const { return pose(t).rotation(); }

  Vector3 velocity_b(double t) const {
    const Rot3 nRb = rotation(t);
    return nRb.transpose() * velocity_n(t);
  }

  Vector3 acceleration_b(double t) const {
    const Rot3 nRb = rotation(t);
    return nRb.transpose() * acceleration_n(t);
  }
};

/// Scenario with constant twist 3D trajectory.
class ExpmapScenario : public Scenario {
 public:
  /// Construct scenario with constant twist [w,v]
  ExpmapScenario(const Vector3& w, const Vector3& v)
      : twist_((Vector6() << w, v).finished()),
        a_b_(twist_.head<3>().cross(twist_.tail<3>())) {}

  Pose3 pose(double t) const { return Pose3::Expmap(twist_ * t); }
  Rot3 rotation(double t) const { return Rot3::Expmap(twist_.head<3>() * t); }
  Vector3 omega_b(double t) const { return twist_.head<3>(); }
  Vector3 velocity_n(double t) const {
    return rotation(t).matrix() * twist_.tail<3>();
  }
  Vector3 acceleration_n(double t) const { return rotation(t) * a_b_; }

 private:
  const Vector6 twist_;
  const Vector3 a_b_;  // constant centripetal acceleration in body = w_b * v_b
};

/// Accelerating from an arbitrary initial state
class AcceleratingScenario : public Scenario {
 public:
  /// Construct scenario with constant twist [w,v]
  AcceleratingScenario(const Rot3& nRb, const Point3& p0, const Vector3& v0,
                       const Vector3& accelerationInBody)
      : nRb_(nRb), p0_(p0.vector()), v0_(v0), a_n_(nRb_ * accelerationInBody) {}

  Pose3 pose(double t) const {
    return Pose3(nRb_, p0_ + v0_ * t + a_n_ * t * t / 2.0);
  }
  Vector3 omega_b(double t) const { return Vector3::Zero(); }
  Vector3 velocity_n(double t) const { return v0_ + a_n_ * t; }
  Vector3 acceleration_n(double t) const { return a_n_; }

 private:
  const Rot3 nRb_;
  const Vector3 p0_, v0_, a_n_;
};

}  // namespace gtsam

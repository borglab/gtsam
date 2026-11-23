/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Unicycle.h
 * @brief   Dynamics model of a unicycle on Pose2, and its dynamics factor
 * @date    November 17, 2026
 * @author  Akash Ratheesh
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam_unstable/planning/dynamics/StagDynamicsFactor.h>
#include <gtsam_unstable/planning/dynamics/StagDynamicsModel.h>

#include <cmath>

namespace gtsam {

/**
 * A unicycle on Pose2 with control [forward speed, yaw rate], stepped with
 * forward Euler: the pose moves by the body-frame increment
 * Pose2(v dt, 0, omega dt).
 */
class Unicycle : public StagDynamicsModel<Pose2, Vector2> {
 public:
  /// Construct with the default time step dt (s).
  using StagDynamicsModel::StagDynamicsModel;
  using StagDynamicsModel::predict;  // keep the default-dt overload visible

  /// One Euler step over dt (s), with the Jacobians chained through compose.
  Pose2 predict(const Pose2& state, const Vector2& control, double dt,
                OptionalJacobian<3, 3> H_state = {},
                OptionalJacobian<3, 2> H_control = {}) const override {
    const double turn = control(1) * dt;
    const Pose2 increment(control(0) * dt, 0.0, turn);
    Matrix3 H_increment;
    const Pose2 next = state.compose(increment, H_state, H_increment);
    if (H_control) {
      // d(increment)/d(control), in the increment's tangent space.
      const Matrix32 dIncrement{{dt * std::cos(turn), 0.0},
                                {-dt * std::sin(turn), 0.0},
                                {0.0, dt}};
      *H_control = H_increment * dIncrement;
    }
    return next;
  }
};

/// Dynamics factor for a Unicycle model, on x_k, u_k and x_{k+1}.
using UnicycleDynamicsFactor = StagDynamicsFactor<Pose2, Vector2>;

}  // namespace gtsam

/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    PointMass.h
 * @brief   Dynamics model of a point mass in the plane, and its dynamics
 *          factor
 * @date    November 17, 2026
 * @author  Akash Ratheesh
 * @author  Frank Dellaert
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam_unstable/planning/dynamics/StagDynamicsFactor.h>
#include <gtsam_unstable/planning/dynamics/StagDynamicsModel.h>

#include <stdexcept>

namespace gtsam {

/**
 * A point mass in the plane driven by its acceleration, stepped with forward
 * Euler. The state is [px, py, vx, vy] and the control is [ax, ay]; over dt
 * the position moves by v dt and the velocity by a dt.
 */
class PointMass : public StagDynamicsModel<Vector, Vector> {
 public:
  /// Construct with the default time step dt (s).
  using StagDynamicsModel::StagDynamicsModel;
  using StagDynamicsModel::predict;  // keep the default-dt overload visible

  /// Velocity in state coordinates for the given planar acceleration.
  static Vector dynamics(const Vector& state, const Vector& control) {
    if (state.size() != 4 || control.size() != 2)
      throw std::invalid_argument(
          "PointMass expects a 4-vector state and a 2-vector acceleration.");
    return Vector4{state(2), state(3), control(0), control(1)};
  }

  /// One forward Euler step over dt (s), with the Jacobians.
  Vector predict(const Vector& state, const Vector& control, double dt,
                 OptionalJacobian<StateDim, StateDim> H_state = {},
                 OptionalJacobian<StateDim, ControlDim> H_control = {}) const override {
    const Vector velocity = dynamics(state, control);
    if (H_state) {
      *H_state = Matrix4{{1.0, 0.0, dt, 0.0},
                         {0.0, 1.0, 0.0, dt},
                         {0.0, 0.0, 1.0, 0.0},
                         {0.0, 0.0, 0.0, 1.0}};
    }
    if (H_control) {
      *H_control = Matrix42{{0.0, 0.0}, {0.0, 0.0}, {dt, 0.0}, {0.0, dt}};
    }
    return state + dt * velocity;
  }
};

/// Dynamics factor for a PointMass model, on x_k, u_k and x_{k+1}.
using PointMassDynamicsFactor = StagDynamicsFactor<Vector, Vector>;

}  // namespace gtsam

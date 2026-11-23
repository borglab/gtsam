/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    StagDynamicsModel.h
 * @brief   Base class for the dynamics models of the STAG graph
 * @date    November 8, 2026
 * @author  Akash Ratheesh
 */

#pragma once

#include <gtsam/base/Manifold.h>
#include <gtsam/base/OptionalJacobian.h>

namespace gtsam {

/**
 * Base class for a discrete-time dynamics model x_{k+1} = f(x_k, u_k, dt).
 *
 * Derived models implement predict() with an explicit time step dt. They may
 * also override predict() without dt; if they don't, the base version
 * predicts over the model's default time step dt().
 *
 * A derived class that overrides only one predict() overload should declare
 * `using StagDynamicsModel<STATE, CONTROL>::predict;` to keep the other one
 * visible.
 *
 * The Jacobians are H_state = d(x_{k+1})/d(x_k) and
 * H_control = d(x_{k+1})/d(u_k), in the tangent spaces of the state and
 * control.
 *
 * @tparam STATE    State type, a GTSAM manifold type.
 * @tparam CONTROL  Control type, a GTSAM manifold type.
 */
template <class STATE, class CONTROL>
class StagDynamicsModel {
  static_assert(IsManifold<STATE>::value,
                "StagDynamicsModel: STATE must be a GTSAM manifold type.");
  static_assert(IsManifold<CONTROL>::value,
                "StagDynamicsModel: CONTROL must be a GTSAM manifold type.");

 public:
  /// Tangent-space dimension of the state.
  static constexpr int StateDim = traits<STATE>::dimension;

  /// Tangent-space dimension of the control.
  static constexpr int ControlDim = traits<CONTROL>::dimension;

  /// Construct with the default time step dt (s) used by predict().
  explicit StagDynamicsModel(double dt) : dt_(dt) {}

  /// Virtual destructor.
  virtual ~StagDynamicsModel() = default;

  /// The default time step (s).
  double dt() const { return dt_; }

  /**
   * Predict the next state over the time step dt (s), with optional
   * Jacobians. Derived models implement this.
   */
  virtual STATE predict(
      const STATE& state, const CONTROL& control, double dt,
      OptionalJacobian<StateDim, StateDim> H_state = {},
      OptionalJacobian<StateDim, ControlDim> H_control = {}) const = 0;

  /**
   * Predict the next state over the model's default time step dt(), with
   * optional Jacobians. Derived models may override this; by default it
   * calls predict() with dt().
   */
  virtual STATE predict(
      const STATE& state, const CONTROL& control,
      OptionalJacobian<StateDim, StateDim> H_state = {},
      OptionalJacobian<StateDim, ControlDim> H_control = {}) const {
    return predict(state, control, dt_, H_state, H_control);
  }

 private:
  double dt_;  ///< Default time step (s) used by predict().
};

}  // namespace gtsam

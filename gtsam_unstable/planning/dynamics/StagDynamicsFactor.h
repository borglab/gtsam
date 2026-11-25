/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    StagDynamicsFactor.h
 * @brief   Dynamics factor of the STAG graph's future half
 * @date    November 8, 2026
 * @author  Akash Ratheesh
 */

#pragma once

#include <gtsam/base/Manifold.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>
#include <gtsam_unstable/planning/dynamics/StagDynamicsModel.h>

#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace gtsam {

namespace internal {

/**
 * The error of a STAG dynamics factor, Local(predict(state, control, dt),
 * nextState), with its Jacobians by the chain rule: H_state = D_local *
 * F_state and H_control = D_local * F_control, where F_* are the model's
 * Jacobians and D_local is the Jacobian of Local with respect to the
 * prediction.
 */
template <class STATE, class CONTROL>
typename traits<STATE>::TangentVector stagDynamicsError(
    const StagDynamicsModel<STATE, CONTROL>& dynamics, const STATE& state,
    const CONTROL& control, double dt, const STATE& nextState,
    OptionalMatrixType H_state, OptionalMatrixType H_control,
    OptionalMatrixType H_nextState) {
  static_assert(HasLocalJacobians<STATE>::value,
                "STAG dynamics factors: STATE must provide "
                "traits<STATE>::Local with Jacobians.");
  constexpr int StateDim = traits<STATE>::dimension;
  constexpr int ControlDim = traits<CONTROL>::dimension;

  Eigen::Matrix<double, StateDim, StateDim> predicted_H_state;
  Eigen::Matrix<double, StateDim, ControlDim> predicted_H_control;
  const STATE predicted = dynamics.predict(
      state, control, dt, H_state ? &predicted_H_state : nullptr,
      H_control ? &predicted_H_control : nullptr);

  if constexpr (StateDim == Eigen::Dynamic) {
    if (traits<STATE>::GetDimension(predicted) !=
        traits<STATE>::GetDimension(nextState))
      throw std::invalid_argument(
          "The predicted and supplied next state dimensions differ.");
  }
  Eigen::Matrix<double, StateDim, StateDim> error_H_predicted;
  const typename traits<STATE>::TangentVector error = traits<STATE>::Local(
      predicted, nextState,
      (H_state || H_control) ? &error_H_predicted : nullptr, H_nextState);

  if (H_state) *H_state = error_H_predicted * predicted_H_state;
  if (H_control) *H_control = error_H_predicted * predicted_H_control;
  return error;
}

/// Returns the dynamics model, or throws std::invalid_argument if it is null.
template <class MODEL>
std::shared_ptr<const MODEL> checkedStagDynamicsModel(
    std::shared_ptr<const MODEL> dynamics) {
  if (!dynamics) {
    throw std::invalid_argument(
        "STAG dynamics factors: the dynamics model is null.");
  }
  return dynamics;
}

}  // namespace internal

/**
 * Dynamics factor for the future half of the STAG graph (planning), on the
 * state x_k, the control u_k and the next state x_{k+1}, where the control
 * is a variable. The error is Local(predict(x_k, u_k, dt), x_{k+1}), which is
 * zero when x_{k+1} equals the model's prediction.
 *
 * @tparam STATE    State type, a GTSAM manifold type.
 * @tparam CONTROL  Control type, a GTSAM manifold type.
 */
template <class STATE, class CONTROL>
class StagDynamicsFactor
    : public NoiseModelFactorT<typename traits<STATE>::TangentVector, STATE,
                               CONTROL, STATE> {
 public:
  /// The dynamics model the factor wraps.
  using Model = StagDynamicsModel<STATE, CONTROL>;

  /// Error vector: a tangent vector of the state.
  using ErrorVector = typename traits<STATE>::TangentVector;

  using Base = NoiseModelFactorT<ErrorVector, STATE, CONTROL, STATE>;
  using Base::evaluateError;

  /**
   * Construct with the model's default time step.
   * @throws std::invalid_argument if the dynamics model is null.
   */
  StagDynamicsFactor(Key stateKey, Key controlKey, Key nextStateKey,
                     std::shared_ptr<const Model> dynamics,
                     const SharedNoiseModel& noiseModel)
      : Base(noiseModel, stateKey, controlKey, nextStateKey),
        dynamics_(internal::checkedStagDynamicsModel(std::move(dynamics))),
        dt_(dynamics_->dt()) {}

  /**
   * Construct with the given time step dt (s).
   * @throws std::invalid_argument if the dynamics model is null.
   */
  StagDynamicsFactor(Key stateKey, Key controlKey, Key nextStateKey,
                     std::shared_ptr<const Model> dynamics, double dt,
                     const SharedNoiseModel& noiseModel)
      : Base(noiseModel, stateKey, controlKey, nextStateKey),
        dynamics_(internal::checkedStagDynamicsModel(std::move(dynamics))),
        dt_(dt) {}

  /// The dynamics model this factor uses.
  const std::shared_ptr<const Model>& dynamics() const { return dynamics_; }

  /// The time step (s) this factor predicts over.
  double dt() const { return dt_; }

  /// Predict the next state with this factor's model and time step.
  STATE predict(const STATE& state, const CONTROL& control) const {
    return dynamics_->predict(state, control, dt_);
  }

  /// Copy of this factor; it shares the same dynamics model.
  NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<StagDynamicsFactor>(*this);
  }

  /// Print the keys, time step and noise model.
  void print(const std::string& s = "",
             const KeyFormatter& keyFormatter =
                 DefaultKeyFormatter) const override {
    std::cout << s << "StagDynamicsFactor(" << keyFormatter(this->key1())
              << "," << keyFormatter(this->key2()) << ","
              << keyFormatter(this->key3()) << ")\n";
    std::cout << "  dt: " << dt_ << "\n";
    this->noiseModel_->print("  noise model: ");
  }

  /**
   * Equal if the keys and noise models are equal, the time steps agree to
   * within tol, and both factors use the same dynamics model object.
   */
  bool equals(const NonlinearFactor& expected,
              double tol = 1e-9) const override {
    const auto* other = dynamic_cast<const StagDynamicsFactor*>(&expected);
    return other != nullptr && Base::equals(*other, tol) &&
           dynamics_ == other->dynamics_ && std::abs(dt_ - other->dt_) <= tol;
  }

  /// Error Local(predict(state, control, dt), nextState) and Jacobians.
  ErrorVector evaluateError(const STATE& state, const CONTROL& control,
                            const STATE& nextState, OptionalMatrixType H_state,
                            OptionalMatrixType H_control,
                            OptionalMatrixType H_nextState) const override {
    return internal::stagDynamicsError(*dynamics_, state, control, dt_,
                                       nextState, H_state, H_control,
                                       H_nextState);
  }

 private:
  std::shared_ptr<const Model> dynamics_;  ///< The shared dynamics model.
  double dt_;                              ///< Time step (s).
};

/// traits
template <class STATE, class CONTROL>
struct traits<StagDynamicsFactor<STATE, CONTROL>>
    : public Testable<StagDynamicsFactor<STATE, CONTROL>> {};

}  // namespace gtsam

// Copyright (c) Georgia Tech Research Corporation. See LICENSE for details.
/**
 * @file PlanningDynamics.h
 * @author Frank Dellaert
 * @author Akash Ratheesh
 */
#pragma once

#include <gtsam/base/Manifold.h>

#include <stdexcept>
#include <utility>

namespace gtsam {

/// A discrete dynamics function returning the next state for a state and
/// control.
template <class Step>
struct DiscreteTransition {
  Step step;
  bool sendable;
};

/**
 * Mark a callable step(x,u) as an exact discrete transition.
 * Set sendable to false when the callable must run on the planning thread.
 */
template <class Step>
DiscreteTransition<Step> discreteTransition(Step step, bool sendable = true) {
  return {std::move(step), sendable};
}

namespace internal {

/// Vector fields used for parallel rollouts must be safe to evaluate
/// concurrently.
template <class Field>
bool sendableDynamics(const Field&) {
  return true;
}

/// A discrete model can request that all its predictions run on the caller.
template <class Step>
bool sendableDynamics(const DiscreteTransition<Step>& transition) {
  return transition.sendable;
}

/// Integrate a tangent vector at the current state with one Euler retraction.
template <class State, class Control, class Field>
State nextState(const Field& field, const State& state, const Control& control,
                double dt) {
  const auto velocity = field(state, control);
  if (static_cast<size_t>(velocity.size()) !=
      traits<State>::GetDimension(state))
    throw std::invalid_argument(
        "Vector field returned the wrong tangent dimension.");
  return traits<State>::Retract(state, dt * velocity);
}

/// Advance one step with the supplied state transition.
template <class State, class Control, class Step>
State nextState(const DiscreteTransition<Step>& model, const State& state,
                const Control& control, double) {
  return model.step(state, control);
}

}  // namespace internal
}  // namespace gtsam

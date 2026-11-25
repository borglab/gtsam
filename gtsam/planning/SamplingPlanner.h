// Copyright (c) Georgia Tech Research Corporation. See LICENSE for details.

/**
 * @file SamplingPlanner.h
 * @author Frank Dellaert
 * @author Akash Ratheesh
 */

#pragma once

#include <gtsam/base/Manifold.h>
#include <gtsam/base/VectorSpace.h>
#include <gtsam/planning/CandidateScoring.h>
#include <gtsam/planning/SamplingAlgorithms.h>

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

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

/**
 * Sampling-based trajectory optimization with a factor-graph objective.
 * Dynamics can be a vector field (x,u) returning a tangent vector, or a
 * discreteTransition returning the next state. Controls are Eigen column
 * vectors. Each proposal draws complete control trajectories and the
 * algorithm uses their graph costs to update the proposal's search state.
 */
template <class State, class Control>
class SamplingPlanner {
  static constexpr int ControlDim = traits<Control>::dimension;
  static_assert(std::is_same_v<Control, Eigen::Matrix<double, ControlDim, 1>>,
                "Control must be an Eigen column vector.");
  using Costs = internal::TrajectoryCosts<State, Control>;

 public:
  /**
   * Set the H+1 state keys, the H control keys and the vector-field time step.
   * Cost factors are scored on the calling thread by default. Concurrent
   * scoring requires explicit opt-in and sendable factors throughout the graph.
   */
  SamplingPlanner(KeyVector stateKeys, KeyVector controlKeys, double dt,
                  bool parallelCandidateScoring = false)
      : stateKeys_(std::move(stateKeys)),
        controlKeys_(std::move(controlKeys)),
        dt_(dt),
        parallelCandidateScoring_(parallelCandidateScoring) {
    if (controlKeys_.empty() || stateKeys_.size() != controlKeys_.size() + 1 ||
        !std::isfinite(dt_) || dt_ <= 0)
      throw std::invalid_argument(
          "Expected H+1 states, H nonempty controls and a positive finite dt.");
    KeySet keys(stateKeys_.begin(), stateKeys_.end());
    keys.insert(controlKeys_.begin(), controlKeys_.end());
    if (keys.size() != stateKeys_.size() + controlKeys_.size())
      throw std::invalid_argument("State and control keys must be distinct.");
  }

  /**
   * Roll out one control sequence from the initial state. Insert or replace
   * future states and controls in a copy of initial, preserving auxiliary keys.
   */
  template <class Dynamics>
  Values rollout(const Values& initial,
                 const Eigen::Ref<const Matrix>& controls,
                 const Dynamics& dynamics) const {
    if (!validControlDimension(controls.rows()) ||
        controls.cols() != static_cast<Eigen::Index>(controlKeys_.size()))
      throw std::invalid_argument(
          "Expected one control column per planning step.");
    std::vector<State> states(stateKeys_.size(),
                              initial.at<State>(stateKeys_.front()));
    rolloutStates(states, controls, dynamics);
    Values result = initial;
    writeRollout(result, states, controls);
    return result;
  }

  /// Extract the nominal controls from initial in horizon order.
  Matrix nominalControls(const Values& initial) const {
    const Control first = initial.at<Control>(controlKeys_.front());
    if (!validControlDimension(first.size()))
      throw std::invalid_argument("Expected a nonempty control vector.");
    Matrix nominal(first.size(), controlKeys_.size());
    nominal.col(0) = first;
    for (size_t k = 1; k < controlKeys_.size(); ++k) {
      const Control control = initial.at<Control>(controlKeys_[k]);
      if (control.size() != first.size())
        throw std::invalid_argument("Controls must have a common dimension.");
      nominal.col(k) = control;
    }
    return nominal;
  }

  /// Roll out each candidate and sum its factor errors in graph order.
  template <class Dynamics>
  Vector rolloutAndScore(const NonlinearFactorGraph& cost,
                         const Values& initial, const Matrix& controls,
                         const Dynamics& dynamics) const {
    return score(
        Costs(cost, stateKeys_, controlKeys_), initial, controls, dynamics,
        parallelCandidateScoring_ && internal::allFactorsSendable(cost));
  }

  /// Draw and evaluate one batch and let the algorithm update the caller's
  /// search.
  template <class Dynamics, class Proposal, class Algorithm, class Search>
  void iterate(const NonlinearFactorGraph& cost, const Values& initial,
               const Dynamics& dynamics, const Proposal& proposal,
               const Algorithm& algorithm, Search& search) const {
    iterate(Costs(cost, stateKeys_, controlKeys_), initial, dynamics, proposal,
            algorithm, search,
            parallelCandidateScoring_ && internal::allFactorsSendable(cost));
  }

  /**
   * Initialize the proposal from the supplied controls, run the requested
   * iterations, then roll out the final mean controls. The initial Values
   * must contain the first state and every nominal control. Use a separate
   * proposal per concurrent solve because the sampler owns its random stream.
   */
  template <class Dynamics, class Proposal, class Algorithm>
  Values optimize(const NonlinearFactorGraph& cost, const Values& initial,
                  const Dynamics& dynamics, const Proposal& proposal,
                  const Algorithm& algorithm, size_t iterations) const {
    auto search = proposal.initialize(nominalControls(initial));
    const Costs costs(cost, stateKeys_, controlKeys_);
    const bool parallel =
        parallelCandidateScoring_ && internal::allFactorsSendable(cost);
    for (size_t i = 0; i < iterations; ++i)
      iterate(costs, initial, dynamics, proposal, algorithm, search, parallel);
    return rollout(initial, search.mean, dynamics);
  }

 private:
  /// Whether a matrix row count describes a control of this planner.
  static bool validControlDimension(Eigen::Index rows) {
    return rows > 0 && (ControlDim == Eigen::Dynamic || rows == ControlDim);
  }

  /// Predict all states from states[0] using the controls of one candidate.
  template <class Dynamics>
  void rolloutStates(std::vector<State>& states,
                     const Eigen::Ref<const Matrix>& controls,
                     const Dynamics& dynamics) const {
    for (size_t k = 0; k < controlKeys_.size(); ++k) {
      const Control control = controls.col(k);
      states[k + 1] = internal::nextState(dynamics, states[k], control, dt_);
    }
  }

  /// Write typed controls and states into the trajectory's graph variables.
  void writeRollout(Values& values, const std::vector<State>& states,
                    const Eigen::Ref<const Matrix>& controls) const {
    for (size_t k = 0; k < controlKeys_.size(); ++k) {
      values.insert_or_assign(controlKeys_[k], Control(controls.col(k)));
      values.insert_or_assign(stateKeys_[k + 1], states[k + 1]);
    }
  }

  /// Score a batch with worker-private trajectory scratch and ordered factor
  /// errors.
  template <class Dynamics>
  Vector score(const Costs& costs, const Values& initial,
               const Matrix& controls, const Dynamics& dynamics,
               bool parallelScoring) const {
    const Eigen::Index horizon = controlKeys_.size();
    if (!validControlDimension(controls.rows()) || controls.cols() % horizon)
      throw std::invalid_argument(
          "Expected a batch of complete control sequences.");
    struct Trajectory {
      std::vector<State> states;
      Values values;
    };
    const Trajectory start{
        std::vector<State>(stateKeys_.size(),
                           initial.at<State>(stateKeys_.front())),
        costs.needsValues() ? initial : Values{}};
    return internal::scoreCandidates(
        controls.cols() / horizon, start, internal::sendableDynamics(dynamics),
        parallelScoring,
        [&](size_t i, Trajectory& trajectory) {
          const auto candidate = controls.middleCols(i * horizon, horizon);
          rolloutStates(trajectory.states, candidate, dynamics);
          if (costs.needsValues())
            writeRollout(trajectory.values, trajectory.states, candidate);
        },
        [&](size_t i, const Trajectory& trajectory) {
          return costs.error(trajectory.states,
                             controls.middleCols(i * horizon, horizon),
                             trajectory.values);
        });
  }

  /// Share the prepared graph costs across the optimization's iterations.
  template <class Dynamics, class Proposal, class Algorithm, class Search>
  void iterate(const Costs& costs, const Values& initial,
               const Dynamics& dynamics, const Proposal& proposal,
               const Algorithm& algorithm, Search& search,
               bool parallelScoring) const {
    if (!validControlDimension(search.mean.rows()) ||
        search.mean.cols() != static_cast<Eigen::Index>(controlKeys_.size()))
      throw std::invalid_argument(
          "Search state must match the planner horizon.");
    auto batch =
        proposal.sample(search, initial, dt_, algorithm.includeIncumbent);
    if (batch.controls.rows() != search.mean.rows() || !batch.controls.cols())
      throw std::invalid_argument(
          "Incompatible proposal batch for this nominal plan.");
    const Vector errors =
        score(costs, initial, batch.controls, dynamics, parallelScoring);
    algorithm.apply(proposal, search, batch, errors);
  }

  KeyVector stateKeys_;
  KeyVector controlKeys_;
  double dt_;
  bool parallelCandidateScoring_;
};

}  // namespace gtsam

/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GenericMPPIController.h
 * @brief   Model predictive path integral control on a factor graph
 * @date    November 8, 2026
 * @author  Akash Ratheesh
 * @author  Frank Dellaert
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/VectorSpace.h>
#include <gtsam/planning/MPCController.h>
#include <gtsam/planning/SamplingPlanner.h>

#include <algorithm>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <type_traits>
#include <typeinfo>
#include <unordered_set>
#include <utility>
#include <vector>

namespace gtsam {

/**
 * Parameters of the generic MPPI controller, shared by every sampler. A
 * controller that fixes the sampler adds the sampler's own parameters, as
 * VanillaMPPIParams does.
 */
struct GenericMPPIParams : public MPCControllerParams {
  size_t iterations = 1;            ///< MPPI iterations per control step.
  double inverseTemperature = 1.0;  ///< Inverse temperature, 1 / lambda.
  /// Enable concurrent scoring on the same factor instances only when all
  /// cost factors support it. Non-sendable factors always run on the caller.
  bool parallelCandidateScoring = false;
};

/**
 * MPPI controller for a graph of future states and controls.
 *
 * In every iteration, optimize() samples around the nominal controls, rolls
 * the samples out, scores them with the graph error, weights them, and
 * replaces the nominal controls with the weighted mean of the samples. It
 * returns the final nominal controls and the states rolled out with them.
 *
 * Each rollout calls predict(state, control) on the dynamics factor for
 * each step. The factor owns any model or time step it needs. Its error
 * must be zero at the predicted next state. A sendable dynamics factor's
 * predict method may run concurrently on different candidates with TBB.
 * Graph cost factors run on the calling thread by default.
 *
 * The control samples come from the SAMPLER passed in at construction. It
 * must provide `const Matrix& sampleControls(const Matrix& nominalControls)
 * const` (returning a Matrix by value works too): K control sequences sampled
 * around the nominal ones (ControlDim x H), in a ControlDim x (K*H) matrix
 * whose column i*H + k holds sample i at step k. K is taken from it.
 * VanillaMPPIController fixes the sampler to a Gaussian noise sampler.
 *
 * @tparam DYNAMICS_FACTOR A ternary nonlinear factor on (state, control,
 *     next state), with the X1, X2 and X3 type aliases of NoiseModelFactorT
 *     and a const predict(state, control) method returning the next state.
 *     The control must be a fixed-size Eigen column vector.
 * @tparam SAMPLER  Control sampler, as described above.
 */
template <class DYNAMICS_FACTOR, class SAMPLER>
class GenericMPPIController : public MPCController {
  using State = typename DYNAMICS_FACTOR::X1;
  using Control = typename DYNAMICS_FACTOR::X2;
  static_assert(std::is_same_v<State, typename DYNAMICS_FACTOR::X3>,
                "The dynamics factor must predict the same state type.");
  static_assert(
      traits<Control>::dimension != Eigen::Dynamic &&
          std::is_same_v<Control,
                         Eigen::Matrix<double, traits<Control>::dimension, 1>>,
      "GenericMPPIController: Control must be a fixed-size Eigen "
      "vector, e.g. Vector2.");

 public:
  /// The underlying planner using this factor's state and control types.
  using Planner = SamplingPlanner<State, Control>;

  /// Control dimension M.
  static constexpr int ControlDim = traits<Control>::dimension;

  /// The future graph's dynamics factor, on (x_k, u_k, x_{k+1}).
  using DynamicsFactor = DYNAMICS_FACTOR;

  /// The dynamics factor of each control key, in the order of the keys.
  using Chain = std::vector<std::shared_ptr<const DynamicsFactor>>;

  /// Construct with the given parameters and control sampler.
  GenericMPPIController(const GenericMPPIParams& params, SAMPLER sampler)
      : params_(params), proposal_(std::move(sampler)) {}

  /// The controller's parameters.
  const GenericMPPIParams& params() const { return params_; }

  /// The control sampler.
  const SAMPLER& sampler() const { return proposal_.sampler(); }

  /** Roll the nominal controls from the measured state and create typed
   * initial values for every key in the dynamics chain. */
  Values initialValues(const NonlinearFactorGraph& graph,
                       const KeyVector& controlKeys, const State& start,
                       const Matrix& controls) const {
    Chain chain;
    updateDynamicsFactors(graph, controlKeys, chain);
    const Planner planner = plannerFor(chain);
    Values initial;
    initial.insert(chain.front()->keys().at(0), start);
    return planner.rollout(initial, controls, transition(chain));
  }

  /// Sample controls around the nominal ones, with the sampler.
  decltype(auto) sampleControls(const Matrix& nominalControls) const {
    return sampler().sampleControls(nominalControls);
  }

  /**
   * Get the nominal control sequence from initial: ubar gets one column per
   * control key, in the order of controlKeys. ubar is resized to
   * ControlDim x controlKeys.size().
   * @throws ValuesKeyDoesNotExist if initial has no value for a control key.
   */
  void getControl(const Values& initial, const KeyVector& controlKeys,
                  Matrix& ubar) const {
    ubar.resize(ControlDim, controlKeys.size());
    for (size_t k = 0; k < controlKeys.size(); ++k) {
      ubar.col(k) = initial.at<Control>(controlKeys[k]);
    }
  }

  /**
   * Update chain to the future graph's dynamics factor of each control key,
   * in the order of controlKeys: chain[k] is the dynamics factor whose
   * control key is controlKeys[k]. chain is reset to controlKeys.size()
   * entries first.
   * @throws std::invalid_argument unless exactly one dynamics factor matches
   * each requested control key.
   */
  void updateDynamicsFactors(const NonlinearFactorGraph& futureGraph,
                             const KeyVector& controlKeys, Chain& chain) const {
    const size_t H = controlKeys.size();
    chain.assign(H, nullptr);
    for (const auto& factor : futureGraph) {
      const auto dynamics =
          std::dynamic_pointer_cast<const DynamicsFactor>(factor);
      if (!dynamics) continue;
      for (size_t k = 0; k < H; ++k) {
        if (dynamics->keys().at(1) != controlKeys[k]) continue;
        if (chain[k])
          throw std::invalid_argument(
              "Multiple dynamics factors match a control key.");
        chain[k] = dynamics;
      }
    }
    for (size_t k = 0; k < H; ++k) {
      if (!chain[k]) {
        throw std::invalid_argument(
            "GenericMPPIController::updateDynamicsFactors: the future graph "
            "has no matching dynamics factor for a control key.");
      }
    }
  }

  /**
   * Check that values holds exactly a State at the state keys and a Control
   * at the control key of every factor in the chain, as rollout() needs.
   * @throws ValuesKeyDoesNotExist if a key is missing, or ValuesIncorrectType
   * if a value has another type, as Values::update does.
   */
  void checkRolloutTypes(const Chain& chain, const Values& values) const {
    const auto check = [&values](Key key, const std::type_info& expected) {
      const Value& value = values.at(key);
      if (typeid(value) != expected) {
        throw ValuesIncorrectType(key, typeid(value), expected);
      }
    };
    for (const auto& f : chain) {
      if (!f) throw std::invalid_argument("Missing dynamics factor in chain.");
      check(f->keys().at(0), typeid(GenericValue<State>));
      check(f->keys().at(1), typeid(GenericValue<Control>));
      check(f->keys().at(2), typeid(GenericValue<State>));
    }
  }

  /**
   * Roll one control sequence out through the chain of dynamics factors:
   * for each step k, write u_k and x_{k+1} = factor.predict(x_k, u_k) into
   * values, where x_k is read from values. controls has one column per step.
   * The values must already hold the states and controls in the chain.
   */
  void rollout(const Chain& chain, const Eigen::Ref<const Matrix>& controls,
               Values& values) const {
    const Planner planner = plannerFor(chain);
    checkRolloutTypes(chain, values);
    values = planner.rollout(values, controls, transition(chain));
  }

  /**
   * Roll every sampled control sequence out through the chain of dynamics
   * factors, starting from initial, and score it with the errors of the
   * future graph's factors, summed in graph order for each sample.
   * The chain's own dynamics factors are skipped: a rollout satisfies them, so
   * their error is zero. Every other factor is scored, including dynamics
   * factors that are not in the chain. For this, the chain must be in step
   * order: the state of chain[k] is the next state of chain[k-1].
   *
   * Each sample is rolled out into a vector of states. A factor of exactly
   * the type PriorFactor<State> on a state key of the chain, or
   * PriorFactor<Control> on a control key, with a noise model of its
   * dimension, is scored directly on the rolled-out state or control, with
   * the same arithmetic as NoiseModelFactor::error. Every other factor is
   * scored through a copy of initial into which the rollout is written. For
   * this, the chain's keys must be distinct.
   *
   * sampledControls uses the sampler's layout: column i*H + k holds sample i
   * at step k, with H = chain.size(). With TBB the samples are rolled out in
   * parallel if the dynamics factors are sendable. Scratch is private to each
   * concurrent rollout. By default, the caller scores each completed sample
   * in order. Concurrent candidate scoring requires an explicit opt-in and
   * sendable factors throughout the graph.
   * @return the cost of each sample.
   * @throws std::invalid_argument if the chain is empty, not in step order or
   * its keys are not distinct, or if sampledControls does not have ControlDim
   * rows and a multiple of H columns; ValuesKeyDoesNotExist or
   * ValuesIncorrectType if initial does not hold the chain's keys with their
   * types (see checkRolloutTypes()); and whatever the scored factors throw.
   */
  Vector rolloutAndScore(const NonlinearFactorGraph& futureGraph,
                         const Chain& chain, const Matrix& sampledControls,
                         const Values& initial) const {
    const Planner planner = plannerFor(chain);
    checkRolloutTypes(chain, initial);
    return planner.rolloutAndScore(costGraph(futureGraph, chain), initial,
                                   sampledControls, transition(chain));
  }

  /**
   * Run MPPI on the future graph. The nominal control sequence ubar is
   * collected from initial at controlKeys (see getControl()), and the
   * dynamics factor of each control key from the future graph (see
   * updateDynamicsFactors()). Each iteration samples K sequences around ubar
   * with the sampler, rolls every sample out through the graph's dynamics
   * factors and scores it with the graph error S_i (see rolloutAndScore()),
   * weights the samples, w_i = exp(-(S_i - min S) * inverseTemperature)
   * normalized to sum to 1 (weights below the smallest normal double are set
   * to 0, which does not change ubar), and sets ubar to the weighted mean of
   * the samples, sum_i w_i v_i. It returns initial with the controls set to
   * the final ubar and the states rolled out with them through the dynamics
   * factors.
   * @throws std::invalid_argument if the control keys do not identify one
   * nonempty dynamics chain with distinct trajectory keys. Missing keys or
   * incorrect types in initial throw before the sampler draws a candidate.
   */
  Values optimize(const NonlinearFactorGraph& futureGraph,
                  const Values& initial,
                  const KeyVector& controlKeys) override {
    // Report a missing or mistyped nominal control before inspecting the graph.
    for (Key key : controlKeys) initial.at<Control>(key);
    Chain chain;
    updateDynamicsFactors(futureGraph, controlKeys, chain);
    const Planner planner = plannerFor(chain);
    checkRolloutTypes(chain, initial);
    return planner.optimize(
        costGraph(futureGraph, chain), initial, transition(chain), proposal_,
        MPPI(params_.inverseTemperature), params_.iterations);
  }

 protected:
  /// The parameters, as the base parameter type.
  const MPCControllerParams& _params() const override { return params_; }

 private:
  using Proposal =
      std::conditional_t<std::is_same_v<SAMPLER, GaussianNoiseSampler>,
                         GaussianProposal, SampledControlProposal<SAMPLER>>;

  /// Derive the planner's ordered keys from one continuous dynamics chain.
  Planner plannerFor(const Chain& chain) const {
    if (chain.empty() || !chain.front())
      throw std::invalid_argument(
          "Expected a nonempty chain of dynamics factors.");
    KeyVector states{chain.front()->keys().at(0)}, controls;
    for (const auto& factor : chain) {
      if (!factor || factor->keys().at(0) != states.back())
        throw std::invalid_argument(
            "Expected a continuous chain of state keys.");
      controls.push_back(factor->keys().at(1));
      states.push_back(factor->keys().at(2));
    }
    // Each discrete factor owns the time step used by its prediction.
    return Planner(std::move(states), std::move(controls), 1.,
                   params_.parallelCandidateScoring);
  }

  /// Select the original prediction for each step and respect model thread
  /// affinity.
  static auto transition(const Chain& chain) {
    const bool sendable =
        std::all_of(chain.begin(), chain.end(),
                    [](const auto& factor) { return factor->sendable(); });
    return discreteTransition(
        [&chain](size_t k, const State& state, const Control& control) {
          return chain[k]->predict(state, control);
        },
        sendable);
  }

  /// Preserve graph order and skip exactly the dynamics satisfied by the
  /// rollout.
  static NonlinearFactorGraph costGraph(const NonlinearFactorGraph& graph,
                                        const Chain& chain) {
    std::unordered_set<const NonlinearFactor*> dynamics;
    for (const auto& factor : chain) dynamics.insert(factor.get());
    NonlinearFactorGraph costs;
    costs.reserve(graph.size());
    for (const auto& factor : graph) {
      if (factor && !dynamics.count(factor.get())) costs.push_back(factor);
    }
    return costs;
  }

  const GenericMPPIParams params_;
  Proposal proposal_;
};

}  // namespace gtsam

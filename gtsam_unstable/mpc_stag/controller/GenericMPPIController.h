/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GenericMPPIController.h
 * @brief   MPPI controller on the future half of the STAG graph, generic over
 *          the control noise sampler, and its parameters
 * @date    November 8, 2026
 * @author  Akash Ratheesh
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/VectorSpace.h>
#include <gtsam/config.h>  // for GTSAM_USE_TBB
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam_unstable/mpc_stag/controller/StagMPCController.h>
#include <gtsam_unstable/mpc_stag/dynamics/StagDynamicsFactor.h>

#ifdef GTSAM_USE_TBB
#include <tbb/enumerable_thread_specific.h>
#include <tbb/parallel_for.h>
#include <tbb/task_arena.h>
#endif

#include <algorithm>
#include <cstddef>
#include <limits>
#include <memory>
#include <stdexcept>
#include <type_traits>
#include <typeinfo>
#include <unordered_map>
#include <utility>
#include <vector>

namespace gtsam {

/**
 * Parameters of the generic MPPI controller, shared by every sampler. A
 * controller that fixes the sampler adds the sampler's own parameters, as
 * VanillaMPPIParams does.
 */
struct GenericMPPIParams : public StagMPCControllerParams {
  size_t iterations = 1;            ///< MPPI iterations per control step.
  double inverseTemperature = 1.0;  ///< Inverse temperature, 1 / lambda.
};

/**
 * MPPI controller on the future half of the STAG graph, generic over the
 * control noise sampler.
 *
 * In every iteration, optimize() samples around the nominal controls, rolls
 * the samples out, scores them with the graph error, weights them, and
 * replaces the nominal controls with the weighted mean of the samples. It
 * returns the final nominal controls and the states rolled out with them.
 *
 * The rollout uses the future graph's own dynamics: the
 * StagDynamicsFactor<STATE, CONTROL> of each control key gives the state
 * keys, the dynamics model and the time step.
 *
 * The control samples come from the SAMPLER passed in at construction. It
 * must provide `const Matrix& sampleControls(const Matrix& nominalControls)
 * const` (returning a Matrix by value works too): K control sequences sampled
 * around the nominal ones (ControlDim x H), in a ControlDim x (K*H) matrix
 * whose column i*H + k holds sample i at step k. K is taken from it.
 * VanillaMPPIController fixes the sampler to a Gaussian noise sampler.
 *
 * @tparam STATE    State type of the future graph's dynamics factors.
 * @tparam CONTROL  Control type, a fixed-size Eigen vector such as Vector2
 *                  (Vector1 for a scalar control), since samples are
 *                  v = u + eps.
 * @tparam SAMPLER  Control sampler, as described above.
 */
template <class STATE, class CONTROL, class SAMPLER>
class GenericMPPIController : public StagMPCController {
  static_assert(traits<CONTROL>::dimension != Eigen::Dynamic &&
                    std::is_same_v<CONTROL,
                                   Eigen::Matrix<double,
                                                 traits<CONTROL>::dimension,
                                                 1>>,
                "GenericMPPIController: CONTROL must be a fixed-size Eigen "
                "vector, e.g. Vector2.");

 public:
  /// Control dimension M.
  static constexpr int ControlDim = traits<CONTROL>::dimension;

  /// The future graph's dynamics factor, on (x_k, u_k, x_{k+1}).
  using DynamicsFactor = StagDynamicsFactor<STATE, CONTROL>;

  /// The dynamics factor of each control key, in the order of the keys.
  using Chain = std::vector<std::shared_ptr<const DynamicsFactor>>;

  /// Construct with the given parameters and control sampler.
  GenericMPPIController(const GenericMPPIParams& params, SAMPLER sampler)
      : params_(params), sampler_(std::move(sampler)) {}

  /// The controller's parameters.
  const GenericMPPIParams& params() const { return params_; }

  /// The control sampler.
  const SAMPLER& sampler() const { return sampler_; }

  /// Sample controls around the nominal ones, with the sampler.
  decltype(auto) sampleControls(const Matrix& nominalControls) const {
    return sampler_.sampleControls(nominalControls);
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
      ubar.col(k) = initial.at<CONTROL>(controlKeys[k]);
    }
  }

  /**
   * Update chain to the future graph's dynamics factor of each control key,
   * in the order of controlKeys: chain[k] is the StagDynamicsFactor whose
   * control key is controlKeys[k]. chain is reset to controlKeys.size()
   * entries first.
   * @throws std::invalid_argument if the future graph has no
   * StagDynamicsFactor for a control key.
   */
  void updateDynamicsFactors(const NonlinearFactorGraph& futureGraph,
                             const KeyVector& controlKeys,
                             Chain& chain) const {
    const size_t H = controlKeys.size();
    chain.assign(H, nullptr);
    for (const auto& factor : futureGraph) {
      const auto dynamics = std::dynamic_pointer_cast<const DynamicsFactor>(
          factor);
      if (!dynamics) continue;
      for (size_t k = 0; k < H; ++k) {
        if (dynamics->keys()[1] == controlKeys[k]) chain[k] = dynamics;
      }
    }
    for (size_t k = 0; k < H; ++k) {
      if (!chain[k]) {
        throw std::invalid_argument(
            "GenericMPPIController::updateDynamicsFactors: the future graph "
            "has no StagDynamicsFactor for a control key.");
      }
    }
  }

  /**
   * Check that values holds exactly a STATE at the state keys and a CONTROL
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
      check(f->keys()[0], typeid(GenericValue<STATE>));
      check(f->keys()[1], typeid(GenericValue<CONTROL>));
      check(f->keys()[2], typeid(GenericValue<STATE>));
    }
  }

  /**
   * Roll one control sequence out through the chain of dynamics factors:
   * for each step k, write u_k and x_{k+1} = f(x_k, u_k, dt) into values,
   * where x_k is read from values. controls has one column per step. The
   * values are overwritten in place, so values must already hold exactly a
   * STATE at each factor's state keys and a CONTROL at its control key; see
   * checkRolloutTypes(). A missing key throws ValuesKeyDoesNotExist, but a
   * value of another type is not checked here.
   */
  void rollout(const Chain& chain, const Eigen::Ref<const Matrix>& controls,
               Values& values) const {
    for (size_t k = 0; k < chain.size(); ++k) {
      const auto& f = chain[k];
      const CONTROL u = controls.col(k);
      // Values stores its own non-const copies, so writing through atRef's
      // const reference is well defined.
      const_cast<CONTROL&>(values.atRef<CONTROL>(f->keys()[1])) = u;
      const_cast<STATE&>(values.atRef<STATE>(f->keys()[2])) =
          f->dynamics()->predict(values.atRef<STATE>(f->keys()[0]), u,
                                 f->dt());
    }
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
   * the type PriorFactor<STATE> on a state key of the chain, or
   * PriorFactor<CONTROL> on a control key, with a noise model of its
   * dimension, is scored directly on the rolled-out state or control, with
   * the same arithmetic as NoiseModelFactor::error. Every other factor is
   * scored through a copy of initial into which the rollout is written. For
   * this, the chain's keys must be distinct.
   *
   * sampledControls uses the sampler's layout: column i*H + k holds sample i
   * at step k, with H = chain.size(). With TBB the samples are rolled out in
   * parallel, each thread into its own states and copy of initial; the costs
   * are the same for any number of threads.
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
    const size_t H = chain.size();
    if (H == 0 || sampledControls.rows() != ControlDim ||
        sampledControls.cols() % H != 0) {
      throw std::invalid_argument(
          "GenericMPPIController::rolloutAndScore: the chain is empty, or "
          "sampledControls does not have ControlDim rows and a multiple of "
          "H columns.");
    }
    for (size_t k = 1; k < H; ++k) {
      if (chain[k]->keys()[0] != chain[k - 1]->keys()[2]) {
        throw std::invalid_argument(
            "GenericMPPIController::rolloutAndScore: the chain is not in step "
            "order: the state of each factor must be the next state of the "
            "factor before it.");
      }
    }
    checkRolloutTypes(chain, initial);
    const std::vector<CostTerm> terms = costTerms(futureGraph, chain);
    const bool needValues =
        std::any_of(terms.begin(), terms.end(),
                    [](const CostTerm& term) { return term.needsValues(); });
    const size_t K = sampledControls.cols() / H;
    Vector costs(K);

    // Roll sample i out into states, and into values if a term needs them,
    // and sum its terms' errors in graph order.
    const auto rolloutAndScoreSample = [&](size_t i, std::vector<STATE>& states,
                                           Values* values) {
      const Eigen::Ref<const Matrix> controls =
          sampledControls.middleCols(i * H, H);
      rolloutStates(chain, controls, states);
      if (values) writeRollout(chain, controls, states, *values);
      double cost = 0.0;
      for (const CostTerm& term : terms) {
        cost += term.error(states, controls, values);
      }
      costs(i) = cost;
    };

    // Every rollout starts from x_0 of initial.
    const std::vector<STATE> startStates(
        H + 1, initial.at<STATE>(chain.front()->keys()[0]));
#ifdef GTSAM_USE_TBB
    // Each thread rolls its samples out into its own states, and its own
    // copy of initial, made on first use.
    tbb::enumerable_thread_specific<std::vector<STATE>> threadStates(
        startStates);
    tbb::enumerable_thread_specific<Values> threadValues(
        [&initial] { return initial; });
    tbb::parallel_for(size_t(0), K, [&](size_t i) {
      std::vector<STATE>& states = threadStates.local();
      Values* values = needValues ? &threadValues.local() : nullptr;
      // Isolated, so that a thread waiting on nested TBB work inside the
      // dynamics or the factors never takes another sample, which would
      // reuse these states and values.
      tbb::this_task_arena::isolate(
          [&] { rolloutAndScoreSample(i, states, values); });
    });
#else
    std::vector<STATE> states = startStates;
    Values values = needValues ? initial : Values();
    for (size_t i = 0; i < K; ++i) {
      rolloutAndScoreSample(i, states, needValues ? &values : nullptr);
    }
#endif
    return costs;
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
   * @throws std::invalid_argument if controlKeys is empty, or if the future
   * graph has no StagDynamicsFactor for a control key; ValuesKeyDoesNotExist
   * or ValuesIncorrectType if initial does not hold the dynamics factors'
   * keys with their types (see checkRolloutTypes()), before sampling; and
   * whatever the sampler throws.
   */
  Values optimize(const NonlinearFactorGraph& futureGraph,
                  const Values& initial,
                  const KeyVector& controlKeys) override {
    if (controlKeys.empty()) {
      throw std::invalid_argument(
          "GenericMPPIController::optimize: controlKeys is empty.");
    }

    const size_t M = ControlDim;
    const size_t H = controlKeys.size();

    // The nominal control sequence, one column per control key.
    Matrix ubar(M, H);
    getControl(initial, controlKeys, ubar);

    // The dynamics factor of each control key, from the future graph.
    Chain chain(H);
    updateDynamicsFactors(futureGraph, controlKeys, chain);
    checkRolloutTypes(chain, initial);

    for (size_t iteration = 0; iteration < params_.iterations; ++iteration) {
      // Sample around ubar: column i*H + k holds sample i at step k.
      const Matrix& sampled = sampler_.sampleControls(ubar);
      const Vector costs =
          rolloutAndScore(futureGraph, chain, sampled, initial);
      const size_t K = costs.size();

      // Weight the samples, w_i = exp(-(S_i - S_min) / lambda), normalized.
      // The best sample gets exp(0) = 1 before normalizing, so the sum is at
      // least 1 and never underflows to zero.
      const double minCost = costs.minCoeff();
      Vector weights =
          (-(costs.array() - minCost) * params_.inverseTemperature).exp();
      weights /= weights.sum();
      // Set subnormal weights to 0: they cannot change ubar, and arithmetic
      // with them is slow.
      weights = (weights.array() < std::numeric_limits<double>::min())
                    .select(0.0, weights.array())
                    .matrix();

      // ubar = sum_i w_i * v_i
      Eigen::Map<Vector>(ubar.data(), M * H) =
          Eigen::Map<const Matrix>(sampled.data(), M * H, K) * weights;
    }

    Values result = initial;
    rollout(chain, ubar, result);
    return result;
  }

 protected:
  /// The parameters, as the base parameter type.
  const StagMPCControllerParams& _params() const override { return params_; }

 private:
  /**
   * A factor that rolloutAndScore() scores. A prior on a rolled-out state or
   * control is scored on it directly; any other factor through Values.
   */
  struct CostTerm {
    const NonlinearFactor* factor;  ///< The factor.
    /// The factor, if it is a prior on the state x_step.
    const PriorFactor<STATE>* statePrior = nullptr;
    /// The factor, if it is a prior on the control u_step.
    const PriorFactor<CONTROL>* controlPrior = nullptr;
    size_t step = 0;  ///< The step of the prior's state or control.

    /// Whether the factor is scored through Values.
    bool needsValues() const { return !statePrior && !controlPrior; }

    /**
     * The factor's error for one rollout: states x_0 to x_H, the controls,
     * one column per step, and the rollout written into values, which may be
     * null if needsValues() is false.
     */
    double error(const std::vector<STATE>& states,
                 const Eigen::Ref<const Matrix>& controls,
                 const Values* values) const {
      if (statePrior) return statePrior->error(states[step]);
      if (controlPrior) return controlPrior->error(CONTROL(controls.col(step)));
      return factor->error(*values);
    }
  };

  /**
   * The factor as a PriorFactor<T> if it is exactly of that type, not a
   * subclass, and its noise model has the prior's dimension; else null.
   */
  template <class T>
  static const PriorFactor<T>* exactPrior(const NonlinearFactor& factor) {
    if (typeid(factor) != typeid(PriorFactor<T>)) return nullptr;
    const auto& prior = static_cast<const PriorFactor<T>&>(factor);
    const SharedNoiseModel& noise = prior.noiseModel();
    const auto dimension =
        static_cast<size_t>(traits<T>::GetDimension(prior.prior()));
    if (!noise || noise->dim() != dimension) return nullptr;
    return &prior;
  }

  /**
   * The factors of the future graph that rolloutAndScore() scores, in graph
   * order: every factor except the chain's, by identity. A unary factor on a
   * state key of the chain is scored on the rolled-out state if exactPrior()
   * gives a PriorFactor<STATE>, and one on a control key on the control if it
   * gives a PriorFactor<CONTROL>.
   * @throws std::invalid_argument if the chain's keys are not distinct.
   */
  std::vector<CostTerm> costTerms(const NonlinearFactorGraph& futureGraph,
                                  const Chain& chain) const {
    // Where each key of the chain is in a rollout: x_step or u_step.
    struct Slot {
      bool isControl;
      size_t step;
    };
    std::unordered_map<Key, Slot> slots;
    const auto addSlot = [&slots](Key key, bool isControl, size_t step) {
      if (!slots.emplace(key, Slot{isControl, step}).second) {
        throw std::invalid_argument(
            "GenericMPPIController::rolloutAndScore: the chain's keys are not "
            "distinct.");
      }
    };
    addSlot(chain.front()->keys()[0], false, 0);
    for (size_t k = 0; k < chain.size(); ++k) {
      addSlot(chain[k]->keys()[1], true, k);
      addSlot(chain[k]->keys()[2], false, k + 1);
    }

    std::vector<CostTerm> terms;
    for (const auto& factor : futureGraph) {
      const bool inChain = std::any_of(
          chain.begin(), chain.end(),
          [&factor](const auto& f) { return f.get() == factor.get(); });
      if (!factor || inChain) continue;
      CostTerm term{factor.get()};
      const auto slot =
          factor->size() == 1 ? slots.find(factor->front()) : slots.end();
      if (slot != slots.end()) {
        term.step = slot->second.step;
        if (slot->second.isControl) {
          term.controlPrior = exactPrior<CONTROL>(*factor);
        } else {
          term.statePrior = exactPrior<STATE>(*factor);
        }
      }
      terms.push_back(term);
    }
    return terms;
  }

  /**
   * Roll one control sequence out into states: for each step k,
   * states[k+1] = f(states[k], u_k, dt), starting from states[0].
   */
  void rolloutStates(const Chain& chain,
                     const Eigen::Ref<const Matrix>& controls,
                     std::vector<STATE>& states) const {
    for (size_t k = 0; k < chain.size(); ++k) {
      const auto& f = chain[k];
      const CONTROL u = controls.col(k);
      states[k + 1] = f->dynamics()->predict(states[k], u, f->dt());
    }
  }

  /**
   * Write a rollout into values in place: the controls u_k and the states
   * x_{k+1}, at each factor's keys, which must already hold exactly a CONTROL
   * and a STATE (see rollout()).
   */
  void writeRollout(const Chain& chain,
                    const Eigen::Ref<const Matrix>& controls,
                    const std::vector<STATE>& states, Values& values) const {
    for (size_t k = 0; k < chain.size(); ++k) {
      const auto& f = chain[k];
      const_cast<CONTROL&>(values.atRef<CONTROL>(f->keys()[1])) =
          controls.col(k);
      const_cast<STATE&>(values.atRef<STATE>(f->keys()[2])) = states[k + 1];
    }
  }

  const GenericMPPIParams params_;  ///< Generic MPPI parameters.
  const SAMPLER sampler_;           ///< Control sampler.
};

}  // namespace gtsam

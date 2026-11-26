// Copyright (c) Georgia Tech Research Corporation. See LICENSE for details.

/**
 * @file CandidateScoring.h
 * @author Frank Dellaert
 * @author Akash Ratheesh
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>

#include <algorithm>
#include <cstddef>
#include <typeinfo>
#include <unordered_map>
#include <vector>

#ifdef GTSAM_USE_TBB
#include <tbb/parallel_for.h>
#include <tbb/task_arena.h>
#endif

namespace gtsam::internal {

/// A non-sendable factor must be evaluated on the thread that called the
/// planner.
inline bool allFactorsSendable(const NonlinearFactorGraph& graph) {
  return std::all_of(graph.begin(), graph.end(), [](const auto& factor) {
    return !factor || factor->sendable();
  });
}

/**
 * Evaluate trajectory costs in graph order. Ordinary priors on trajectory keys
 * can be evaluated directly on a state or a control. Other factors receive
 * the rolled-out Values with any auxiliary graph variables preserved.
 */
template <class State, class Control>
class TrajectoryCosts {
 public:
  /// Identify the cost factors that can be scored without building Values.
  TrajectoryCosts(const NonlinearFactorGraph& graph, const KeyVector& stateKeys,
                  const KeyVector& controlKeys) {
    std::unordered_map<Key, size_t> states, controls;
    for (size_t k = 0; k < stateKeys.size(); ++k)
      states.emplace(stateKeys[k], k);
    for (size_t k = 0; k < controlKeys.size(); ++k)
      controls.emplace(controlKeys[k], k);
    terms_.reserve(graph.size());
    for (const auto& factor : graph) {
      if (!factor) continue;
      CostTerm term{factor.get()};
      if (factor->size() == 1) {
        const Key key = factor->front();
        if (const auto it = states.find(key); it != states.end()) {
          term.step = it->second;
          term.statePrior = exactPrior<State>(*factor);
        } else if (const auto it = controls.find(key); it != controls.end()) {
          term.step = it->second;
          term.controlPrior = exactPrior<Control>(*factor);
        }
      }
      needsValues_ |= term.needsValues();
      terms_.push_back(term);
    }
  }

  /// Whether at least one factor must read a complete Values object.
  bool needsValues() const { return needsValues_; }

  /// Sum this trajectory's factor errors on the current thread in graph order.
  double error(const std::vector<State>& states,
               const Eigen::Ref<const Matrix>& controls,
               const Values& values) const {
    double result = 0.;
    for (const auto& term : terms_)
      result += term.error(states, controls, values);
    return result;
  }

 private:
  /**
   * A factor of the cost graph. A prior on a rolled-out state or
   * control is scored on it directly; any other factor through Values.
   */
  struct CostTerm {
    const NonlinearFactor* factor;  ///< The factor.
    /// The factor, if it is a prior on the state x_step.
    const PriorFactor<State>* statePrior = nullptr;
    /// The factor, if it is a prior on the control u_step.
    const PriorFactor<Control>* controlPrior = nullptr;
    size_t step = 0;  ///< The step of the prior's state or control.

    /// Whether the factor is scored through Values.
    bool needsValues() const { return !statePrior && !controlPrior; }

    /**
     * The factor's error for one rollout: states x_0 to x_H, the controls,
     * one column per step, and the rollout written into values when
     * needsValues() is true.
     */
    double error(const std::vector<State>& states,
                 const Eigen::Ref<const Matrix>& controls,
                 const Values& values) const {
      if (statePrior) return statePrior->error(states[step]);
      if (controlPrior) return controlPrior->error(Control(controls.col(step)));
      return factor->error(values);
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

  std::vector<CostTerm> terms_;
  bool needsValues_ = false;
};

/**
 * Roll out disjoint candidates with private scratch. The calling thread scores
 * their completed trajectories in sample order unless the caller explicitly
 * permits concurrent scoring. Reuse a bounded set of trajectories across
 * blocks so scratch memory depends on worker count rather than sample count.
 */
template <class Scratch, class Rollout, class Score>
Vector scoreCandidates(size_t count, const Scratch& initialScratch,
                       bool parallelRollouts, bool parallelScoring,
                       const Rollout& rollout, const Score& score) {
  Vector costs(count);
#ifdef GTSAM_USE_TBB
  if (parallelRollouts && count > 1) {
    const size_t workers = tbb::this_task_arena::max_concurrency();
    std::vector<Scratch> scratch(std::min(count, 2 * workers), initialScratch);
    for (size_t first = 0; first < count; first += scratch.size()) {
      const size_t size = std::min(count - first, scratch.size());
      tbb::parallel_for(size_t{0}, size, [&](size_t slot) {
        const size_t candidate = first + slot;
        rollout(candidate, scratch[slot]);
        if (parallelScoring) costs[candidate] = score(candidate, scratch[slot]);
      });
      if (!parallelScoring) {
        for (size_t slot = 0; slot < size; ++slot) {
          const size_t candidate = first + slot;
          costs[candidate] = score(candidate, scratch[slot]);
        }
      }
    }
    return costs;
  }
#endif
  Scratch scratch(initialScratch);
  for (size_t i = 0; i < count; ++i) {
    rollout(i, scratch);
    costs[i] = score(i, scratch);
  }
  return costs;
}

}  // namespace gtsam::internal

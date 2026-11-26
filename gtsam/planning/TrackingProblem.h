// Copyright (c) Georgia Tech Research Corporation. See LICENSE for details.
/**
 * @file TrackingProblem.h
 * @author Frank Dellaert
 * @author Akash Ratheesh
 */
#pragma once

#include <gtsam/planning/CandidateScoring.h>
#include <gtsam/planning/TrackingFeedback.h>

#include <array>
#include <functional>
#include <type_traits>
#include <utility>
#include <vector>

namespace gtsam::internal {

/** Shared trajectory layout for the two tracking optimizers. State uses
 * manifold coordinates and Control is a fixed or dynamic Eigen column vector,
 * as in SamplingPlanner. Preserve the concrete control type in Values.
 */
template <class StateType, class Control>
class TrackingProblem {
 public:
  using State = StateType;
  using Transition = std::function<State(const State&, const Control&)>;
  using VectorField = std::function<Vector(const State&, const Control&)>;

 protected:
  static_assert(IsManifold<State>::value, "State must be a GTSAM manifold.");
  static_assert(
      std::is_same_v<Control,
                     Eigen::Matrix<double, traits<Control>::dimension, 1>>,
      "Control must be an Eigen column vector.");

  /// Validate the horizon layout once when constructing a tracker.
  TrackingProblem(KeyVector stateKeys, KeyVector controlKeys)
      : stateKeys_(std::move(stateKeys)),
        controlKeys_(std::move(controlKeys)),
        keyMap_(buildStageKeyMap(stateKeys_, controlKeys_)) {}

  /// Collect the typed controls from the initial trajectory.
  Matrix initialControls(const Values& initial, const Vector& lower,
                         const Vector& upper) const {
    const auto m = initial.at<Control>(controlKeys_.front()).size();
    validateControlLimits(m, lower, upper);
    Matrix controls(m, controlKeys_.size());
    for (size_t k = 0; k < controlKeys_.size(); ++k) {
      const Control u = initial.at<Control>(controlKeys_[k]);
      if (u.size() != m || !u.allFinite())
        throw std::invalid_argument("Invalid initial tracking control.");
      controls.col(k) = u;
    }
    return controls;
  }

  /// Roll out the bounded controls, applying feedback about the nominal states.
  template <class Step>
  void rollout(const State& x0, Matrix& controls,
               const std::vector<State>* base, const TrackingSweep* sweep,
               double alpha, const Vector& lower, const Vector& upper,
               std::vector<State>& states, const Step& step) const {
    if constexpr (std::is_same_v<State, Vector>) {
      if (!x0.allFinite())
        throw std::invalid_argument("Nonfinite initial tracking state.");
    }
    states.clear();
    states.reserve(stateKeys_.size());
    states.push_back(x0);
    for (size_t k = 0; k < controlKeys_.size(); ++k) {
      auto u = controls.col(k);
      if (sweep) {
        const Vector dx = traits<State>::Local((*base)[k], states[k]);
        u += alpha * sweep->feedforward.col(k) + sweep->gains[k] * dx;
      }
      if (lower.size()) u = u.cwiseMax(lower);
      if (upper.size()) u = u.cwiseMin(upper);
      const Control control(u);
      State next = step(states[k], control);
      if constexpr (std::is_same_v<State, Vector>) {
        if (next.size() != x0.size())
          throw std::invalid_argument(
              "Tracking transition has wrong dimension.");
      }
      states.push_back(std::move(next));
    }
  }

  KeyVector stateKeys_, controlKeys_;
  StageKeyMap keyMap_;
};

/** A stage-local cost graph prepared for repeated linearization. Gaussian
 * priors on vector spaces have constant curvature. Cache their precision and
 * translate the linear and constant terms to each new nominal trajectory.
 * All other costs are linearized by the original nonlinear factors.
 */
template <class State, class Control>
class TrackingObjective {
 public:
  /// Prepare the cost factors for repeated scoring and linearization.
  TrackingObjective(const NonlinearFactorGraph& graph, const Values& initial,
                    const KeyVector& xs, const KeyVector& us,
                    const StageKeyMap& stages, Eigen::Index n, Eigen::Index m)
      : initial_(initial),
        stateKeys_(xs),
        controlKeys_(us),
        stages_(stages),
        scoring_(graph, xs, us),
        quadratic_{{},
                   HessianFactor(xs.back(), Matrix::Zero(n, n), Vector::Zero(n),
                                 0.0)} {
    const std::array<Eigen::Index, 2> dims{m, n};
    quadratic_.stageFactors.reserve(us.size());
    for (size_t k = 0; k < us.size(); ++k) {
      SymmetricBlockMatrix info(dims, true);
      info.setZero();
      quadratic_.stageFactors.emplace_back(KeyVector{us[k], xs[k]},
                                           std::move(info));
    }
    for (const auto& factor : graph) {
      if (!factor) continue;
      if (factor->size() == 1) {
        const auto it = stages.find(factor->front());
        if (it != stages.end()) {
          const auto [k, isControl] = it->second;
          if (isControl ? cachePrior(*factor, k, 0, m, controlPriors_)
                        : cachePrior(*factor, k, k < us.size() ? 1 : 0, n,
                                     statePriors_))
            continue;
        }
      }
      nonlinear_.push_back(factor);
    }
  }

  /// Write the trajectory into Values, preserving auxiliary variables.
  Values trajectoryValues(const std::vector<State>& states,
                          const Matrix& controls) const {
    Values values(initial_);
    for (size_t k = 0; k < controlKeys_.size(); ++k) {
      values.insert_or_assign(controlKeys_[k], Control(controls.col(k)));
      values.insert_or_assign(stateKeys_[k + 1], states[k + 1]);
    }
    return values;
  }

  /// Score the trajectory with the same factor evaluation as SamplingPlanner.
  double error(const std::vector<State>& states, const Matrix& controls) const {
    if (scoring_.needsValues())
      return scoring_.error(states, controls,
                            trajectoryValues(states, controls));
    return scoring_.error(states, controls, initial_);
  }

  /// Assemble the exact local Gaussian factors about the nominal trajectory.
  StageLinearizedCosts linearize(const std::vector<State>& states,
                                 const Matrix& controls) const {
    auto result = quadratic_;
    for (const auto& prior : statePriors_)
      prior.recenter(states[prior.stage], stageFactor(result, prior.stage));
    for (const auto& prior : controlPriors_)
      prior.recenter(Control(controls.col(prior.stage)),
                     result.stageFactors[prior.stage]);
    if (nonlinear_.empty()) return result;
    const Values values = trajectoryValues(states, controls);
    for (const auto& factor : nonlinear_) {
      const auto linear = factor->linearize(values);
      if (linear && !linear->empty()) accumulate(result, *linear);
    }
    return result;
  }

 private:
  /// Increments about x have residual dx - Local(x, mean) for a vector space.
  template <class T>
  struct QuadraticPrior {
    size_t stage, block;
    const T& mean;
    Matrix precision;

    void recenter(const T& x, HessianFactor& joint) const {
      const Vector delta = traits<T>::Local(x, mean);
      const Vector eta = precision * delta;
      joint.info().updateOffDiagonalBlock(block, joint.size(), eta);
      joint.constantTerm() += delta.dot(eta);
    }
  };

  static HessianFactor& stageFactor(StageLinearizedCosts& costs, size_t stage) {
    return stage == costs.stageFactors.size() ? costs.terminalFactor
                                              : costs.stageFactors[stage];
  }

  template <class T>
  bool cachePrior(const NonlinearFactor& factor, size_t stage, size_t block,
                  Eigen::Index dimension,
                  std::vector<QuadraticPrior<T>>& priors) {
    if constexpr (std::is_base_of_v<vector_space_tag,
                                    typename traits<T>::structure_category>) {
      if (typeid(factor) == typeid(PriorFactor<T>)) {
        const auto& prior = static_cast<const PriorFactor<T>&>(factor);
        const auto* model =
            dynamic_cast<const noiseModel::Gaussian*>(prior.noiseModel().get());
        if (model && !model->isConstrained() &&
            model->dim() == size_t(dimension) &&
            traits<T>::GetDimension(prior.prior()) == size_t(dimension)) {
          priors.push_back({stage, block, prior.prior(), model->information()});
          stageFactor(quadratic_, stage)
              .info()
              .updateDiagonalBlock(block, priors.back().precision);
          return true;
        }
      }
    }
    return false;
  }

  void accumulate(StageLinearizedCosts& out,
                  const GaussianFactor& linear) const {
    const auto it = stages_.find(linear.front());
    if (it == stages_.end() || linear.size() > 2)
      throw std::invalid_argument(
          "Expected a unary or binary trajectory cost on known keys.");
    const auto [stage, isControl] = it->second;
    if (linear.size() == 2) {
      const auto other = stages_.find(linear.back());
      if (other == stages_.end() || other->second.first != stage ||
          other->second.second == isControl)
        throw std::invalid_argument(
            "Binary trajectory costs must couple the state and control of one "
            "stage.");
    }
    auto& joint = stageFactor(out, stage);
    linear.updateHessian(joint.keys(), &joint.info());
  }

  const Values& initial_;
  const KeyVector& stateKeys_;
  const KeyVector& controlKeys_;
  const StageKeyMap& stages_;
  TrajectoryCosts<State, Control> scoring_;
  std::vector<QuadraticPrior<State>> statePriors_;
  std::vector<QuadraticPrior<Control>> controlPriors_;
  StageLinearizedCosts quadratic_;
  NonlinearFactorGraph nonlinear_;
};

}  // namespace gtsam::internal

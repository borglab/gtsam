// Copyright (c) Georgia Tech Research Corporation. See LICENSE for details.
/**
 * @file DDPTracking.h
 * @author Frank Dellaert
 * @author Akash Ratheesh
 */
#pragma once

#include <gtsam/base/Manifold.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/planning/TrackingProblem.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

namespace gtsam {

/**
 * Manifold-generic full second-order finite-difference DDP for trajectory
 * optimization and tracking on any GTSAM manifold (including Vector, Pose2,
 * Pose3, Rot3, Unit3), sharing the exact same NonlinearFactorGraph + Values API
 * and HessianFactor::eliminateCholesky -> GaussianBayesNet elimination engine
 * as ILQRTracking<State, Control>.
 */
template <class State = Vector, class Control = Vector>
class DDPTracking : public internal::TrackingProblem<State, Control> {
  using Base = internal::TrackingProblem<State, Control>;
  using Base::controlKeys_;
  using Base::keyMap_;
  using Base::stateKeys_;

 public:
  using typename Base::Transition;
  using typename Base::VectorField;
  /// Associate the horizon with ordered state (H+1) and control (H) keys
  /// (matching SamplingPlanner and ILQRTracking).
  DDPTracking(KeyVector stateKeys, KeyVector controlKeys)
      : Base(std::move(stateKeys), std::move(controlKeys)) {}

  /** Optimize trajectory Values and GaussianBayesNet policy for a
   * NonlinearFactorGraph objective using a discrete transition
   * x_{k+1} = f(x_k, u_k).
   */
  Values optimize(const NonlinearFactorGraph& cost, const Values& initial,
                  const Transition& transition, size_t iterations,
                  const Vector& lower = {}, const Vector& upper = {}) {
    if (!transition) throw std::invalid_argument("Invalid DDP transition.");
    const size_t h = controlKeys_.size();
    const State x0 = initial.at<State>(stateKeys_.front());
    const Eigen::Index n =
        static_cast<Eigen::Index>(traits<State>::GetDimension(x0));
    Matrix controls = this->initialControls(initial, lower, upper);
    const Eigen::Index m = controls.rows();
    const Eigen::Index d = n + m;

    auto rollout = [&](Matrix& u, const std::vector<State>* base,
                       const internal::TrackingSweep* sweep, double alpha,
                       std::vector<State>& statesOut) {
      this->rollout(x0, u, base, sweep, alpha, lower, upper, statesOut,
                    transition);
    };

    auto backward = [&](const std::vector<State>& states, const Matrix& u,
                        const internal::StageLinearizedCosts& stageCosts,
                        double reg) -> internal::TrackingSweep {
      const auto horizon = static_cast<Eigen::Index>(h);
      HessianFactor valueToGo = stageCosts.terminalFactor;
      internal::TrackingSweep sweep{
          GaussianBayesNet(), Matrix::Zero(m, horizon), std::vector<Matrix>(h)};
      sweep.policy.resize(h);

      for (Eigen::Index k = horizon - 1; k >= 0; --k) {
        const auto idx = static_cast<size_t>(k);
        const State& xk = states[idx];
        const Control uk = u.col(k);
        const State& center = states[idx + 1];
        Vector delta(d);
        if constexpr (std::is_same_v<State, Vector>)
          delta.head(n) = 1e-4 * (1.0 + xk.array().abs());
        else
          delta.head(n).setConstant(1e-4);
        delta.tail(m) = 1e-4 * (1.0 + uk.array().abs());

        // Evaluate local tangent displacement relative to nominal x_{k+1}.
        auto fLocal = [&](const Vector& dz) -> Vector {
          const State next = transition(traits<State>::Retract(xk, dz.head(n)),
                                        uk + dz.tail(m));
          return traits<State>::Local(center, next);
        };

        // Central-difference Jacobian (A, B) and second-order value-gradient
        // contraction (valueGrad = -valueToGo.linearTerm()).
        const Vector valueGrad = -valueToGo.linearTerm();
        Matrix jacobian(n, d), secondOrder = Matrix::Zero(d, d);
        for (Eigen::Index i = 0; i < d; ++i) {
          Vector shift = Vector::Zero(d);
          shift[i] = delta[i];
          const Vector plus = fLocal(shift), minus = fLocal(-shift);
          jacobian.col(i) = (plus - minus) / (2.0 * delta[i]);
          secondOrder(i, i) =
              valueGrad.dot(plus + minus) / (delta[i] * delta[i]);
          for (Eigen::Index j = 0; j < i; ++j) {
            Vector cross = Vector::Zero(d);
            cross[j] = delta[j];
            const Vector mixed = fLocal(shift + cross) - fLocal(shift - cross) -
                                 fLocal(-shift + cross) +
                                 fLocal(-shift - cross);
            secondOrder(i, j) = secondOrder(j, i) =
                valueGrad.dot(mixed) / (4.0 * delta[i] * delta[j]);
          }
        }

        // Pull valueToGo back through (A, B), combine with the second-order
        // dynamics HessianFactor and linearized NonlinearFactorGraph stage
        // cost factors.
        const HessianFactor curvature(
            controlKeys_[idx], stateKeys_[idx],
            secondOrder.bottomRightCorner(m, m),
            secondOrder.bottomLeftCorner(m, n), Vector::Zero(m),
            secondOrder.topLeftCorner(n, n), Vector::Zero(n), 0.0);
        auto conditional = internal::eliminateControlStage(
            jacobian.leftCols(n), jacobian.rightCols(m),
            stageCosts.stageFactors[idx], reg, valueToGo, &curvature);
        if (!conditional) return {};
        internal::extractControlLaw(*conditional, sweep.feedforward.col(k),
                                    sweep.gains[idx]);
        sweep.policy[idx] = std::move(conditional);
      }

      return sweep;
    };

    const internal::TrackingObjective<State, Control> trackingCost(
        cost, initial, stateKeys_, controlKeys_, keyMap_, n, m);
    std::vector<State> states;
    rollout(controls, nullptr, nullptr, 0.0, states);
    double objective = trackingCost.error(states, controls);
    if (!std::isfinite(objective))
      throw std::domain_error("Nonfinite initial DDP rollout.");
    double damping = 1e-9;
    auto stageCosts = trackingCost.linearize(states, controls);

    auto solveBackward = [&](double& reg) {
      auto sweep = backward(states, controls, stageCosts, reg);
      while (sweep.gains.empty() && reg < 1e12) {
        reg = std::max(1e-6, 10.0 * reg);
        sweep = backward(states, controls, stageCosts, reg);
      }
      return sweep;
    };

    for (size_t i = 0;; ++i) {
      auto sweep = solveBackward(damping);
      if (sweep.gains.empty())
        throw std::domain_error("DDP control Hessian remained indefinite.");

      bool accepted = false;
      if (i < iterations && sweep.feedforward.norm() > 1e-10) {
        for (int step = 0; step < 8; ++step) {
          const double alpha = std::ldexp(1.0, -step);
          Matrix trial = controls;
          std::vector<State> trialStates;
          rollout(trial, &states, &sweep, alpha, trialStates);
          const double value = trackingCost.error(trialStates, trial);
          if (std::isfinite(value) && value < objective) {
            controls = std::move(trial);
            states = std::move(trialStates);
            objective = value;
            damping = std::max(1e-9, 0.5 * damping);
            accepted = true;
            break;
          }
        }
      }

      if (accepted) {
        stageCosts = trackingCost.linearize(states, controls);
        continue;
      }

      if (damping > 1e-9) {
        double finalDamping = 1e-9;
        auto finalSweep = solveBackward(finalDamping);
        if (!finalSweep.gains.empty()) sweep = std::move(finalSweep);
      }
      states_ = std::move(states);
      policy_ = std::move(sweep.policy);
      gains_ = std::move(sweep.gains);
      return trackingCost.trajectoryValues(states_, controls);
    }
  }

  /** Optimize trajectory Values and GaussianBayesNet policy using a
   * continuous-time vector field xi = vectorField(x, u) integrated via
   * traits<State>::Retract(x, dt * xi) (matching SamplingPlanner and
   * ILQRTracking).
   */
  Values optimize(const NonlinearFactorGraph& cost, const Values& initial,
                  const VectorField& vectorField, double dt, size_t iterations,
                  const Vector& lower = {}, const Vector& upper = {}) {
    if (!vectorField || !std::isfinite(dt) || !(dt > 0.0))
      throw std::invalid_argument(
          "Expected a valid vector field and positive finite dt.");
    const Transition discrete = [&vectorField, dt](const State& x,
                                                   const Control& u) -> State {
      const Vector xi = vectorField(x, u);
      if (static_cast<size_t>(xi.size()) != traits<State>::GetDimension(x))
        throw std::invalid_argument(
            "Vector field returned the wrong tangent dimension.");
      return traits<State>::Retract(x, dt * xi);
    };
    return optimize(cost, initial, discrete, iterations, lower, upper);
  }

  /// State keys X(0..H).
  const KeyVector& stateKeys() const { return stateKeys_; }

  /// Control keys U(0..H-1).
  const KeyVector& controlKeys() const { return controlKeys_; }

  /// Optimized state trajectory on State ordered in time.
  const std::vector<State>& states() const { return states_; }

  /// Eliminated GaussianBayesNet policy containing conditionals P(du_k | dx_k)
  /// ordered in time k = 0..H-1.
  const GaussianBayesNet& policy() const { return policy_; }

  /// Gain multiplying traits<State>::Local(reference, actual) at a given step.
  const Matrix& gain(size_t step) const { return gains_.at(step); }

 private:
  std::vector<State> states_;
  GaussianBayesNet policy_;
  std::vector<Matrix> gains_;
};

}  // namespace gtsam

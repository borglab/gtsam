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
#include <gtsam/planning/TrackingFeedback.h>

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
 * optimization and tracking on any GTSAM Manifold M (including Vector, Pose2,
 * Pose3, Rot3, Unit3), sharing the exact same NonlinearFactorGraph + Values API
 * and HessianFactor::eliminateCholesky -> GaussianBayesNet elimination engine
 * as ILQRTracking<M>.
 */
template <class M = Vector>
class DDPTracking {
  static_assert(IsManifold<M>::value,
                "Template parameter M must be a GTSAM Manifold.");

 public:
  using State = M;
  using Transition = std::function<M(const M&, const Vector&)>;
  using VectorField = std::function<Vector(const M&, const Vector&)>;

  /// Associate the horizon with ordered state (H+1) and control (H) keys
  /// (matching SamplingPlanner and ILQRTracking).
  DDPTracking(KeyVector stateKeys, KeyVector controlKeys)
      : stateKeys_(std::move(stateKeys)),
        controlKeys_(std::move(controlKeys)),
        keyMap_(internal::buildStageKeyMap(stateKeys_, controlKeys_)) {}

  /** Optimize trajectory Values and GaussianBayesNet policy for a
   * NonlinearFactorGraph objective using a discrete transition
   * x_{k+1} = f(x_k, u_k).
   */
  Values optimize(const NonlinearFactorGraph& cost, const Values& initial,
                  const Transition& transition, size_t iterations,
                  const Vector& lower = {}, const Vector& upper = {}) {
    if (!transition) throw std::invalid_argument("Invalid DDP transition.");
    const size_t h = controlKeys_.size();
    const M x0 = initial.at<M>(stateKeys_.front());
    const Eigen::Index n =
        static_cast<Eigen::Index>(traits<M>::GetDimension(x0));
    const Eigen::Index m = initial.at<Vector>(controlKeys_.front()).size();
    const Eigen::Index d = n + m;
    if constexpr (std::is_same_v<M, Vector>) {
      if (!x0.allFinite())
        throw std::invalid_argument("Nonfinite initial DDP state.");
    }
    internal::validateControlLimits(m, lower, upper);

    Matrix controls(m, static_cast<Eigen::Index>(h));
    for (size_t k = 0; k < h; ++k) {
      const Vector uk = initial.at<Vector>(controlKeys_[k]);
      if (uk.size() != m || !uk.allFinite())
        throw std::invalid_argument("Invalid initial DDP control.");
      controls.col(static_cast<Eigen::Index>(k)) = uk;
    }

    auto rolloutValues = [&](Matrix& u, const std::vector<M>* base,
                             const internal::TrackingSweep* sweep, double alpha,
                             std::vector<M>& statesOut) {
      statesOut.clear();
      statesOut.reserve(h + 1);
      statesOut.push_back(x0);
      Values vals = initial;
      vals.insert_or_assign(stateKeys_[0], x0);
      for (size_t k = 0; k < h; ++k) {
        const auto col = static_cast<Eigen::Index>(k);
        if (sweep && base) {
          const Vector dx = traits<M>::Local((*base)[k], statesOut[k]);
          u.col(col) +=
              alpha * sweep->feedforward.col(col) + sweep->gains[k] * dx;
        }
        if (lower.size()) u.col(col) = u.col(col).cwiseMax(lower);
        if (upper.size()) u.col(col) = u.col(col).cwiseMin(upper);
        M next = transition(statesOut[k], u.col(col));
        if constexpr (std::is_same_v<M, Vector>) {
          if (next.size() != n)
            throw std::invalid_argument(
                "Tracking transition has wrong dimension.");
        }
        vals.insert_or_assign(controlKeys_[k], Vector(u.col(col)));
        vals.insert_or_assign(stateKeys_[k + 1], next);
        statesOut.push_back(std::move(next));
      }
      return vals;
    };

    auto backward = [&](const std::vector<M>& states, const Matrix& u,
                        const internal::StageLinearizedCosts& stageCosts,
                        double reg) -> internal::TrackingSweep {
      const auto horizon = static_cast<Eigen::Index>(h);
      HessianFactor valueToGo = stageCosts.terminalFactor;
      internal::TrackingSweep sweep{
          GaussianBayesNet(), Matrix::Zero(m, horizon), std::vector<Matrix>(h)};
      std::vector<GaussianConditional::shared_ptr> conditionals(h);

      for (Eigen::Index k = horizon - 1; k >= 0; --k) {
        const auto idx = static_cast<size_t>(k);
        const M& xk = states[idx];
        const Vector uk = u.col(k);
        const M& center = states[idx + 1];
        Vector delta(d);
        if constexpr (std::is_same_v<M, Vector>)
          delta.head(n) = 1e-4 * (1.0 + xk.array().abs());
        else
          delta.head(n).setConstant(1e-4);
        delta.tail(m) = 1e-4 * (1.0 + uk.array().abs());

        // Evaluate local tangent displacement relative to nominal x_{k+1}.
        auto fLocal = [&](const Vector& dz) -> Vector {
          const M next =
              transition(traits<M>::Retract(xk, dz.head(n)), uk + dz.tail(m));
          return traits<M>::Local(center, next);
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
        HessianFactor stageFactor = internal::pullBackDynamics(
            stateKeys_[idx], controlKeys_[idx], jacobian.leftCols(n),
            jacobian.rightCols(m), valueToGo);
        HessianFactor(controlKeys_[idx], stateKeys_[idx],
                      secondOrder.bottomRightCorner(m, m),
                      secondOrder.bottomLeftCorner(m, n), Vector::Zero(m),
                      secondOrder.topLeftCorner(n, n), Vector::Zero(n), 0.0)
            .updateHessian(&stageFactor);
        for (const auto& factor : stageCosts.stageFactors[idx])
          factor->updateHessian(stageFactor.keys(), &stageFactor.info());
        if (reg > 0.0)
          stageFactor.info().updateDiagonalBlock(0,
                                                 reg * Matrix::Identity(m, m));

        auto [conditional, nextValueToGo] = internal::eliminateControlFactor(
            std::move(stageFactor), controlKeys_[idx]);
        if (!conditional) return {};
        internal::extractControlLaw(*conditional, sweep.feedforward.col(k),
                                    sweep.gains[idx]);
        conditionals[idx] = std::move(conditional);
        valueToGo = std::move(nextValueToGo);
      }

      sweep.policy.reserve(h);
      for (auto& cond : conditionals) sweep.policy.push_back(std::move(cond));
      return sweep;
    };

    std::vector<M> states;
    Values values = rolloutValues(controls, nullptr, nullptr, 0.0, states);
    double objective = cost.error(values);
    if (!std::isfinite(objective))
      throw std::domain_error("Nonfinite initial DDP rollout.");
    double damping = 1e-9;
    auto stageCosts = internal::linearizeStageCosts(cost, values, stateKeys_,
                                                    controlKeys_, keyMap_, n);

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
          std::vector<M> trialStates;
          Values trialValues =
              rolloutValues(trial, &states, &sweep, alpha, trialStates);
          const double value = cost.error(trialValues);
          if (std::isfinite(value) && value < objective) {
            controls = std::move(trial);
            states = std::move(trialStates);
            values = std::move(trialValues);
            objective = value;
            damping = std::max(1e-9, 0.5 * damping);
            accepted = true;
            break;
          }
        }
      }

      if (accepted) {
        stageCosts = internal::linearizeStageCosts(cost, values, stateKeys_,
                                                   controlKeys_, keyMap_, n);
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
      return values;
    }
  }

  /** Optimize trajectory Values and GaussianBayesNet policy using a
   * continuous-time vector field xi = vectorField(x, u) integrated via
   * traits<M>::Retract(x, dt * xi) (matching SamplingPlanner and ILQRTracking).
   */
  Values optimize(const NonlinearFactorGraph& cost, const Values& initial,
                  const VectorField& vectorField, double dt, size_t iterations,
                  const Vector& lower = {}, const Vector& upper = {}) {
    if (!vectorField || !std::isfinite(dt) || !(dt > 0.0))
      throw std::invalid_argument(
          "Expected a valid vector field and positive finite dt.");
    const Transition discrete = [&vectorField, dt](const M& x,
                                                   const Vector& u) -> M {
      const Vector xi = vectorField(x, u);
      if (static_cast<size_t>(xi.size()) != traits<M>::GetDimension(x))
        throw std::invalid_argument(
            "Vector field returned the wrong tangent dimension.");
      return traits<M>::Retract(x, dt * xi);
    };
    return optimize(cost, initial, discrete, iterations, lower, upper);
  }

  /// State keys X(0..H).
  const KeyVector& stateKeys() const { return stateKeys_; }

  /// Control keys U(0..H-1).
  const KeyVector& controlKeys() const { return controlKeys_; }

  /// Optimized state trajectory on M ordered in time.
  const std::vector<M>& states() const { return states_; }

  /// Eliminated GaussianBayesNet policy containing conditionals P(du_k | dx_k)
  /// ordered in time k = 0..H-1.
  const GaussianBayesNet& policy() const { return policy_; }

  /// Gain multiplying traits<M>::Local(reference, actual) at a given step.
  const Matrix& gain(size_t step) const { return gains_.at(step); }

 private:
  KeyVector stateKeys_, controlKeys_;
  internal::StageKeyMap keyMap_;
  std::vector<M> states_;
  GaussianBayesNet policy_;
  std::vector<Matrix> gains_;
};

}  // namespace gtsam

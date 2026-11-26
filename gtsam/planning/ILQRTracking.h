// Copyright (c) Georgia Tech Research Corporation. See LICENSE for details.
/**
 * @file ILQRTracking.h
 * @author Frank Dellaert
 * @author Akash Ratheesh
 */
#pragma once

#include <gtsam/base/Manifold.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/linear/GaussianBayesNet.h>
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

namespace internal {

struct ILQRLinearization {
  std::vector<Matrix> A;
  std::vector<Matrix> B;
};

/// Reverse-time chain elimination of linearized stage/terminal cost factors
/// using eliminateControlStage, returning a
/// GaussianBayesNet policy P(du_k | dx_k).
GTSAM_EXPORT TrackingSweep backwardILQR(const StageLinearizedCosts& costs,
                                        const ILQRLinearization& lin,
                                        double damping);

}  // namespace internal

/**
 * Manifold-generic factor-graph TVLQR / iLQR for trajectory optimization and
 * tracking on any GTSAM manifold (including Vector, Pose2, Pose3, Rot3,
 * Unit3), aligned with ManifoldEKF<State> and SamplingPlanner<State, Control>.
 *
 * Each outer iteration linearizes the NonlinearFactorGraph objective and
 * dynamics in local tangent coordinates and performs reverse-time chain
 * elimination via HessianFactor::eliminateCholesky, returning the optimized
 * Values and a GaussianBayesNet policy of conditionals P(du_k | dx_k).
 *
 * Callers can supply:
 * - an analytic linearized transition f(x_k, u_k, Hx, Hu) following GTSAM's
 *   OptionalJacobian convention (exact tangent Jacobians, zero differencing),
 * - a discrete transition x_{k+1} = f(x_k, u_k) (central differences in local
 *   tangent coordinates),
 * - or a continuous-time vector field (f, dt) integrated via
 *   traits<State>::Retract(x, dt * f(x, u)) (matching SamplingPlanner).
 */
template <class State = Vector, class Control = Vector>
class ILQRTracking : public internal::TrackingProblem<State, Control> {
  using Base = internal::TrackingProblem<State, Control>;
  using Base::controlKeys_;
  using Base::keyMap_;
  using Base::stateKeys_;

 public:
  using typename Base::Transition;
  using typename Base::VectorField;
  using OptionalMatrix = OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic>;
  using LinearizedTransition = std::function<State(
      const State&, const Control&, OptionalMatrix, OptionalMatrix)>;

  /// Associate the horizon with ordered state (H+1) and control (H) keys
  /// (matching SamplingPlanner).
  ILQRTracking(KeyVector stateKeys, KeyVector controlKeys)
      : Base(std::move(stateKeys), std::move(controlKeys)) {}

  /** Optimize trajectory Values and GaussianBayesNet policy for a
   * NonlinearFactorGraph objective using an analytic OptionalJacobian
   * transition f(x_k, u_k, Hx, Hu).
   */
  Values optimize(const NonlinearFactorGraph& cost, const Values& initial,
                  const LinearizedTransition& transition, size_t iterations,
                  const Vector& lower = {}, const Vector& upper = {}) {
    if (!transition) throw std::invalid_argument("Invalid iLQR transition.");
    const size_t h = controlKeys_.size();
    const State x0 = initial.at<State>(stateKeys_.front());
    const Eigen::Index n =
        static_cast<Eigen::Index>(traits<State>::GetDimension(x0));
    Matrix controls = this->initialControls(initial, lower, upper);
    const Eigen::Index m = controls.rows();

    auto rollout = [&](Matrix& u, const std::vector<State>* base,
                       const internal::TrackingSweep* sweep, double alpha,
                       std::vector<State>& statesOut) {
      this->rollout(x0, u, base, sweep, alpha, lower, upper, statesOut,
                    [&](const State& x, const Control& input) {
                      return transition(x, input, {}, {});
                    });
    };

    auto linearizeDynamics = [&](const std::vector<State>& x, const Matrix& u) {
      internal::ILQRLinearization lin{std::vector<Matrix>(h),
                                      std::vector<Matrix>(h)};
      for (size_t k = 0; k < h; ++k) {
        Matrix A(n, n), B(n, m);
        transition(x[k], u.col(static_cast<Eigen::Index>(k)), A, B);
        if (A.rows() != n || A.cols() != n || B.rows() != n || B.cols() != m ||
            !A.allFinite() || !B.allFinite())
          throw std::domain_error("Invalid iLQR Jacobian.");
        lin.A[k] = std::move(A);
        lin.B[k] = std::move(B);
      }
      return lin;
    };

    const internal::TrackingObjective<State, Control> trackingCost(
        cost, initial, stateKeys_, controlKeys_, keyMap_, n, m);
    std::vector<State> states;
    rollout(controls, nullptr, nullptr, 0.0, states);
    double objective = trackingCost.error(states, controls);
    if (!std::isfinite(objective))
      throw std::domain_error("Nonfinite initial iLQR rollout.");
    double damping = 0.0;
    auto lin = linearizeDynamics(states, controls);
    auto stageCosts = trackingCost.linearize(states, controls);

    for (size_t i = 0;; ++i) {
      auto sweep = internal::backwardILQR(stageCosts, lin, damping);
      while (sweep.gains.empty() && damping < 1e6) {
        damping = damping == 0.0 ? 1e-4 : 10.0 * damping;
        sweep = internal::backwardILQR(stageCosts, lin, damping);
      }
      if (sweep.gains.empty())
        throw std::domain_error("iLQR control Hessian remained indefinite.");

      bool accepted = false;
      bool converged = false;
      if (i < iterations && sweep.feedforward.norm() > 1e-9) {
        for (int step = 0; step < 8; ++step) {
          const double alpha = std::ldexp(1.0, -step);
          Matrix trial = controls;
          std::vector<State> trialStates;
          rollout(trial, &states, &sweep, alpha, trialStates);
          const double value = trackingCost.error(trialStates, trial);
          if (std::isfinite(value) && value < objective) {
            const double decrease = objective - value;
            controls = std::move(trial);
            states = std::move(trialStates);
            objective = value;
            damping = damping > 1e-9 ? 0.5 * damping : 0.0;
            accepted = true;
            if (decrease <= 1e-12 * (1.0 + objective)) converged = true;
            break;
          }
        }
        if (!accepted && damping < 1e6) {
          damping = damping == 0.0 ? 1e-4 : 10.0 * damping;
          continue;
        }
      }

      if (accepted && !converged) {
        lin = linearizeDynamics(states, controls);
        stageCosts = trackingCost.linearize(states, controls);
        continue;
      }

      if (accepted) {
        lin = linearizeDynamics(states, controls);
        stageCosts = trackingCost.linearize(states, controls);
      }
      if (accepted || damping > 0.0) {
        auto finalSweep = internal::backwardILQR(stageCosts, lin, 0.0);
        if (!finalSweep.gains.empty()) sweep = std::move(finalSweep);
      }
      states_ = std::move(states);
      policy_ = std::move(sweep.policy);
      gains_ = std::move(sweep.gains);
      return trackingCost.trajectoryValues(states_, controls);
    }
  }

  /** Optimize trajectory Values and GaussianBayesNet policy using a discrete
   * transition x_{k+1} = f(x_k, u_k) differentiated by central differences in
   * local tangent coordinates.
   */
  Values optimize(const NonlinearFactorGraph& cost, const Values& initial,
                  const Transition& transition, size_t iterations,
                  const Vector& lower = {}, const Vector& upper = {}) {
    if (!transition) throw std::invalid_argument("Invalid iLQR transition.");
    const LinearizedTransition fdTransition =
        [&transition](const State& x, const Control& u, OptionalMatrix Hx,
                      OptionalMatrix Hu) -> State {
      const Eigen::Index n =
          static_cast<Eigen::Index>(traits<State>::GetDimension(x));
      const Eigen::Index m = u.size();
      const State next = transition(x, u);
      if (Hx) {
        Matrix A(n, n);
        for (Eigen::Index i = 0; i < n; ++i) {
          double eps = 1e-5;
          if constexpr (std::is_same_v<State, Vector>)
            eps = 1e-5 * (1.0 + std::abs(x[i]));
          Vector dx = Vector::Zero(n);
          dx[i] = eps;
          const State plus = transition(traits<State>::Retract(x, dx), u);
          const State minus = transition(traits<State>::Retract(x, -dx), u);
          A.col(i) = (traits<State>::Local(next, plus) -
                      traits<State>::Local(next, minus)) /
                     (2.0 * eps);
        }
        *Hx = std::move(A);
      }
      if (Hu) {
        Matrix B(n, m);
        for (Eigen::Index i = 0; i < m; ++i) {
          const double eps = 1e-5 * (1.0 + std::abs(u[i]));
          Vector du = Vector::Zero(m);
          du[i] = eps;
          const State plus = transition(x, u + du);
          const State minus = transition(x, u - du);
          B.col(i) = (traits<State>::Local(next, plus) -
                      traits<State>::Local(next, minus)) /
                     (2.0 * eps);
        }
        *Hu = std::move(B);
      }
      return next;
    };
    return optimize(cost, initial, fdTransition, iterations, lower, upper);
  }

  /** Optimize trajectory Values and GaussianBayesNet policy using a
   * continuous-time vector field xi = vectorField(x, u) integrated via
   * traits<State>::Retract(x, dt * xi) (matching SamplingPlanner).
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

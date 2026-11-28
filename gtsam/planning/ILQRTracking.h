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
#include <gtsam/base/numericalDerivative.h>
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

/// Quadratic stage costs and the linearized deterministic transitions.
struct ILQRLinearization {
  StageLinearizedCosts costs;
  std::vector<Matrix> A, B;

  /// Eliminate the controls in reverse time to obtain P(du_k | dx_k).
  GaussianBayesNet eliminateSequential(double damping) const {
    return eliminateTrackingChain(costs, [&](size_t k, HessianFactor& stage,
                                             const HessianFactor& valueToGo) {
      return eliminateControlStage(A[k], B[k], stage, damping, valueToGo);
    });
  }
};

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
  using typename Base::Objective;
  using typename Base::Trajectory;

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
    const auto step = [&](const State& x, const Control& u) {
      return transition(x, u, {}, {});
    };
    auto nominal = this->initialTrajectory(initial, lower, upper, step);
    const auto objective = this->prepareObjective(cost, initial, nominal);
    auto linear = linearize(objective, nominal, transition);
    double damping = 0.0;
    for (size_t i = 0;; ++i) {
      auto policy = internal::eliminateWithDamping(
          damping, 1e6,
          [&](double reg) { return linear.eliminateSequential(reg); },
          increaseDamping);
      if (policy.empty())
        throw std::domain_error("iLQR control Hessian remained indefinite.");
      auto law = internal::solveConditionals(policy);
      bool refreshPolicy = damping > 0.0;
      if (i < iterations && law.feedforward.norm() > 1e-9) {
        const double previous = nominal.error;
        if (this->lineSearch(nominal, objective, law, step, lower, upper)) {
          damping = damping > 1e-9 ? 0.5 * damping : 0.0;
          linear = linearize(objective, nominal, transition);
          if (previous - nominal.error > 1e-12 * (1.0 + nominal.error))
            continue;
          refreshPolicy =
              true;  // Return the policy at the accepted trajectory.
        } else if (damping < 1e6) {
          damping = increaseDamping(damping);
          continue;
        }
      }
      if (refreshPolicy) {
        auto finalPolicy = linear.eliminateSequential(0.0);
        if (!finalPolicy.empty()) {
          policy = std::move(finalPolicy);
          law = internal::solveConditionals(policy);
        }
      }
      return this->finish(std::move(nominal), policy, std::move(law),
                          objective);
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
    const LinearizedTransition fdTransition = [&](const State& x,
                                                  const Control& u,
                                                  OptionalMatrix Hx,
                                                  OptionalMatrix Hu) {
      const State next = transition(x, u);
      const auto n = static_cast<Eigen::Index>(traits<State>::GetDimension(x));
      if (Hx) {
        Vector eps = Vector::Constant(n, 1e-5);
        if constexpr (std::is_same_v<State, Vector>)
          eps = 1e-5 * (1.0 + x.array().abs());
        *Hx = numericalDerivativeAt(next, eps, [&](const Vector& dx) {
          return transition(traits<State>::Retract(x, dx), u);
        });
      }
      if (Hu) {
        const Vector eps = 1e-5 * (1.0 + u.array().abs());
        *Hu = numericalDerivativeAt(
            next, eps, [&](const Vector& du) { return transition(x, u + du); });
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
    return optimize(cost, initial, Base::discretize(vectorField, dt),
                    iterations, lower, upper);
  }

 private:
  /// Start regularization at 1e-4 and increase it tenfold on retries.
  static double increaseDamping(double damping) {
    return damping == 0.0 ? 1e-4 : 10.0 * damping;
  }

  /// Linearize the transition and stage factors about the same nominal
  /// trajectory.
  static internal::ILQRLinearization linearize(
      const Objective& objective, const Trajectory& nominal,
      const LinearizedTransition& transition) {
    const auto horizon = nominal.states.size() - 1;
    const auto n = static_cast<Eigen::Index>(
        traits<State>::GetDimension(nominal.states.front()));
    const auto m = nominal.controls.rows();
    std::vector<Matrix> A(horizon), B(horizon);
    for (size_t k = 0; k < horizon; ++k) {
      A[k].resize(n, n);
      B[k].resize(n, m);
      transition(nominal.states[k], nominal.controls.col(k), A[k], B[k]);
      if (A[k].rows() != n || A[k].cols() != n || B[k].rows() != n ||
          B[k].cols() != m || !A[k].allFinite() || !B[k].allFinite())
        throw std::domain_error("Invalid iLQR Jacobian.");
    }
    return {objective.linearize(nominal.states, nominal.controls), std::move(A),
            std::move(B)};
  }
};

}  // namespace gtsam

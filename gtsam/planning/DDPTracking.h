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
  using typename Base::Trajectory;

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
    auto nominal = this->initialTrajectory(initial, lower, upper, transition);
    const auto objective = this->prepareObjective(cost, initial, nominal);
    constexpr double minDamping = 1e-9;
    double damping = minDamping;
    for (size_t i = 0;; ++i) {
      const auto costs = objective.linearize(nominal.states, nominal.controls);
      auto eliminate = [&](double reg) {
        return eliminateSequential(costs, nominal, transition, reg);
      };
      auto policy = internal::eliminateWithDamping(damping, 1e12, eliminate,
                                                   increaseDamping);
      if (policy.empty())
        throw std::domain_error("DDP control Hessian remained indefinite.");
      auto law = internal::solveConditionals(policy);
      if (i < iterations && law.feedforward.norm() > 1e-10 &&
          this->lineSearch(nominal, objective, law, transition, lower, upper)) {
        damping = std::max(minDamping, 0.5 * damping);
        continue;
      }
      if (damping > minDamping) {
        double finalDamping = minDamping;
        auto finalPolicy = internal::eliminateWithDamping(
            finalDamping, 1e12, eliminate, increaseDamping);
        if (!finalPolicy.empty()) {
          policy = std::move(finalPolicy);
          law = internal::solveConditionals(policy);
        }
      }
      return this->finish(std::move(nominal), policy, std::move(law),
                          objective);
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
    return optimize(cost, initial, Base::discretize(vectorField, dt),
                    iterations, lower, upper);
  }

 private:
  /// Increase regularization when a DDP control Hessian is indefinite.
  static double increaseDamping(double damping) {
    return std::max(1e-6, 10.0 * damping);
  }

  /// Jacobian of the transition and its Hessian contracted with the value
  /// gradient.
  struct DynamicsDerivatives {
    Matrix jacobian, curvature;
  };

  /// Differentiate the dynamics in tangent coordinates at one nominal step.
  static DynamicsDerivatives linearizeDynamics(const State& xk,
                                               const Control& uk,
                                               const State& center,
                                               const Transition& transition,
                                               const Vector& valueGrad) {
    const Eigen::Index n = valueGrad.size(), m = uk.size(), d = n + m;
    Vector delta(d);
    if constexpr (std::is_same_v<State, Vector>)
      delta.head(n) = 1e-4 * (1.0 + xk.array().abs());
    else
      delta.head(n).setConstant(1e-4);
    delta.tail(m) = 1e-4 * (1.0 + uk.array().abs());

    // Evaluate local tangent displacement relative to nominal x_{k+1}.
    auto fLocal = [&](const Vector& dz) -> Vector {
      const State next =
          transition(traits<State>::Retract(xk, dz.head(n)), uk + dz.tail(m));
      return traits<State>::Local(center, next);
    };

    // Reuse one perturbation for the diagonal and four-point mixed stencils.
    Matrix jacobian(n, d), secondOrder(d, d);
    Vector shift = Vector::Zero(d);
    for (Eigen::Index i = 0; i < d; ++i) {
      shift[i] = delta[i];
      const Vector plus = fLocal(shift);
      shift[i] = -delta[i];
      const Vector minus = fLocal(shift);
      jacobian.col(i) = (plus - minus) / (2.0 * delta[i]);
      secondOrder(i, i) = valueGrad.dot(plus + minus) / (delta[i] * delta[i]);
      for (Eigen::Index j = 0; j < i; ++j) {
        const auto sample = [&](double di, double dj) {
          shift[i] = di;
          shift[j] = dj;
          return fLocal(shift);
        };
        const Vector mixed =
            sample(delta[i], delta[j]) - sample(delta[i], -delta[j]) -
            sample(-delta[i], delta[j]) + sample(-delta[i], -delta[j]);
        secondOrder(i, j) = secondOrder(j, i) =
            valueGrad.dot(mixed) / (4.0 * delta[i] * delta[j]);
        shift[j] = 0.0;
      }
      shift[i] = 0.0;
    }
    return {std::move(jacobian), std::move(secondOrder)};
  }

  /** Reverse-time elimination supplies each stage with the gradient of its
   * incoming cost-to-go, which weights the second-order dynamics term in DDP.
   */
  static GaussianBayesNet eliminateSequential(
      const internal::StageLinearizedCosts& costs, const Trajectory& nominal,
      const Transition& transition, double damping) {
    return internal::eliminateTrackingChain(
        costs,
        [&](size_t k, HessianFactor& stage, const HessianFactor& valueToGo) {
          const auto dynamics = linearizeDynamics(
              nominal.states[k], nominal.controls.col(k), nominal.states[k + 1],
              transition, -valueToGo.linearTerm());
          const auto n = dynamics.jacobian.rows(), m = nominal.controls.rows();
          return internal::eliminateControlStage(
              dynamics.jacobian.leftCols(n), dynamics.jacobian.rightCols(m),
              stage, damping, valueToGo, &dynamics.curvature);
        });
  }
};

}  // namespace gtsam

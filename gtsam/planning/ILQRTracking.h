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
#include <gtsam/planning/TrackingFeedback.h>

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
/// using pullBackDynamics + eliminateControlFactor, returning a
/// GaussianBayesNet policy P(du_k | dx_k).
GTSAM_EXPORT TrackingSweep backwardILQR(const StageLinearizedCosts& costs,
                                        const KeyVector& stateKeys,
                                        const KeyVector& controlKeys,
                                        const ILQRLinearization& lin,
                                        double damping);

}  // namespace internal

/**
 * Manifold-generic factor-graph TVLQR / iLQR for trajectory optimization and
 * tracking on any GTSAM Manifold M (including Vector, Pose2, Pose3, Rot3,
 * Unit3), aligned with ManifoldEKF<M> and SamplingPlanner<State, Control>.
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
 *   traits<M>::Retract(x, dt * f(x, u)) (matching SamplingPlanner).
 */
template <class M = Vector>
class ILQRTracking {
  static_assert(IsManifold<M>::value,
                "Template parameter M must be a GTSAM Manifold.");

 public:
  using State = M;
  using Transition = std::function<M(const M&, const Vector&)>;
  using VectorField = std::function<Vector(const M&, const Vector&)>;
  using OptionalMatrix = OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic>;
  using LinearizedTransition =
      std::function<M(const M&, const Vector&, OptionalMatrix, OptionalMatrix)>;

  /// Associate the horizon with ordered state (H+1) and control (H) keys
  /// (matching SamplingPlanner).
  ILQRTracking(KeyVector stateKeys, KeyVector controlKeys)
      : stateKeys_(std::move(stateKeys)),
        controlKeys_(std::move(controlKeys)),
        keyMap_(internal::buildStageKeyMap(stateKeys_, controlKeys_)) {}

  /** Optimize trajectory Values and GaussianBayesNet policy for a
   * NonlinearFactorGraph objective using an analytic OptionalJacobian
   * transition f(x_k, u_k, Hx, Hu).
   */
  Values optimize(const NonlinearFactorGraph& cost, const Values& initial,
                  const LinearizedTransition& transition, size_t iterations,
                  const Vector& lower = {}, const Vector& upper = {}) {
    if (!transition) throw std::invalid_argument("Invalid iLQR transition.");
    const size_t h = controlKeys_.size();
    const M x0 = initial.at<M>(stateKeys_.front());
    const Eigen::Index n =
        static_cast<Eigen::Index>(traits<M>::GetDimension(x0));
    const Eigen::Index m = initial.at<Vector>(controlKeys_.front()).size();
    if constexpr (std::is_same_v<M, Vector>) {
      if (!x0.allFinite())
        throw std::invalid_argument("Nonfinite initial iLQR state.");
    }
    internal::validateControlLimits(m, lower, upper);

    Matrix controls(m, static_cast<Eigen::Index>(h));
    for (size_t k = 0; k < h; ++k) {
      const Vector uk = initial.at<Vector>(controlKeys_[k]);
      if (uk.size() != m || !uk.allFinite())
        throw std::invalid_argument("Invalid initial iLQR control.");
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
        M next = transition(statesOut[k], u.col(col), {}, {});
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

    auto linearizeDynamics = [&](const std::vector<M>& x, const Matrix& u) {
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

    std::vector<M> states;
    Values values = rolloutValues(controls, nullptr, nullptr, 0.0, states);
    double objective = cost.error(values);
    if (!std::isfinite(objective))
      throw std::domain_error("Nonfinite initial iLQR rollout.");
    double damping = 0.0;
    auto lin = linearizeDynamics(states, controls);
    auto stageCosts = internal::linearizeStageCosts(cost, values, stateKeys_,
                                                    controlKeys_, keyMap_, n);

    for (size_t i = 0;; ++i) {
      auto sweep = internal::backwardILQR(stageCosts, stateKeys_, controlKeys_,
                                          lin, damping);
      while (sweep.gains.empty() && damping < 1e6) {
        damping = damping == 0.0 ? 1e-4 : 10.0 * damping;
        sweep = internal::backwardILQR(stageCosts, stateKeys_, controlKeys_,
                                       lin, damping);
      }
      if (sweep.gains.empty())
        throw std::domain_error("iLQR control Hessian remained indefinite.");

      bool accepted = false;
      bool converged = false;
      if (i < iterations && sweep.feedforward.norm() > 1e-9) {
        for (int step = 0; step < 8; ++step) {
          const double alpha = std::ldexp(1.0, -step);
          Matrix trial = controls;
          std::vector<M> trialStates;
          Values trialValues =
              rolloutValues(trial, &states, &sweep, alpha, trialStates);
          const double value = cost.error(trialValues);
          if (std::isfinite(value) && value < objective) {
            const double decrease = objective - value;
            controls = std::move(trial);
            states = std::move(trialStates);
            values = std::move(trialValues);
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
        stageCosts = internal::linearizeStageCosts(cost, values, stateKeys_,
                                                   controlKeys_, keyMap_, n);
        continue;
      }

      if (accepted) {
        lin = linearizeDynamics(states, controls);
        stageCosts = internal::linearizeStageCosts(cost, values, stateKeys_,
                                                   controlKeys_, keyMap_, n);
      }
      if (accepted || damping > 0.0) {
        auto finalSweep = internal::backwardILQR(stageCosts, stateKeys_,
                                                 controlKeys_, lin, 0.0);
        if (!finalSweep.gains.empty()) sweep = std::move(finalSweep);
      }
      states_ = std::move(states);
      policy_ = std::move(sweep.policy);
      gains_ = std::move(sweep.gains);
      return values;
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
        [&transition](const M& x, const Vector& u, OptionalMatrix Hx,
                      OptionalMatrix Hu) -> M {
      const Eigen::Index n =
          static_cast<Eigen::Index>(traits<M>::GetDimension(x));
      const Eigen::Index m = u.size();
      const M next = transition(x, u);
      if (Hx) {
        Matrix A(n, n);
        for (Eigen::Index i = 0; i < n; ++i) {
          double eps = 1e-5;
          if constexpr (std::is_same_v<M, Vector>)
            eps = 1e-5 * (1.0 + std::abs(x[i]));
          Vector dx = Vector::Zero(n);
          dx[i] = eps;
          const M plus = transition(traits<M>::Retract(x, dx), u);
          const M minus = transition(traits<M>::Retract(x, -dx), u);
          A.col(i) =
              (traits<M>::Local(next, plus) - traits<M>::Local(next, minus)) /
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
          const M plus = transition(x, u + du);
          const M minus = transition(x, u - du);
          B.col(i) =
              (traits<M>::Local(next, plus) - traits<M>::Local(next, minus)) /
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
   * traits<M>::Retract(x, dt * xi) (matching SamplingPlanner).
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

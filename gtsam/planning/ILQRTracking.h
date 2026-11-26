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
#include <gtsam/base/VectorSpace.h>
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

/// Reverse-time factor-graph iLQR backward pass in local tangent coordinates
/// using GTSAM's HessianFactor::eliminateCholesky. When errorJacobians is
/// non-empty (size H+1), each C_k maps perturbations in T_{x_k} M to
/// perturbations of Local(target_k, x_k).
GTSAM_EXPORT TrackingSweep
backwardILQR(const Matrix& stateErrors, const Matrix& controlErrors,
             const Vector& terminalError, const ILQRLinearization& lin,
             const Matrix& q, const Matrix& r, const Matrix& terminal,
             double damping, const std::vector<Matrix>& errorJacobians = {});

}  // namespace internal

/**
 * Manifold-generic factor-graph TVLQR / iLQR for trajectory tracking on any
 * GTSAM Manifold M (including Vector, Pose2, Pose3, Rot3, Unit3), mirroring
 * ManifoldEKF<M> and SamplingPlanner<M, Control>.
 *
 * State errors live in the tangent space via traits<M>::Local(target, state),
 * perturbations apply via traits<M>::Retract(state, delta), and each backward
 * pass eliminates the control block using HessianFactor::eliminateCholesky.
 * For vector-space types (such as Vector), the chart Jacobian is identically I
 * and skipped at compile time.
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
  static constexpr bool kIsVectorSpace =
      std::is_base_of_v<vector_space_tag,
                        typename traits<M>::structure_category>;

 public:
  using State = M;
  using Transition = std::function<M(const M&, const Vector&)>;
  using VectorField = std::function<Vector(const M&, const Vector&)>;
  using OptionalMatrix = OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic>;
  using LinearizedTransition =
      std::function<M(const M&, const Vector&, OptionalMatrix, OptionalMatrix)>;

  /// Supply running state/control and terminal state precision matrices in
  /// local tangent coordinates.
  ILQRTracking(const Matrix& statePrecision, const Matrix& controlPrecision,
               const Matrix& terminalPrecision)
      : q_(statePrecision), r_(controlPrecision), terminal_(terminalPrecision) {
    internal::checkTrackingPrecision(q_, false);
    internal::checkTrackingPrecision(r_, true);
    internal::checkTrackingPrecision(terminal_, false);
    if (q_.rows() != terminal_.rows())
      throw std::invalid_argument("Inconsistent state precisions.");
  }

  /** Fit H controls and H tangent-space gains to H+1 target states on M using
   * an analytic linearized discrete transition f(x_k, u_k, Hx, Hu).
   */
  Matrix optimize(const M& initial, const std::vector<M>& targetStates,
                  const Matrix& targetControls,
                  const LinearizedTransition& transition, size_t iterations,
                  const Vector& lower = {}, const Vector& upper = {}) {
    const Eigen::Index h = targetControls.cols(),
                       n = static_cast<Eigen::Index>(
                           traits<M>::GetDimension(initial)),
                       m = targetControls.rows();
    if (!h || n != q_.rows() || m != r_.rows() ||
        targetStates.size() != static_cast<size_t>(h + 1) ||
        !targetControls.allFinite() || !transition)
      throw std::invalid_argument(
          "Invalid iLQR horizon, targets or transition.");
    if constexpr (std::is_same_v<M, Vector>) {
      if (!initial.allFinite())
        throw std::invalid_argument("Nonfinite initial iLQR state.");
      for (const Vector& target : targetStates)
        if (target.size() != n || !target.allFinite())
          throw std::invalid_argument("Invalid iLQR target state.");
    }
    internal::validateControlLimits(m, lower, upper);

    auto rollout = [&](Matrix& u, const std::vector<M>* base,
                       const internal::TrackingSweep* sweep, double alpha) {
      std::vector<M> x;
      x.reserve(static_cast<size_t>(h + 1));
      x.push_back(initial);
      for (Eigen::Index k = 0; k < h; ++k) {
        const auto idx = static_cast<size_t>(k);
        if (sweep && base) {
          const Vector dx = traits<M>::Local((*base)[idx], x[idx]);
          u.col(k) +=
              alpha * sweep->feedforward.col(k) + sweep->gains[idx] * dx;
        }
        if (lower.size()) u.col(k) = u.col(k).cwiseMax(lower);
        if (upper.size()) u.col(k) = u.col(k).cwiseMin(upper);
        M next = transition(x[idx], u.col(k), {}, {});
        if constexpr (std::is_same_v<M, Vector>) {
          if (next.size() != n)
            throw std::invalid_argument(
                "Tracking transition has wrong dimension.");
        }
        x.push_back(std::move(next));
      }
      return x;
    };

    auto errorsAndCost = [&](const std::vector<M>& x, const Matrix& u,
                             Matrix& stateErr, Vector& termErr) {
      stateErr.resize(n, h);
      for (Eigen::Index k = 0; k < h; ++k)
        stateErr.col(k) = traits<M>::Local(targetStates[static_cast<size_t>(k)],
                                           x[static_cast<size_t>(k)]);
      termErr = traits<M>::Local(targetStates[static_cast<size_t>(h)],
                                 x[static_cast<size_t>(h)]);
      const Matrix du = u - targetControls;
      return 0.5 * ((stateErr.array() * (q_ * stateErr).array()).sum() +
                    (du.array() * (r_ * du).array()).sum() +
                    termErr.dot(terminal_ * termErr));
    };

    auto linearize = [&](const std::vector<M>& x, const Matrix& u) {
      internal::ILQRLinearization lin{
          std::vector<Matrix>(static_cast<size_t>(h)),
          std::vector<Matrix>(static_cast<size_t>(h))};
      for (Eigen::Index k = 0; k < h; ++k) {
        const auto idx = static_cast<size_t>(k);
        Matrix A(n, n), B(n, m);
        transition(x[idx], u.col(k), A, B);
        if (A.rows() != n || A.cols() != n || B.rows() != n || B.cols() != m ||
            !A.allFinite() || !B.allFinite())
          throw std::domain_error("Invalid iLQR Jacobian.");
        lin.A[idx] = std::move(A);
        lin.B[idx] = std::move(B);
      }
      return lin;
    };

    auto computeErrorJacobians = [&](const std::vector<M>& x) {
      std::vector<Matrix> C;
      if constexpr (!kIsVectorSpace) {
        C.resize(static_cast<size_t>(h + 1));
        constexpr double eps = 1e-5;
        for (Eigen::Index k = 0; k <= h; ++k) {
          const auto idx = static_cast<size_t>(k);
          Matrix Ck(n, n);
          for (Eigen::Index i = 0; i < n; ++i) {
            Vector dx = Vector::Zero(n);
            dx[i] = eps;
            const Vector plus = traits<M>::Local(
                targetStates[idx], traits<M>::Retract(x[idx], dx));
            const Vector minus = traits<M>::Local(
                targetStates[idx], traits<M>::Retract(x[idx], -dx));
            Ck.col(i) = (plus - minus) / (2.0 * eps);
          }
          C[idx] = std::move(Ck);
        }
      }
      return C;
    };

    Matrix controls = targetControls;
    std::vector<M> states = rollout(controls, nullptr, nullptr, 0.0);
    Matrix stateErr;
    Vector termErr;
    double objective = errorsAndCost(states, controls, stateErr, termErr);
    if (!std::isfinite(objective))
      throw std::domain_error("Nonfinite initial iLQR rollout.");
    double damping = 0.0;
    auto lin = linearize(states, controls);
    auto errJac = computeErrorJacobians(states);

    for (size_t i = 0;; ++i) {
      const Matrix controlErr = controls - targetControls;
      auto sweep = internal::backwardILQR(stateErr, controlErr, termErr, lin,
                                          q_, r_, terminal_, damping, errJac);
      while (sweep.gains.empty() && damping < 1e6) {
        damping = damping == 0.0 ? 1e-4 : 10.0 * damping;
        sweep = internal::backwardILQR(stateErr, controlErr, termErr, lin, q_,
                                       r_, terminal_, damping, errJac);
      }
      if (sweep.gains.empty())
        throw std::domain_error("iLQR control Hessian remained indefinite.");

      bool accepted = false;
      bool converged = false;
      if (i < iterations && sweep.feedforward.norm() > 1e-9) {
        for (int step = 0; step < 8; ++step) {
          const double alpha = std::ldexp(1.0, -step);
          Matrix trial = controls;
          std::vector<M> x = rollout(trial, &states, &sweep, alpha);
          Matrix trialErr;
          Vector trialTerm;
          const double value = errorsAndCost(x, trial, trialErr, trialTerm);
          if (std::isfinite(value) && value < objective) {
            const double decrease = objective - value;
            controls = std::move(trial);
            states = std::move(x);
            stateErr = std::move(trialErr);
            termErr = std::move(trialTerm);
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
        lin = linearize(states, controls);
        errJac = computeErrorJacobians(states);
        continue;
      }

      if (accepted) {
        lin = linearize(states, controls);
        errJac = computeErrorJacobians(states);
      }
      if (accepted || damping > 0.0) {
        auto finalSweep =
            internal::backwardILQR(stateErr, controls - targetControls, termErr,
                                   lin, q_, r_, terminal_, 0.0, errJac);
        if (!finalSweep.gains.empty()) sweep = std::move(finalSweep);
      }
      states_ = std::move(states);
      gains_ = std::move(sweep.gains);
      return controls;
    }
  }

  /** Fit H controls and H gains using a discrete transition x_{k+1} = f(x_k,
   * u_k) differentiated by central differences in local tangent coordinates.
   */
  Matrix optimize(const M& initial, const std::vector<M>& targetStates,
                  const Matrix& targetControls, const Transition& transition,
                  size_t iterations, const Vector& lower = {},
                  const Vector& upper = {}) {
    if (!transition)
      throw std::invalid_argument(
          "Invalid iLQR horizon, targets or transition.");
    const LinearizedTransition fdTransition = [&](const M& x, const Vector& u,
                                                  OptionalMatrix Hx,
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
    return optimize(initial, targetStates, targetControls, fdTransition,
                    iterations, lower, upper);
  }

  /** Fit H controls and H gains using a continuous-time vector field
   * xi = vectorField(x, u) integrated via traits<M>::Retract(x, dt * xi)
   * (matching SamplingPlanner).
   */
  Matrix optimize(const M& initial, const std::vector<M>& targetStates,
                  const Matrix& targetControls, const VectorField& vectorField,
                  double dt, size_t iterations, const Vector& lower = {},
                  const Vector& upper = {}) {
    if (!vectorField || !std::isfinite(dt) || !(dt > 0.0))
      throw std::invalid_argument(
          "Expected a valid vector field and positive finite dt.");
    const Transition discrete = [vectorField, dt](const M& x,
                                                  const Vector& u) -> M {
      const Vector xi = vectorField(x, u);
      if (static_cast<size_t>(xi.size()) != traits<M>::GetDimension(x))
        throw std::invalid_argument(
            "Vector field returned the wrong tangent dimension.");
      return traits<M>::Retract(x, dt * xi);
    };
    return optimize(initial, targetStates, targetControls, discrete, iterations,
                    lower, upper);
  }

  /// Convenience Matrix-column target overloads when M = Vector.
  template <class T = M, std::enable_if_t<std::is_same_v<T, Vector>, int> = 0>
  Matrix optimize(const Vector& initial, const Matrix& targetStates,
                  const Matrix& targetControls,
                  const LinearizedTransition& transition, size_t iterations,
                  const Vector& lower = {}, const Vector& upper = {}) {
    return optimize(initial, matrixToVectorStates(targetStates), targetControls,
                    transition, iterations, lower, upper);
  }

  template <class T = M, std::enable_if_t<std::is_same_v<T, Vector>, int> = 0>
  Matrix optimize(const Vector& initial, const Matrix& targetStates,
                  const Matrix& targetControls, const Transition& transition,
                  size_t iterations, const Vector& lower = {},
                  const Vector& upper = {}) {
    return optimize(initial, matrixToVectorStates(targetStates), targetControls,
                    transition, iterations, lower, upper);
  }

  template <class T = M, std::enable_if_t<std::is_same_v<T, Vector>, int> = 0>
  Matrix optimize(const Vector& initial, const Matrix& targetStates,
                  const Matrix& targetControls, const VectorField& vectorField,
                  double dt, size_t iterations, const Vector& lower = {},
                  const Vector& upper = {}) {
    return optimize(initial, matrixToVectorStates(targetStates), targetControls,
                    vectorField, dt, iterations, lower, upper);
  }

  /// Optimized state trajectory on M ordered in time.
  const std::vector<M>& states() const { return states_; }

  /// Convenience Matrix-column view of states() when M = Vector.
  template <class T = M, std::enable_if_t<std::is_same_v<T, Vector>, int> = 0>
  Matrix stateMatrix() const {
    if (states_.empty()) return Matrix();
    Matrix out(states_.front().size(),
               static_cast<Eigen::Index>(states_.size()));
    for (size_t k = 0; k < states_.size(); ++k)
      out.col(static_cast<Eigen::Index>(k)) = states_[k];
    return out;
  }

  /// Gain multiplying traits<M>::Local(reference, actual) at a given step.
  const Matrix& gain(size_t step) const { return gains_.at(step); }

 private:
  static std::vector<Vector> matrixToVectorStates(const Matrix& cols) {
    std::vector<Vector> out;
    out.reserve(static_cast<size_t>(cols.cols()));
    for (Eigen::Index k = 0; k < cols.cols(); ++k)
      out.emplace_back(cols.col(k));
    return out;
  }

  Matrix q_, r_, terminal_;
  std::vector<M> states_;
  std::vector<Matrix> gains_;
};

}  // namespace gtsam

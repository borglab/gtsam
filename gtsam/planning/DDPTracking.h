// Copyright (c) Georgia Tech Research Corporation. See LICENSE for details.
/**
 * @file DDPTracking.h
 * @author Frank Dellaert
 * @author Akash Ratheesh
 */
#pragma once

#include <gtsam/base/Manifold.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/VectorSpace.h>
#include <gtsam/planning/TrackingFeedback.h>

#include <Eigen/Cholesky>
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
 * tracking on any GTSAM Manifold M (including Vector, Pose2, Pose3, Rot3,
 * Unit3), sharing the exact same API as ILQRTracking<M>.
 */
template <class M = Vector>
class DDPTracking {
  static_assert(IsManifold<M>::value,
                "Template parameter M must be a GTSAM Manifold.");
  static constexpr bool kIsVectorSpace =
      std::is_base_of_v<vector_space_tag,
                        typename traits<M>::structure_category>;

 public:
  using State = M;
  using Transition = std::function<M(const M&, const Vector&)>;
  using VectorField = std::function<Vector(const M&, const Vector&)>;

  /// Supply running state/control and terminal state precision matrices in
  /// local tangent coordinates.
  DDPTracking(const Matrix& statePrecision, const Matrix& controlPrecision,
              const Matrix& terminalPrecision)
      : q_(statePrecision), r_(controlPrecision), terminal_(terminalPrecision) {
    internal::checkTrackingPrecision(q_, false);
    internal::checkTrackingPrecision(r_, true);
    internal::checkTrackingPrecision(terminal_, false);
    if (q_.rows() != terminal_.rows())
      throw std::invalid_argument("Inconsistent state precisions.");
  }

  /** Fit H controls and H tangent-space gains to H+1 target states on M using
   * a discrete transition x_{k+1} = f(x_k, u_k).
   */
  Matrix optimize(const M& initial, const std::vector<M>& targetStates,
                  const Matrix& targetControls, const Transition& transition,
                  size_t iterations, const Vector& lower = {},
                  const Vector& upper = {}) {
    const Eigen::Index h = targetControls.cols(),
                       n = static_cast<Eigen::Index>(
                           traits<M>::GetDimension(initial)),
                       m = targetControls.rows(), d = n + m;
    if (!h || n != q_.rows() || m != r_.rows() ||
        targetStates.size() != static_cast<size_t>(h + 1) ||
        !targetControls.allFinite() || !transition)
      throw std::invalid_argument(
          "Invalid DDP horizon, targets or transition.");
    if constexpr (std::is_same_v<M, Vector>) {
      if (!initial.allFinite())
        throw std::invalid_argument("Nonfinite initial DDP state.");
      for (const Vector& target : targetStates)
        if (target.size() != n || !target.allFinite())
          throw std::invalid_argument("Invalid DDP target state.");
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
        M next = transition(x[idx], u.col(k));
        if constexpr (std::is_same_v<M, Vector>) {
          if (next.size() != n)
            throw std::invalid_argument(
                "Tracking transition has wrong dimension.");
        }
        x.push_back(std::move(next));
      }
      return x;
    };

    auto cost = [&](const std::vector<M>& x, const Matrix& u) {
      Matrix stateErr(n, h);
      for (Eigen::Index k = 0; k < h; ++k)
        stateErr.col(k) = traits<M>::Local(targetStates[static_cast<size_t>(k)],
                                           x[static_cast<size_t>(k)]);
      const Vector end = traits<M>::Local(targetStates[static_cast<size_t>(h)],
                                          x[static_cast<size_t>(h)]);
      const Matrix du = u - targetControls;
      return 0.5 * ((stateErr.array() * (q_ * stateErr).array()).sum() +
                    (du.array() * (r_ * du).array()).sum() +
                    end.dot(terminal_ * end));
    };

    auto errorJacobian = [&](const M& target, const M& x) {
      Matrix C(n, n);
      constexpr double eps = 1e-5;
      for (Eigen::Index i = 0; i < n; ++i) {
        Vector dx = Vector::Zero(n);
        dx[i] = eps;
        const Vector plus = traits<M>::Local(target, traits<M>::Retract(x, dx));
        const Vector minus =
            traits<M>::Local(target, traits<M>::Retract(x, -dx));
        C.col(i) = (plus - minus) / (2.0 * eps);
      }
      return C;
    };

    auto backward = [&](const std::vector<M>& states, const Matrix& u,
                        double reg) -> internal::TrackingSweep {
      Vector gradient;
      Matrix curvature;
      const Vector termErr = traits<M>::Local(
          targetStates[static_cast<size_t>(h)], states[static_cast<size_t>(h)]);
      if constexpr (!kIsVectorSpace) {
        const Matrix Cterm = errorJacobian(targetStates[static_cast<size_t>(h)],
                                           states[static_cast<size_t>(h)]);
        gradient = Cterm.transpose() * terminal_ * termErr;
        curvature = Cterm.transpose() * terminal_ * Cterm;
      } else {
        gradient = terminal_ * termErr;
        curvature = terminal_;
      }
      internal::TrackingSweep sweep{
          Matrix::Zero(m, h), std::vector<Matrix>(static_cast<size_t>(h))};

      for (Eigen::Index k = h - 1; k >= 0; --k) {
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

        Matrix jacobian(n, d), hessian = Matrix::Zero(d, d);
        for (Eigen::Index i = 0; i < d; ++i) {
          Vector shift = Vector::Zero(d);
          shift[i] = delta[i];
          const Vector plus = fLocal(shift), minus = fLocal(-shift);
          jacobian.col(i) = (plus - minus) / (2.0 * delta[i]);
          hessian(i, i) = gradient.dot(plus + minus) / (delta[i] * delta[i]);
          for (Eigen::Index j = 0; j < i; ++j) {
            Vector cross = Vector::Zero(d);
            cross[j] = delta[j];
            const Vector mixed = fLocal(shift + cross) - fLocal(shift - cross) -
                                 fLocal(-shift + cross) +
                                 fLocal(-shift - cross);
            hessian(i, j) = hessian(j, i) =
                gradient.dot(mixed) / (4.0 * delta[i] * delta[j]);
          }
        }

        hessian += jacobian.transpose() * curvature * jacobian;
        hessian.bottomRightCorner(m, m) += r_;
        Vector first = jacobian.transpose() * gradient;
        const Vector errk = traits<M>::Local(targetStates[idx], xk);
        if constexpr (!kIsVectorSpace) {
          const Matrix Ck = errorJacobian(targetStates[idx], xk);
          hessian.topLeftCorner(n, n) += Ck.transpose() * q_ * Ck;
          first.head(n) += Ck.transpose() * q_ * errk;
        } else {
          hessian.topLeftCorner(n, n) += q_;
          first.head(n) += q_ * errk;
        }
        first.tail(m) += r_ * (uk - targetControls.col(k));

        Matrix uu = hessian.bottomRightCorner(m, m);
        uu.diagonal().array() += reg;
        Eigen::LLT<Matrix> factor(uu);
        if (factor.info() != Eigen::Success) return {};
        const Matrix ux = hessian.bottomLeftCorner(m, n);
        Matrix gain = factor.solve(-ux);
        Vector du = factor.solve(-first.tail(m));

        gradient = first.head(n) + gain.transpose() * first.tail(m) +
                   ux.transpose() * du +
                   gain.transpose() * hessian.bottomRightCorner(m, m) * du;
        curvature = hessian.topLeftCorner(n, n) + gain.transpose() * ux +
                    ux.transpose() * gain +
                    gain.transpose() * hessian.bottomRightCorner(m, m) * gain;
        curvature = (0.5 * (curvature + curvature.transpose())).eval();
        sweep.gains[idx] = std::move(gain);
        sweep.feedforward.col(k) = du;
      }
      return sweep;
    };

    Matrix controls = targetControls;
    std::vector<M> states = rollout(controls, nullptr, nullptr, 0.0);
    double objective = cost(states, controls);
    if (!std::isfinite(objective))
      throw std::domain_error("Nonfinite initial DDP rollout.");
    double damping = 1e-9;

    auto solveBackward = [&](double& reg) {
      auto sweep = backward(states, controls, reg);
      while (sweep.gains.empty() && reg < 1e12) {
        reg = std::max(1e-6, 10.0 * reg);
        sweep = backward(states, controls, reg);
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
          std::vector<M> x = rollout(trial, &states, &sweep, alpha);
          const double value = cost(x, trial);
          if (std::isfinite(value) && value < objective) {
            controls = std::move(trial);
            states = std::move(x);
            objective = value;
            damping = std::max(1e-9, 0.5 * damping);
            accepted = true;
            break;
          }
        }
      }

      if (!accepted) {
        if (damping > 1e-9) {
          double finalDamping = 1e-9;
          auto finalSweep = solveBackward(finalDamping);
          if (!finalSweep.gains.empty()) sweep = std::move(finalSweep);
        }
        states_ = std::move(states);
        gains_ = std::move(sweep.gains);
        return controls;
      }
    }
  }

  /** Fit H controls and H gains using a continuous-time vector field
   * xi = vectorField(x, u) integrated via traits<M>::Retract(x, dt * xi)
   * (matching SamplingPlanner and ILQRTracking).
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

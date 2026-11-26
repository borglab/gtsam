// Copyright (c) Georgia Tech Research Corporation. See LICENSE for details.
/**
 * @file TrackingFeedback.cpp
 * @author Frank Dellaert
 * @author Akash Ratheesh
 */
#include <gtsam/planning/TrackingFeedback.h>

#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace gtsam {
namespace {

void checkPrecision(const Matrix& p, bool definite) {
  if (!p.rows() || p.rows() != p.cols() || !p.allFinite() ||
      !p.isApprox(p.transpose()))
    throw std::invalid_argument(
        "Expected a finite symmetric precision matrix.");
  Eigen::LDLT<Matrix> ldlt(p);
  if (ldlt.info() != Eigen::Success || !ldlt.isPositive() ||
      (definite && (ldlt.vectorD().array() <= 0).any()))
    throw std::invalid_argument("Invalid tracking precision.");
}

struct Sweep {
  Matrix feedforward;
  std::vector<Matrix> gains;
};

/// Form the local quadratic Bellman model, including the value-gradient
/// contraction with the transition Hessian that distinguishes DDP from iLQR.
Sweep backward(const Matrix& states, const Matrix& controls, const Matrix& goal,
               const Matrix& reference, const DDPTracking::Transition& step,
               const Matrix& q, const Matrix& r, const Matrix& terminal,
               double damping) {
  const Eigen::Index n = states.rows(), m = controls.rows(),
                     h = controls.cols(), d = n + m;
  Vector gradient = terminal * (states.col(h) - goal.col(h));
  Matrix curvature = terminal;
  Sweep sweep{Matrix::Zero(m, h), std::vector<Matrix>(h)};

  for (Eigen::Index k = h - 1; k >= 0; --k) {
    Vector z(d);
    z << states.col(k), controls.col(k);
    const Vector delta = 1e-4 * (1 + z.array().abs());
    auto f = [&](const Vector& arg) -> Vector {
      Vector next = step(arg.head(n), arg.tail(m));
      if (next.size() != n || !next.allFinite())
        throw std::domain_error("Invalid DDP transition.");
      return next;
    };

    // Central-difference Jacobian and value-gradient Hessian contraction.
    const Vector center = states.col(k + 1);
    Matrix jacobian(n, d), hessian = Matrix::Zero(d, d);
    for (Eigen::Index i = 0; i < d; ++i) {
      Vector shift = Vector::Zero(d);
      shift[i] = delta[i];
      Vector plus = f(z + shift), minus = f(z - shift);
      jacobian.col(i) = (plus - minus) / (2 * delta[i]);
      hessian(i, i) =
          gradient.dot(plus + minus - 2 * center) / (delta[i] * delta[i]);
      for (Eigen::Index j = 0; j < i; ++j) {
        Vector cross = Vector::Zero(d);
        cross[j] = delta[j];
        const Vector mixed = f(z + shift + cross) - f(z + shift - cross) -
                             f(z - shift + cross) + f(z - shift - cross);
        hessian(i, j) = hessian(j, i) =
            gradient.dot(mixed) / (4 * delta[i] * delta[j]);
      }
    }

    // Assemble the local quadratic Q-function at step k.
    hessian += jacobian.transpose() * curvature * jacobian;
    hessian.topLeftCorner(n, n) += q;
    hessian.bottomRightCorner(m, m) += r;
    Vector first = jacobian.transpose() * gradient;
    first.head(n) += q * (states.col(k) - goal.col(k));
    first.tail(m) += r * (controls.col(k) - reference.col(k));

    // Factor damped Q_uu and solve for feedback gain and feedforward step.
    Matrix uu = hessian.bottomRightCorner(m, m);
    uu.diagonal().array() += damping;
    Eigen::LLT<Matrix> factor(uu);
    if (factor.info() != Eigen::Success) return {};
    const Matrix ux = hessian.bottomLeftCorner(m, n);
    Matrix gain = factor.solve(-ux);
    Vector du = factor.solve(-first.tail(m));

    // Back up value gradient and symmetrized curvature to step k.
    gradient = first.head(n) + gain.transpose() * first.tail(m) +
               ux.transpose() * du +
               gain.transpose() * hessian.bottomRightCorner(m, m) * du;
    curvature = hessian.topLeftCorner(n, n) + gain.transpose() * ux +
                ux.transpose() * gain +
                gain.transpose() * hessian.bottomRightCorner(m, m) * gain;
    curvature = (0.5 * (curvature + curvature.transpose())).eval();
    sweep.gains[k] = std::move(gain);
    sweep.feedforward.col(k) = du;
  }
  return sweep;
}

}  // namespace

Vector contractionFeedback(const Vector& error, const Matrix& metric,
                           const Matrix& inputJacobian,
                           const Vector& driftDifference, double rate) {
  checkPrecision(metric, true);
  if (!(rate > 0) || !std::isfinite(rate) || error.size() != metric.rows() ||
      driftDifference.size() != error.size() ||
      inputJacobian.rows() != error.size() || !inputJacobian.cols() ||
      !error.allFinite() || !inputJacobian.allFinite() ||
      !driftDifference.allFinite())
    throw std::invalid_argument(
        "Invalid contraction tracking dimensions or rate.");
  const Vector dual = metric * error;
  const Vector lhs = 2 * inputJacobian.transpose() * dual;
  const double rhs = -2 * dual.dot(rate * error + driftDifference);
  if (rhs >= 0 || lhs.squaredNorm() == 0)
    return Vector::Zero(inputJacobian.cols());
  return (rhs / lhs.squaredNorm()) * lhs;
}

DDPTracking::DDPTracking(const Matrix& statePrecision,
                         const Matrix& controlPrecision,
                         const Matrix& terminalPrecision)
    : q_(statePrecision), r_(controlPrecision), terminal_(terminalPrecision) {
  checkPrecision(q_, false);
  checkPrecision(r_, true);
  checkPrecision(terminal_, false);
  if (q_.rows() != terminal_.rows())
    throw std::invalid_argument("Inconsistent state precisions.");
}

Matrix DDPTracking::optimize(const Vector& initial, const Matrix& targetStates,
                             const Matrix& targetControls,
                             const Transition& transition, size_t iterations,
                             const Vector& lower, const Vector& upper) {
  const Eigen::Index h = targetControls.cols(), n = initial.size();
  if (!h || initial.size() != q_.rows() || targetControls.rows() != r_.rows() ||
      targetStates.rows() != n || targetStates.cols() != h + 1 ||
      !initial.allFinite() || !targetStates.allFinite() ||
      !targetControls.allFinite() || !transition)
    throw std::invalid_argument(
        "Invalid DDP tracking horizon, targets or transition.");
  Matrix controls = targetControls;
  const auto m = controls.rows();
  if ((lower.size() &&
       (lower.size() != m || lower.hasNaN() ||
        (lower.array() == std::numeric_limits<double>::infinity()).any())) ||
      (upper.size() &&
       (upper.size() != m || upper.hasNaN() ||
        (upper.array() == -std::numeric_limits<double>::infinity()).any())) ||
      (lower.size() && upper.size() && (lower.array() > upper.array()).any()))
    throw std::invalid_argument("Invalid DDP control limits.");
  auto rollout = [&](Matrix& u, const Matrix& base, const Sweep* sweep,
                     double alpha) {
    Matrix x(n, h + 1);
    x.col(0) = initial;
    for (Eigen::Index k = 0; k < h; ++k) {
      if (sweep)
        u.col(k) += alpha * sweep->feedforward.col(k) +
                    sweep->gains[k] * (x.col(k) - base.col(k));
      if (lower.size()) u.col(k) = u.col(k).cwiseMax(lower);
      if (upper.size()) u.col(k) = u.col(k).cwiseMin(upper);
      const Vector next = transition(x.col(k), u.col(k));
      if (next.size() != n)
        throw std::invalid_argument("DDP transition has wrong dimension.");
      x.col(k + 1) = next;
    }
    return x;
  };
  auto cost = [&](const Matrix& x, const Matrix& u) {
    const Matrix dx = x.leftCols(h) - targetStates.leftCols(h),
                 du = u - targetControls;
    const Vector end = x.col(h) - targetStates.col(h);
    return 0.5 *
           ((dx.array() * (q_ * dx).array()).sum() +
            (du.array() * (r_ * du).array()).sum() + end.dot(terminal_ * end));
  };
  // Evaluate the initial clamped nominal trajectory.
  Matrix states = rollout(controls, {}, nullptr, 0);
  if (!states.allFinite())
    throw std::domain_error("Nonfinite initial DDP rollout.");
  double objective = cost(states, controls), damping = 1e-9;

  // Run a damped backward sweep, increasing regularization until Q_uu is PD.
  auto solveBackward = [&](double& reg) {  // NOLINT
    auto sweep = backward(states, controls, targetStates, targetControls,
                          transition, q_, r_, terminal_, reg);
    while (sweep.gains.empty() && reg < 1e12) {
      reg = std::max(1e-6, 10 * reg);
      sweep = backward(states, controls, targetStates, targetControls,
                       transition, q_, r_, terminal_, reg);
    }
    return sweep;
  };

  for (size_t i = 0;; ++i) {
    auto sweep = solveBackward(damping);
    if (sweep.gains.empty())
      throw std::domain_error("DDP control Hessian remained indefinite.");

    // Backtracking line search along the feedforward/feedback policy.
    bool accepted = false;
    if (i < iterations && sweep.feedforward.norm() > 1e-10) {
      for (int step = 0; step < 8; ++step) {
        const double alpha = std::ldexp(1.0, -step);
        Matrix trial = controls;
        Matrix x = rollout(trial, states, &sweep, alpha);
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
      // Recompute final gains from minimum damping so they are not biased by
      // leftover Levenberg regularization from earlier iterations.
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
}  // namespace gtsam

/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, Akash Ratheesh

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file timeILQRTracking.cpp
 * @brief Benchmark factor-graph ILQRTracking (EliminateQR,
 *        EliminatePreferCholesky, and soft-dynamics EliminateCholesky) against
 *        a bespoke second-order finite-difference DDP tracker.
 */

#include <gtsam/constrained/ActiveSetSolver.h>
#include <gtsam/constrained/QpProblem.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/planning/DDPTracking.h>
#include <gtsam/planning/ILQRTracking.h>

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "internal/TimingUtils.h"

using namespace gtsam;
using symbol_shorthand::U;
using symbol_shorthand::X;

namespace {

/// Factor-graph iLQR variant allowing choice of elimination routine and
/// optional soft dynamics sigma (0 = hard noiseModel::Constrained).
class FactorGraphILQRVariant {
 public:
  using Transition = ILQRTracking<>::Transition;
  enum class Mode { kQRHard, kPreferCholeskyHard, kCholeskySoft };

  FactorGraphILQRVariant(const Matrix& statePrecision,
                         const Matrix& controlPrecision,
                         const Matrix& terminalPrecision, Mode mode,
                         double dynamicsSigma = 0.0)
      : q_(statePrecision),
        r_(controlPrecision),
        terminal_(terminalPrecision),
        sqrtQ_(precisionSquareRoot(statePrecision)),
        sqrtR_(precisionSquareRoot(controlPrecision)),
        sqrtTerminal_(precisionSquareRoot(terminalPrecision)),
        mode_(mode),
        dynamicsSigma_(dynamicsSigma) {}

  Matrix optimize(const Vector& initial, const Matrix& targetStates,
                  const Matrix& targetControls, const Transition& transition,
                  size_t iterations) {
    const Eigen::Index h = targetControls.cols(), n = initial.size(),
                       m = targetControls.rows();
    Matrix controls = targetControls;

    auto rollout = [&](Matrix& u, const Matrix& base, const Matrix* ff,
                       const std::vector<Matrix>* gains, double alpha) {
      Matrix x(n, h + 1);
      x.col(0) = initial;
      for (Eigen::Index k = 0; k < h; ++k) {
        if (ff && gains)
          u.col(k) += alpha * ff->col(k) + (*gains)[static_cast<size_t>(k)] *
                                               (x.col(k) - base.col(k));
        x.col(k + 1) = transition(x.col(k), u.col(k));
      }
      return x;
    };

    auto cost = [&](const Matrix& x, const Matrix& u) {
      const Matrix dx = x.leftCols(h) - targetStates.leftCols(h),
                   du = u - targetControls;
      const Vector end = x.col(h) - targetStates.col(h);
      return 0.5 * ((dx.array() * (q_ * dx).array()).sum() +
                    (du.array() * (r_ * du).array()).sum() +
                    end.dot(terminal_ * end));
    };

    auto backward =
        [&](const Matrix& states, const Matrix& u,
            double damping) -> std::pair<Matrix, std::vector<Matrix>> {
      GaussianFactorGraph graph;
      const SharedDiagonal dynModel =
          mode_ == Mode::kCholeskySoft
              ? static_cast<SharedDiagonal>(
                    noiseModel::Isotropic::Sigma(n, dynamicsSigma_))
              : static_cast<SharedDiagonal>(noiseModel::Constrained::All(n));
      const Matrix negIdentity = -Matrix::Identity(n, n);
      const Vector zeroState = Vector::Zero(n);
      const Vector zeroControl = Vector::Zero(m);
      const Matrix sqrtDamping = std::sqrt(damping) * Matrix::Identity(m, m);

      for (Eigen::Index k = 0; k < h; ++k) {
        const Vector xk = states.col(k), uk = u.col(k);
        Matrix A(n, n), B(n, m);
        for (Eigen::Index i = 0; i < n; ++i) {
          const double eps = 1e-5 * (1.0 + std::abs(xk[i]));
          Vector dx = Vector::Zero(n);
          dx[i] = eps;
          A.col(i) =
              (transition(xk + dx, uk) - transition(xk - dx, uk)) / (2.0 * eps);
        }
        for (Eigen::Index i = 0; i < m; ++i) {
          const double eps = 1e-5 * (1.0 + std::abs(uk[i]));
          Vector du = Vector::Zero(m);
          du[i] = eps;
          B.col(i) =
              (transition(xk, uk + du) - transition(xk, uk - du)) / (2.0 * eps);
        }
        graph.emplace_shared<JacobianFactor>(X(k), A, U(k), B, X(k + 1),
                                             negIdentity, zeroState, dynModel);
        if (sqrtQ_.rows() > 0)
          graph.emplace_shared<JacobianFactor>(
              X(k), sqrtQ_, -sqrtQ_ * (xk - targetStates.col(k)));
        graph.emplace_shared<JacobianFactor>(
            U(k), sqrtR_, -sqrtR_ * (uk - targetControls.col(k)));
        if (damping > 0.0)
          graph.emplace_shared<JacobianFactor>(U(k), sqrtDamping, zeroControl);
      }
      if (sqrtTerminal_.rows() > 0)
        graph.emplace_shared<JacobianFactor>(
            X(h), sqrtTerminal_,
            -sqrtTerminal_ * (states.col(h) - targetStates.col(h)));

      Ordering ordering;
      ordering.reserve(static_cast<size_t>(2 * h + 1));
      ordering.push_back(X(h));
      for (Eigen::Index k = h - 1; k >= 0; --k) {
        ordering.push_back(U(k));
        ordering.push_back(X(k));
      }

      const auto bayesNet =
          mode_ == Mode::kCholeskySoft
              ? graph.eliminateSequential(ordering, EliminateCholesky)
              : (mode_ == Mode::kPreferCholeskyHard
                     ? graph.eliminateSequential(ordering,
                                                 EliminatePreferCholesky)
                     : graph.eliminateSequential(ordering, EliminateQR));

      Matrix ff = Matrix::Zero(m, h);
      std::vector<Matrix> gains(static_cast<size_t>(h));
      for (const auto& conditional : *bayesNet) {
        const Symbol sym(conditional->firstFrontalKey());
        if (sym.chr() != 'u') continue;
        const auto k = static_cast<Eigen::Index>(sym.index());
        const Matrix R = conditional->R();
        const Vector d = conditional->d();
        const auto lu = R.partialPivLu();
        ff.col(k) = lu.solve(d);
        gains[static_cast<size_t>(k)] =
            conditional->nrParents() > 0 ? Matrix(-lu.solve(conditional->S()))
                                         : Matrix::Zero(m, n);
      }
      return {std::move(ff), std::move(gains)};
    };

    Matrix states = rollout(controls, {}, nullptr, nullptr, 0);
    double objective = cost(states, controls), damping = 0.0;
    for (size_t i = 0;; ++i) {
      auto [ff, gains] = backward(states, controls, damping);
      bool accepted = false;
      if (i < iterations && ff.norm() > 1e-10) {
        for (int step = 0; step < 8; ++step) {
          const double alpha = std::ldexp(1.0, -step);
          Matrix trial = controls;
          Matrix x = rollout(trial, states, &ff, &gains, alpha);
          const double value = cost(x, trial);
          if (std::isfinite(value) && value < objective) {
            controls = std::move(trial);
            states = std::move(x);
            objective = value;
            damping = damping > 1e-9 ? 0.5 * damping : 0.0;
            accepted = true;
            break;
          }
        }
        if (!accepted && damping < 1e6) {
          damping = damping == 0.0 ? 1e-4 : 10.0 * damping;
          continue;
        }
      }
      if (!accepted) {
        if (damping > 0.0)
          std::tie(ff, gains) = backward(states, controls, 0.0);
        states_ = std::move(states);
        gains_ = std::move(gains);
        return controls;
      }
    }
  }

  const Matrix& states() const { return states_; }
  const Matrix& gain(size_t step) const { return gains_.at(step); }

 private:
  static Matrix precisionSquareRoot(const Matrix& p) {
    Eigen::SelfAdjointEigenSolver<Matrix> eig(p);
    const Vector values = eig.eigenvalues();
    const Matrix vectors = eig.eigenvectors();
    const double tol = 1e-12 * std::max(1.0, values.maxCoeff());
    Eigen::Index kept = 0;
    for (Eigen::Index i = 0; i < values.size(); ++i)
      if (values[i] > tol) ++kept;
    Matrix root(kept, p.cols());
    Eigen::Index row = 0;
    for (Eigen::Index i = 0; i < values.size(); ++i) {
      if (values[i] > tol) {
        root.row(row) = std::sqrt(values[i]) * vectors.col(i).transpose();
        ++row;
      }
    }
    return root;
  }

  Matrix q_, r_, terminal_, sqrtQ_, sqrtR_, sqrtTerminal_, states_;
  std::vector<Matrix> gains_;
  Mode mode_;
  double dynamicsSigma_;
};

struct BenchmarkCase {
  std::string name;
  Matrix q, r, terminal;
  Vector x0;
  Matrix targetStates, targetControls;
  ILQRTracking<>::Transition transition;
  ILQRTracking<>::LinearizedTransition linearizedTransition;
  size_t iterations;
};

/// Open-loop SQP/iLQR using GTSAM's QpProblem + ActiveSetSolver (Sparse or
/// Dense backend). Solves the same equality-constrained quadratic subproblem
/// per iteration (note: produces open-loop feedforward steps without Riccati
/// feedback gains K_k).
class QpTrackingVariant {
 public:
  using Transition = ILQRTracking<>::Transition;
  using Backend = ActiveSetSolverParams::QpSubproblemSolver;

  QpTrackingVariant(const Matrix& statePrecision,
                    const Matrix& controlPrecision,
                    const Matrix& terminalPrecision, Backend backend)
      : q_(statePrecision),
        r_(controlPrecision),
        terminal_(terminalPrecision),
        sqrtQ_(precisionSquareRoot(statePrecision)),
        sqrtR_(precisionSquareRoot(controlPrecision)),
        sqrtTerminal_(precisionSquareRoot(terminalPrecision)),
        backend_(backend) {}

  Matrix optimize(const Vector& initial, const Matrix& targetStates,
                  const Matrix& targetControls, const Transition& transition,
                  size_t iterations) {
    const Eigen::Index h = targetControls.cols(), n = initial.size(),
                       m = targetControls.rows();
    Matrix controls = targetControls;

    auto rollout = [&](const Matrix& u) {
      Matrix x(n, h + 1);
      x.col(0) = initial;
      for (Eigen::Index k = 0; k < h; ++k)
        x.col(k + 1) = transition(x.col(k), u.col(k));
      return x;
    };

    auto cost = [&](const Matrix& x, const Matrix& u) {
      const Matrix dx = x.leftCols(h) - targetStates.leftCols(h),
                   du = u - targetControls;
      const Vector end = x.col(h) - targetStates.col(h);
      return 0.5 * ((dx.array() * (q_ * dx).array()).sum() +
                    (du.array() * (r_ * du).array()).sum() +
                    end.dot(terminal_ * end));
    };

    Values zeroInit;
    Ordering ordering;
    ordering.reserve(static_cast<size_t>(2 * h + 1));
    ordering.push_back(X(h));
    zeroInit.insert<Vector>(X(h), Vector::Zero(n));
    for (Eigen::Index k = h - 1; k >= 0; --k) {
      ordering.push_back(U(k));
      ordering.push_back(X(k));
      zeroInit.insert<Vector>(U(k), Vector::Zero(m));
      zeroInit.insert<Vector>(X(k), Vector::Zero(n));
    }

    auto params = std::make_shared<ActiveSetSolverParams>();
    params->qpSubproblemSolver = backend_;
    params->setOrdering(ordering);

    auto solveSubproblem = [&](const Matrix& states, const Matrix& u,
                               double damping) -> Matrix {
      QpProblem problem;
      const Matrix identity = Matrix::Identity(n, n);
      const Matrix negIdentity = -identity;
      const Vector zeroState = Vector::Zero(n);
      const Vector zeroControl = Vector::Zero(m);
      const Matrix sqrtDamping = std::sqrt(damping) * Matrix::Identity(m, m);

      problem.addConstraint(
          LinearConstraint::Equal(JacobianFactor(X(0), identity, zeroState)));

      for (Eigen::Index k = 0; k < h; ++k) {
        const Vector xk = states.col(k), uk = u.col(k);
        Matrix A(n, n), B(n, m);
        for (Eigen::Index i = 0; i < n; ++i) {
          const double eps = 1e-5 * (1.0 + std::abs(xk[i]));
          Vector dx = Vector::Zero(n);
          dx[i] = eps;
          A.col(i) =
              (transition(xk + dx, uk) - transition(xk - dx, uk)) / (2.0 * eps);
        }
        for (Eigen::Index i = 0; i < m; ++i) {
          const double eps = 1e-5 * (1.0 + std::abs(uk[i]));
          Vector du = Vector::Zero(m);
          du[i] = eps;
          B.col(i) =
              (transition(xk, uk + du) - transition(xk, uk - du)) / (2.0 * eps);
        }
        problem.addConstraint(LinearConstraint::Equal(JacobianFactor(
            X(k), A, U(k), B, X(k + 1), negIdentity, zeroState)));
        if (sqrtQ_.rows() > 0)
          problem.addCost(JacobianFactor(X(k), sqrtQ_,
                                         -sqrtQ_ * (xk - targetStates.col(k))));
        problem.addCost(JacobianFactor(U(k), sqrtR_,
                                       -sqrtR_ * (uk - targetControls.col(k))));
        if (damping > 0.0)
          problem.addCost(JacobianFactor(U(k), sqrtDamping, zeroControl));
      }
      if (sqrtTerminal_.rows() > 0)
        problem.addCost(JacobianFactor(
            X(h), sqrtTerminal_,
            -sqrtTerminal_ * (states.col(h) - targetStates.col(h))));

      ActiveSetSolver solver(problem, params);
      const Values sol = solver.optimize(zeroInit);
      Matrix ff(m, h);
      for (Eigen::Index k = 0; k < h; ++k) ff.col(k) = sol.at<Vector>(U(k));
      return ff;
    };

    Matrix states = rollout(controls);
    double objective = cost(states, controls), damping = 0.0;
    for (size_t i = 0;; ++i) {
      const Matrix ff = solveSubproblem(states, controls, damping);
      bool accepted = false;
      if (i < iterations && ff.norm() > 1e-10) {
        for (int step = 0; step < 8; ++step) {
          const double alpha = std::ldexp(1.0, -step);
          Matrix trial = controls + alpha * ff;
          Matrix x = rollout(trial);
          const double value = cost(x, trial);
          if (std::isfinite(value) && value < objective) {
            controls = std::move(trial);
            states = std::move(x);
            objective = value;
            damping = damping > 1e-9 ? 0.5 * damping : 0.0;
            accepted = true;
            break;
          }
        }
        if (!accepted && damping < 1e6) {
          damping = damping == 0.0 ? 1e-4 : 10.0 * damping;
          continue;
        }
      }
      if (!accepted) return controls;
    }
  }

 private:
  static Matrix precisionSquareRoot(const Matrix& p) {
    Eigen::SelfAdjointEigenSolver<Matrix> eig(p);
    const Vector values = eig.eigenvalues();
    const Matrix vectors = eig.eigenvectors();
    const double tol = 1e-12 * std::max(1.0, values.maxCoeff());
    Eigen::Index kept = 0;
    for (Eigen::Index i = 0; i < values.size(); ++i)
      if (values[i] > tol) ++kept;
    Matrix root(kept, p.cols());
    Eigen::Index row = 0;
    for (Eigen::Index i = 0; i < values.size(); ++i) {
      if (values[i] > tol) {
        root.row(row) = std::sqrt(values[i]) * vectors.col(i).transpose();
        ++row;
      }
    }
    return root;
  }

  Matrix q_, r_, terminal_, sqrtQ_, sqrtR_, sqrtTerminal_;
  Backend backend_;
};

std::tuple<KeyVector, KeyVector, NonlinearFactorGraph, Values> buildCaseGraph(
    const BenchmarkCase& c) {
  const auto h = static_cast<size_t>(c.targetControls.cols());
  KeyVector xKeys, uKeys;
  xKeys.reserve(h + 1);
  uKeys.reserve(h);
  NonlinearFactorGraph graph;
  Values initial;
  initial.insert(X(0), c.x0);
  const auto qNoise = noiseModel::Diagonal::Precisions(c.q.diagonal());
  const auto rNoise = noiseModel::Diagonal::Precisions(c.r.diagonal());
  const auto termNoise =
      noiseModel::Diagonal::Precisions(c.terminal.diagonal());
  for (size_t k = 0; k < h; ++k) {
    const auto col = static_cast<Eigen::Index>(k);
    xKeys.push_back(X(k));
    uKeys.push_back(U(k));
    initial.insert(U(k), Vector(c.targetControls.col(col)));
    graph.addPrior(X(k), Vector(c.targetStates.col(col)), qNoise);
    graph.addPrior(U(k), Vector(c.targetControls.col(col)), rNoise);
  }
  xKeys.push_back(X(h));
  graph.addPrior(X(h), Vector(c.targetStates.col(static_cast<Eigen::Index>(h))),
                 termNoise);
  return {std::move(xKeys), std::move(uKeys), std::move(graph),
          std::move(initial)};
}

Matrix extractControls(const Values& vals, const KeyVector& uKeys,
                       Eigen::Index m) {
  Matrix out(m, static_cast<Eigen::Index>(uKeys.size()));
  for (size_t k = 0; k < uKeys.size(); ++k)
    out.col(static_cast<Eigen::Index>(k)) = vals.at<Vector>(uKeys[k]);
  return out;
}

void runCase(const BenchmarkCase& c, size_t warmups, size_t repetitions) {
  KeyVector xKeys, uKeys;
  NonlinearFactorGraph graph;
  Values initial;
  std::tie(xKeys, uKeys, graph, initial) = buildCaseGraph(c);
  ILQRTracking<> ilqrAna(xKeys, uKeys);
  ILQRTracking<> ilqrFd(xKeys, uKeys);
  FactorGraphILQRVariant ilqrPref(
      c.q, c.r, c.terminal, FactorGraphILQRVariant::Mode::kPreferCholeskyHard);
  FactorGraphILQRVariant ilqrCholSoft(
      c.q, c.r, c.terminal, FactorGraphILQRVariant::Mode::kCholeskySoft, 1e-5);
  QpTrackingVariant qpSparse(c.q, c.r, c.terminal,
                             ActiveSetSolverParams::QpSubproblemSolver::Sparse);
  QpTrackingVariant qpDense(c.q, c.r, c.terminal,
                            ActiveSetSolverParams::QpSubproblemSolver::Dense);
  DDPTracking<> ddp(xKeys, uKeys);

  Values planAna, planFd, planDdp;
  Matrix uPref, uChol, uQpSp, uQpDn;
  const auto anaSamples = timing::measureMilliseconds(
      [&] {
        planAna = ilqrAna.optimize(graph, initial, c.linearizedTransition,
                                   c.iterations);
      },
      warmups, repetitions);
  const auto fdSamples = timing::measureMilliseconds(
      [&] {
        planFd = ilqrFd.optimize(graph, initial, c.transition, c.iterations);
      },
      warmups, repetitions);
  const auto prefSamples = timing::measureMilliseconds(
      [&] {
        uPref = ilqrPref.optimize(c.x0, c.targetStates, c.targetControls,
                                  c.transition, c.iterations);
      },
      warmups, repetitions);
  const auto cholSamples = timing::measureMilliseconds(
      [&] {
        uChol = ilqrCholSoft.optimize(c.x0, c.targetStates, c.targetControls,
                                      c.transition, c.iterations);
      },
      warmups, repetitions);
  const auto qpSpSamples = timing::measureMilliseconds(
      [&] {
        uQpSp = qpSparse.optimize(c.x0, c.targetStates, c.targetControls,
                                  c.transition, c.iterations);
      },
      warmups, repetitions);
  const auto qpDnSamples = timing::measureMilliseconds(
      [&] {
        uQpDn = qpDense.optimize(c.x0, c.targetStates, c.targetControls,
                                 c.transition, c.iterations);
      },
      warmups, repetitions);
  const auto ddpSamples = timing::measureMilliseconds(
      [&] {
        planDdp = ddp.optimize(graph, initial, c.transition, c.iterations);
      },
      warmups, repetitions);

  const auto med = [](const std::vector<double>& s) {
    return timing::summarizeSamples(s, timing::MedianPolicy::kAverageMiddle)
        .median;
  };

  const Matrix uAna = extractControls(planAna, uKeys, c.targetControls.rows());
  const Matrix uDdp = extractControls(planDdp, uKeys, c.targetControls.rows());
  const double tAna = med(anaSamples);
  const double tDdp = med(ddpSamples);
  std::cout << "| " << std::left << std::setw(34) << c.name << " | "
            << std::right << std::fixed << std::setprecision(3) << std::setw(7)
            << tAna << " | " << std::setw(7) << med(fdSamples) << " | "
            << std::setw(8) << med(prefSamples) << " | " << std::setw(8)
            << med(cholSamples) << " | " << std::setw(8) << med(qpSpSamples)
            << " | " << std::setw(8) << med(qpDnSamples) << " | "
            << std::setw(7) << tDdp << " | " << std::setprecision(2)
            << std::setw(5) << (tDdp / tAna) << "x | " << std::scientific
            << std::setprecision(1) << std::setw(7)
            << (uAna - uDdp).cwiseAbs().maxCoeff() << " |\n";
}

}  // namespace

int main(int argc, char* argv[]) {
  timing::Arguments args(argc, argv);
  const size_t warmups = args.sizeValue("--warmups", 3);
  const size_t repetitions = args.sizeValue("--repetitions", 20);
  args.validateAllConsumed();

  std::vector<BenchmarkCase> cases;

  // 1. Planar double integrator (DoubleIntegratorCoRL.ipynb), H = 10 and H
  // = 50.
  for (Eigen::Index h : {Eigen::Index{10}, Eigen::Index{50}}) {
    const double dt = 0.02;
    Matrix4 A = Matrix4::Identity();
    A(0, 2) = dt;
    A(1, 3) = dt;
    Matrix42 B = Matrix42::Zero();
    B(2, 0) = dt;
    B(3, 1) = dt;
    cases.push_back({"DoubleIntegrator (n=4,m=2,H=" + std::to_string(h) + ")",
                     dt * Vector4(500., 500., 100., 100.).asDiagonal(),
                     dt * Matrix2::Identity(), Matrix4::Identity(),
                     Vector4(0.3, -0.2, 0.5, -0.4), Matrix::Zero(4, h + 1),
                     Matrix::Zero(2, h),
                     [dt](const Vector& x, const Vector& u) -> Vector {
                       return Vector4(x[0] + dt * x[2], x[1] + dt * x[3],
                                      x[2] + dt * u[0], x[3] + dt * u[1]);
                     },
                     [dt, A, B](const Vector& x, const Vector& u,
                                ILQRTracking<>::OptionalMatrix Hx,
                                ILQRTracking<>::OptionalMatrix Hu) -> Vector {
                       if (Hx) *Hx = A;
                       if (Hu) *Hu = B;
                       return Vector4(x[0] + dt * x[2], x[1] + dt * x[3],
                                      x[2] + dt * u[0], x[3] + dt * u[1]);
                     },
                     5});
  }

  // 2. Nonlinear pendulum swing-up and feasible-reference tracking
  // (n=2,m=1,H=25).
  {
    const double dt = 0.05;
    const Eigen::Index h = 25;
    auto pendulum = [dt](const Vector& x, const Vector& u) -> Vector {
      return Vector2(x[0] + dt * x[1],
                     x[1] + dt * (-9.81 * std::sin(x[0]) + u[0]));
    };
    ILQRTracking<>::LinearizedTransition pendulumLin =
        [dt](const Vector& x, const Vector& u,
             ILQRTracking<>::OptionalMatrix Hx,
             ILQRTracking<>::OptionalMatrix Hu) -> Vector {
      if (Hx) *Hx = Matrix2{{1.0, dt}, {-dt * 9.81 * std::cos(x[0]), 1.0}};
      if (Hu) *Hu = Vector2(0.0, dt);
      return Vector2(x[0] + dt * x[1],
                     x[1] + dt * (-9.81 * std::sin(x[0]) + u[0]));
    };
    Matrix targetStates = Matrix::Zero(2, h + 1);
    targetStates.row(0).setConstant(M_PI);
    const Matrix targetControls = Matrix::Zero(1, h);
    const Matrix q = Vector2(2., 0.2).asDiagonal();
    const Matrix r = Matrix1::Identity() * 0.1;
    const Matrix terminal = Vector2(20., 2.).asDiagonal();
    const Vector2 x0(0.2, 0.0);

    cases.push_back({"Pendulum swing-up (n=2,m=1,H=25)", q, r, terminal, x0,
                     targetStates, targetControls, pendulum, pendulumLin, 50});

    // Build a feasible nonlinear reference trajectory for Tube/Robust tracking.
    const auto [refXKeys, refUKeys, refGraph, refInit] =
        buildCaseGraph(cases.back());
    DDPTracking<> refSolver(refXKeys, refUKeys);
    const Values refPlan = refSolver.optimize(refGraph, refInit, pendulum, 50);
    Matrix refStates(2, h + 1);
    for (size_t k = 0; k <= static_cast<size_t>(h); ++k)
      refStates.col(static_cast<Eigen::Index>(k)) =
          refPlan.at<Vector>(refXKeys[k]);
    const Matrix uRef = extractControls(refPlan, refUKeys, 1);
    cases.push_back({"Pendulum tracking (n=2,m=1,H=25)", q, r, terminal,
                     x0 + Vector2(0.05, -0.05), refStates, uRef, pendulum,
                     pendulumLin, 20});
  }

  // 3. Nonlinear cart-pole tracking (CartPoleMPPI.ipynb dynamics,
  // n=4,m=1,H=50).
  {
    const double dt = 0.02;
    const Eigen::Index h = 50;
    auto cartPole = [dt](const Vector& x, const Vector& u) -> Vector {
      const double speed = x[1], angle = x[2], rate = x[3], force = u[0];
      const double s = std::sin(angle), c = std::cos(angle);
      const double denom = 1.0 + s * s;
      const double acc = (force + s * (rate * rate + 9.81 * c)) / denom;
      const double angAcc =
          (-force * c - rate * rate * c * s - 2.0 * 9.81 * s) / denom;
      return Vector4(x[0] + dt * speed, speed + dt * acc, angle + dt * rate,
                     rate + dt * angAcc);
    };
    ILQRTracking<>::LinearizedTransition cartPoleLin =
        [dt](const Vector& x, const Vector& u,
             ILQRTracking<>::OptionalMatrix Hx,
             ILQRTracking<>::OptionalMatrix Hu) -> Vector {
      const double speed = x[1], angle = x[2], rate = x[3], force = u[0];
      const double s = std::sin(angle), c = std::cos(angle);
      const double denom = 1.0 + s * s;
      const double numA = force + s * (rate * rate + 9.81 * c);
      const double numG = -force * c - rate * rate * c * s - 2.0 * 9.81 * s;
      const double acc = numA / denom;
      const double angAcc = numG / denom;
      if (Hx) {
        const double dDenom = 2.0 * s * c;
        const double dNumA = c * (rate * rate + 9.81 * c) - 9.81 * s * s;
        const double dNumG =
            force * s - rate * rate * (c * c - s * s) - 2.0 * 9.81 * c;
        Matrix4 A = Matrix4::Identity();
        A(0, 1) = dt;
        A(1, 2) = dt * (dNumA * denom - numA * dDenom) / (denom * denom);
        A(1, 3) = dt * (2.0 * s * rate) / denom;
        A(2, 3) = dt;
        A(3, 2) = dt * (dNumG * denom - numG * dDenom) / (denom * denom);
        A(3, 3) = 1.0 + dt * (-2.0 * rate * c * s) / denom;
        *Hx = A;
      }
      if (Hu) {
        *Hu = Vector4(0.0, dt / denom, 0.0, -dt * c / denom);
      }
      return Vector4(x[0] + dt * speed, speed + dt * acc, angle + dt * rate,
                     rate + dt * angAcc);
    };
    Matrix targetStates = Matrix::Zero(4, h + 1);
    targetStates.row(2).setConstant(M_PI);
    cases.push_back({"CartPole balance (n=4,m=1,H=50)",
                     Vector4(1., 0.1, 10., 0.5).asDiagonal(),
                     Matrix1::Identity() * 0.1,
                     Vector4(5., 1., 50., 2.).asDiagonal(),
                     Vector4(0.1, 0.0, M_PI - 0.15, 0.0), targetStates,
                     Matrix::Zero(1, h), cartPole, cartPoleLin, 20});
  }

  // 4. Coupled 3D nonlinear dynamics (n=6,m=3,H=25) to show dimension scaling.
  {
    const double dt = 0.05;
    const Eigen::Index h = 25;
    auto coupled3d = [dt](const Vector& x, const Vector& u) -> Vector {
      Vector6 next = x;
      next.head<3>() += dt * x.tail<3>();
      next[3] +=
          dt * (u[0] - 0.2 * x[3] * std::abs(x[3]) + 0.1 * std::sin(x[1]));
      next[4] +=
          dt * (u[1] - 0.2 * x[4] * std::abs(x[4]) + 0.1 * std::sin(x[2]));
      next[5] +=
          dt * (u[2] - 0.2 * x[5] * std::abs(x[5]) + 0.1 * std::sin(x[0]));
      return next;
    };
    ILQRTracking<>::LinearizedTransition coupled3dLin =
        [dt](const Vector& x, const Vector& u,
             ILQRTracking<>::OptionalMatrix Hx,
             ILQRTracking<>::OptionalMatrix Hu) -> Vector {
      if (Hx) {
        Matrix6 A = Matrix6::Identity();
        A.topRightCorner<3, 3>() = dt * Matrix3::Identity();
        A(3, 1) = dt * 0.1 * std::cos(x[1]);
        A(3, 3) = 1.0 - dt * 0.4 * std::abs(x[3]);
        A(4, 2) = dt * 0.1 * std::cos(x[2]);
        A(4, 4) = 1.0 - dt * 0.4 * std::abs(x[4]);
        A(5, 0) = dt * 0.1 * std::cos(x[0]);
        A(5, 5) = 1.0 - dt * 0.4 * std::abs(x[5]);
        *Hx = A;
      }
      if (Hu) {
        Matrix63 B = Matrix63::Zero();
        B.bottomRows<3>() = dt * Matrix3::Identity();
        *Hu = B;
      }
      Vector6 next = x;
      next.head<3>() += dt * x.tail<3>();
      next[3] +=
          dt * (u[0] - 0.2 * x[3] * std::abs(x[3]) + 0.1 * std::sin(x[1]));
      next[4] +=
          dt * (u[1] - 0.2 * x[4] * std::abs(x[4]) + 0.1 * std::sin(x[2]));
      next[5] +=
          dt * (u[2] - 0.2 * x[5] * std::abs(x[5]) + 0.1 * std::sin(x[0]));
      return next;
    };
    cases.push_back({"Coupled 3D nonlinear (n=6,m=3,H=25)", Matrix6::Identity(),
                     0.2 * Matrix3::Identity(), 5.0 * Matrix6::Identity(),
                     (Vector6() << 0.5, -0.4, 0.3, 0.1, -0.2, 0.1).finished(),
                     Matrix::Zero(6, h + 1), Matrix::Zero(3, h), coupled3d,
                     coupled3dLin, 20});
  }

  std::cout << "ILQRTracking (Analytic vs FD) vs FG-PrefCh vs FG-Chol(1e-5) "
               "vs QpSparse vs QpDense vs DDPTracking"
            << " (" << warmups << " warmups, " << repetitions << " reps)\n\n";
  std::cout << "| Benchmark                          | Ana(ms) |  FD(ms) |"
            << " Pref(ms) | Chol(ms) | QpSp(ms) | QpDn(ms) | DDP(ms) | Speed  |"
            << " |du|    |\n";
  std::cout << "|------------------------------------|---------|---------|"
            << "----------|----------|----------|----------|---------|--------|"
            << "---------|\n";
  for (const auto& c : cases) runCase(c, warmups, repetitions);
  return 0;
}

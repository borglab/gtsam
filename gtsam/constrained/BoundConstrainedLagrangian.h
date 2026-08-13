/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    BoundConstrainedLagrangian.h
 * @brief   Augmented Lagrangian method for nonlinear constrained optimization.
 * @author  Yetong Zhang
 * @date    Aug 3, 2024
 */

#pragma once

#include <gtsam/constrained/AugmentedLagrangianOptimizer.h>
#include <gtsam/constrained/ConstrainedOptimizer.h>

namespace gtsam {

/// Parameters for the bound-constrained augmented Lagrangian outer loop.
class GTSAM_EXPORT BoundConstrainedLagrangianParams
    : public AugmentedLagrangianParams {
 public:
  using Base = AugmentedLagrangianParams;
  using This = BoundConstrainedLagrangianParams;
  using shared_ptr = std::shared_ptr<BoundConstrainedLagrangianParams>;

  /// Multiplicative factor used to increase the equality penalty `muEq`
  /// when the iterate is not yet feasible enough for a multiplier update.
  /// This fixed factor is a simplified choice; the paper uses
  /// algorithm-dependent penalty rules.
  double k = 2;

  /// Exponent used in the BCL threshold update
  /// `eta_{k+1} = eta_k / mu_k^alpha` after an accepted multiplier step.
  /// This update is a simplified variant of the paper's parameter rules.
  double alpha = 0.5;

  /// Initial feasibility threshold for the equality-constraint violation.
  double eta0 = 1.0;

  /// Initial stationarity threshold for the cost-gradient infinity norm.
  double omega0 = 1.0;

  /// Stop when feasibility threshold `eta` has been reduced below this value.
  double eta_threshold = 1e-3;

  /// Stop when threshold `omega` has been reduced below this value.
  double omega_threshold = 1e-3;

  BoundConstrainedLagrangianParams() {
    // The paper uses an adaptive inner stopping rule. This implementation
    // instead limits the unconstrained LM inner solve to one iteration.
    lmParams.maxIterations = 1;
  }
};

class GTSAM_EXPORT BoundConstrainedLagrangianState
    : public AugmentedLagrangianState {
 public:
  using Base = AugmentedLagrangianState;
  using This = BoundConstrainedLagrangianState;
  using shared_ptr = std::shared_ptr<This>;

  double omega;  // Threshold to compare gradient against;
  double eta;    // Threshold to compare constraint violation against;

  using Base::Base;
};

/**
 * Augmented Lagrangian method with BCL globalization strategy.
 *
 * This implementation is inspired by A. R. Conn, N. I. M. Gould, and
 * P. L. Toint, "A Globally Convergent Augmented Lagrangian Algorithm for
 * Optimization with General Constraints and Simple Bounds," SIAM Journal on
 * Numerical Analysis, 28(2):545-572, 1991,
 * https://doi.org/10.1137/0728030. It is not an entirely faithful
 * implementation of that paper.
 */
class GTSAM_EXPORT BoundConstrainedLagrangian : public ConstrainedOptimizer {
 public:
  using Base = ConstrainedOptimizer;
  using This = BoundConstrainedLagrangian;
  using shared_ptr = std::shared_ptr<This>;

  using Params = BoundConstrainedLagrangianParams;
  using State = BoundConstrainedLagrangianState;
  using Progress = std::vector<BoundConstrainedLagrangianState>;

 protected:
  Params::shared_ptr p_;
  mutable Progress progress_;

 public:
  /// Constructor.
  BoundConstrainedLagrangian(const ConstrainedOptProblem& problem,
                             const Values& initialValues,
                             Params::shared_ptr p = std::make_shared<Params>())
      : Base(problem, initialValues), p_(p) {}

  /// Solve one step of optimization, updating multipliers and parameters.
  State iterate(const State& state) const;

  /// Run optimization for the bound-constrained problem and return the result.
  Values optimize() const override;

  /// Return progress of iterations as a read-only sequence of states.
  const Progress& progress() const { return progress_; }

 protected:
  /// Create an unconstrained optimizer that solves the augmented Lagrangian.
  /// The paper uses a bound-constrained inner method; this implementation uses
  /// an unconstrained Levenberg-Marquardt optimizer instead.
  SharedOptimizer createUnconstrainedOptimizer(
      const NonlinearFactorGraph& graph, const Values& values) const;

  /// Update the Lagrange multipliers using dual ascent.
  void updateMultipliers(const State& prev_state, State* state) const;

  /// Build the augmented Lagrangian factor graph for the given state.
  NonlinearFactorGraph augmentedLagrangianFunction(
      const State& state, const double epsilon = 1.0) const;

  /// Store and log the initial state of the optimization.
  void logInitialState(const State& state) const;

  /// Store and log the state after an iteration of the optimization.
  void logIteration(const State& state) const;

  /// Customized convergence check for bound-constrained optimization.
  bool checkConvergenceBC(const State& state, const State& previousState,
                          const Params& params) const;
};

}  // namespace gtsam

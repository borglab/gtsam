/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    AugmentedLagrangianOptimizer.h
 * @brief   Augmented Lagrangian method for nonlinear constrained optimization.
 * @author  Yetong Zhang
 * @date    Aug 3, 2024
 */

#pragma once

#include <gtsam/constrained/AugmentedLagrangianOptimizer.h>
#include <gtsam/constrained/ConstrainedOptimizer.h>

namespace gtsam {

/// Parameters for Augmented Lagrangian method
class GTSAM_EXPORT BoundConstrainedLagrangianParams
    : public AugmentedLagrangianParams {
 public:
  using Base = AugmentedLagrangianParams;
  using This = BoundConstrainedLagrangianParams;
  using shared_ptr = std::shared_ptr<BoundConstrainedLagrangianParams>;

  double k = 2;  // mu increase rate factor
  double alpha = 0.5;
  double ita0 = 10.0;
  double omega0 = 1.0;
  double mu0;
  double ita_threshold = 1e-3;
  double omega_threshold = 1e-3;

  BoundConstrainedLagrangianParams() {
    // Set default inner optimizer to only iterate once.
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
  double ita;    // Threshold to compare constraint violation against;

  using Base::Base;
};

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

  /// Solve one step of optimization, update multipliers and parameters.
  State iterate(const State& state) const;

  /// Run optimization with equality constraints only.
  Values optimize() const override;

  /// Return progress of iterations.
  const Progress& progress() const { return progress_; }

 protected:
  /// Create an unconstrained optimizer that solves the augmented Lagrangian.
  SharedOptimizer createUnconstrainedOptimizer(
      const NonlinearFactorGraph& graph, const Values& values) const;

  /** Update the Lagrange multipliers using dual ascent. */
  void updateMultipliers(const State& prev_state, State* state) const;

  NonlinearFactorGraph augmentedLagrangianFunction(
      const State& state, const double epsilon = 1.0) const;

  /// Store and log the initial state of the optimization.
  void logInitialState(const State& state) const;

  /// Store and log the state after any iteration of the optimization.
  void logIteration(const State& state) const;
};

}  // namespace gtsam

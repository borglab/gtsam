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
 * @author  Frank Dellaert (codex assisted)
 * @date    Aug 3, 2024
 */

#pragma once

#include <gtsam/constrained/AugmentedLagrangianSolver.h>
#include <gtsam/constrained/ConstrainedOptimizer.h>

#include <limits>

namespace gtsam {

/** Diagnostics and algorithm state for one outer ALM iteration. */
class AugmentedLagrangianState : public PenaltyOptimizerState,
                                 public AugmentedLagrangianDiagnostics {
 public:
  using Base = PenaltyOptimizerState;
  using This = AugmentedLagrangianState;
  using shared_ptr = std::shared_ptr<This>;

  /** Initialize Lagrange multipliers as zeros. */
  GTSAM_EXPORT void initializeLagrangeMultipliers(
      const ConstrainedOptProblem& problem);

  using Base::Base;
};

/**
 * Augmented Lagrangian solver for nonlinear least-squares problems with
 * equalities h(x)=0 and scalar inequalities g(x)<=0. Inequalities use the
 * Powell--Hestenes--Rockafellar term
 *
 *   (max(0, lambda + rho g)^2 - lambda^2) / (2 rho).
 *
 * The BCL policy is inspired by Algorithm 1 of Conn, Gould, and Toint,
 * "A Globally Convergent Augmented Lagrangian Algorithm for Optimization with
 * General Constraints and Simple Bounds," SIAM J. Numer. Anal. 28(2), 1991,
 * https://doi.org/10.1137/0728030, but is not entirely faithful. It replaces
 * the paper's projected bound-constrained inner solve with unconstrained LM
 * and full augmented-Lagrangian gradient stationarity, and handles
 * inequalities directly with PHR terms rather than bounded slack variables.
 * Consequently, the paper's convergence proof does not apply to this
 * implementation.
 *
 * Custom smoothed inequality penalties are intentionally unsupported because
 * they invalidate the projected PHR multiplier identity.
 *
 * Both ALM frontends delegate penalty updates, damping, inner solves, and
 * convergence tests to one internal engine. Each optimize() call owns a
 * persistent Gaussian step graph and NonlinearMultifrontalSolver. Original
 * nonlinear linearizations supply both the step and stationarity tests. Costs
 * must linearize to Gaussian factors supported by that solver, with fixed
 * factor slots, keys, and dimensions. lmParams controls damping, ordering,
 * multifrontal storage, iteration limits, and Aggressive convergence
 * tolerances; its linearSolverType, iterationHook, verbosity, and logFile
 * settings are not used.
 */
class GTSAM_EXPORT AugmentedLagrangianOptimizer : public ConstrainedOptimizer {
 public:
  using Base = ConstrainedOptimizer;
  using This = AugmentedLagrangianOptimizer;
  using shared_ptr = std::shared_ptr<This>;

  using Params = AugmentedLagrangianParams;
  using State = AugmentedLagrangianState;
  using Progress = std::vector<AugmentedLagrangianState>;

 protected:
  Params::shared_ptr p_;
  mutable Progress progress_;

 public:
  /// Construct from a constrained optimization problem.
  AugmentedLagrangianOptimizer(
      const ConstrainedOptProblem& problem, const Values& initialValues,
      Params::shared_ptr p = std::make_shared<Params>())
      : Base(problem, initialValues), p_(p), progress_() {}

  /// Construct by unpacking costs and constraints from one factor graph.
  AugmentedLagrangianOptimizer(
      const NonlinearFactorGraph& graph, const Values& initialValues,
      Params::shared_ptr p = std::make_shared<Params>())
      : Base(graph, initialValues), p_(p), progress_() {}

  /// Run the selected augmented-Lagrangian update policy and return its values.
  Values optimize() const override;

  /** Run the same optimization and return its final values and diagnostics.
   * Each call starts from initialValues_ with fresh multipliers and retains one
   * workspace across its outer iterations. The returned state describes either
   * convergence or the stopping condition; callers must check converged.
   * On BCL convergence, its projected multipliers correspond to the reported
   * augmented-Lagrangian stationarity, even if the intermediate feasibility
   * target would otherwise request a penalty increase.
   * No iteration history is stored unless storeOptProgress is enabled.
   */
  State optimizeWithState() const;

  /**
   * Solve one fixed-multiplier subproblem and perform one outer update.
   * `muEq` and `muIneq` are the direct penalties used for this subproblem and
   * the returned pair contains the penalties for the next subproblem. BCL
   * requires the two supplied penalties to be equal because Algorithm 1 uses
   * one penalty for the combined constraint vector. Aggressive permits distinct
   * equality and inequality penalties.
   */
  std::tuple<State, double, double> iterate(const State& state, double muEq,
                                            double muIneq) const;

  /// Return stored progress; populated when storeOptProgress is true.
  const Progress& progress() const { return progress_; }

  /**
   * Build the least-squares representation of the fixed-parameter augmented
   * Lagrangian. The represented objective differs from the mathematical PHR
   * objective only by constants independent of x. `epsilon` is retained for
   * source compatibility and ignored.
   */
  NonlinearFactorGraph augmentedLagrangianFunction(const State& state,
                                                   double epsilon = 1.0) const;

 protected:
  /// Private workspace shared by all subproblems in one optimize() call.
  struct Workspace;

  /** Solve and update using a lazily initialized, persistent workspace. */
  std::tuple<State, double, double> iterate(
      const State& state, double muEq, double muIneq,
      std::unique_ptr<Workspace>& workspace) const;

  /// Update Aggressive multipliers from the newly solved point.
  void updateLagrangeMultiplier(const State& subproblemState,
                                State* solvedState) const;

  /// Compute Aggressive direct penalties for the next subproblem.
  std::pair<double, double> updatePenaltyParameter(
      const State& previousState, const State& solvedState) const;

  /// Validate policy parameters and supported inequality constraints.
  void validateConfiguration() const;

  /// Store and log the initial state of the optimization.
  void logInitialState(const State& state) const;

  /// Store and log the state after an iteration of the optimization.
  void logIteration(const State& state) const;
};

}  // namespace gtsam

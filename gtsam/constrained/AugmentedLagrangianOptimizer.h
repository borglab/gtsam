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

#include <gtsam/constrained/ConstrainedOptimizer.h>
#include <gtsam/constrained/PenaltyOptimizer.h>

#include <limits>

namespace gtsam {

/** Outer update policy used by AugmentedLagrangianOptimizer. */
enum class AugmentedLagrangianUpdatePolicy { Aggressive, BCL };

/** Update performed after an augmented-Lagrangian subproblem. */
enum class AugmentedLagrangianUpdateType {
  None,
  Multiplier,
  Penalty,
  MultiplierAndPenalty
};

/** Parameters for the augmented Lagrangian method. */
class GTSAM_EXPORT AugmentedLagrangianParams : public PenaltyOptimizerParams {
 public:
  using Base = PenaltyOptimizerParams;
  using This = AugmentedLagrangianParams;
  using shared_ptr = std::shared_ptr<AugmentedLagrangianParams>;

  /// Outer update policy. BCL is the default; Aggressive remains available.
  AugmentedLagrangianUpdatePolicy updatePolicy =
      AugmentedLagrangianUpdatePolicy::BCL;

  /// Maximum equality-multiplier step for the Aggressive policy.
  double maxDualStepSizeEq = 10.0;
  /// Maximum inequality-multiplier step for the Aggressive policy.
  double maxDualStepSizeIneq = 10.0;
  /// Equality dual-step factor for the Aggressive policy.
  double dualStepSizeFactorEq = 1.0;
  /// Inequality dual-step factor for the Aggressive policy.
  double dualStepSizeFactorIneq = 1.0;
  /// Required violation reduction before Aggressive keeps its penalty fixed.
  double muIncreaseThreshold = 0.25;

  /// Absolute full augmented-Lagrangian gradient tolerance for BCL convergence.
  double absoluteStationarityTolerance = 1e-5;

  /**
   * Initial common direct penalty rho for BCL (paper default: 1 / mu_0).
   * Algorithm 1 has one penalty for the complete constraint vector, so BCL
   * applies this rho to both equality and inequality blocks. Constraint sigmas
   * provide fixed relative scaling. Aggressive may use separate penalties.
   */
  double bclInitialPenalty = 10.0;
  /// BCL direct-penalty growth factor (paper default: 1 / tau).
  double bclPenaltyIncreaseRate = 100.0;
  /// BCL omega_0 stationarity scale.
  double bclOmega0 = 1.0;
  /// BCL eta_0 feasibility scale.
  double bclEta0 = 1.0;
  /// BCL gamma_1 cap used to define alpha.
  double bclGamma1 = 0.1;
  /// BCL alpha_omega exponent used when resetting omega.
  double bclAlphaOmega = 1.0;
  /// BCL beta_omega exponent used when tightening omega.
  double bclBetaOmega = 1.0;
  /// BCL alpha_eta exponent used when resetting eta.
  double bclAlphaEta = 0.1;
  /// BCL beta_eta exponent used when tightening eta.
  double bclBetaEta = 0.9;

  using Base::Base;
};

/** Diagnostics and algorithm state for one outer ALM iteration. */
class GTSAM_EXPORT AugmentedLagrangianState : public PenaltyOptimizerState {
 public:
  using Base = PenaltyOptimizerState;
  using This = AugmentedLagrangianState;
  using shared_ptr = std::shared_ptr<This>;

  /// Equality Lagrange multipliers, in whitened constraint coordinates.
  std::vector<Vector> lambdaEq;
  /// Nonnegative inequality Lagrange multipliers, one per scalar constraint.
  std::vector<double> lambdaIneq;

  /// Infinity norm of the full augmented-Lagrangian gradient.
  double augmentedLagrangianStationarity =
      std::numeric_limits<double>::infinity();
  /// max(||h||_inf, ||q||_inf), including inequality complementarity.
  double generalizedConstraintViolation =
      std::numeric_limits<double>::infinity();
  /// Infinity norm of max(g, 0) for whitened inequalities.
  double primalInequalityViolation = 0.0;
  /// Infinity norm of lambda^+ times g for whitened inequalities.
  double complementarity = 0.0;

  /// Total LM iterations accumulated across all outer iterations.
  size_t totalUnconstrainedIterations = 0;
  /// Whether LM met the requested BCL stationarity target.
  bool innerConverged = true;
  /// Whether the constrained convergence test was met.
  bool converged = false;
  /// Outer update performed after solving this subproblem.
  AugmentedLagrangianUpdateType updateType =
      AugmentedLagrangianUpdateType::None;

  /// BCL alpha for the next subproblem.
  double bclAlpha = 0.0;
  /// BCL stationarity tolerance for the next subproblem.
  double bclOmega = 0.0;
  /// BCL generalized-feasibility tolerance for the next subproblem.
  double bclEta = 0.0;

  /** Initialize Lagrange multipliers as zeros. */
  void initializeLagrangeMultipliers(const ConstrainedOptProblem& problem);

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

  /// Run the selected augmented-Lagrangian update policy.
  Values optimize() const override;

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
  /// Create the LM optimizer used for an augmented-Lagrangian subproblem.
  SharedOptimizer createUnconstrainedOptimizer(
      const NonlinearFactorGraph& graph, const Values& values) const;

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

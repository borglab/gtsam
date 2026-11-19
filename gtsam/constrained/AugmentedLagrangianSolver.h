/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/** @file AugmentedLagrangianSolver.h
 * Gaussian-factor augmented Lagrangian solver and shared ALM parameters.
 */
#pragma once

#include <gtsam/constrained/PenaltyOptimizer.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>

#include <cmath>
#include <limits>

namespace gtsam {

/** Outer update policy shared by the Gaussian solver and nonlinear optimizer.
 */
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

  /**
   * Absolute full augmented-Lagrangian gradient tolerance for BCL convergence.
   * Also bounds the inner stationarity target from below, so feasibility
   * updates do not require solving subproblems beyond the final accuracy.
   */
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

  /// Collect phase wall times in statistics; leave false for timing
  /// comparisons.
  bool enableProfiling = false;

  using Base::Base;
};

/** Reason an ALM run returned. Limits return the best available point. */
enum class AugmentedLagrangianStatus {
  NotConverged,
  Converged,
  InnerIterationLimit,
  OuterIterationLimit,
  DampingLimit,
  NumericalFailure
};

/** Per-call work counters; no numerical workspace is shared between calls. */
struct AugmentedLagrangianStatistics {
  size_t modelLinearizations =
      0;                    ///< Nonlinear model evaluations with Jacobians.
  size_t modelUpdates = 0;  ///< Updates of the Gaussian step model.
  size_t symbolicFactorizations = 0;  ///< Symbolic workspace constructions.
  size_t numericalEliminations = 0;   ///< Attempted numerical eliminations.
  size_t objectiveFactorCopies = 0;  ///< Explicit copies during Gaussian setup.
  size_t constraintCoefficientUpdates = 0;  ///< Updated augmented row matrices.
  size_t roundoffChecks = 0;  ///< Trial-gradient checks at cost roundoff.
  double preparationSeconds =
      0;  ///< Gaussian preparation and symbolic analysis.
  double evaluationSeconds =
      0;  ///< Objective/constraint evaluation and derivatives.
  double assemblySeconds =
      0;  ///< Gaussian step-model/RHS assembly and loading.
  double eliminationSeconds = 0;  ///< Numerical elimination including damping.
  double backSubstitutionSeconds = 0;  ///< Triangular solution recovery.
  double diagnosticsSeconds =
      0;  ///< Gradients, predicted reduction and diagnostics.
};

/** Shared primal-independent ALM diagnostics and multiplier state. */
struct AugmentedLagrangianDiagnostics {
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

  /// Reason the run returned.
  AugmentedLagrangianStatus status = AugmentedLagrangianStatus::NotConverged;
  /// Work performed by this optimization call.
  AugmentedLagrangianStatistics statistics;
};

/** Final Gaussian ALM state. Constraint multipliers use whitened coordinates.
 * Equality blocks follow the input graph; inequality multipliers follow scalar
 * rows in graph order. Native convergence tests stationarity and generalized
 * feasibility, not an absolute QP duality gap.
 */
struct AugmentedLagrangianSolverState : AugmentedLagrangianDiagnostics {
  VectorValues values;               ///< Current direct vector variables.
  size_t iteration = 0;              ///< Number of completed outer iterations.
  double cost = 0;                   ///< Original quadratic objective.
  double eqConstraintViolation = 0;  ///< Equality residual Euclidean norm.
  double ineqConstraintViolation = 0;  ///< Positive inequality residual norm.
  double time = 0;  ///< Last outer iteration wall time in seconds.
  double muEq = 0;  ///< Direct equality penalty used by the last subproblem.
  double muIneq =
      0;  ///< Direct inequality penalty used by the last subproblem.
  size_t unconstrainedIterations = 0;  ///< Last inner accepted-step count.

  /// Combined constraint violation used by the Aggressive policy.
  double violation() const {
    return std::hypot(eqConstraintViolation, ineqConstraintViolation);
  }
};

/** Augmented Lagrangian solver for convex QPs supplied as Gaussian factors.
 * Objective factors retain native Jacobian/Hessian representations. Equality
 * rows encode E*x=d, and inequality rows encode A*x<=b. Constraints must be
 * Jacobian factors with unit or finite positive diagonal noise scales; their
 * graph determines equality/inequality semantics, not constrained noise.
 * Numerical workspaces are local to a call; optional history is retained in
 * the solver object. Inputs must remain unchanged during optimization. Initial values may violate constraints.
 */
class GTSAM_EXPORT AugmentedLagrangianSolver {
 public:
  using Params = AugmentedLagrangianParams;
  using State = AugmentedLagrangianSolverState;
  using Progress = std::vector<State>;

  /** Store Gaussian inputs and parameters; prepare numerical storage per call.
   */
  AugmentedLagrangianSolver(const GaussianFactorGraph& costs,
                            const GaussianFactorGraph& equalities,
                            const GaussianFactorGraph& inequalities,
                            Params::shared_ptr params = nullptr);

  /// Optimize from zeros inferred from all objective and constraint keys.
  VectorValues optimize() const;
  /// Optimize from finite, dimensionally consistent direct vector values.
  VectorValues optimize(const VectorValues& initialValues) const;
  /// Optimize from zeros and retain the final convergence diagnostics.
  State optimizeWithState() const;
  /// Optimize from supplied vectors and retain final convergence diagnostics.
  State optimizeWithState(const VectorValues& initialValues) const;
  /// Return history populated only when storeOptProgress is enabled.
  const Progress& progress() const { return progress_; }

 private:
  /// Shared implementation for explicit and inferred initial values.
  State run(const VectorValues* initialValues) const;

  GaussianFactorGraph costs_, equalities_, inequalities_;
  Params::shared_ptr params_;
  mutable Progress progress_;
};

}  // namespace gtsam

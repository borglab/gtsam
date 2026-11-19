/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    ActiveSetSolver.h
 * @brief   Generic active-set solver for linearly constrained LPs and QPs.
 * @author  Frank Dellaert
 */

#pragma once

#include <gtsam/constrained/LpProblem.h>
#include <gtsam/constrained/QpProblem.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>

#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace gtsam {

/** Parameters for ActiveSetSolver. */
class GTSAM_EXPORT ActiveSetSolverParams {
 public:
  using shared_ptr = std::shared_ptr<ActiveSetSolverParams>;

  /// Formulation used by optimize(); independent of the numerical backend.
  enum class QpFormulation { Primal, Dual, Automatic };

  /** Primal preserves feasible active-set descent. Dual starts with nonnegative
   * multipliers and permits primal-infeasible initial values. Automatic chooses
   * Dual for cold starts and retains the formulation of a converged warm start
   * whose supplied point is feasible. Equality-only and fully fixed problems
   * use Primal. Singular reduced proximal curvature falls back to Primal during
   * setup only. Explicit
   * Dual requires invertible reduced curvature (positive regularization permits
   * semidefinite objectives). LPs resolve Automatic to Primal and reject Dual.
   * computeFeasibleStep() always uses Primal.
   */
  QpFormulation qpFormulation = QpFormulation::Automatic;

  /// Strategy used to solve equality-constrained QP active-set subproblems.
  enum class QpSubproblemSolver {
    /// Eliminate a sparse working graph, reusing its ordering and optionally
    /// its Bayes tree while the variable keys stay unchanged.
    Sparse,
    /// Pack dense objective/constraint blocks and solve rank-aware working
    /// systems, preserving original rows and square-root costs.
    Dense
  };

  /** How a cold Primal QP selects its initial point and working set. All modes keep
   * the original objective and constraints; only the last mode may move the
   * supplied point. An explicit working-set warm start bypasses these modes.
   */
  enum class QpInitialization {
    /// Preserve the point and initially activate every tight inequality.
    TightConstraints,
    /// Preserve the point, then release strictly inward rows using the
    /// negative objective gradient projected onto the feasible tangent cone.
    ProjectedGradient,
    /// Attempt one feasible working-face descent step before selecting tight
    /// rows and projecting the gradient at the resulting point. An unhelpful
    /// step retains the starting point; numerical failures propagate.
    FeasibleStepThenProjectedGradient
  };

  /// Maximum number of main working solves across active-set/proximal iterations,
  /// including the preliminary face solve in FeasibleStepThenProjectedGradient.
  size_t maxIterations = 1000;
  /// Distance at which an inequality is initially considered active.
  double activeTolerance = 1e-7;
  /// Step size used to inspect LP and unregularized QP optimality.
  double stepTolerance = 1e-7;
  /// Maximum allowed primal constraint residual.
  double feasibilityTolerance = 1e-7;
  /// Absolute tolerance for multiplier signs and stationarity. Also supplies
  /// the absolute and relative tolerances for the constant-free QP gap, with
  /// an additional allowance for floating-point evaluation error.
  double multiplierTolerance = 1e-7;
  /// Recentered proximal weight for QP working subproblems. Both backends
  /// certify the original objective; this never adds a permanent penalty.
  double regularization = 1e-9;
  /// Maximum residual accepted from the cold-start feasibility phase.
  double phaseOneFeasibilityTolerance = 1e-6;
  /// Numerical backend, defaulting to Sparse independently of formulation.
  QpSubproblemSolver qpSubproblemSolver = QpSubproblemSolver::Sparse;
  /** Reduce Dense QPs to permanent equality-feasible coordinates before solving.
   * When false, keep the original coordinates and enforce equalities in the
   * working factorization. Applies to both Primal and Dual; ignored by Sparse.
   */
  bool projectDenseQpEqualities = true;
  /// Reuse primal sparse elimination across working-set and objective RHS changes.
  /// Dual uses its own fixed-factorization cache regardless of this option.
  /// A proximal-center change uses cached triangular solves without elimination.
  bool useIncrementalSparseQp = true;
  /** Primal initialization for optimize() and computeFeasibleStep(). Ignored
   * by dual optimization; computeFeasibleStep() always honors this option.
   * FeasibleStepThenProjectedGradient requires a fully feasible starting point
   * and counts its single face-solve attempt against maxIterations. Both
   * projection modes solve independent tangent-cone components; each auxiliary
   * projection has its own maxIterations budget and propagates failure. Their
   * iterations are excluded from main iteration counts, but all initialization
   * work is part of the call. LPs accept only TightConstraints. Supplied values
   * are the starting point; supplying a working-set warm start bypasses this
   * option. Backend, regularization, tolerances and ordering remain in effect.
   * For example:
   * @code
   * params.qpInitialization = ActiveSetSolverParams::QpInitialization::
   *     FeasibleStepThenProjectedGradient;
   * @endcode
   */
  QpInitialization qpInitialization = QpInitialization::TightConstraints;

  /** Optional complete permutation of the problem's variable keys. Sparse
   * working graphs and auxiliary projections restrict it to their current
   * keys; phase one appends its slack key. Dense QPs use it for vector
   * packing and cost factorization. Multiplier keys are internal and use a separate COLAMD
   * ordering. Unset means automatic COLAMD.
   */
  std::optional<Ordering> ordering;

  /// Set a custom variable-elimination ordering.
  void setOrdering(const Ordering& ordering) { this->ordering = ordering; }
  /// Return to automatic COLAMD ordering.
  void clearOrdering() { ordering.reset(); }
  /// Return whether a custom ordering was supplied.
  bool hasOrdering() const { return ordering.has_value(); }
};

/** Final active-set state, including row-ordered multipliers. */
class GTSAM_EXPORT ActiveSetSolverState {
 public:
  using shared_ptr = std::shared_ptr<ActiveSetSolverState>;

  /// Actual optimization formulation, resolved once during setup.
  ActiveSetSolverParams::QpFormulation qpFormulation =
      ActiveSetSolverParams::QpFormulation::Primal;
  Values values;
  std::vector<Vector> equalityMultipliers;
  std::vector<Vector> inequalityMultipliers;
  std::vector<bool> activeInequalityRows;
  bool converged = false;
  /// Main iterations including any preliminary initialization face solve;
  /// excludes auxiliary projected-gradient solves.
  size_t iterations = 0;
};

/** A feasible-step outcome, with no claim of QP optimality or dual validity. */
class GTSAM_EXPORT ActiveSetFeasibleStepResult {
 public:
  /// Outcome of the bounded search, including the selected QP initialization.
  enum class Status { FeasibleProgress, NoProgress, NumericalFailure, IterationLimit };

  Values values;
  std::vector<bool> activeInequalityRows;
  Status status = Status::NoProgress;
  /// Main working-face solves attempted, including the preliminary step;
  /// excludes auxiliary projection solves.
  size_t iterations = 0;
  double initialObjective = 0;
  double objective = 0;
  /// Original-objective decrease evaluated without subtracting constant terms;
  /// more reliable than initialObjective - objective when offsets are large.
  double objectiveDecrease = 0;
  double constraintViolation = 0;
  std::string message;

  /** Return a 0/1 working-set mask in original row order for array-based callers. */
  Vector activeInequalityMask() const {
    Vector mask(activeInequalityRows.size());
    for (size_t row = 0; row < activeInequalityRows.size(); ++row)
      mask(row) = activeInequalityRows[row] ? 1.0 : 0.0;
    return mask;
  }
};

/**
 * Active-set solver for LPs and QPs with shared LinearConstraint constraints.
 *
 * QP subproblems use sparse working factor graphs with optional Bayes-tree
 * reuse, or packed dense systems with rank-aware constraint handling. Both
 * backends regularize working subproblems around the current point and check
 * convergence against the original, unregularized QP. LP subproblems project
 * the negative objective gradient onto the active constraint surface and
 * detect unbounded descent when no inactive inequality blocks that direction.
 */
class GTSAM_EXPORT ActiveSetSolver {
 public:
  using Params = ActiveSetSolverParams;
  using State = ActiveSetSolverState;
  using shared_ptr = std::shared_ptr<ActiveSetSolver>;

  /** Construct a solver for a QpProblem, copying and validating its parameters. */
  explicit ActiveSetSolver(
      const QpProblem& problem,
      Params::shared_ptr params = std::make_shared<Params>());

  /** Construct a solver for a LpProblem, copying and validating its parameters. */
  explicit ActiveSetSolver(
      const LpProblem& problem,
      Params::shared_ptr params = std::make_shared<Params>());

  /** Optimize from finite values, inequality-feasible when using Primal.
   * Equality constraints are enforced by the working subproblems. Primal
   * two-stage initialization additionally requires equality-feasible values.
   * Dual accepts infeasible values as a proximal center and skips initialization.
   */
  Values optimize(const Values& initialValues) const;

  /** Optimize from an initial point and return the final state. */
  std::pair<Values, State> optimizeWithState(const Values& initialValues) const;

  /** Optimize with active-set warm start from a previous state, bypassing
   * qpInitialization and using initialValues as the starting point.
   */
  std::pair<Values, State> optimizeWithState(const Values& initialValues,
                                             const State& warmStart) const;

  /** Optimize from a zero proximal center with Dual, or find a vector-valued
   * feasible point before Primal optimization. */
  Values optimize() const;

  /** Optimize with the selected formulation and return the final state. Dual
   * starts from a zero proximal center; Primal first finds a feasible point. */
  std::pair<Values, State> optimizeWithState() const;

  /**
   * Seek a feasible QP descent step from a finite, feasible initial point.
   * Select the initial face using the same initialization options as optimize.
   * Respect the configured QP backend, regularization and tolerances.
   * Truncate at the first blocking
   * inequality; zero-length blocks update the working set within maxIterations.
   * A useful step must exceed stepTolerance in infinity norm and strictly
   * decrease the original objective. No multiplier or optimality test is made.
   * FeasibleStepThenProjectedGradient completes its preliminary face solve and
   * projected working-set selection, then returns any useful descent already
   * obtained there without taking another face step. Its preliminary attempt
   * shares the iteration budget and sparse workspace with subsequent search.
   *
   * Invalid initial values, infeasible points and LPs throw. Numerical failures
   * and iteration limits are returned explicitly, including initialization
   * failures. Non-progress outcomes retain the supplied point. NoProgress does
   * not establish optimality. Returned flags describe the selected working set
   * in original inequality-row order; they are not multiplier certificates.
   */
  ActiveSetFeasibleStepResult computeFeasibleStep(const Values& initialValues) const;

 private:
  enum class ProblemType { LP, QP };
  struct DenseQpWorkspace;
  struct SolveWorkspace;

  struct ValueShape {
    enum class Type { Vector, Matrix };

    Type type = Type::Vector;
    size_t rows = 0;
    size_t cols = 1;

    size_t dim() const { return rows * cols; }
  };

  struct ConstraintRow {
    size_t constraintIndex = 0;
    size_t row = 0;
    Key multiplierKey = 0;
    JacobianFactor::shared_ptr factor;
  };

  struct SparseConstraintData {
    std::vector<ConstraintRow> equalities;
    std::vector<size_t> equalityDims;
    std::vector<ConstraintRow> inequalities;
    std::vector<size_t> inequalityDims;
  };

  struct InternalState {
    VectorValues values;
    VectorValues duals;
    std::vector<bool> activeInequalityRows;
    bool converged = false;
    size_t iterations = 0;
  };

  ProblemType problemType_;
  const QpProblem* qpProblem_ = nullptr;
  const LpProblem* lpProblem_ = nullptr;
  Params::shared_ptr params_;
  std::map<Key, size_t> keyDims_;
  GaussianFactorGraph qpCostGraph_;
  VectorValues qpGradientAtZero_;
  std::map<Key, Vector> lpGradient_;
  SparseConstraintData constraints_;
  std::shared_ptr<DenseQpWorkspace> denseQpWorkspace_;

  /// Resolve formulation and prepare equality-constrained dual algebra once.
  bool prepareDualQp(SolveWorkspace* workspace, bool preferPrimal = false) const;

  /// Reject malformed row masks, multiplier blocks, or unresolved formulations.
  void validateWarmStart(const State& warmStart) const;
  /// Optimize dual multipliers, recentering until the original QP is certified.
  std::pair<Values, State> optimizeDualQp(const Values& initialValues,
      const State* warmStart, SolveWorkspace* workspace) const;

  /// Run the active-set loop with this call's feasibility/initialization cache.
  std::pair<Values, State> optimizeWithState(const Values& initialValues,
                                             const State* warmStart,
                                             SolveWorkspace* workspace) const;
  /// Advance the active set using this optimization call's backend caches.
  InternalState iterate(const InternalState& state,
                        SolveWorkspace* workspace) const;

 private:
  // Internal helpers: setup and value representation.
  /// Record the dimensions of all keys in the problem.
  void initializeDimensions();
  /** Validate the initialization mode, tolerances and complete custom ordering. */
  void validateParameters() const;
  /// Build the stored QP graph or LP gradient.
  void initializeObjective();
  /// Allocate the workspace used only by dense QP mode.
  void initializeDenseQpWorkspace();
  /// Pack original weighted Jacobians and separate Hessian-native curvature.
  void packDenseObjective();
  /// Convert native PSD Hessian curvature once without adding a damping floor.
  void packDenseHessian(const Matrix& hessian, size_t rowStart);
  /// Pack every original constraint row once in stable multiplier order.
  void packDenseConstraints();
  /// Split and normalize constraints into scalar rows.
  void initializeConstraints();
  /// Check the types and dimensions of supplied initial values.
  void validateInitialValues(const Values& values) const;
  /// Create zero vectors for every key in the problem.
  Values zeroVectorValues() const;
  /// Record whether each initial value was a vector or matrix.
  std::map<Key, ValueShape> valueShapes(const Values& values) const;
  /// Flatten supported Values entries into vector-valued entries.
  VectorValues valuesToVectorValues(const Values& values) const;
  /// Restore the original vector and matrix value shapes.
  Values vectorValuesToValues(const VectorValues& values,
                              const Values& prototype) const;

  // Internal helpers: working set and optimality.
  /** Search for feasible descent using the shared main-iteration budget and
   * elimination workspace. Failed attempts still update state.iterations.
   */
  void searchFeasibleStep(InternalState* state, const Values& prototype,
                          SolveWorkspace* workspace,
                          ActiveSetFeasibleStepResult* result) const;
  /** Solve the configured QP working face and truncate at the first blocker.
   * Count the attempt before elimination and check every original constraint.
   */
  void takeFeasibleStep(InternalState* state, SolveWorkspace* workspace) const;
  /// Check and record a feasible displacement without assigning optimality.
  bool inspectFeasibleStep(const InternalState& next, const Values& prototype,
                           ActiveSetFeasibleStepResult* result) const;
  /// Evaluate original quadratic decrease independently of additive constants.
  double quadraticDecrease(const VectorValues& initial,
                            const VectorValues& candidate) const;
  /// Assemble the LP gradient projection and its active constraints.
  GaussianFactorGraph buildLpWorkingGraph(
      const VectorValues& values,
      const std::vector<bool>& activeInequalityRows) const;
  /// Solve a sparse working graph using this call's ordering and tree caches.
  VectorValues solveSparseSubproblem(const InternalState& state,
                                     SolveWorkspace* workspace) const;
  /// Assemble A_active^T lambda = gradient for the original or proximal QP.
  /// A nonnull center adds regularization * (values - center) to the gradient.
  /// An optional original gradient reuses this same candidate evaluation.
  GaussianFactorGraph buildDualGraph(
      const SparseConstraintData& constraints,
      const std::vector<bool>& activeInequalityRows, const VectorValues& values,
      const VectorValues* proximalCenter = nullptr,
      const VectorValues* originalGradient = nullptr) const;
  /// Decide when a working-set point is ready for a multiplier check.
  bool shouldInspectOptimality(const InternalState& state,
                               const VectorValues& candidate) const;
  /** Reuse the dense primal LDLT and permanent equality factors for multipliers.
   * Return false when dependent rows or proximal-only stationarity require
   * graph recovery. Share the exact original gradient with the final checks.
   */
  bool recoverDenseMultipliers(const VectorValues& candidate,
                               const SolveWorkspace& workspace,
                               VectorValues* originalGradient,
                               VectorValues* duals) const;
  /// Recover original-QP multipliers, or return false for a proximal-only step.
  bool recoverMultipliers(const InternalState& state,
                           const VectorValues& candidate,
                           SolveWorkspace* workspace,
                           VectorValues* duals,
                           VectorValues* originalGradient) const;
  /// Certify the original QP's primal residual, stationarity, and gap.
  /// A supplied gradient belongs to these exact values, within this iteration.
  bool originalQpKktSatisfied(const VectorValues& values,
                               const std::vector<bool>& activeRows,
                               const VectorValues& duals,
                               const VectorValues* originalGradient = nullptr) const;
  /// Line-search toward a working-set point and add the first blocking row.
  void moveTowardCandidate(const InternalState& state,
                           const VectorValues& candidate,
                           InternalState* next) const;
  /// Evaluate the QP or LP objective gradient at one key.
  Vector objectiveGradient(Key key, const VectorValues& values) const;

  // Internal helpers: dense QP subproblems.
  /// Prepare one per-call Dense backend, shared from feasibility through solving.
  void prepareDenseWorkspace(SolveWorkspace* workspace) const;
  /// Flatten vector values using the dense workspace ordering.
  Vector vectorValuesToDenseVector(const VectorValues& values) const;
  /// Recover vector values from the dense workspace ordering.
  VectorValues denseVectorToVectorValues(const Vector& vector) const;
  /// Solve a dense working face with this optimization call's numerical cache.
  VectorValues solveDenseQpSubproblem(const InternalState& state,
                                      SolveWorkspace* workspace) const;

  // Internal helpers: state and cold-start feasibility.
  /** Complete cold QP initialization using the main solve's workspace.
   * Callers with explicit working-set warm starts bypass this helper.
   */
  void initializeQp(InternalState* state, SolveWorkspace* workspace) const;
  /** Attempt one face step from a fully feasible point. Accept only useful
   * original-objective descent, then rebuild all tight rows before projection.
   */
  void initializeFeasiblePoint(InternalState* state,
                               SolveWorkspace* workspace) const;
  /** Rebuild the working set from all tight rows at the current point. */
  void initializeTightRows(InternalState* state) const;
  /** Select tight rows using the projected gradient without moving the point.
   * Components follow equality and tight-inequality connectivity, independently
   * of objective coupling. Strict inward derivatives release rows; all original
   * constraints remain in the main solve.
   */
  void initializeProjectedGradient(InternalState* state,
                                    SolveWorkspace* workspace) const;
  /** Project one component through the existing Dense active-set solver in
   * cached equality-free tangent coordinates. Preserve its gradient scale and
   * iteration budget, then check the lifted direction against original rows.
   */
  Values projectDenseGradient(
      const VectorValues& gradient, const std::set<Key>& keys,
      const std::vector<size_t>& rows, double scale,
      SolveWorkspace* workspace) const;
  /** Preserve supplied values and select warm-start rows or all tight rows.
   * The QP initialization strategy is applied separately by initializeQp().
   */
  InternalState initialState(const Values& values,
                             const State* warmStart) const;
  /// Restore constraint row order and value shapes in the public state.
  State buildPublicState(const InternalState& state,
                         const Values& prototype) const;
  /// Find a feasible starting point for a vector-valued problem.
  Values findVectorFeasiblePoint(SolveWorkspace* workspace) const;
  /// Reuse Dense equality preparation, or solve the sparse minimum-norm graph.
  bool tryEqualityProjection(const Values& zeroValues, Values* feasible,
                              SolveWorkspace* workspace) const;
  /// Solve phase one and discard its synthetic slack variable.
  Values solvePhaseOne(const Values& zeroValues) const;
};

}  // namespace gtsam

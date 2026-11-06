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
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace gtsam {

/** Parameters for ActiveSetSolver. */
class GTSAM_EXPORT ActiveSetSolverParams {
 public:
  using shared_ptr = std::shared_ptr<ActiveSetSolverParams>;

  /// Strategy used to solve equality-constrained QP active-set subproblems.
  enum class QpSubproblemSolver {
    /// Eliminate a sparse working graph, reusing its ordering and optionally
    /// its Bayes tree while the variable keys stay unchanged.
    Sparse,
    /// Reuse one Hessian factorization and solve dense active-set Schur
    /// systems.
    Dense
  };

  /// Maximum number of active-set or proximal iterations.
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
  /// Sparse QPs use this as a recentered proximal weight; dense QPs add it to
  /// the factored Hessian.
  double regularization = 1e-9;
  /// Maximum residual accepted from the cold-start feasibility phase.
  double phaseOneFeasibilityTolerance = 1e-6;
  QpSubproblemSolver qpSubproblemSolver = QpSubproblemSolver::Sparse;
  /// Reuse sparse QP elimination when additional regularization is zero.
  bool useIncrementalSparseQp = true;
  /** Seed cold QP working sets using componentwise Euclidean projection of the
   * negative gradient onto the feasible tangent cone. Explicit warm starts take
   * precedence; LPs are unchanged. Each component projection uses maxIterations
   * independently of the main solve and propagates failure. Disabled by
   * default.
   */
  bool useProjectedGradientInitialization = false;
};

/** Final active-set state, including row-ordered multipliers. */
class GTSAM_EXPORT ActiveSetSolverState {
 public:
  using shared_ptr = std::shared_ptr<ActiveSetSolverState>;

  Values values;
  std::vector<Vector> equalityMultipliers;
  std::vector<Vector> inequalityMultipliers;
  std::vector<bool> activeInequalityRows;
  bool converged = false;
  size_t iterations = 0;
};

/** A feasible-step outcome, with no claim of QP optimality or dual validity. */
class GTSAM_EXPORT ActiveSetFeasibleStepResult {
 public:
  /// Outcome of the bounded search, including projected-gradient initialization.
  enum class Status { FeasibleProgress, NoProgress, NumericalFailure, IterationLimit };

  Values values;
  std::vector<bool> activeInequalityRows;
  Status status = Status::NoProgress;
  /// Main working-face solves attempted; excludes auxiliary projection solves.
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
 * QP subproblems can either rebuild sparse working factor graphs or reuse one
 * Hessian factorization with dense active-set Schur complements. Sparse QPs
 * regularize each working graph around the current feasible point and check
 * convergence against the original, unregularized QP. LP subproblems project
 * the negative objective gradient onto the active constraint surface and
 * detect unbounded descent when no inactive inequality blocks that direction.
 */
class GTSAM_EXPORT ActiveSetSolver {
 public:
  using Params = ActiveSetSolverParams;
  using State = ActiveSetSolverState;
  using shared_ptr = std::shared_ptr<ActiveSetSolver>;

  /** Construct a solver for a QpProblem. */
  explicit ActiveSetSolver(
      const QpProblem& problem,
      Params::shared_ptr params = std::make_shared<Params>());

  /** Construct a solver for a LpProblem. */
  explicit ActiveSetSolver(
      const LpProblem& problem,
      Params::shared_ptr params = std::make_shared<Params>());

  /** Optimize from an initial point. */
  Values optimize(const Values& initialValues) const;

  /** Optimize from an initial point and return the final state. */
  std::pair<Values, State> optimizeWithState(const Values& initialValues) const;

  /** Optimize with active-set warm start from a previous state. */
  std::pair<Values, State> optimizeWithState(const Values& initialValues,
                                             const State& warmStart) const;

  /** Find a vector-valued feasible point and optimize. */
  Values optimize() const;

  /** Find a vector-valued feasible point, optimize, and return final state. */
  std::pair<Values, State> optimizeWithState() const;

  /**
   * Seek a feasible QP descent step from a finite, feasible initial point.
   * Select the initial face using the same initialization options as optimize.
   * Respect the configured QP backend, regularization and tolerances.
   * Truncate at the first blocking
   * inequality; zero-length blocks update the working set within maxIterations.
   * A useful step must exceed stepTolerance in infinity norm and strictly
   * decrease the original objective. No multiplier or optimality test is made.
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
  struct DenseConstraintSystem;
  struct SparseWorkspace;

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

  /// Run the active-set loop from the supplied initial values and state.
  std::pair<Values, State> optimizeWithState(const Values& initialValues,
                                             const State* warmStart) const;
  /// Advance the active set, reusing this optimization call's sparse orderings.
  InternalState iterate(const InternalState& state,
                        SparseWorkspace* workspace) const;

 private:
  // Internal helpers: setup and value representation.
  /// Record the dimensions of all keys in the problem.
  void initializeDimensions();
  /// Build the stored QP graph or LP gradient.
  void initializeObjective();
  /// Allocate the workspace used only by dense QP mode.
  void initializeDenseQpWorkspace();
  /// Factor the dense-mode cost, retrying with its regularization floor.
  void factorDenseQpCost();
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

  // Internal helpers: sparse working set and optimality.
  /// Run bounded working-face solves, stopping at the first feasible descent.
  void searchFeasibleStep(InternalState state, const Values& prototype,
                          ActiveSetFeasibleStepResult* result) const;
  /// Check and record a feasible displacement without assigning optimality.
  bool inspectFeasibleStep(const InternalState& next, const Values& prototype,
                           ActiveSetFeasibleStepResult* result) const;
  /// Evaluate original quadratic decrease independently of additive constants.
  double quadraticDecrease(const VectorValues& initial,
                            const VectorValues& candidate) const;
  /// Assemble the cost, proximal or LP descent term, and active constraints.
  GaussianFactorGraph buildWorkingGraph(
      const VectorValues& values,
      const std::vector<bool>& activeInequalityRows) const;
  /// Solve a sparse working graph using this call's ordering and tree caches.
  VectorValues solveSparseSubproblem(const InternalState& state,
                                     SparseWorkspace* workspace) const;
  /// Update active inequality factors and solve the unregularized QP tree.
  VectorValues solveIncrementalSparseQp(
      const GaussianFactorGraph& graph, const std::vector<bool>& activeRows,
      const GaussianFactorGraph::Eliminate& eliminate,
      SparseWorkspace* workspace) const;
  /// Assemble A_active^T lambda = gradient for the original or proximal QP.
  /// A nonnull center adds regularization * (values - center) to the gradient.
  GaussianFactorGraph buildDualGraph(
      const SparseConstraintData& constraints,
      const std::vector<bool>& activeInequalityRows, const VectorValues& values,
      const VectorValues* proximalCenter = nullptr) const;
  /// Decide when a working-set point is ready for a multiplier check.
  bool shouldInspectOptimality(const InternalState& state,
                               const VectorValues& candidate,
                               bool useDenseQp) const;
  /// Recover original-QP multipliers, or return false for a proximal-only step.
  bool recoverSparseMultipliers(const InternalState& state,
                                const VectorValues& candidate,
                                SparseWorkspace* workspace,
                                VectorValues* duals) const;
  /// Certify the original sparse QP's primal residual, stationarity, and gap.
  bool originalSparseKktSatisfied(const VectorValues& values,
                                  const std::vector<bool>& activeRows,
                                  const VectorValues& duals) const;
  /// Line-search toward a working-set point and add the first blocking row.
  void moveTowardCandidate(const InternalState& state,
                           const VectorValues& candidate,
                           InternalState* next) const;
  /// Evaluate the QP or LP objective gradient at one key.
  Vector objectiveGradient(Key key, const VectorValues& values) const;

  // Internal helpers: dense QP subproblems.
  /// Flatten vector values using the dense workspace ordering.
  Vector vectorValuesToDenseVector(const VectorValues& values) const;
  /// Recover vector values from the dense workspace ordering.
  VectorValues denseVectorToVectorValues(const Vector& vector) const;
  /// Apply the cached inverse Hessian through its factorization.
  Vector applyInverseDenseHessian(const Vector& vector) const;
  /// Solve the current equality-constrained dense QP.
  std::pair<VectorValues, VectorValues> solveDenseQpSubproblem(
      const std::vector<bool>& activeInequalityRows) const;
  /// Assemble active rows in the dense workspace's stable variable order.
  DenseConstraintSystem denseConstraintSystem(
      const std::vector<bool>& activeInequalityRows) const;
  /// Test whether the dense QP step is within the configured tolerance.
  bool denseQpStepIsSmall(const VectorValues& candidateValues,
                          const VectorValues& values) const;

  // Internal helpers: state and cold-start feasibility.
  /** Select tight rows using the projected gradient without moving the point.
   * Components follow equality and tight-inequality connectivity, independently
   * of objective coupling. Strict inward derivatives release rows; all original
   * constraints remain in the main solve.
   */
  void initializeProjectedGradient(InternalState* state) const;
  /// Initialize active rows from values or a warm start.
  InternalState initialState(const Values& values,
                             const State* warmStart) const;
  /// Restore constraint row order and value shapes in the public state.
  State buildPublicState(const InternalState& state,
                         const Values& prototype) const;
  /// Find a feasible starting point for a vector-valued problem.
  Values findVectorFeasiblePoint() const;
  /// Try the sparse QP's minimum-norm projection onto its equalities.
  bool tryEqualityProjection(const Values& zeroValues, Values* feasible) const;
  /// Solve phase one and discard its synthetic slack variable.
  Values solvePhaseOne(const Values& zeroValues) const;
};

}  // namespace gtsam

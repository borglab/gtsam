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
#include <vector>

namespace gtsam {

/** Parameters for ActiveSetSolver. */
class GTSAM_EXPORT ActiveSetSolverParams {
 public:
  using shared_ptr = std::shared_ptr<ActiveSetSolverParams>;

  /// Strategy used to solve equality-constrained QP active-set subproblems.
  enum class QpSubproblemSolver {
    /// Rebuild and eliminate a sparse working factor graph each iteration.
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
  /// Tolerance for multiplier signs, stationarity, and QP gap checks.
  double multiplierTolerance = 1e-7;
  /// Sparse QPs use this as a recentered proximal weight; dense QPs add it to
  /// the factored Hessian.
  double regularization = 1e-9;
  /// Maximum residual accepted from the cold-start feasibility phase.
  double phaseOneFeasibilityTolerance = 1e-6;
  QpSubproblemSolver qpSubproblemSolver = QpSubproblemSolver::Sparse;
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

 private:
  enum class ProblemType { LP, QP };
  struct DenseQpWorkspace;
  struct DenseConstraintSystem;

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
  std::map<Key, Vector> lpGradient_;
  SparseConstraintData constraints_;
  std::shared_ptr<DenseQpWorkspace> denseQpWorkspace_;

  /// Run the active-set loop from the supplied initial values and state.
  std::pair<Values, State> optimizeWithState(const Values& initialValues,
                                             const State* warmStart) const;
  /// Advance the current active set by one subproblem solve.
  InternalState iterate(const InternalState& state) const;

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
  /// Assemble the cost, proximal or LP descent term, and active constraints.
  GaussianFactorGraph buildWorkingGraph(
      const VectorValues& values,
      const std::vector<bool>& activeInequalityRows) const;
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

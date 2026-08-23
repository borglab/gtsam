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

  size_t maxIterations = 1000;
  double activeTolerance = 1e-7;
  double stepTolerance = 1e-7;
  double feasibilityTolerance = 1e-7;
  double multiplierTolerance = 1e-7;
  double regularization = 1e-9;
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
 * Hessian factorization with dense active-set Schur complements. LP subproblems
 * project the negative objective gradient onto the active constraint surface
 * and detect unbounded descent when no inactive inequality blocks that
 * direction.
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

  void initializeDimensions();
  void initializeObjective();
  void initializeDenseQpWorkspace();
  void initializeConstraints();
  void validateInitialValues(const Values& values) const;
  Values zeroVectorValues() const;
  Values findVectorFeasiblePoint() const;
  std::map<Key, ValueShape> valueShapes(const Values& values) const;
  VectorValues valuesToVectorValues(const Values& values) const;
  Values vectorValuesToValues(const VectorValues& values,
                              const Values& prototype) const;
  GaussianFactorGraph buildWorkingGraph(
      const VectorValues& values,
      const std::vector<bool>& activeInequalityRows) const;
  GaussianFactorGraph buildDualGraph(
      const SparseConstraintData& constraints,
      const std::vector<bool>& activeInequalityRows,
      const VectorValues& values) const;
  Vector vectorValuesToDenseVector(const VectorValues& values) const;
  VectorValues denseVectorToVectorValues(const Vector& vector) const;
  Vector applyInverseDenseHessian(const Vector& vector) const;
  std::pair<VectorValues, VectorValues> solveDenseQpSubproblem(
      const std::vector<bool>& activeInequalityRows) const;
  bool denseQpStepIsSmall(const VectorValues& candidateValues,
                          const VectorValues& values) const;
  Vector objectiveGradient(Key key, const VectorValues& values) const;
  InternalState initialState(const Values& values,
                             const State* warmStart) const;
  std::pair<Values, State> optimizeWithState(const Values& initialValues,
                                             const State* warmStart) const;
  InternalState iterate(const InternalState& state) const;
  State buildPublicState(const InternalState& state,
                         const Values& prototype) const;
};

}  // namespace gtsam

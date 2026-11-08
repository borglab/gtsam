/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file ActiveSetSolver.cpp
 * Shared active-set iteration and public solve entry points for LPs and QPs.
 *
 * Setup and value conversion live in ActiveSetSolverSetup.cpp; cold feasibility
 * and working-set initialization live in ActiveSetSolverInitialization.cpp.
 * Dense and sparse working solves, and sparse multiplier algebra, have separate
 * internal implementations. This file owns blocking, row release, feasible-step
 * status, and the original-problem convergence certificates.
 */

#include <gtsam/constrained/internal/ActiveSetSolverInternal.h>
#include <gtsam/constrained/internal/SparseDualSolver.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace gtsam {

using internal::IterationLimitError;
using internal::NextAvailableKey;
using internal::RowResidual;
using internal::SingleValue;
using internal::maximumStep;
using internal::primalFeasible;
using internal::restrictOrdering;
using internal::DualSolveResult;
using internal::solveSparseDualGraph;

namespace {

/* ************************************************************************* */
const Ordering& cachedOrdering(const GaussianFactorGraph& graph, KeySet* keys,
                               Ordering* ordering,
                               const std::optional<Ordering>& requested = {}) {
  const KeySet graphKeys = graph.keys();
  if (graphKeys != *keys) {
    *ordering = requested ? restrictOrdering(*requested, graphKeys)
                          : Ordering::Colamd(graph);
    *keys = graphKeys;
  }
  return *ordering;
}

/* ************************************************************************* */
// Exclude arbitrary additive constants from relative gap scaling. The error
// allowance covers cancellation in quadratic/linear terms at a floating-point
// optimum; primal, dual-sign and stationarity checks remain independent.
bool scaledGapSatisfied(double gap, double quadratic, double linear,
                        double dualRhs, double dualRhsMagnitude, double tolerance) {
  const double primal = 0.5 * quadratic + linear;
  const double dual = -0.5 * quadratic + dualRhs;
  const double scale = 1 + std::max(std::abs(primal), std::abs(dual));
  const double roundoff = 64 * std::numeric_limits<double>::epsilon() *
      (1 + std::abs(quadratic) + std::abs(linear) + dualRhsMagnitude);
  return std::isfinite(gap) && std::isfinite(scale) &&
         std::isfinite(roundoff) && std::abs(gap) <= tolerance * scale + roundoff;
}

/* ************************************************************************* */
double RowDotProduct(const JacobianFactor& factor, const VectorValues& values) {
  double result = 0.0;
  for (auto it = factor.begin(); it != factor.end(); ++it) {
    result += factor.getA(it).row(0).dot(values.at(*it));
  }
  return result;
}

/* ************************************************************************* */
void CompleteMissingKeys(VectorValues* values, const VectorValues& fallback,
                         const std::map<Key, size_t>& keyDims) {
  for (const auto& [key, _] : keyDims) {
    if (!values->exists(key)) {
      values->insert(key, fallback.at(key));
    }
  }
}

/* ************************************************************************* */
template <typename Row>
double constraintViolation(const std::vector<Row>& equalities,
                           const std::vector<Row>& inequalities,
                           const VectorValues& values) {
  double violation = 0;
  for (const auto& row : equalities)
    violation = std::max(violation, std::abs(RowResidual(*row.factor, values)));
  for (const auto& row : inequalities)
    violation = std::max(violation, RowResidual(*row.factor, values));
  return violation;
}

/* ************************************************************************* */
template <typename Row>
int leavingInequality(const std::vector<Row>& inequalities,
                      const std::vector<bool>& activeRows,
                      const VectorValues& duals, double tolerance) {
  int leavingRow = -1;
  double largestMultiplier = tolerance;
  for (size_t row = 0; row < inequalities.size(); ++row) {
    if (!activeRows[row]) continue;
    const Key key = inequalities[row].multiplierKey;
    const double multiplier = duals.exists(key) ? duals.at(key)(0) : 0.0;
    if (multiplier > largestMultiplier) {
      leavingRow = static_cast<int>(row);
      largestMultiplier = multiplier;
    }
  }
  return leavingRow;
}

}  // namespace

/* ************************************************************************* */
GaussianFactorGraph ActiveSetSolver::buildLpWorkingGraph(
    const VectorValues& values,
    const std::vector<bool>& activeInequalityRows) const {
  // Project a unit negative-gradient step onto the active LP constraint surface.
  // The shared line search decides how far to move along this descent direction.
  GaussianFactorGraph graph;
  for (const auto& [key, dimension] : keyDims_) {
    const Vector gradient = objectiveGradient(key, values);
    graph.emplace_shared<JacobianFactor>(
        key, Matrix::Identity(dimension, dimension), values.at(key) - gradient);
  }
  for (const ConstraintRow& row : constraints_.equalities)
    graph.push_back(row.factor);
  for (size_t row = 0; row < constraints_.inequalities.size(); ++row)
    if (activeInequalityRows.at(row))
      graph.push_back(constraints_.inequalities[row].factor);
  return graph;
}

/* ************************************************************************* */
VectorValues ActiveSetSolver::solveSparseSubproblem(
    const InternalState& state, SolveWorkspace* workspace) const {
  if (problemType_ == ProblemType::LP) {
    const GaussianFactorGraph graph =
        buildLpWorkingGraph(state.values, state.activeInequalityRows);
    const Ordering& ordering = cachedOrdering(
        graph, &workspace->lpKeys, &workspace->lpOrdering, params_->ordering);
    return graph.optimize(ordering, EliminatePreferCholesky);
  }
  // Retain factor pointers and per-call options; the numerical backend
  // belongs to this call, including initialization and every working-face solve.
  if (!workspace->sparse) {
    internal::SparseQpData data;
    data.objective = qpCostGraph_;
    data.keyDimensions = keyDims_;
    data.regularization = params_->regularization;
    data.useIncremental = params_->useIncrementalSparseQp;
    data.ordering = params_->ordering;
    for (const auto& row : constraints_.equalities)
      data.equalities.push_back(row.factor);
    for (const auto& row : constraints_.inequalities)
      data.inequalities.push_back(row.factor);
    workspace->sparse =
        std::make_unique<internal::SparseQpSolver>(std::move(data));
  }
  return workspace->sparse->solve(state.activeInequalityRows, state.values);
}

/* ************************************************************************* */
GaussianFactorGraph ActiveSetSolver::buildDualGraph(
    const SparseConstraintData& constraints,
    const std::vector<bool>& activeInequalityRows, const VectorValues& values,
    const VectorValues* proximalCenter,
    const VectorValues* originalGradient) const {
  GaussianFactorGraph dualGraph;
  // Stationarity for the current working set is A_active^T * lambda = grad.
  // With the normalized A*x-b<=0 convention, a positive active inequality
  // multiplier indicates a row that should leave the active set.
  for (const auto& [key, _] : keyDims_) {
    std::vector<std::pair<Key, Matrix>> terms;
    for (const ConstraintRow& row : constraints.equalities) {
      auto it = row.factor->find(key);
      if (it != row.factor->end()) {
        terms.emplace_back(row.multiplierKey, row.factor->getA(it).transpose());
      }
    }
    for (size_t index = 0; index < constraints.inequalities.size(); ++index) {
      if (!activeInequalityRows.at(index)) {
        continue;
      }
      const ConstraintRow& row = constraints.inequalities[index];
      auto it = row.factor->find(key);
      if (it != row.factor->end()) {
        terms.emplace_back(row.multiplierKey, row.factor->getA(it).transpose());
      }
    }
    if (terms.empty()) continue;
    Vector gradient = originalGradient ? originalGradient->at(key)
                                       : objectiveGradient(key, values);
    if (proximalCenter)
      gradient +=
          params_->regularization * (values.at(key) - proximalCenter->at(key));
    dualGraph.emplace_shared<JacobianFactor>(terms, gradient);
  }
  return dualGraph;
}

/* ************************************************************************* */
void ActiveSetSolver::prepareDenseWorkspace(SolveWorkspace* workspace) const {
  // Equality preparation belongs to one optimization call and is read-only
  // once built; working-set factors remain mutable in that same backend.
  if (!workspace->dense)
    workspace->dense = std::make_unique<internal::DenseQpSolver>(
        denseQpWorkspace_->data, params_->projectDenseQpEqualities);
}

/* ************************************************************************* */
VectorValues ActiveSetSolver::solveDenseQpSubproblem(
    const InternalState& state, SolveWorkspace* workspace) const {
  // One mutable backend instance serves this call's initialization and main
  // loop. Concurrent optimizations never share its working-set factorization.
  prepareDenseWorkspace(workspace);
  return denseVectorToVectorValues(workspace->dense->solve(
      state.activeInequalityRows, vectorValuesToDenseVector(state.values)));
}

/* ************************************************************************* */
Vector ActiveSetSolver::objectiveGradient(Key key,
                                          const VectorValues& values) const {
  const auto dim = keyDims_.at(key);
  if (problemType_ == ProblemType::LP) {
    const auto it = lpGradient_.find(key);
    return it != lpGradient_.end() ? it->second : Vector::Zero(dim);
  }

  Vector gradient = Vector::Zero(dim);
  for (const auto& factor : qpCostGraph_) {
    auto it = factor->find(key);
    if (it != factor->end()) {
      gradient += factor->gradient(key, values);
    }
  }
  return gradient;
}

/* ************************************************************************* */
ActiveSetSolver::InternalState ActiveSetSolver::iterate(
    const InternalState& state, SolveWorkspace* workspace) const {
  const bool useDenseQp =
      problemType_ == ProblemType::QP &&
      params_->qpSubproblemSolver == Params::QpSubproblemSolver::Dense;
  VectorValues candidate;
  if (useDenseQp) {
    candidate = solveDenseQpSubproblem(state, workspace);
  } else {
    candidate = solveSparseSubproblem(state, workspace);
    CompleteMissingKeys(&candidate, state.values, keyDims_);
  }

  InternalState next = state;
  next.iterations = state.iterations + 1;
  if (!shouldInspectOptimality(state, candidate)) {
    moveTowardCandidate(state, candidate, &next);
    return next;
  }

  // Both backends recover multipliers for the original objective and use the
  // same proximal fallback, release rule, and original-QP certificates.
  // One candidate owns its original gradient; reuse never crosses iterations.
  VectorValues originalGradient;
  if (!recoverMultipliers(state, candidate, workspace, &next.duals,
                          &originalGradient)) {
    next.values = candidate;
    return next;
  }
  next.values = candidate;
  const int leavingRow =
      leavingInequality(constraints_.inequalities, state.activeInequalityRows,
                        next.duals, params_->multiplierTolerance);
  if (leavingRow >= 0) {
    next.activeInequalityRows[leavingRow] = false;
  } else {
    next.converged = problemType_ == ProblemType::LP ||
                     originalQpKktSatisfied(
                         candidate, state.activeInequalityRows, next.duals,
                         originalGradient.size() == 0 ? nullptr : &originalGradient);
  }
  return next;
}

/* ************************************************************************* */
bool ActiveSetSolver::shouldInspectOptimality(const InternalState& state,
                                              const VectorValues& candidate) const {
  if (problemType_ == ProblemType::QP) {
    // A feasible working-set minimum can be inspected immediately, without
    // repeating the same factorization just to observe a zero step. When
    // regularization is positive, its proximal residual bounds the difference
    // from original stationarity, including movement along flat directions.
    double bound = 0.0;
    for (const auto& [key, _] : keyDims_)
      bound = std::max(bound, params_->regularization *
                                  (candidate.at(key) - state.values.at(key))
                                      .lpNorm<Eigen::Infinity>());
    return bound <= params_->multiplierTolerance &&
           primalFeasible(constraints_.equalities, constraints_.inequalities,
                          candidate, params_->feasibilityTolerance);
  }
  return candidate.equals(state.values, params_->stepTolerance);
}

/* ************************************************************************* */
bool ActiveSetSolver::recoverDenseMultipliers(const VectorValues& candidate,
                                             const SolveWorkspace& workspace,
                                             VectorValues* originalGradient,
                                             VectorValues* duals) const {
  if (!workspace.dense || !workspace.dense->canRecoverMultipliers()) return false;
  Vector gradient(denseQpWorkspace_->totalDim);
  // Read the original factors, including Hessian-native costs, without a
  // Hessian-to-Jacobian conversion or any proximal forcing in this certificate.
  for (Key key : denseQpWorkspace_->hessianOrdering) {
    const Vector block = objectiveGradient(key, candidate);
    gradient.segment(denseQpWorkspace_->keyOffsets.at(key), keyDims_.at(key)) = block;
    originalGradient->insert(key, block);
  }
  const auto multipliers = workspace.dense->multipliers(
      vectorValuesToDenseVector(candidate), gradient, params_->multiplierTolerance);
  if (!multipliers) return false;
  *duals = VectorValues();
  size_t index = 0;
  for (const auto* rows : {&constraints_.equalities, &constraints_.inequalities})
    for (const auto& row : *rows) {
      const double multiplier = (*multipliers)(index++);
      if (multiplier != 0)
        duals->insert(row.multiplierKey, Vector::Constant(1, multiplier));
    }
  return true;
}

/* ************************************************************************* */
bool ActiveSetSolver::recoverMultipliers(const InternalState& state,
                                               const VectorValues& candidate,
                                               SolveWorkspace* workspace,
                                               VectorValues* duals,
                                               VectorValues* originalGradient) const {
  // Reuse multipliers produced alongside the Dense primal candidate when the
  // original-gradient checks agree. Difficult systems retain graph recovery.
  if (recoverDenseMultipliers(candidate, *workspace, originalGradient, duals))
    return true;
  const VectorValues* gradient = originalGradient->size() == 0 ? nullptr : originalGradient;
  // The original gradient is the only one that can certify a solution.
  const GaussianFactorGraph dualGraph =
      buildDualGraph(constraints_, state.activeInequalityRows, candidate, nullptr,
                     gradient);
  const Ordering& ordering = cachedOrdering(
      dualGraph, &workspace->dualKeys, &workspace->dualOrdering);
  const DualSolveResult original = solveSparseDualGraph(
      dualGraph, ordering, params_->multiplierTolerance,
      problemType_ == ProblemType::QP);
  if (original.consistent()) {
    *duals = original.multipliers;
    return true;
  }

  if (problemType_ == ProblemType::QP && params_->regularization > 0.0) {
    const DualSolveResult proximal = solveSparseDualGraph(
        buildDualGraph(constraints_, state.activeInequalityRows, candidate,
                       &state.values, gradient),
        ordering, params_->multiplierTolerance);
    if (proximal.consistent()) {
      // Only the proximal subproblem is stationary. The next iteration must
      // recenter at candidate; its multipliers cannot certify the original QP.
      return false;
    }
    std::ostringstream message;
    message << std::setprecision(16)
            << "ActiveSetSolver: multiplier stationarity is inconsistent "
               "for both the original and proximal QP (residuals "
            << original.residual << ", " << proximal.residual << "; thresholds "
            << original.threshold << ", " << proximal.threshold << ").";
    throw std::runtime_error(message.str());
  }

  std::ostringstream message;
  message << std::setprecision(16)
          << "ActiveSetSolver: multiplier stationarity is inconsistent "
             "(residual "
          << original.residual << "; threshold " << original.threshold << ").";
  throw std::runtime_error(message.str());
}

/* ************************************************************************* */
bool ActiveSetSolver::originalQpKktSatisfied(
    const VectorValues& values, const std::vector<bool>& activeRows,
    const VectorValues& duals, const VectorValues* originalGradient) const {
  if (!primalFeasible(constraints_.equalities, constraints_.inequalities,
                      values, params_->feasibilityTolerance))
    return false;
  VectorValues stationarity;
  double gap = 0.0, quadratic = 0.0, linear = 0.0;
  double dualRhs = 0.0, dualRhsMagnitude = 0.0;
  for (const auto& [key, _] : keyDims_) {
    const Vector gradient = originalGradient ? originalGradient->at(key)
                                             : objectiveGradient(key, values);
    stationarity.insert(key, gradient);
    gap += values.at(key).dot(gradient);
    const double linearTerm = qpGradientAtZero_.exists(key)
        ? values.at(key).dot(qpGradientAtZero_.at(key)) : 0.0;
    linear += linearTerm;
    quadratic += values.at(key).dot(gradient) - linearTerm;
  }

  auto subtractRow = [&](const ConstraintRow& row) {
    if (!duals.exists(row.multiplierKey)) return;
    const double multiplier = duals.at(row.multiplierKey)(0);
    const double rhsTerm = row.factor->getb()(0) * multiplier;
    gap -= rhsTerm;
    dualRhs += rhsTerm;
    dualRhsMagnitude += std::abs(rhsTerm);
    for (auto key = row.factor->begin(); key != row.factor->end(); ++key)
      stationarity.at(*key) -=
          row.factor->getA(key).transpose() * Vector::Constant(1, multiplier);
  };
  for (const ConstraintRow& row : constraints_.equalities) subtractRow(row);
  for (size_t row = 0; row < constraints_.inequalities.size(); ++row)
    if (activeRows[row]) subtractRow(constraints_.inequalities[row]);

  for (const auto& [_, residual] : stationarity)
    if (!residual.allFinite() ||
        residual.lpNorm<Eigen::Infinity>() > params_->multiplierTolerance)
      return false;
  return scaledGapSatisfied(gap, quadratic, linear, dualRhs, dualRhsMagnitude,
                             params_->multiplierTolerance);
}

/* ************************************************************************* */
void ActiveSetSolver::moveTowardCandidate(const InternalState& state,
                                          const VectorValues& candidate,
                                          InternalState* next) const {
  const VectorValues direction = candidate - state.values;
  // Stop at the first inactive inequality reached along the working-set
  // direction. An unblocked LP direction proves unboundedness.
  double step = problemType_ == ProblemType::QP
                    ? 1.0
                    : std::numeric_limits<double>::infinity();
  int blockingRow = -1;
  for (size_t row = 0; row < constraints_.inequalities.size(); ++row) {
    if (state.activeInequalityRows[row]) continue;
    const JacobianFactor& factor = *constraints_.inequalities[row].factor;
    const double denominator = RowDotProduct(factor, direction);
    if (denominator <= 0.0) continue;
    const double candidateStep =
        -RowResidual(factor, state.values) / denominator;
    if (candidateStep < step) {
      step = std::max(0.0, candidateStep);
      blockingRow = static_cast<int>(row);
    }
  }
  if (problemType_ == ProblemType::LP && blockingRow < 0)
    throw std::runtime_error("ActiveSetSolver: LP is unbounded.");

  next->values = state.values + step * direction;
  if (blockingRow >= 0) next->activeInequalityRows[blockingRow] = true;
}

/* ************************************************************************* */
ActiveSetSolver::State ActiveSetSolver::buildPublicState(
    const InternalState& state, const Values& prototype) const {
  State publicState;
  publicState.values = vectorValuesToValues(state.values, prototype);
  publicState.activeInequalityRows = state.activeInequalityRows;
  publicState.converged = state.converged;
  publicState.iterations = state.iterations;

  publicState.equalityMultipliers.reserve(constraints_.equalityDims.size());
  for (size_t dim : constraints_.equalityDims) {
    publicState.equalityMultipliers.push_back(Vector::Zero(dim));
  }
  // Internal multipliers are scalar values keyed by synthetic multiplier keys;
  // the public state restores the original constraint grouping and row order.
  for (const ConstraintRow& row : constraints_.equalities) {
    if (state.duals.exists(row.multiplierKey)) {
      publicState.equalityMultipliers[row.constraintIndex](row.row) =
          state.duals.at(row.multiplierKey)(0);
    }
  }

  publicState.inequalityMultipliers.reserve(constraints_.inequalityDims.size());
  for (size_t dim : constraints_.inequalityDims) {
    publicState.inequalityMultipliers.push_back(Vector::Zero(dim));
  }
  for (const ConstraintRow& row : constraints_.inequalities) {
    if (state.duals.exists(row.multiplierKey)) {
      publicState.inequalityMultipliers[row.constraintIndex](row.row) =
          state.duals.at(row.multiplierKey)(0);
    }
  }

  return publicState;
}

/* ************************************************************************* */
std::pair<Values, ActiveSetSolver::State> ActiveSetSolver::optimizeWithState(
    const Values& initialValues) const {
  SolveWorkspace workspace;
  return optimizeWithState(initialValues, nullptr, &workspace);
}

/* ************************************************************************* */
std::pair<Values, ActiveSetSolver::State> ActiveSetSolver::optimizeWithState(
    const Values& initialValues, const State& warmStart) const {
  SolveWorkspace workspace;
  return optimizeWithState(initialValues, &warmStart, &workspace);
}

/* ************************************************************************* */
Values ActiveSetSolver::optimize(const Values& initialValues) const {
  return optimizeWithState(initialValues).first;
}

/* ************************************************************************* */
std::pair<Values, ActiveSetSolver::State> ActiveSetSolver::optimizeWithState()
    const {
  SolveWorkspace workspace;
  if (prepareDualQp(&workspace))
    return optimizeDualQp(zeroVectorValues(), nullptr, &workspace);
  const Values feasible = workspace.fixedPoint
      ? vectorValuesToValues(*workspace.fixedPoint, zeroVectorValues())
      : findVectorFeasiblePoint(&workspace);
  return optimizeWithState(feasible, nullptr, &workspace);
}

/* ************************************************************************* */
Values ActiveSetSolver::optimize() const { return optimizeWithState().first; }

/* ************************************************************************* */
ActiveSetFeasibleStepResult ActiveSetSolver::computeFeasibleStep(
    const Values& initialValues) const {
  if (problemType_ != ProblemType::QP)
    throw std::invalid_argument("ActiveSetSolver: feasible steps require a QP.");
  validateInitialValues(initialValues);
  const VectorValues initial = valuesToVectorValues(initialValues);
  if (!primalFeasible(constraints_.equalities, constraints_.inequalities, initial,
                      params_->feasibilityTolerance))
    throw std::invalid_argument("ActiveSetSolver: feasible step needs a feasible point.");
  ActiveSetFeasibleStepResult result;
  result.values = initialValues;
  result.objective = result.initialObjective = qpCostGraph_.error(initial);
  result.constraintViolation = constraintViolation(
      constraints_.equalities, constraints_.inequalities, initial);
  InternalState state;
  SolveWorkspace workspace;
  try {
    if (!std::isfinite(result.initialObjective))
      throw std::runtime_error("ActiveSetSolver: non-finite initial objective.");
    state = initialState(initialValues, nullptr);
    initializeQp(&state, &workspace);
    result.activeInequalityRows = state.activeInequalityRows;
    // A successful preliminary step already satisfies this API's contract.
    if (!inspectFeasibleStep(state, initialValues, &result))
      searchFeasibleStep(&state, initialValues, &workspace, &result);
  } catch (const IterationLimitError& error) {
    result.status = ActiveSetFeasibleStepResult::Status::IterationLimit;
    result.message = error.what();
  } catch (const std::exception& error) {
    result.status = ActiveSetFeasibleStepResult::Status::NumericalFailure;
    result.message = error.what();
  }
  result.iterations = state.iterations;
  return result;
}

/* ************************************************************************* */
void ActiveSetSolver::searchFeasibleStep(
    InternalState* state, const Values& prototype, SolveWorkspace* workspace,
    ActiveSetFeasibleStepResult* result) const {
  // Retry zero-length blockers within the same budget and elimination cache.
  while (state->iterations < params_->maxIterations) {
    const auto previousRows = state->activeInequalityRows;
    takeFeasibleStep(state, workspace);
    result->activeInequalityRows = state->activeInequalityRows;
    if (inspectFeasibleStep(*state, prototype, result)) return;
    if (previousRows == state->activeInequalityRows) {
      result->message = "No useful displacement on the selected working face.";
      return;
    }
  }
  result->status = ActiveSetFeasibleStepResult::Status::IterationLimit;
  result->message = "ActiveSetSolver: feasible-step iteration limit reached.";
}

/* ************************************************************************* */
void ActiveSetSolver::takeFeasibleStep(InternalState* state,
                                       SolveWorkspace* workspace) const {
  // Count attempted work before elimination so failure reports remain honest.
  ++state->iterations;
  VectorValues candidate =
      params_->qpSubproblemSolver == Params::QpSubproblemSolver::Dense
          ? solveDenseQpSubproblem(*state, workspace)
          : solveSparseSubproblem(*state, workspace);
  CompleteMissingKeys(&candidate, state->values, keyDims_);
  (void)maximumStep(candidate, candidate);
  InternalState next = *state;
  moveTowardCandidate(*state, candidate, &next);
  if (!primalFeasible(constraints_.equalities, constraints_.inequalities,
                      next.values, params_->feasibilityTolerance))
    throw std::runtime_error(
        "ActiveSetSolver: feasible step failed constraint checks.");
  *state = std::move(next);
}

/* ************************************************************************* */
bool ActiveSetSolver::inspectFeasibleStep(
    const InternalState& next,
    const Values& prototype, ActiveSetFeasibleStepResult* result) const {
  if (!primalFeasible(constraints_.equalities, constraints_.inequalities,
                      next.values, params_->feasibilityTolerance))
    throw std::runtime_error("ActiveSetSolver: feasible step failed constraint checks.");
  const VectorValues initial = valuesToVectorValues(prototype);
  if (maximumStep(next.values, initial) <= params_->stepTolerance)
    return false;
  const double objective = qpCostGraph_.error(next.values);
  if (!std::isfinite(objective) || !std::isfinite(result->initialObjective))
    throw std::runtime_error("ActiveSetSolver: non-finite feasible-step objective.");
  const double decrease = quadraticDecrease(initial, next.values);
  if (!std::isfinite(decrease))
    throw std::runtime_error("ActiveSetSolver: non-finite quadratic decrease.");
  if (decrease > 0) {
    result->values = vectorValuesToValues(next.values, prototype);
    result->objective = objective;
    result->objectiveDecrease = decrease;
    result->constraintViolation = constraintViolation(
        constraints_.equalities, constraints_.inequalities, next.values);
    result->status = ActiveSetFeasibleStepResult::Status::FeasibleProgress;
  } else {
    result->message = "The working-face step did not decrease the original objective.";
  }
  return true;
}

/* ************************************************************************* */
double ActiveSetSolver::quadraticDecrease(const VectorValues& initial,
                                         const VectorValues& candidate) const {
  const VectorValues firstGradient = qpCostGraph_.gradient(initial);
  const VectorValues lastGradient = qpCostGraph_.gradient(candidate);
  long double change = 0;
  for (const auto& [key, gradient] : firstGradient) {
    const Vector step = candidate.at(key) - initial.at(key);
    change += 0.5L * step.cast<long double>().dot(
        gradient.cast<long double>() + lastGradient.at(key).cast<long double>());
  }
  return -static_cast<double>(change);
}

/* ************************************************************************* */
std::pair<Values, ActiveSetSolver::State> ActiveSetSolver::optimizeWithState(
    const Values& initialValues, const State* warmStart,
    SolveWorkspace* workspace) const {
  validateInitialValues(initialValues);
  if (warmStart) validateWarmStart(*warmStart);
  const bool preferPrimal = warmStart && warmStart->converged &&
      warmStart->qpFormulation == Params::QpFormulation::Primal &&
      primalFeasible(constraints_.equalities, constraints_.inequalities,
                     valuesToVectorValues(initialValues), params_->feasibilityTolerance);
  if (prepareDualQp(workspace, preferPrimal))
    return optimizeDualQp(initialValues, warmStart, workspace);
  InternalState state = workspace->fixedPoint
      ? initialState(vectorValuesToValues(*workspace->fixedPoint, initialValues), nullptr)
      : initialState(initialValues, warmStart);
  // Explicit row masks take precedence over every cold-initialization mode.
  if (!warmStart && !workspace->fixedPoint) initializeQp(&state, workspace);
  while (!state.converged) {
    if (state.iterations >= params_->maxIterations) {
      throw IterationLimitError("ActiveSetSolver: failed to converge.");
    }
    state = iterate(state, workspace);
  }
  State publicState = buildPublicState(state, initialValues);
  return {publicState.values, publicState};
}

}  // namespace gtsam

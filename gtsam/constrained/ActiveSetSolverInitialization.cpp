/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file ActiveSetSolverInitialization.cpp
 * Cold feasibility search and initial working-set selection.
 *
 * Equality projection, phase one, projected-gradient selection, and the optional
 * feasible first step share ActiveSetSolver's existing state and working-solve
 * workspace. Their definitions are separated from the main iteration without
 * changing initialization policy, warm-start precedence, or iteration budgets.
 */

#include <gtsam/constrained/internal/ActiveSetSolverInternal.h>
#include <gtsam/base/DSFMap.h>
#include <gtsam/linear/linearExceptions.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace gtsam {

using internal::IterationLimitError;
using internal::NextAvailableKey;
using internal::RowResidual;
using internal::SingleValue;
using internal::maximumStep;
using internal::primalFeasible;
using internal::restrictOrdering;

namespace {

/* ************************************************************************* */
template <typename Row>
LpProblem buildPhaseOneProblem(const std::vector<Row>& equalities,
                               const std::vector<Row>& inequalities,
                               Key slackKey, double* initialSlack) {
  // Minimize a nonnegative scalar slack. Each equality becomes two relaxed
  // inequalities, and all rows share the same slack variable.
  LpProblem problem;
  problem.addCost(
      JacobianFactor(slackKey, Matrix::Identity(1, 1), Vector::Zero(1)));
  auto rowTerms = [&](const Row& row, double sign) {
    std::vector<std::pair<Key, Matrix>> terms;
    terms.reserve(row.factor->size() + 1);
    for (auto key = row.factor->begin(); key != row.factor->end(); ++key)
      terms.emplace_back(*key, sign * row.factor->getA(key));
    terms.emplace_back(slackKey, -Matrix::Identity(1, 1));
    return terms;
  };

  *initialSlack = 1.0;
  for (const Row& row : equalities) {
    const double rhs = row.factor->getb()(0);
    problem.addConstraint(LinearConstraint::LessEqual(
        JacobianFactor(rowTerms(row, 1.0), SingleValue(rhs))));
    problem.addConstraint(LinearConstraint::LessEqual(
        JacobianFactor(rowTerms(row, -1.0), SingleValue(-rhs))));
    *initialSlack = std::max(*initialSlack, std::abs(rhs) + 1.0);
  }
  for (const Row& row : inequalities) {
    const double rhs = row.factor->getb()(0);
    problem.addConstraint(LinearConstraint::LessEqual(
        JacobianFactor(rowTerms(row, 1.0), SingleValue(rhs))));
    *initialSlack = std::max(*initialSlack, -rhs + 1.0);
  }
  problem.addConstraint(LinearConstraint::LessEqual(
      JacobianFactor(slackKey, -Matrix::Identity(1, 1), Vector::Zero(1))));
  return problem;
}

// Tangent equations have zero RHS. Only the objective forcing and tight
// inequality rows change; the permanent equality basis is already prepared.
QpProblem denseTangentProblem(
    const internal::DenseQpEqualityProjection& projection,
    const internal::DenseQpData& original, const Vector& gradient,
    const std::vector<bool>& active) {
  QpProblem result;
  const Eigen::Index dimension = projection.nullspace.cols();
  result.addCost(JacobianFactor(0, Matrix::Identity(dimension, dimension),
      Vector(-projection.nullspace.transpose() * gradient)));
  for (size_t row = 0; row < active.size(); ++row) {
    if (!active[row]) continue;
    const double norm = original.normals.row(original.equalities + row).stableNorm();
    result.addConstraint(LinearConstraint::LessEqual(JacobianFactor(
        0, Matrix(projection.reduced.normals.row(row) / (norm > 0 ? norm : 1)),
        Vector1::Zero())));
  }
  return result;
}

// Check lifted directions against normalized original tangent rows before the
// caller classifies strictly inward constraints. No affine offset belongs in d.
template <typename Row>
void checkTangentDirection(
    const std::vector<Row>& equalities, const std::vector<Row>& inequalities,
    const VectorValues& direction, double feasibilityTolerance,
    const std::vector<bool>& active) {
  auto derivative = [&](const Row& row) {
    const double norm = row.factor->getA().stableNorm();
    double value = 0;
    for (auto key = row.factor->begin(); key != row.factor->end(); ++key)
      value += ((row.factor->getA(key) / (norm > 0 ? norm : 1)) *
                direction.at(*key))(0);
    return value;
  };
  for (const auto& row : equalities) {
    const double value = derivative(row);
    if (!std::isfinite(value) || std::abs(value) > feasibilityTolerance)
      throw std::runtime_error("Projected direction violates an original tangent equality.");
  }
  for (size_t row = 0; row < inequalities.size(); ++row) {
    if (!active[row]) continue;
    const double value = derivative(inequalities[row]);
    if (!std::isfinite(value) || value > feasibilityTolerance)
      throw std::runtime_error("Projected direction violates an original tangent inequality.");
  }
}

}  // namespace

/* ************************************************************************* */
ActiveSetSolver::InternalState ActiveSetSolver::initialState(
    const Values& values, const State* warmStart) const {
  // Preserve caller values; a working-set warm start overrides cold selection.
  InternalState state;
  state.values = valuesToVectorValues(values);
  state.activeInequalityRows.assign(constraints_.inequalities.size(), false);

  for (size_t row = 0; row < constraints_.inequalities.size(); ++row) {
    const double violation =
        RowResidual(*constraints_.inequalities[row].factor, state.values);
    if (violation > params_->feasibilityTolerance) {
      throw std::invalid_argument(
          "ActiveSetSolver: initial values are infeasible.");
    }
  }

  if (warmStart) {
    if (warmStart->activeInequalityRows.size() != constraints_.inequalities.size())
      throw std::invalid_argument("ActiveSetSolver: warm-start row count does not match the problem.");
    state.activeInequalityRows = warmStart->activeInequalityRows;
    return state;
  }

  initializeTightRows(&state);
  return state;
}

/* ************************************************************************* */
void ActiveSetSolver::initializeTightRows(InternalState* state) const {
  // Include every tight row, including coincident blockers at a new point.
  for (size_t row = 0; row < constraints_.inequalities.size(); ++row) {
    const double violation =
        RowResidual(*constraints_.inequalities[row].factor, state->values);
    state->activeInequalityRows[row] =
        std::abs(violation) <= params_->activeTolerance;
  }
}

/* ************************************************************************* */
void ActiveSetSolver::initializeQp(InternalState* state,
                                   SolveWorkspace* workspace) const {
  // Both public QP entry points share this pipeline without recursive solves.
  if (problemType_ != ProblemType::QP ||
      params_->qpInitialization == Params::QpInitialization::TightConstraints)
    return;
  if (params_->qpInitialization ==
      Params::QpInitialization::FeasibleStepThenProjectedGradient)
    initializeFeasiblePoint(state, workspace);
  initializeProjectedGradient(state, workspace);
}

/* ************************************************************************* */
void ActiveSetSolver::initializeFeasiblePoint(
    InternalState* state, SolveWorkspace* workspace) const {
  // Truncation preserves affine equalities only from an already feasible point.
  if (!primalFeasible(constraints_.equalities, constraints_.inequalities,
                      state->values, params_->feasibilityTolerance))
    throw std::invalid_argument(
        "ActiveSetSolver: two-stage initialization needs a feasible point.");
  if (state->iterations >= params_->maxIterations)
    throw IterationLimitError(
        "ActiveSetSolver: initialization iteration limit reached.");
  const VectorValues initial = state->values;
  takeFeasibleStep(state, workspace);
  const double decrease = quadraticDecrease(initial, state->values);
  if (!std::isfinite(decrease) ||
      !std::isfinite(qpCostGraph_.error(state->values)))
    throw std::runtime_error(
        "ActiveSetSolver: non-finite initialization objective.");
  // Accept only useful original-objective descent; keep the attempted count.
  if (maximumStep(state->values, initial) <= params_->stepTolerance ||
      decrease <= 0)
    state->values = initial;
  initializeTightRows(state);
}

/* ************************************************************************* */
void ActiveSetSolver::initializeProjectedGradient(
    InternalState* state, SolveWorkspace* workspace) const {
  if (std::none_of(state->activeInequalityRows.begin(),
                   state->activeInequalityRows.end(),
                   [](bool row) { return row; }))
    return;

  // Prepare equalities once even when projection is the first numerical stage.
  if (denseQpWorkspace_ && !constraints_.equalities.empty())
    prepareDenseWorkspace(workspace);
  const bool reuseDense = workspace->dense &&
                         workspace->dense->equalityProjection();

  // The identity projection objective is separable even when the QP is coupled.
  DSFMap<Key> partition;
  auto connect = [&](const ConstraintRow& row) {
    for (Key key : *row.factor) partition.merge(row.factor->front(), key);
  };
  for (const auto& row : constraints_.equalities) connect(row);
  for (size_t row = 0; row < constraints_.inequalities.size(); ++row)
    if (state->activeInequalityRows[row])
      connect(constraints_.inequalities[row]);

  struct Component {
    QpProblem projection;
    std::vector<size_t> inequalities;
  };
  std::map<Key, Component> components;
  // Normalize homogeneous rows so projection and classification are insensitive
  // to a constraint's positive scale. A constant row remains in the working
  // set.
  auto tangentRow = [](const JacobianFactor& factor) {
    JacobianFactor tangent(factor);
    tangent.getb().setZero();
    const double norm = tangent.getA().stableNorm();
    if (norm > 0) tangent.getA() /= norm;
    return tangent;
  };
  for (const auto& row : constraints_.equalities) {
    if (row.factor->empty()) continue;
    components[partition.find(row.factor->front())].projection.addConstraint(
        LinearConstraint::Equal(tangentRow(*row.factor)));
  }
  for (size_t row = 0; row < constraints_.inequalities.size(); ++row) {
    const auto& factor = *constraints_.inequalities[row].factor;
    if (!state->activeInequalityRows[row] || factor.empty()) continue;
    auto& component = components[partition.find(factor.front())];
    component.projection.addConstraint(
        LinearConstraint::LessEqual(tangentRow(factor)));
    component.inequalities.push_back(row);
  }

  const VectorValues gradient = qpCostGraph_.gradient(state->values);
  auto projectionParams = std::make_shared<Params>(*params_);
  projectionParams->qpFormulation = Params::QpFormulation::Primal;
  // Auxiliary cone projections must not recursively initialize themselves.
  projectionParams->qpInitialization =
      Params::QpInitialization::TightConstraints;
  projectionParams->qpSubproblemSolver = Params::QpSubproblemSolver::Sparse;
  projectionParams->regularization = 0;
  for (const auto& [root, keys] : partition.sets()) {
    auto& component = components.at(root);
    if (component.inequalities.empty()) continue;
    // Cone projection is positively homogeneous: scaling the whole component
    // gradient preserves its binding rows while bounding numerical magnitudes.
    double scale = 1;
    for (Key key : keys)
      if (gradient.exists(key))
        scale = std::max(scale, gradient.at(key).lpNorm<Eigen::Infinity>());
    Values zero;
    for (Key key : keys) {
      const size_t dim = keyDims_.at(key);
      Vector target = Vector::Zero(dim);
      if (gradient.exists(key)) target = -gradient.at(key) / scale;
      component.projection.addCost(
          JacobianFactor(key, Matrix::Identity(dim, dim), target));
      zero.insert(key, Vector::Zero(dim).eval());
    }
    Values direction;
    if (params_->ordering)
      projectionParams->ordering = restrictOrdering(
          *params_->ordering, KeySet(keys.begin(), keys.end()));
    try {
      direction = reuseDense
          ? projectDenseGradient(gradient, keys, component.inequalities, scale,
                                 workspace)
          : ActiveSetSolver(component.projection, projectionParams).optimize(zero);
    } catch (const IterationLimitError& error) {
      throw IterationLimitError(
          std::string("ActiveSetSolver: projected-gradient initialization failed: ") +
          error.what());
    } catch (const std::runtime_error& error) {
      throw std::runtime_error(
          std::string(
              "ActiveSetSolver: projected-gradient initialization failed: ") +
          error.what());
    }
    for (size_t row : component.inequalities) {
      const auto& factor = *constraints_.inequalities[row].factor;
      const double norm = factor.getA().stableNorm();
      if (norm == 0) continue;
      double derivative = 0;
      for (auto key = factor.begin(); key != factor.end(); ++key)
        derivative += ((factor.getA(key) / norm) * direction.at<Vector>(*key))(0);
      if (derivative < -params_->activeTolerance)
        state->activeInequalityRows[row] = false;
    }
  }
}

/* ************************************************************************* */
Values ActiveSetSolver::projectDenseGradient(
    const VectorValues& gradient, const std::set<Key>& keys,
    const std::vector<size_t>& rows, double scale,
    SolveWorkspace* workspace) const {
  const auto& equality = *workspace->dense->equalityProjection();
  VectorValues direction = VectorValues::Zero(gradient);
  if (equality.nullspace.cols() > 0) {
    Vector forcing = Vector::Zero(denseQpWorkspace_->totalDim);
    for (Key key : keys)
      if (gradient.exists(key))
        forcing.segment(denseQpWorkspace_->keyOffsets.at(key), keyDims_.at(key)) =
            gradient.at(key) / scale;
    std::vector<bool> active(constraints_.inequalities.size(), false);
    for (size_t row : rows) active[row] = true;
    const QpProblem projection = denseTangentProblem(
        equality, denseQpWorkspace_->data, forcing, active);
    auto parameters = std::make_shared<Params>(*params_);
    parameters->qpFormulation = Params::QpFormulation::Primal;
    parameters->qpInitialization = Params::QpInitialization::TightConstraints;
    parameters->regularization = 0;
    // This auxiliary key represents reduced coordinates, not original keys.
    parameters->ordering = Ordering{0};
    Values zero;
    zero.insert(0, Vector::Zero(equality.nullspace.cols()).eval());
    const Values reduced = ActiveSetSolver(projection, parameters).optimize(zero);
    direction = denseVectorToVectorValues(
        equality.nullspace * reduced.at<Vector>(0));
    checkTangentDirection(constraints_.equalities, constraints_.inequalities,
                          direction, params_->feasibilityTolerance, active);
  }
  // The caller classifies only this original component, with its own budget.
  Values result;
  for (Key key : keys) result.insert(key, direction.at(key));
  return result;
}

/* ************************************************************************* */
Values ActiveSetSolver::zeroVectorValues() const {
  Values values;
  for (const auto& [key, dim] : keyDims_) {
    const Vector zero = Vector::Zero(dim);
    values.insert(key, zero);
  }
  return values;
}

/* ************************************************************************* */
Values ActiveSetSolver::findVectorFeasiblePoint(SolveWorkspace* workspace) const {
  const Values zeroValues = zeroVectorValues();
  Values feasible;
  if (problemType_ == ProblemType::QP &&
      tryEqualityProjection(zeroValues, &feasible, workspace))
    return feasible;
  return solvePhaseOne(zeroValues);
}

/* ************************************************************************* */
bool ActiveSetSolver::tryEqualityProjection(const Values& zeroValues,
                                            Values* feasible,
                                            SolveWorkspace* workspace) const {
  if (denseQpWorkspace_) {
    try {
      prepareDenseWorkspace(workspace);
      const auto* projection = workspace->dense->equalityProjection();
      if (projection || constraints_.equalities.empty()) {
        const VectorValues candidate = projection
            ? denseVectorToVectorValues(projection->point)
            : valuesToVectorValues(zeroValues);
        if (!primalFeasible(constraints_.equalities, constraints_.inequalities,
                            candidate, params_->feasibilityTolerance))
          return false;
        *feasible = vectorValuesToValues(candidate, zeroValues);
        return true;
      }
    } catch (const std::runtime_error&) {
      // Preserve phase one when original equalities cannot supply a point.
      return false;
    }
  }
  // Unit unary factors select a minimum-norm point on the equality manifold.
  GaussianFactorGraph graph;
  for (const auto& [key, dim] : keyDims_)
    graph.emplace_shared<JacobianFactor>(key, Matrix::Identity(dim, dim),
                                         Vector::Zero(dim));
  for (const ConstraintRow& row : constraints_.equalities)
    graph.push_back(row.factor);

  try {
    const VectorValues candidate = params_->ordering
        ? graph.optimize(*params_->ordering) : graph.optimize();
    for (const auto& [key, _] : keyDims_)
      if (!candidate.exists(key) || !candidate.at(key).allFinite())
        return false;
    if (!primalFeasible(constraints_.equalities, constraints_.inequalities,
                        candidate, params_->feasibilityTolerance))
      return false;
    *feasible = vectorValuesToValues(candidate, zeroValues);
    return true;
  } catch (const IndeterminateSystemException&) {
    // Phase one handles an inconsistent or singular projection.
    return false;
  }
}

/* ************************************************************************* */
Values ActiveSetSolver::solvePhaseOne(const Values& zeroValues) const {
  const Key slackKey = NextAvailableKey(keyDims_);
  double initialSlack;
  const LpProblem phaseProblem =
      buildPhaseOneProblem(constraints_.equalities, constraints_.inequalities,
                           slackKey, &initialSlack);
  Values initial = zeroValues;
  initial.insert(slackKey, Vector::Constant(1, initialSlack).eval());
  auto phaseParams = std::make_shared<Params>(*params_);
  phaseParams->qpFormulation = Params::QpFormulation::Primal;
  // Phase one is an LP; the outer QP alone owns its initialization strategy.
  phaseParams->qpInitialization = Params::QpInitialization::TightConstraints;
  if (params_->ordering) {
    KeySet phaseKeys;
    for (const auto& row : constraints_.equalities)
      phaseKeys.insert(row.factor->begin(), row.factor->end());
    for (const auto& row : constraints_.inequalities)
      phaseKeys.insert(row.factor->begin(), row.factor->end());
    phaseParams->ordering = restrictOrdering(*params_->ordering, phaseKeys);
    phaseParams->ordering->push_back(slackKey);
  }
  const Values result =
      ActiveSetSolver(phaseProblem, phaseParams).optimize(initial);
  if (result.at<Vector>(slackKey)(0) > params_->phaseOneFeasibilityTolerance)
    throw std::runtime_error(
        "ActiveSetSolver: could not find a feasible point.");

  Values feasible;
  for (const auto& [key, _] : keyDims_)
    feasible.insert(key, result.exists(key) ? result.at<Vector>(key)
                                            : zeroValues.at<Vector>(key));
  return feasible;
}

}  // namespace gtsam

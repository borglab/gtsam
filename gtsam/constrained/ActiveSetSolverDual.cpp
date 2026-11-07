/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */

/** @file ActiveSetSolverDual.cpp
 * Formulation selection, dual warm starts, and original-QP certification.
 */
#include <gtsam/constrained/internal/ActiveSetSolverInternal.h>

namespace gtsam {

bool ActiveSetSolver::prepareDualQp(SolveWorkspace* workspace,
                                   bool preferPrimal) const {
  if (workspace->formulationPrepared) return bool(workspace->dual);
  workspace->formulationPrepared = true;
  if (problemType_ != ProblemType::QP ||
      params_->qpFormulation == Params::QpFormulation::Primal) return false;
  const bool automatic = params_->qpFormulation == Params::QpFormulation::Automatic;
  if (automatic && (preferPrimal || constraints_.inequalities.empty())) return false;
  internal::SparseQpData data;
  data.objective = qpCostGraph_;
  data.keyDimensions = keyDims_;
  data.regularization = params_->regularization;
  data.ordering = params_->ordering;
  for (const auto& row : constraints_.equalities) data.equalities.push_back(row.factor);
  for (const auto& row : constraints_.inequalities) data.inequalities.push_back(row.factor);
  const bool dense = params_->qpSubproblemSolver == Params::QpSubproblemSolver::Dense;
  if (dense) prepareDenseWorkspace(workspace);
  auto dual = std::make_unique<internal::DualQpSolver>(std::move(data),
      dense ? workspace->dense.get() : nullptr,
      dense ? denseQpWorkspace_->hessianOrdering : Ordering{});
  if (automatic && dual->reducedDimension() == 0) {
    Values point;
    if (!tryEqualityProjection(zeroVectorValues(), &point, workspace))
      throw std::runtime_error("ActiveSetSolver: fully fixed point violates the constraints.");
    workspace->fixedPoint = valuesToVectorValues(point);
    return false;
  }
  if (!dual->prepare()) {
    if (automatic) return false;
    throw std::runtime_error("ActiveSetSolver: Dual requires invertible reduced curvature; "
                             "use positive proximal regularization for a singular objective.");
  }
  workspace->dual = std::move(dual);
  return true;
}

void ActiveSetSolver::validateWarmStart(const State& warmStart) const {
  if (warmStart.activeInequalityRows.size() != constraints_.inequalities.size())
    throw std::invalid_argument("ActiveSetSolver: warm-start row count does not match the problem.");
  if (warmStart.qpFormulation != Params::QpFormulation::Primal &&
      warmStart.qpFormulation != Params::QpFormulation::Dual)
    throw std::invalid_argument("ActiveSetSolver: warm-start formulation must be resolved.");
  auto validate = [](const std::vector<Vector>& values, const std::vector<size_t>& dimensions) {
    if (values.empty()) return;
    if (values.size() != dimensions.size())
      throw std::invalid_argument("ActiveSetSolver: invalid warm-start multiplier count.");
    for (size_t i = 0; i < values.size(); ++i)
      if (size_t(values[i].size()) != dimensions[i] || !values[i].allFinite())
        throw std::invalid_argument("ActiveSetSolver: invalid warm-start multipliers.");
  };
  validate(warmStart.equalityMultipliers, constraints_.equalityDims);
  validate(warmStart.inequalityMultipliers, constraints_.inequalityDims);
}

std::pair<Values, ActiveSetSolver::State> ActiveSetSolver::optimizeDualQp(
    const Values& initialValues, const State* warmStart,
    SolveWorkspace* workspace) const {
  validateInitialValues(initialValues);
  InternalState state;
  state.values = valuesToVectorValues(initialValues);
  state.activeInequalityRows.assign(constraints_.inequalities.size(), false);
  Vector multipliers = Vector::Zero(constraints_.inequalities.size());
  if (warmStart) {
    state.activeInequalityRows = warmStart->activeInequalityRows;
    if (!warmStart->inequalityMultipliers.empty())
      for (size_t i = 0; i < constraints_.inequalities.size(); ++i) {
        const auto& row = constraints_.inequalities[i];
        if (state.activeInequalityRows[i])
          multipliers(i) = std::max(0.0,
              -warmStart->inequalityMultipliers[row.constraintIndex](row.row));
      }
  }
  while (!state.converged) {
    state.values = workspace->dual->solve(state.values, &multipliers,
        &state.activeInequalityRows, &state.iterations, params_->maxIterations,
        params_->feasibilityTolerance, params_->multiplierTolerance);
    VectorValues gradient;
    for (const auto& [key, dimension] : keyDims_)
      gradient.insert(key, objectiveGradient(key, state.values));
    VectorValues residual = gradient;
    state.duals = VectorValues();
    for (size_t i = 0; i < constraints_.inequalities.size(); ++i) {
      const auto& row = constraints_.inequalities[i];
      state.duals.insert(row.multiplierKey, Vector1{-multipliers(i)});
      for (auto key = row.factor->begin(); key != row.factor->end(); ++key)
        residual.at(*key) += multipliers(i) * row.factor->getA(key).row(0).transpose();
    }
    const Vector equalities = workspace->dual->equalityMultipliers(
        residual, params_->multiplierTolerance);
    for (size_t i = 0; i < constraints_.equalities.size(); ++i)
      state.duals.insert(constraints_.equalities[i].multiplierKey, Vector1{equalities(i)});
    state.converged = originalQpKktSatisfied(state.values,
        state.activeInequalityRows, state.duals, &gradient);
    if (!state.converged && params_->regularization == 0)
      throw std::runtime_error("ActiveSetSolver: Dual failed the original QP certificate.");
  }
  State result = buildPublicState(state, initialValues);
  result.qpFormulation = Params::QpFormulation::Dual;
  return {result.values, result};
}

}  // namespace gtsam

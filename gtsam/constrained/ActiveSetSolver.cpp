/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    ActiveSetSolver.cpp
 * @brief   Generic active-set solver for linearly constrained LPs and QPs.
 * @author  Frank Dellaert
 */

#include <gtsam/base/GenericValue.h>
#include <gtsam/constrained/ActiveSetSolver.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/inference/VariableIndex.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/linearExceptions.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace gtsam {

// Dense QP mode is allocated only when requested. It stores the reusable
// Hessian factorization and the key-to-offset map needed by dense vectors.
struct ActiveSetSolver::DenseQpWorkspace {
  Ordering hessianOrdering;
  GaussianBayesNet::shared_ptr costBayesNet;
  std::map<Key, size_t> keyOffsets;
  size_t totalDim = 0;
  Vector unconstrainedMinimum;
};

namespace {

constexpr double kMinimumDenseRegularization = 1e-6;

/* ************************************************************************* */
Vector DirectVectorOrMatrixValue(const Values& values, Key key) {
  const Value& value = values.at(key);
  if (const auto* vectorValue =
          dynamic_cast<const GenericValue<Vector>*>(&value)) {
    return vectorValue->value();
  }
  if (const auto* matrixValue =
          dynamic_cast<const GenericValue<Matrix>*>(&value)) {
    const Matrix& matrix = matrixValue->value();
    return Eigen::Map<const Vector>(matrix.data(), matrix.size());
  }
  throw std::invalid_argument(
      "ActiveSetSolver: only Vector and Matrix Values "
      "entries are supported.");
}

/* ************************************************************************* */
void AddOrCheckDim(std::map<Key, size_t>* keyDims, Key key, size_t dim) {
  const auto [it, inserted] = keyDims->emplace(key, dim);
  if (!inserted && it->second != dim) {
    throw std::invalid_argument(
        "ActiveSetSolver: inconsistent key dimensions.");
  }
}

/* ************************************************************************* */
void AddFactorDims(std::map<Key, size_t>* keyDims,
                   const JacobianFactor& factor) {
  for (auto it = factor.begin(); it != factor.end(); ++it) {
    AddOrCheckDim(keyDims, *it, static_cast<size_t>(factor.getDim(it)));
  }
}

/* ************************************************************************* */
void AddFactorDims(std::map<Key, size_t>* keyDims,
                   const HessianFactor& factor) {
  for (auto it = factor.begin(); it != factor.end(); ++it) {
    AddOrCheckDim(keyDims, *it, static_cast<size_t>(factor.getDim(it)));
  }
}

/* ************************************************************************* */
const LinearConstraint& RequireLinearConstraint(
    const NonlinearEqualityConstraint::shared_ptr& constraint) {
  const auto linear =
      std::dynamic_pointer_cast<LinearEqualityConstraintFactor>(constraint);
  if (!linear) {
    throw std::invalid_argument(
        "ActiveSetSolver: equality constraints must be "
        "LinearConstraint factors.");
  }
  return linear->linearConstraint();
}

/* ************************************************************************* */
const LinearConstraint& RequireLinearConstraint(
    const NonlinearInequalityConstraint::shared_ptr& constraint) {
  const auto linear =
      std::dynamic_pointer_cast<LinearInequalityConstraintFactor>(constraint);
  if (!linear) {
    throw std::invalid_argument(
        "ActiveSetSolver: inequality constraints must be "
        "LinearConstraint factors.");
  }
  return linear->linearConstraint();
}

/* ************************************************************************* */
double ConstraintSign(LinearConstraint::Sense sense) {
  // The solver normalizes every constraint row to A*x-b<=0. Equalities use the
  // same row representation and always stay in the working set; inequalities
  // enter and leave one row at a time after multi-row factors are split.
  // GreaterEqual rows are multiplied by -1 so the active-set tests can use a
  // single feasibility convention: residual A*x - b is non-positive.
  return sense == LinearConstraint::Sense::GreaterEqual ? -1.0 : 1.0;
}

/* ************************************************************************* */
Vector SingleValue(double value) {
  Vector vector(1);
  vector << value;
  return vector;
}

/* ************************************************************************* */
JacobianFactor::shared_ptr OneRowConstraintFactor(const JacobianFactor& factor,
                                                  size_t row, double sign) {
  std::vector<std::pair<Key, Matrix>> terms;
  terms.reserve(factor.size());
  for (auto it = factor.begin(); it != factor.end(); ++it) {
    terms.emplace_back(*it, sign * Matrix(factor.getA(it).row(row)));
  }
  return std::make_shared<JacobianFactor>(
      terms, SingleValue(sign * factor.getb()(row)),
      noiseModel::Constrained::All(1));
}

/* ************************************************************************* */
double RowResidual(const JacobianFactor& factor, const VectorValues& values) {
  return factor.unweighted_error(values)(0);
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
Key NextAvailableKey(const std::map<Key, size_t>& keyDims) {
  if (keyDims.empty()) return 1;
  return keyDims.rbegin()->first + 1;
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
void AddRegularization(GaussianFactorGraph* graph,
                       const std::map<Key, size_t>& keyDims,
                       double regularization) {
  const double sigma = std::sqrt(regularization);
  for (const auto& [key, dim] : keyDims) {
    graph->push_back(JacobianFactor(key, sigma * Matrix::Identity(dim, dim),
                                    Vector::Zero(dim)));
  }
}

/* ************************************************************************* */
bool DenseSchurComplementIsSingular(const Eigen::LDLT<Matrix>& ldlt,
                                    const Matrix& schurComplement) {
  if (ldlt.info() != Eigen::Success) {
    return true;
  }
  const Vector diagonal = ldlt.vectorD();
  if (diagonal.size() == 0) {
    return false;
  }
  const double tolerance = std::numeric_limits<double>::epsilon() *
                           static_cast<double>(diagonal.size()) *
                           std::max(1.0, schurComplement.norm());
  return diagonal.cwiseAbs().minCoeff() <= tolerance;
}

}  // namespace

/* ************************************************************************* */
ActiveSetSolver::ActiveSetSolver(const QpProblem& problem,
                                 Params::shared_ptr params)
    : problemType_(ProblemType::QP),
      qpProblem_(&problem),
      params_(std::move(params)) {
  if (!params_) {
    params_ = std::make_shared<Params>();
  }
  initializeDimensions();
  initializeObjective();
  initializeConstraints();
}

/* ************************************************************************* */
ActiveSetSolver::ActiveSetSolver(const LpProblem& problem,
                                 Params::shared_ptr params)
    : problemType_(ProblemType::LP),
      lpProblem_(&problem),
      params_(std::move(params)) {
  if (!params_) {
    params_ = std::make_shared<Params>();
  }
  initializeDimensions();
  initializeObjective();
  initializeConstraints();
}

/* ************************************************************************* */
void ActiveSetSolver::initializeDimensions() {
  if (problemType_ == ProblemType::QP) {
    if (qpProblem_->costs().empty()) {
      throw std::invalid_argument(
          "ActiveSetSolver: QP must have at least one "
          "cost.");
    }
    for (const auto& factor : qpProblem_->costs()) {
      const auto cost = std::dynamic_pointer_cast<QpCost>(factor);
      if (!cost) {
        throw std::invalid_argument(
            "ActiveSetSolver: QP costs must be QpCost "
            "factors.");
      }
      AddFactorDims(&keyDims_, cost->hessianFactor());
    }
    for (const auto& constraint : qpProblem_->eConstraints()) {
      AddFactorDims(&keyDims_, RequireLinearConstraint(constraint).factor());
    }
    for (const auto& constraint : qpProblem_->iConstraints()) {
      AddFactorDims(&keyDims_, RequireLinearConstraint(constraint).factor());
    }
  } else {
    if (lpProblem_->linearCosts().empty()) {
      throw std::invalid_argument(
          "ActiveSetSolver: LP must have at least one "
          "cost.");
    }
    for (const LpCost& cost : lpProblem_->linearCosts()) {
      AddFactorDims(&keyDims_, cost.factor());
    }
    for (const auto& constraint : lpProblem_->eConstraints()) {
      AddFactorDims(&keyDims_, RequireLinearConstraint(constraint).factor());
    }
    for (const auto& constraint : lpProblem_->iConstraints()) {
      AddFactorDims(&keyDims_, RequireLinearConstraint(constraint).factor());
    }
  }
}

/* ************************************************************************* */
void ActiveSetSolver::initializeObjective() {
  if (problemType_ == ProblemType::LP) {
    for (const auto& [key, dim] : keyDims_) {
      lpGradient_[key] = Vector::Zero(dim);
    }
    for (const LpCost& cost : lpProblem_->linearCosts()) {
      const JacobianFactor& factor = cost.factor();
      for (auto it = factor.begin(); it != factor.end(); ++it) {
        const Key key = *it;
        // LpCost stores each linear term as a Jacobian row. Summing the rows
        // gives the constant objective gradient for that key.
        lpGradient_.at(key) += factor.getA(it).colwise().sum().transpose();
      }
    }
    return;
  }

  for (const auto& factor : qpProblem_->costs()) {
    const auto cost = std::dynamic_pointer_cast<QpCost>(factor);
    qpCostGraph_.push_back(cost->hessianFactor());
  }
  if (params_->qpSubproblemSolver == Params::QpSubproblemSolver::Dense) {
    initializeDenseQpWorkspace();
  }
}

/* ************************************************************************* */
void ActiveSetSolver::initializeDenseQpWorkspace() {
  denseQpWorkspace_ = std::make_shared<DenseQpWorkspace>();
  DenseQpWorkspace& workspace = *denseQpWorkspace_;

  // Reuse the same ordering for the Hessian factorization and for all dense
  // vectors. This makes each key's offset stable across active-set iterations.
  workspace.hessianOrdering = Ordering::Colamd(VariableIndex(qpCostGraph_));
  size_t offset = 0;
  for (Key key : workspace.hessianOrdering) {
    workspace.keyOffsets[key] = offset;
    offset += keyDims_.at(key);
  }
  for (const auto& [key, _] : keyDims_) {
    if (workspace.keyOffsets.find(key) == workspace.keyOffsets.end()) {
      throw std::invalid_argument(
          "ActiveSetSolver: constrained variable is missing from QP cost.");
    }
  }
  workspace.totalDim = offset;

  // The dense subproblem path assumes every constrained variable is present in
  // the quadratic cost, because H^{-1} is applied through this Bayes net.
  auto eliminate = [&](const GaussianFactorGraph& graph) {
    return graph.eliminateSequential(workspace.hessianOrdering);
  };

  GaussianFactorGraph regularizedCostGraph = qpCostGraph_;
  if (params_->regularization > 0.0) {
    AddRegularization(&regularizedCostGraph, keyDims_, params_->regularization);
  }

  try {
    workspace.costBayesNet = eliminate(regularizedCostGraph);
  } catch (const IndeterminantLinearSystemException&) {
    if (params_->regularization <= 0.0 ||
        params_->regularization >= kMinimumDenseRegularization) {
      throw std::runtime_error(
          "ActiveSetSolver: dense QP Hessian factorization failed. Increase "
          "regularization or use the sparse QP subproblem solver.");
    }
    // Very small positive regularization can still be treated as rank-deficient
    // by elimination; retry with the dense-mode floor before failing.
    GaussianFactorGraph fallbackCostGraph = qpCostGraph_;
    AddRegularization(&fallbackCostGraph, keyDims_,
                      kMinimumDenseRegularization);
    try {
      workspace.costBayesNet = eliminate(fallbackCostGraph);
    } catch (const IndeterminantLinearSystemException&) {
      throw std::runtime_error(
          "ActiveSetSolver: dense QP Hessian factorization failed even after "
          "minimum dense regularization. Increase regularization or use the "
          "sparse QP subproblem solver.");
    }
  }

  // Store the unconstrained minimizer x0. Constrained solves then only need the
  // active constraint correction x = x0 + H^{-1} A^T lambda.
  workspace.unconstrainedMinimum = Vector::Zero(workspace.totalDim);
  const VectorValues minimum = workspace.costBayesNet->optimize();
  for (const auto& [key, vector] : minimum) {
    const auto offsetIt = workspace.keyOffsets.find(key);
    if (offsetIt != workspace.keyOffsets.end()) {
      workspace.unconstrainedMinimum.segment(offsetIt->second, vector.size()) =
          vector;
    }
  }
}

/* ************************************************************************* */
void ActiveSetSolver::initializeConstraints() {
  Key multiplierKey = NextAvailableKey(keyDims_);

  const auto& eqConstraints = problemType_ == ProblemType::QP
                                  ? qpProblem_->eConstraints()
                                  : lpProblem_->eConstraints();
  const auto& ineqConstraints = problemType_ == ProblemType::QP
                                    ? qpProblem_->iConstraints()
                                    : lpProblem_->iConstraints();

  // Assign synthetic keys to scalar Lagrange multipliers. Keeping one key per
  // row lets the dual solve use ordinary GaussianFactorGraph machinery and lets
  // the public state rebuild row-ordered multiplier vectors later.
  constraints_.equalityDims.reserve(eqConstraints.size());
  for (size_t constraintIndex = 0; constraintIndex < eqConstraints.size();
       ++constraintIndex) {
    const auto constraintFactor = eqConstraints.at(constraintIndex);
    const LinearConstraint& constraint =
        RequireLinearConstraint(constraintFactor);
    const JacobianFactor& factor = constraint.factor();
    const double sign = ConstraintSign(constraint.sense());
    const size_t rows = static_cast<size_t>(factor.getb().size());
    constraints_.equalityDims.push_back(rows);
    for (size_t row = 0; row < rows; ++row) {
      constraints_.equalities.push_back(
          {constraintIndex, row, multiplierKey++,
           OneRowConstraintFactor(factor, row, sign)});
    }
  }

  constraints_.inequalityDims.reserve(ineqConstraints.size());
  for (size_t constraintIndex = 0; constraintIndex < ineqConstraints.size();
       ++constraintIndex) {
    const auto constraintFactor = ineqConstraints.at(constraintIndex);
    const LinearConstraint& constraint =
        RequireLinearConstraint(constraintFactor);
    const JacobianFactor& factor = constraint.factor();
    const double sign = ConstraintSign(constraint.sense());
    const size_t rows = static_cast<size_t>(factor.getb().size());
    constraints_.inequalityDims.push_back(rows);
    for (size_t row = 0; row < rows; ++row) {
      constraints_.inequalities.push_back(
          {constraintIndex, row, multiplierKey++,
           OneRowConstraintFactor(factor, row, sign)});
    }
  }
}

/* ************************************************************************* */
std::map<Key, ActiveSetSolver::ValueShape> ActiveSetSolver::valueShapes(
    const Values& values) const {
  std::map<Key, ValueShape> shapes;
  for (const auto& [key, dim] : keyDims_) {
    const Value& value = values.at(key);
    if (const auto* vectorValue =
            dynamic_cast<const GenericValue<Vector>*>(&value)) {
      const size_t rows = static_cast<size_t>(vectorValue->value().size());
      if (rows != dim) {
        throw std::invalid_argument(
            "ActiveSetSolver: Vector value dimension does "
            "not match problem dimension.");
      }
      shapes[key] = ValueShape{ValueShape::Type::Vector, rows, 1};
    } else if (const auto* matrixValue =
                   dynamic_cast<const GenericValue<Matrix>*>(&value)) {
      const Matrix& matrix = matrixValue->value();
      const size_t rows = static_cast<size_t>(matrix.rows());
      const size_t cols = static_cast<size_t>(matrix.cols());
      if (rows * cols != dim) {
        throw std::invalid_argument(
            "ActiveSetSolver: Matrix value dimension does "
            "not match problem dimension.");
      }
      shapes[key] = ValueShape{ValueShape::Type::Matrix, rows, cols};
    } else {
      throw std::invalid_argument(
          "ActiveSetSolver: only Vector and Matrix Values "
          "entries are supported.");
    }
  }
  return shapes;
}

/* ************************************************************************* */
void ActiveSetSolver::validateInitialValues(const Values& values) const {
  (void)valueShapes(values);
}

/* ************************************************************************* */
VectorValues ActiveSetSolver::valuesToVectorValues(const Values& values) const {
  VectorValues result;
  for (const auto& [key, dim] : keyDims_) {
    const Vector value = DirectVectorOrMatrixValue(values, key);
    if (static_cast<size_t>(value.size()) != dim) {
      throw std::invalid_argument(
          "ActiveSetSolver: Values dimension does not match "
          "problem dimension.");
    }
    result.insert(key, value);
  }
  return result;
}

/* ************************************************************************* */
Values ActiveSetSolver::vectorValuesToValues(const VectorValues& values,
                                             const Values& prototype) const {
  const auto shapes = valueShapes(prototype);
  Values result;
  for (const auto& [key, shape] : shapes) {
    const Vector& segment = values.at(key);
    if (static_cast<size_t>(segment.size()) != shape.dim()) {
      throw std::invalid_argument(
          "ActiveSetSolver: solution dimension does not "
          "match problem dimension.");
    }
    if (shape.type == ValueShape::Type::Vector) {
      result.insert(key, segment);
    } else {
      const Eigen::Map<const Matrix> matrix(segment.data(), shape.rows,
                                            shape.cols);
      result.insert(key, Matrix(matrix));
    }
  }
  return result;
}

/* ************************************************************************* */
GaussianFactorGraph ActiveSetSolver::buildWorkingGraph(
    const VectorValues& values,
    const std::vector<bool>& activeInequalityRows) const {
  GaussianFactorGraph workingGraph;
  if (problemType_ == ProblemType::QP) {
    workingGraph.push_back(qpCostGraph_);
    if (params_->regularization > 0.0) {
      const double sqrtRegularization = std::sqrt(params_->regularization);
      for (const auto& [key, dim] : keyDims_) {
        workingGraph.emplace_shared<JacobianFactor>(
            key, sqrtRegularization * Matrix::Identity(dim, dim),
            Vector::Zero(dim));
      }
    }
  } else {
    // For LPs, the subproblem is the projection of a unit negative-gradient
    // step onto the current active constraint surface. This produces a descent
    // direction; the line search below decides how far we can move.
    for (const auto& [key, dim] : keyDims_) {
      const Vector gradient = objectiveGradient(key, values);
      workingGraph.emplace_shared<JacobianFactor>(
          key, Matrix::Identity(dim, dim), values.at(key) - gradient);
    }
  }

  for (const ConstraintRow& row : constraints_.equalities) {
    workingGraph.push_back(row.factor);
  }
  for (size_t index = 0; index < constraints_.inequalities.size(); ++index) {
    if (activeInequalityRows.at(index)) {
      workingGraph.push_back(constraints_.inequalities[index].factor);
    }
  }
  return workingGraph;
}

/* ************************************************************************* */
GaussianFactorGraph ActiveSetSolver::buildDualGraph(
    const SparseConstraintData& constraints,
    const std::vector<bool>& activeInequalityRows,
    const VectorValues& values) const {
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
    if (!terms.empty()) {
      dualGraph.emplace_shared<JacobianFactor>(terms,
                                               objectiveGradient(key, values));
    }
  }
  return dualGraph;
}

/* ************************************************************************* */
Vector ActiveSetSolver::vectorValuesToDenseVector(
    const VectorValues& values) const {
  if (!denseQpWorkspace_) {
    throw std::logic_error(
        "ActiveSetSolver: dense QP workspace has not been initialized.");
  }
  const DenseQpWorkspace& workspace = *denseQpWorkspace_;
  Vector dense = Vector::Zero(workspace.totalDim);
  for (const auto& [key, dim] : keyDims_) {
    dense.segment(workspace.keyOffsets.at(key), dim) = values.at(key);
  }
  return dense;
}

/* ************************************************************************* */
VectorValues ActiveSetSolver::denseVectorToVectorValues(
    const Vector& vector) const {
  if (!denseQpWorkspace_) {
    throw std::logic_error(
        "ActiveSetSolver: dense QP workspace has not been initialized.");
  }
  const DenseQpWorkspace& workspace = *denseQpWorkspace_;
  VectorValues values;
  for (const auto& [key, dim] : keyDims_) {
    values.insert(key, vector.segment(workspace.keyOffsets.at(key), dim));
  }
  return values;
}

/* ************************************************************************* */
Vector ActiveSetSolver::applyInverseDenseHessian(const Vector& vector) const {
  if (!denseQpWorkspace_) {
    throw std::logic_error(
        "ActiveSetSolver: dense QP workspace has not been initialized.");
  }
  const DenseQpWorkspace& workspace = *denseQpWorkspace_;
  // The Bayes net represents a square-root Hessian factorization. Applying
  // H^{-1} is two triangular solves: first with the transpose, then with the
  // factor itself.
  const VectorValues vectorValues = denseVectorToVectorValues(vector);
  const VectorValues result = workspace.costBayesNet->backSubstitute(
      workspace.costBayesNet->backSubstituteTranspose(vectorValues));
  return vectorValuesToDenseVector(result);
}

/* ************************************************************************* */
std::pair<VectorValues, VectorValues> ActiveSetSolver::solveDenseQpSubproblem(
    const std::vector<bool>& activeInequalityRows) const {
  if (!denseQpWorkspace_) {
    throw std::logic_error(
        "ActiveSetSolver: dense QP workspace has not been initialized.");
  }
  const DenseQpWorkspace& workspace = *denseQpWorkspace_;

  size_t activeInequalityDim = 0;
  for (bool active : activeInequalityRows) {
    if (active) {
      ++activeInequalityDim;
    }
  }
  const size_t equalityDim = constraints_.equalities.size();
  const size_t kktDim = equalityDim + activeInequalityDim;

  VectorValues duals;
  if (kktDim == 0) {
    return {denseVectorToVectorValues(workspace.unconstrainedMinimum), duals};
  }

  Matrix activeA = Matrix::Zero(kktDim, workspace.totalDim);
  Vector activeB = Vector::Zero(kktDim);
  auto addRow = [&](size_t row, const ConstraintRow& constraint) {
    const JacobianFactor& factor = *constraint.factor;
    for (auto it = factor.begin(); it != factor.end(); ++it) {
      const Key key = *it;
      activeA.block(row, workspace.keyOffsets.at(key), 1, keyDims_.at(key)) =
          factor.getA(it);
    }
    activeB(row) = factor.getb()(0);
  };

  size_t row = 0;
  for (const ConstraintRow& equality : constraints_.equalities) {
    addRow(row++, equality);
  }
  for (size_t inequalityIndex = 0;
       inequalityIndex < constraints_.inequalities.size(); ++inequalityIndex) {
    if (activeInequalityRows.at(inequalityIndex)) {
      addRow(row++, constraints_.inequalities[inequalityIndex]);
    }
  }

  Matrix hessianInverseAT(workspace.totalDim, kktDim);
  for (size_t column = 0; column < kktDim; ++column) {
    hessianInverseAT.col(column) =
        applyInverseDenseHessian(activeA.row(column).transpose());
  }

  // Solve the equality-constrained QP through the Schur complement:
  //
  //     (A H^{-1} A^T) lambda = b - A*x0
  //     x = x0 + H^{-1} A^T lambda
  //
  // where x0 is the unconstrained minimizer of the quadratic cost.
  const Matrix schurComplement = activeA * hessianInverseAT;
  Eigen::LDLT<Matrix> ldlt(schurComplement);
  if (DenseSchurComplementIsSingular(ldlt, schurComplement)) {
    throw std::runtime_error(
        "ActiveSetSolver: dense QP Schur complement is singular or "
        "ill-conditioned. Active constraints may be linearly dependent; "
        "remove redundant constraints or use the sparse QP subproblem solver.");
  }
  const Vector multipliers =
      ldlt.solve(activeB - activeA * workspace.unconstrainedMinimum);
  if (!multipliers.allFinite()) {
    throw std::runtime_error(
        "ActiveSetSolver: dense QP Schur complement solve produced non-finite "
        "multipliers. Active constraints may be ill-conditioned; use the "
        "sparse QP subproblem solver.");
  }
  const Vector values =
      workspace.unconstrainedMinimum + hessianInverseAT * multipliers;

  size_t multiplierRow = 0;
  for (const ConstraintRow& equality : constraints_.equalities) {
    duals.insert(equality.multiplierKey,
                 SingleValue(multipliers(multiplierRow++)));
  }
  for (size_t inequalityIndex = 0;
       inequalityIndex < constraints_.inequalities.size(); ++inequalityIndex) {
    if (!activeInequalityRows.at(inequalityIndex)) {
      continue;
    }
    const ConstraintRow& inequality =
        constraints_.inequalities[inequalityIndex];
    duals.insert(inequality.multiplierKey,
                 SingleValue(multipliers(multiplierRow++)));
  }

  return {denseVectorToVectorValues(values), duals};
}

/* ************************************************************************* */
bool ActiveSetSolver::denseQpStepIsSmall(const VectorValues& candidateValues,
                                         const VectorValues& values) const {
  const Vector step = vectorValuesToDenseVector(candidateValues) -
                      vectorValuesToDenseVector(values);
  return step.norm() <= params_->stepTolerance;
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
  if (params_->regularization > 0.0) {
    gradient += params_->regularization * values.at(key);
  }
  return gradient;
}

/* ************************************************************************* */
ActiveSetSolver::InternalState ActiveSetSolver::initialState(
    const Values& values, const State* warmStart) const {
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

  if (warmStart && warmStart->activeInequalityRows.size() ==
                       constraints_.inequalities.size()) {
    state.activeInequalityRows = warmStart->activeInequalityRows;
    return state;
  }

  for (size_t row = 0; row < constraints_.inequalities.size(); ++row) {
    const double violation =
        RowResidual(*constraints_.inequalities[row].factor, state.values);
    state.activeInequalityRows[row] =
        std::abs(violation) <= params_->activeTolerance;
  }
  return state;
}

/* ************************************************************************* */
ActiveSetSolver::InternalState ActiveSetSolver::iterate(
    const InternalState& state) const {
  const bool useDenseQp =
      problemType_ == ProblemType::QP &&
      params_->qpSubproblemSolver == Params::QpSubproblemSolver::Dense;

  VectorValues candidateValues;
  VectorValues subproblemDuals;
  if (useDenseQp) {
    std::tie(candidateValues, subproblemDuals) =
        solveDenseQpSubproblem(state.activeInequalityRows);
  } else {
    candidateValues =
        buildWorkingGraph(state.values, state.activeInequalityRows).optimize();
    CompleteMissingKeys(&candidateValues, state.values, keyDims_);
  }

  InternalState next = state;
  next.iterations = state.iterations + 1;

  const bool smallStep =
      useDenseQp ? denseQpStepIsSmall(candidateValues, state.values)
                 : candidateValues.equals(state.values, params_->stepTolerance);
  if (smallStep) {
    // No primal movement is left for this working set. Check KKT multipliers:
    // if every active inequality multiplier has the correct sign, we are done;
    // otherwise remove the most offending row and solve the reduced set.
    if (useDenseQp) {
      next.duals = subproblemDuals;
    } else {
      const GaussianFactorGraph dualGraph = buildDualGraph(
          constraints_, state.activeInequalityRows, candidateValues);
      next.duals = dualGraph.empty() ? VectorValues() : dualGraph.optimize();
    }

    int leavingRow = -1;
    double largestMultiplier = params_->multiplierTolerance;
    for (size_t row = 0; row < constraints_.inequalities.size(); ++row) {
      if (!state.activeInequalityRows[row]) {
        continue;
      }
      const Key multiplierKey = constraints_.inequalities[row].multiplierKey;
      const double lambda = next.duals.exists(multiplierKey)
                                ? next.duals.at(multiplierKey)(0)
                                : 0.0;
      if (lambda > largestMultiplier) {
        leavingRow = static_cast<int>(row);
        largestMultiplier = lambda;
      }
    }
    next.values = candidateValues;
    if (leavingRow < 0) {
      next.converged = true;
    } else {
      next.activeInequalityRows[leavingRow] = false;
    }
    return next;
  }

  const VectorValues direction = candidateValues - state.values;
  // Move toward the working-set solution until an inactive inequality becomes
  // tight. QPs can take the full step when nothing blocks; LPs use the same
  // calculation to detect unbounded descent.
  double step = problemType_ == ProblemType::QP
                    ? 1.0
                    : std::numeric_limits<double>::infinity();
  int blockingRow = -1;
  for (size_t row = 0; row < constraints_.inequalities.size(); ++row) {
    if (state.activeInequalityRows[row]) {
      continue;
    }
    const JacobianFactor& factor = *constraints_.inequalities[row].factor;
    const double denominator = RowDotProduct(factor, direction);
    if (denominator <= 0.0) {
      continue;
    }
    const double residual = RowResidual(factor, state.values);
    const double candidateStep = -residual / denominator;
    if (candidateStep < step) {
      step = std::max(0.0, candidateStep);
      blockingRow = static_cast<int>(row);
    }
  }

  if (problemType_ == ProblemType::LP && blockingRow < 0) {
    throw std::runtime_error("ActiveSetSolver: LP is unbounded.");
  }

  next.values = state.values + step * direction;
  if (blockingRow >= 0) {
    next.activeInequalityRows[blockingRow] = true;
  }
  return next;
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
  return optimizeWithState(initialValues, nullptr);
}

/* ************************************************************************* */
std::pair<Values, ActiveSetSolver::State> ActiveSetSolver::optimizeWithState(
    const Values& initialValues, const State& warmStart) const {
  return optimizeWithState(initialValues, &warmStart);
}

/* ************************************************************************* */
Values ActiveSetSolver::optimize(const Values& initialValues) const {
  return optimizeWithState(initialValues).first;
}

/* ************************************************************************* */
std::pair<Values, ActiveSetSolver::State> ActiveSetSolver::optimizeWithState()
    const {
  return optimizeWithState(findVectorFeasiblePoint());
}

/* ************************************************************************* */
Values ActiveSetSolver::optimize() const { return optimizeWithState().first; }

/* ************************************************************************* */
std::pair<Values, ActiveSetSolver::State> ActiveSetSolver::optimizeWithState(
    const Values& initialValues, const State* warmStart) const {
  validateInitialValues(initialValues);
  InternalState state = initialState(initialValues, warmStart);
  while (!state.converged) {
    if (state.iterations >= params_->maxIterations) {
      throw std::runtime_error("ActiveSetSolver: failed to converge.");
    }
    state = iterate(state);
  }
  State publicState = buildPublicState(state, initialValues);
  return {publicState.values, publicState};
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
Values ActiveSetSolver::findVectorFeasiblePoint() const {
  const Values zeroValues = zeroVectorValues();
  LpProblem phaseProblem;
  const Key slackKey = NextAvailableKey(keyDims_);
  // Phase one minimizes a non-negative scalar slack s. If the optimum slack is
  // zero within tolerance, the original variables satisfy all constraints.
  phaseProblem.addCost(
      JacobianFactor(slackKey, Matrix::Identity(1, 1), Vector::Zero(1)));

  auto rowTerms = [&](const ConstraintRow& row, double rowSign) {
    std::vector<std::pair<Key, Matrix>> terms;
    terms.reserve(row.factor->size() + 1);
    for (auto it = row.factor->begin(); it != row.factor->end(); ++it) {
      terms.emplace_back(*it, rowSign * row.factor->getA(it));
    }
    terms.emplace_back(slackKey, -Matrix::Identity(1, 1));
    return terms;
  };

  double initialSlack = 1.0;
  for (const ConstraintRow& row : constraints_.equalities) {
    const double b = row.factor->getb()(0);
    // Equality A*x=b is relaxed as both A*x-s<=b and -A*x-s<=-b.
    phaseProblem.addConstraint(LinearConstraint::LessEqual(
        JacobianFactor(rowTerms(row, 1.0), SingleValue(b))));
    phaseProblem.addConstraint(LinearConstraint::LessEqual(
        JacobianFactor(rowTerms(row, -1.0), SingleValue(-b))));
    initialSlack = std::max(initialSlack, std::abs(b) + 1.0);
  }
  for (const ConstraintRow& row : constraints_.inequalities) {
    const double b = row.factor->getb()(0);
    phaseProblem.addConstraint(LinearConstraint::LessEqual(
        JacobianFactor(rowTerms(row, 1.0), SingleValue(b))));
    initialSlack = std::max(initialSlack, -b + 1.0);
  }
  phaseProblem.addConstraint(LinearConstraint::LessEqual(
      JacobianFactor(slackKey, -Matrix::Identity(1, 1), Vector::Zero(1))));

  Values phaseInitial = zeroValues;
  const Vector slackInitial = Vector::Constant(1, initialSlack);
  phaseInitial.insert(slackKey, slackInitial);
  const Values phaseResult =
      ActiveSetSolver(phaseProblem, params_).optimize(phaseInitial);
  const double slack = phaseResult.at<Vector>(slackKey)(0);
  if (slack > params_->phaseOneFeasibilityTolerance) {
    throw std::runtime_error(
        "ActiveSetSolver: could not find a feasible point.");
  }

  Values feasibleValues;
  for (const auto& [key, _] : keyDims_) {
    feasibleValues.insert(key, phaseResult.exists(key)
                                   ? phaseResult.at<Vector>(key)
                                   : zeroValues.at<Vector>(key));
  }
  return feasibleValues;
}

}  // namespace gtsam

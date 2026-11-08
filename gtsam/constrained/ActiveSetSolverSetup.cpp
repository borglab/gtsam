/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file ActiveSetSolverSetup.cpp
 * Input validation, constraint-row representation, and dense packing.
 *
 * These are ActiveSetSolver member definitions using its existing private state.
 * Keeping one-time setup and Values conversion here leaves the main source file
 * focused on working-set iteration. Original factors, row order, parameter
 * validation, and dense packing retain their existing behavior.
 */

#include <gtsam/constrained/internal/ActiveSetSolverInternal.h>
#include <gtsam/base/GenericValue.h>

#include <Eigen/Eigenvalues>
#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

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
// Scatter only Hessian-native costs; original Jacobians retain their square
// roots. This helper does not add curvature or change the original linear term.
void accumulateDenseHessian(const HessianFactor& factor,
                            const std::map<Key, size_t>& offsets,
                            Matrix* hessian, Vector* linear) {
  const Matrix information = factor.information();
  size_t localRow = 0;
  for (auto i = factor.begin(); i != factor.end(); ++i) {
    const size_t ni = factor.getDim(i), gi = offsets.at(*i);
    linear->segment(gi, ni) += factor.linearTerm(i);
    size_t localCol = 0;
    for (auto j = factor.begin(); j != factor.end(); ++j) {
      const size_t nj = factor.getDim(j), gj = offsets.at(*j);
      hessian->block(gi, gj, ni, nj) +=
          information.block(localRow, localCol, ni, nj);
      localCol += nj;
    }
    localRow += ni;
  }
}

/* ************************************************************************* */
// Pack already whitened local blocks without forming objective normal equations.
void packDenseJacobians(const std::vector<JacobianFactor>& factors,
                         const std::map<Key, size_t>& offsets,
                         internal::DenseQpData* data) {
  size_t row = 0;
  for (const auto& factor : factors) {
    for (auto key = factor.begin(); key != factor.end(); ++key)
      data->jacobian.block(row, offsets.at(*key), factor.rows(),
                           factor.getDim(key)) = factor.getA(key);
    data->rhs.segment(row, factor.rows()) = factor.getb();
    row += factor.rows();
  }
}

}  // namespace

/* ************************************************************************* */
ActiveSetSolver::ActiveSetSolver(const QpProblem& problem,
                                 Params::shared_ptr params)
    : problemType_(ProblemType::QP),
      qpProblem_(&problem),
      params_(params ? std::make_shared<Params>(*params) : std::make_shared<Params>()) {
  initializeDimensions();
  validateParameters();
  initializeObjective();
  initializeConstraints();
  if (problemType_ == ProblemType::QP &&
      params_->qpSubproblemSolver == Params::QpSubproblemSolver::Dense)
    initializeDenseQpWorkspace();
}

/* ************************************************************************* */
ActiveSetSolver::ActiveSetSolver(const LpProblem& problem,
                                 Params::shared_ptr params)
    : problemType_(ProblemType::LP),
      lpProblem_(&problem),
      params_(params ? std::make_shared<Params>(*params) : std::make_shared<Params>()) {
  if (params_->qpFormulation == Params::QpFormulation::Automatic)
    params_->qpFormulation = Params::QpFormulation::Primal;
  initializeDimensions();
  validateParameters();
  initializeObjective();
  initializeConstraints();
  if (problemType_ == ProblemType::QP &&
      params_->qpSubproblemSolver == Params::QpSubproblemSolver::Dense)
    initializeDenseQpWorkspace();
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
void ActiveSetSolver::validateParameters() const {
  switch (params_->qpFormulation) {
    case Params::QpFormulation::Primal:
    case Params::QpFormulation::Dual:
    case Params::QpFormulation::Automatic:
      break;
    default:
      throw std::invalid_argument("ActiveSetSolver: unknown QP formulation.");
  }
  if (problemType_ == ProblemType::LP &&
      params_->qpFormulation != Params::QpFormulation::Primal)
    throw std::invalid_argument("ActiveSetSolver: LPs require Primal formulation.");
  // Reject unknown modes and QP-only initialization on an LP explicitly.
  switch (params_->qpInitialization) {
    case Params::QpInitialization::TightConstraints:
    case Params::QpInitialization::ProjectedGradient:
    case Params::QpInitialization::FeasibleStepThenProjectedGradient:
      break;
    default:
      throw std::invalid_argument(
          "ActiveSetSolver: unknown QP initialization.");
  }
  if (problemType_ == ProblemType::LP &&
      params_->qpInitialization != Params::QpInitialization::TightConstraints)
    throw std::invalid_argument(
        "ActiveSetSolver: LPs require TightConstraints initialization.");
  for (double value : {params_->activeTolerance, params_->stepTolerance,
                       params_->feasibilityTolerance, params_->multiplierTolerance,
                       params_->phaseOneFeasibilityTolerance, params_->regularization})
    if (!std::isfinite(value) || value < 0)
      throw std::invalid_argument("ActiveSetSolver: tolerances and regularization must be finite and nonnegative.");
  if (!params_->ordering) return;
  KeySet keys;
  for (Key key : *params_->ordering)
    if (!keyDims_.count(key) || !keys.insert(key).second)
      throw std::invalid_argument("ActiveSetSolver: ordering has unknown or repeated keys.");
  if (keys.size() != keyDims_.size())
    throw std::invalid_argument("ActiveSetSolver: ordering must contain every problem key.");
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
    qpCostGraph_.push_back(cost->gaussianFactor().clone());
  }
  qpGradientAtZero_ = qpCostGraph_.gradientAtZero();
}

/* ************************************************************************* */
void ActiveSetSolver::initializeDenseQpWorkspace() {
  denseQpWorkspace_ = std::make_shared<DenseQpWorkspace>();
  auto& workspace = *denseQpWorkspace_;
  GaussianFactorGraph schema = qpCostGraph_;
  for (const auto& row : constraints_.equalities) schema.push_back(row.factor);
  for (const auto& row : constraints_.inequalities) schema.push_back(row.factor);
  workspace.hessianOrdering = params_->ordering ? *params_->ordering
                                                : Ordering::Colamd(schema);
  for (Key key : workspace.hessianOrdering) {
    workspace.keyOffsets[key] = workspace.totalDim;
    workspace.totalDim += keyDims_.at(key);
  }
  // Constraint-only variables are valid: null-space QR does not assume an
  // invertible unconstrained Hessian or invent missing curvature.
  workspace.data.regularization = params_->regularization;
  packDenseObjective();
  packDenseConstraints();
}

/* ************************************************************************* */
void ActiveSetSolver::packDenseObjective() {
  auto& workspace = *denseQpWorkspace_;
  auto& data = workspace.data;
  const size_t n = workspace.totalDim;
  Matrix hessian = Matrix::Zero(n, n);
  data.linear = Vector::Zero(n);
  std::vector<JacobianFactor> jacobians;
  size_t rowCount = 0;
  for (const auto& factor : qpCostGraph_) {
    if (const auto* jacobian = dynamic_cast<const JacobianFactor*>(factor.get())) {
      jacobians.push_back(jacobian->whiten());
      rowCount += jacobian->rows();
    } else {
      accumulateDenseHessian(dynamic_cast<const HessianFactor&>(*factor),
                              workspace.keyOffsets, &hessian, &data.linear);
    }
  }
  data.jacobian = Matrix::Zero(rowCount + n, n);
  data.rhs = Vector::Zero(rowCount + n);
  // Native positive-definite curvature needs neither an eigensquare root nor
  // a subsequent QR. Test H before adding any proximal term, so regularization
  // cannot hide nonconvexity. Weak pivots retain the rank-aware square-root path.
  if (jacobians.empty() && constraints_.equalities.empty() && n > 0 &&
      hessian.allFinite()) {
    Eigen::LLT<Matrix> cholesky(hessian);
    if (cholesky.info() == Eigen::Success &&
        cholesky.rcond() > std::sqrt(std::numeric_limits<double>::epsilon())) {
      data.jacobian = cholesky.matrixU();
      data.nativeHessian = std::move(hessian);
      return;
    }
  }
  packDenseJacobians(jacobians, workspace.keyOffsets, &data);
  packDenseHessian(hessian, rowCount);
}

/* ************************************************************************* */
void ActiveSetSolver::packDenseHessian(const Matrix& hessian, size_t rowStart) {
  // Convert native PSD curvature once, retaining every positive eigenvalue.
  // Negative roundoff is clipped only within its computed backward-error scale.
  if (hessian.isZero(0)) return;
  const double allowance = 64 * std::numeric_limits<double>::epsilon() *
                           hessian.rows() * hessian.norm();
  // Diagonal native curvature already supplies its eigenvalues. Keep every
  // nonzero off-diagonal coefficient on the general square-root path.
  if (hessian.isDiagonal(0)) {
    if (!hessian.diagonal().allFinite() || hessian.diagonal().minCoeff() < -allowance)
      throw std::runtime_error("ActiveSetSolver: dense QP objective is not convex.");
    denseQpWorkspace_->data.jacobian.middleRows(rowStart, hessian.rows())
        .diagonal() = hessian.diagonal().cwiseMax(0).cwiseSqrt();
    return;
  }
  Eigen::SelfAdjointEigenSolver<Matrix> eigen(hessian);
  if (eigen.info() != Eigen::Success || eigen.eigenvalues().minCoeff() < -allowance)
    throw std::runtime_error("ActiveSetSolver: dense QP objective is not convex.");
  denseQpWorkspace_->data.jacobian.middleRows(rowStart, hessian.rows()) =
      eigen.eigenvalues().cwiseMax(0).cwiseSqrt().asDiagonal() *
      eigen.eigenvectors().transpose();
}

/* ************************************************************************* */
void ActiveSetSolver::packDenseConstraints() {
  auto& workspace = *denseQpWorkspace_;
  auto& data = workspace.data;
  data.equalities = constraints_.equalities.size();
  const size_t count = data.equalities + constraints_.inequalities.size();
  data.normals = Matrix::Zero(count, workspace.totalDim);
  data.bounds.resize(count);
  size_t index = 0;
  auto append = [&](const ConstraintRow& row) {
    for (auto key = row.factor->begin(); key != row.factor->end(); ++key)
      data.normals.block(index, workspace.keyOffsets.at(*key), 1,
                         keyDims_.at(*key)) = row.factor->getA(key);
    data.bounds(index++) = row.factor->getb()(0);
  };
  for (const auto& row : constraints_.equalities) append(row);
  for (const auto& row : constraints_.inequalities) append(row);
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
  auto appendRows = [&](const auto& source, auto* rows, auto* dimensions) {
    dimensions->reserve(source.size());
    for (size_t constraintIndex = 0; constraintIndex < source.size();
         ++constraintIndex) {
      const auto constraintFactor = source.at(constraintIndex);
      const LinearConstraint& constraint =
          RequireLinearConstraint(constraintFactor);
      const JacobianFactor& factor = constraint.factor();
      const double sign = ConstraintSign(constraint.sense());
      const size_t rowCount = static_cast<size_t>(factor.getb().size());
      dimensions->push_back(rowCount);
      for (size_t row = 0; row < rowCount; ++row)
        rows->push_back({constraintIndex, row, multiplierKey++,
                         OneRowConstraintFactor(factor, row, sign)});
    }
  };
  appendRows(eqConstraints, &constraints_.equalities,
             &constraints_.equalityDims);
  appendRows(ineqConstraints, &constraints_.inequalities,
             &constraints_.inequalityDims);
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
  for (const auto& [key, _] : keyDims_)
    if (!DirectVectorOrMatrixValue(values, key).allFinite())
      throw std::invalid_argument("ActiveSetSolver: initial values must be finite.");
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

}  // namespace gtsam

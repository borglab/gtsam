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

#include <Eigen/Cholesky>
#include <Eigen/QR>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
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

// Dense rows are assembled only in dense mode, in the same order as their
// multiplier keys. Sparse mode keeps each row as a factor instead.
struct ActiveSetSolver::DenseConstraintSystem {
  Matrix normals;
  Vector rhs;
  std::vector<Key> multiplierKeys;
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
Vector1 SingleValue(double value) { return Vector1{value}; }

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

/* ************************************************************************* */
// Multipliers may be nonunique. A measured residual distinguishes a valid
// least-squares multiplier set from an inconsistent stationarity equation.
struct DualSolveResult {
  VectorValues multipliers;
  double residual = 0.0;
  double threshold = 0.0;

  bool consistent() const { return residual <= threshold; }
};

/* ************************************************************************* */
std::pair<double, double> DualGraphResidual(const GaussianFactorGraph& graph,
                                            const VectorValues& multipliers) {
  for (const auto& [_, value] : multipliers)
    if (!value.allFinite())
      throw std::runtime_error("ActiveSetSolver: non-finite multipliers.");
  double residual = 0.0, rhsNorm = 0.0;
  for (const auto& factor : graph) {
    const JacobianFactor whitened = JacobianFactor(*factor).whiten();
    residual = std::max(
        residual,
        whitened.unweighted_error(multipliers).lpNorm<Eigen::Infinity>());
    rhsNorm = std::max(rhsNorm, whitened.getb().lpNorm<Eigen::Infinity>());
  }
  return {residual, rhsNorm};
}

/* ************************************************************************* */
// Retain original column scales: a dependent column can reach a later clique
// as roundoff alone, which must not become its new reference scale.
std::map<Key, double> dualColumnScales(const GaussianFactorGraph& graph) {
  std::map<Key, double> scales;
  for (const auto& factor : graph) {
    const JacobianFactor whitened = JacobianFactor(*factor).whiten();
    for (auto key = whitened.begin(); key != whitened.end(); ++key) {
      const auto block = whitened.getA(key);
      if (block.cols() != 1)
        throw std::logic_error("Expected scalar multiplier keys");
      scales[*key] = std::hypot(scales[*key], block.stableNorm());
    }
  }
  return scales;
}

/* ************************************************************************* */
// Pivot only among frontal columns. Separator equations must survive unchanged
// so that the rest of the Bayes tree still solves the original least squares.
KeyVector freeDualFrontalKeys(const JacobianFactor& joint,
                              const Ordering& frontals,
                              const std::map<Key, double>& scales) {
  if (joint.rows() == 0) return KeyVector(frontals.begin(), frontals.end());
  Matrix frontalMatrix(joint.rows(), frontals.size());
  for (size_t column = 0; column < frontals.size(); ++column) {
    const Key key = frontals[column];
    const double scale = scales.at(key);
    frontalMatrix.col(column) =
        joint.getA(joint.find(key)) / (scale > 0.0 ? scale : 1.0);
  }
  Eigen::ColPivHouseholderQR<Matrix> qr(frontalMatrix);
  const double threshold = 64.0 * std::numeric_limits<double>::epsilon() *
                           std::max(frontalMatrix.rows(), frontalMatrix.cols());
  // Eigen's threshold is relative to the largest pivot; ours is relative to
  // the original columns after normalization, including at a nearly zero root.
  qr.setThreshold(qr.maxPivot() > 0.0 ? threshold / qr.maxPivot() : 1.0);
  KeyVector freeKeys;
  for (Eigen::Index column = qr.rank(); column < frontalMatrix.cols(); ++column)
    freeKeys.push_back(frontals[qr.colsPermutation().indices()(column)]);
  return freeKeys;
}

/* ************************************************************************* */
// Dual-only elimination: choose zero for free scalar multipliers, then return
// an ordinary nonsingular conditional through GTSAM's constrained QR. For
// exact dependencies this selects one solution without changing the reduced
// objective on the separator. The caller checks the original graph residual.
GaussianFactorGraph::EliminationResult eliminateDualWithGauge(
    const GaussianFactorGraph& factors, const Ordering& frontals,
    const std::map<Key, double>& scales) {
  auto joint = std::make_shared<JacobianFactor>(factors, frontals);
  *joint = joint->whiten();
  GaussianFactorGraph gauged;
  gauged.push_back(joint);
  for (Key key : freeDualFrontalKeys(*joint, frontals, scales))
    gauged.emplace_shared<JacobianFactor>(key, Matrix::Ones(1, 1),
                                          Vector::Zero(1),
                                          noiseModel::Constrained::All(1));
  return EliminateQR(gauged, frontals);
}

/* ************************************************************************* */
DualSolveResult solveRankDeficientDualGraph(const GaussianFactorGraph& graph,
                                            double tolerance) {
  const auto scales = dualColumnScales(graph);
  DualSolveResult result;
  result.multipliers = graph.optimize(
      [&](const GaussianFactorGraph& factors, const Ordering& frontals) {
        return eliminateDualWithGauge(factors, frontals, scales);
      });
  double rhsNorm;
  std::tie(result.residual, rhsNorm) =
      DualGraphResidual(graph, result.multipliers);
  result.threshold = std::max(1e-12, tolerance) * (1.0 + rhsNorm);
  return result;
}

/* ************************************************************************* */
DualSolveResult SolveSparseDualGraph(const GaussianFactorGraph& graph,
                                     double tolerance,
                                     bool checkStationarity = true) {
  if (graph.empty()) return {};
  try {
    // The usual graph solve is faster, but a successful factorization alone
    // does not prove that an overdetermined stationarity equation is exact.
    DualSolveResult result;
    result.multipliers = graph.optimize();
    if (!checkStationarity) return result;
    double rhsNorm;
    std::tie(result.residual, rhsNorm) =
        DualGraphResidual(graph, result.multipliers);
    result.threshold = std::max(1e-12, tolerance) * (1.0 + rhsNorm);
    if (result.consistent()) return result;
  } catch (const IndeterminateSystemException&) {
    // The graph QR fallback fixes free multiplier coordinates clique by clique.
  }
  DualSolveResult result = solveRankDeficientDualGraph(graph, tolerance);
  if (!checkStationarity && !result.consistent())
    throw std::runtime_error("Multiplier graph QR: inconsistent stationarity");
  return result;
}

/* ************************************************************************* */
// These row helpers need no solver state. Templates keep ConstraintRow private
// to ActiveSetSolver while accepting its row containers at the call site.
template <typename Row>
bool primalFeasible(const std::vector<Row>& equalities,
                    const std::vector<Row>& inequalities,
                    const VectorValues& values, double tolerance) {
  for (const Row& row : equalities) {
    const double residual = RowResidual(*row.factor, values);
    if (!std::isfinite(residual) || std::abs(residual) > tolerance)
      return false;
  }
  for (const Row& row : inequalities) {
    const double residual = RowResidual(*row.factor, values);
    if (!std::isfinite(residual) || residual > tolerance) return false;
  }
  return true;
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
    qpCostGraph_.push_back(cost->gaussianFactor().clone());
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

  factorDenseQpCost();
  // Constrained solves reuse the unconstrained minimizer x0 and apply only
  // the active constraint correction x = x0 + H^{-1} A^T lambda.
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
void ActiveSetSolver::factorDenseQpCost() {
  DenseQpWorkspace& workspace = *denseQpWorkspace_;
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
  } catch (const IndeterminateSystemException&) {
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
    } catch (const IndeterminateSystemException&) {
      throw std::runtime_error(
          "ActiveSetSolver: dense QP Hessian factorization failed even after "
          "minimum dense regularization. Increase regularization or use the "
          "sparse QP subproblem solver.");
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
            sqrtRegularization * values.at(key));
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
    const std::vector<bool>& activeInequalityRows, const VectorValues& values,
    const VectorValues* proximalCenter) const {
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
    Vector gradient = objectiveGradient(key, values);
    if (proximalCenter)
      gradient +=
          params_->regularization * (values.at(key) - proximalCenter->at(key));
    dualGraph.emplace_shared<JacobianFactor>(terms, gradient);
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
ActiveSetSolver::DenseConstraintSystem ActiveSetSolver::denseConstraintSystem(
    const std::vector<bool>& activeInequalityRows) const {
  const DenseQpWorkspace& workspace = *denseQpWorkspace_;
  size_t rowCount = constraints_.equalities.size();
  for (bool active : activeInequalityRows) rowCount += active ? 1 : 0;

  DenseConstraintSystem system;
  system.normals = Matrix::Zero(rowCount, workspace.totalDim);
  system.rhs = Vector::Zero(rowCount);
  system.multiplierKeys.reserve(rowCount);
  auto appendRow = [&](const ConstraintRow& row) {
    const size_t index = system.multiplierKeys.size();
    const JacobianFactor& factor = *row.factor;
    for (auto key = factor.begin(); key != factor.end(); ++key)
      system.normals.block(index, workspace.keyOffsets.at(*key), 1,
                           keyDims_.at(*key)) = factor.getA(key);
    system.rhs(index) = factor.getb()(0);
    system.multiplierKeys.push_back(row.multiplierKey);
  };
  for (const ConstraintRow& row : constraints_.equalities) appendRow(row);
  for (size_t row = 0; row < constraints_.inequalities.size(); ++row)
    if (activeInequalityRows.at(row)) appendRow(constraints_.inequalities[row]);
  return system;
}

/* ************************************************************************* */
std::pair<VectorValues, VectorValues> ActiveSetSolver::solveDenseQpSubproblem(
    const std::vector<bool>& activeInequalityRows) const {
  if (!denseQpWorkspace_)
    throw std::logic_error(
        "ActiveSetSolver: dense QP workspace has not been initialized.");
  const DenseQpWorkspace& workspace = *denseQpWorkspace_;
  const DenseConstraintSystem active =
      denseConstraintSystem(activeInequalityRows);
  const size_t kktDim = active.multiplierKeys.size();
  VectorValues duals;
  if (kktDim == 0)
    return {denseVectorToVectorValues(workspace.unconstrainedMinimum), duals};

  Matrix hessianInverseAT(workspace.totalDim, kktDim);
  for (size_t column = 0; column < kktDim; ++column)
    hessianInverseAT.col(column) =
        applyInverseDenseHessian(active.normals.row(column).transpose());

  // Schur complement: (A H^{-1} A^T) lambda = b - A*x0, followed by
  // x = x0 + H^{-1} A^T lambda. The Hessian factorization is reused.
  const Matrix schurComplement = active.normals * hessianInverseAT;
  Eigen::LDLT<Matrix> ldlt(schurComplement);
  if (DenseSchurComplementIsSingular(ldlt, schurComplement))
    throw std::runtime_error(
        "ActiveSetSolver: dense QP Schur complement is singular or "
        "ill-conditioned. Active constraints may be linearly dependent; "
        "remove redundant constraints or use the sparse QP subproblem solver.");
  const Vector multipliers =
      ldlt.solve(active.rhs - active.normals * workspace.unconstrainedMinimum);
  if (!multipliers.allFinite())
    throw std::runtime_error(
        "ActiveSetSolver: dense QP Schur complement solve produced non-finite "
        "multipliers. Active constraints may be ill-conditioned; use the "
        "sparse QP subproblem solver.");

  const Vector values =
      workspace.unconstrainedMinimum + hessianInverseAT * multipliers;
  for (size_t row = 0; row < kktDim; ++row)
    duals.insert(active.multiplierKeys[row], SingleValue(multipliers(row)));
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
  // Sparse QPs regularize only the working subproblem, not the target cost.
  if (params_->qpSubproblemSolver == Params::QpSubproblemSolver::Dense &&
      params_->regularization > 0.0) {
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
  VectorValues candidate, subproblemDuals;
  if (useDenseQp) {
    std::tie(candidate, subproblemDuals) =
        solveDenseQpSubproblem(state.activeInequalityRows);
  } else {
    // Keep least-squares QPs in square-root form throughout elimination.
    // Forming normal equations can erase weak finite directions, including
    // after an active row leaves the working set.
    const bool squareRootQp =
        problemType_ == ProblemType::QP &&
        std::all_of(qpCostGraph_.begin(), qpCostGraph_.end(),
                    [](const GaussianFactor::shared_ptr& factor) {
                      return dynamic_cast<const JacobianFactor*>(factor.get());
                    });
    const GaussianFactorGraph workingGraph =
        buildWorkingGraph(state.values, state.activeInequalityRows);
    candidate = squareRootQp ? workingGraph.optimize(EliminateQR)
                            : workingGraph.optimize();
    CompleteMissingKeys(&candidate, state.values, keyDims_);
  }

  InternalState next = state;
  next.iterations = state.iterations + 1;
  if (!shouldInspectOptimality(state, candidate, useDenseQp)) {
    moveTowardCandidate(state, candidate, &next);
    return next;
  }

  if (useDenseQp) {
    next.duals = subproblemDuals;
  } else if (!recoverSparseMultipliers(state, candidate, &next.duals)) {
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
    next.converged = useDenseQp || problemType_ == ProblemType::LP ||
                     originalSparseKktSatisfied(
                         candidate, state.activeInequalityRows, next.duals);
  }
  return next;
}

/* ************************************************************************* */
bool ActiveSetSolver::shouldInspectOptimality(const InternalState& state,
                                              const VectorValues& candidate,
                                              bool useDenseQp) const {
  if (!useDenseQp && problemType_ == ProblemType::QP) {
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
  return useDenseQp ? denseQpStepIsSmall(candidate, state.values)
                    : candidate.equals(state.values, params_->stepTolerance);
}

/* ************************************************************************* */
bool ActiveSetSolver::recoverSparseMultipliers(const InternalState& state,
                                               const VectorValues& candidate,
                                               VectorValues* duals) const {
  // The original gradient is the only one that can certify a solution.
  const DualSolveResult original = SolveSparseDualGraph(
      buildDualGraph(constraints_, state.activeInequalityRows, candidate),
      params_->multiplierTolerance, problemType_ == ProblemType::QP);
  if (original.consistent()) {
    *duals = original.multipliers;
    return true;
  }

  if (problemType_ == ProblemType::QP && params_->regularization > 0.0) {
    const DualSolveResult proximal = SolveSparseDualGraph(
        buildDualGraph(constraints_, state.activeInequalityRows, candidate,
                       &state.values),
        params_->multiplierTolerance);
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
bool ActiveSetSolver::originalSparseKktSatisfied(
    const VectorValues& values, const std::vector<bool>& activeRows,
    const VectorValues& duals) const {
  if (!primalFeasible(constraints_.equalities, constraints_.inequalities,
                      values, params_->feasibilityTolerance))
    return false;
  VectorValues stationarity;
  double gap = 0.0;
  for (const auto& [key, _] : keyDims_) {
    const Vector gradient = objectiveGradient(key, values);
    stationarity.insert(key, gradient);
    gap += values.at(key).dot(gradient);
  }

  auto subtractRow = [&](const ConstraintRow& row) {
    if (!duals.exists(row.multiplierKey)) return;
    const double multiplier = duals.at(row.multiplierKey)(0);
    gap -= row.factor->getb()(0) * multiplier;
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
  return std::isfinite(gap) && std::abs(gap) <= params_->multiplierTolerance;
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
  Values feasible;
  if (problemType_ == ProblemType::QP &&
      params_->qpSubproblemSolver == Params::QpSubproblemSolver::Sparse &&
      tryEqualityProjection(zeroValues, &feasible))
    return feasible;
  return solvePhaseOne(zeroValues);
}
/* ************************************************************************* */
bool ActiveSetSolver::tryEqualityProjection(const Values& zeroValues,
                                            Values* feasible) const {
  // Unit unary factors select a minimum-norm point on the equality manifold.
  GaussianFactorGraph graph;
  for (const auto& [key, dim] : keyDims_)
    graph.emplace_shared<JacobianFactor>(key, Matrix::Identity(dim, dim),
                                         Vector::Zero(dim));
  for (const ConstraintRow& row : constraints_.equalities)
    graph.push_back(row.factor);

  try {
    const VectorValues candidate = graph.optimize();
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
  const Values result =
      ActiveSetSolver(phaseProblem, params_).optimize(initial);
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

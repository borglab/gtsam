#include <gtsam/certifiable/LiftedSDPProblem.h>

#include <Eigen/Eigenvalues>

#include <cmath>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>

#ifdef GTSAM_USE_MOSEK
#include <fusion.h>
#include <monty.h>

namespace mf = mosek::fusion;
#endif

namespace gtsam {

#ifdef GTSAM_USE_MOSEK

// Put MOSEK only functions in anonymous namespace.
namespace {

// # copied from gtsam-private
using LiftedVariableXijToSDPVariableViewMap = std::map<std::pair<Key, Key>, mf::Variable::t>;

// # copied from gtsam-private
struct MosekSolveSummary {
  bool solved = false;
  mf::ProblemStatus problemStatus;
  double optimizerTimeSeconds;
};

// # copied from gtsam-private
std::map<std::string, double> DefaultMosekParams() {
  return {
      {"intpntCoTolRelGap", 1e-10},
      {"intpntCoTolDfeas", 1e-10},
      {"intpntCoTolPfeas", 1e-10},
      {"intpntCoTolInfeas", 1e-10},
  };
}

// # copied from gtsam-private
std::map<std::string, double> MergeMosekParams(
    const std::map<std::string, double>& overrides) {
  auto merged = DefaultMosekParams();
  for (const auto& kv : overrides) {
    merged[kv.first] = kv.second;
  }
  return merged;
}

// # copied from gtsam-private
MosekSolveSummary SolveMosekModel(
    const mf::Model::t& M,
    const std::map<std::string, double>& mosek_params) {
  const auto mergedParams = MergeMosekParams(mosek_params);
  for (const auto& kv : mergedParams) {
    M->setSolverParam(kv.first, kv.second);
  }

  MosekSolveSummary summary;
  M->solve();
  summary.problemStatus = M->getProblemStatus();
  summary.optimizerTimeSeconds = M->getSolverDoubleInfo("optimizerTime");
  summary.solved = true;

  return summary;
}

std::map<Key, DenseIndex> CollectQpCostKeyDims(const QcqpProblem& problem,
                                               KeySet* costKeys) {
  std::map<Key, DenseIndex> keyDims;
  for (const auto& factor : problem.costs()) {
    if (!factor) {
      continue;
    }

    const auto* cost = dynamic_cast<const QpCost*>(factor.get());
    if (!cost) {
      throw std::runtime_error(
          "CollectQpCostKeyDims: expected objective factors to be QpCost.");
    }

    const HessianFactor& H = cost->hessianFactor();
    for (auto it = H.begin(); it != H.end(); ++it) {
      const Key key = *it;
      if (costKeys) {
        costKeys->insert(key);
      }
      const DenseIndex dim = H.getDim(it);
      const auto [entry, inserted] = keyDims.emplace(key, dim);
      if (!inserted && entry->second != dim) {
        throw std::runtime_error(
            "CollectQpCostKeyDims: inconsistent QpCost dimension for key.");
      }
    }
  }
  return keyDims;
}

// # copied from gtsam-private
void DisposeMosekModel(const mf::Model::t& M) {
  if (M.get() != nullptr) {
    M->dispose();
  }
}

// # copied from gtsam-private
// Convert Eigen matrix to a Fusion-compatible numeric buffer with one
// allocation and one pass. The buffer is filled in row-major order.
// In principle, this is a performant way of doing things. 
// This returns a 1D array buffer that is the row-major form of mat.
std::shared_ptr<monty::ndarray<double, 1>> convertToMOSEKArray2D(
    const Matrix& mat) {
  const int rows = static_cast<int>(mat.rows());
  const int cols = static_cast<int>(mat.cols());
  auto buffer = monty::new_array_ptr<double, 1>(monty::shape(rows * cols));

  // We use Eigen's type conversion which should be performant. 
  using RowMajorMat = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
  // Eigen overloads the operator= to perform element-wise assignment into the memory referenced by view.
  Eigen::Map<RowMajorMat> view(buffer->raw(), rows, cols);
  view = mat;

  return buffer;
}

// # copied from gtsam-private
mf::Matrix::t convertToMosekDenseMatrix(const Matrix& mat) {
  return mf::Matrix::dense(static_cast<int>(mat.rows()),
                           static_cast<int>(mat.cols()),
                           convertToMOSEKArray2D(mat));
}

mf::Matrix::t convertToMosekDenseMatrix(const Vector& vec) {
  Matrix mat(vec.size(), 1);
  mat.col(0) = vec;
  return convertToMosekDenseMatrix(mat);
}

// # copied from gtsam-private
Matrix ConvertFromMosekLevelColMajor(
    const std::shared_ptr<monty::ndarray<double, 1>>& level, int rows,
    int cols) {
  using ColMajorMat =
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>;
  Eigen::Map<const ColMajorMat> view(level->raw(), rows, cols);
  return Matrix(view);
}

// # copied from gtsam-private
Matrix ExtractSolvedMatrixBlock(const mf::Variable::t& blockView,
                                DenseIndex expectedDim) {
  const auto level = blockView->level();
  const size_t numel = static_cast<size_t>(level->size(0));
  const size_t expectedSize = static_cast<size_t>(expectedDim);
  if (numel != expectedSize * expectedSize) {
    throw std::runtime_error(
        "ExtractSolvedMatrixBlock: solved block size does not match the QCQP "
        "variable dimension.");
  }

  return ConvertFromMosekLevelColMajor(
      level, static_cast<int>(expectedDim), static_cast<int>(expectedDim));
}

// # copied from gtsam-private
double ComputeBlockRankOneRatio(const Matrix& Xii) {
  Eigen::SelfAdjointEigenSolver<Matrix> solver;
  solver.compute(Xii.template selfadjointView<Eigen::Lower>(),
                 Eigen::EigenvaluesOnly);
  if (solver.info() != Eigen::Success) {
    throw std::runtime_error(
        "ComputeBlockRankOneRatio: eigen decomposition failed.");
  }

  const auto eigs = solver.eigenvalues();
  if (eigs.size() < 2) {
    throw std::runtime_error(
        "ComputeBlockRankOneRatio: Xii block too small for rank-one check.");
  }

  const double lambdaMax = eigs(eigs.size() - 1);
  const double lambdaSecond = eigs(eigs.size() - 2);
  return lambdaMax / lambdaSecond;
}

// # copied from gtsam-private
constexpr double kRecoveredBlockRankOneWarningThreshold = 1e5;

// # copied from gtsam-private
Vector RecoverLiftedVector(const Matrix& Xii) {
  if (Xii.rows() == 0 || Xii.cols() == 0 || std::abs(Xii(0, 0)) < 1e-9) {
    throw std::runtime_error(
        "RecoverLiftedVector: homogenization entry is near zero.");
  }
  return Xii.col(0);
}

// # copied from gtsam-private
void RecoverLiftedVectors(
    const LiftedVariableXijToSDPVariableViewMap& XijMap,
    const KeyVector& orderedKeys,
    const std::map<Key, DenseIndex>& orderedKeyDims,
    std::vector<Vector>& recoveredLiftedVectors,
    std::vector<double>& recoveredVariableEVRs) {
  const size_t variableCount = orderedKeys.size();
  recoveredLiftedVectors.resize(variableCount);
  recoveredVariableEVRs.resize(variableCount);

  for (size_t index = 0; index < variableCount; ++index) {
    const Key key = orderedKeys[index];
    const Matrix Xii = ExtractSolvedMatrixBlock(
        XijMap.at({key, key}), orderedKeyDims.at(key));
    recoveredVariableEVRs[index] = ComputeBlockRankOneRatio(Xii);
    if (recoveredVariableEVRs[index] < kRecoveredBlockRankOneWarningThreshold) {
      std::cerr << "WARNING: recovered lifted block for key "
                << DefaultKeyFormatter(key) << " failed rank-1 check with EVR "
                << recoveredVariableEVRs[index] << std::endl;
    }
    recoveredLiftedVectors[index] = RecoverLiftedVector(Xii);
  }
}

mf::Expression::t BuildQpCostObjectiveTerm(
    const QpCost& cost,
    const LiftedVariableXijToSDPVariableViewMap& xijMap) {
  const HessianFactor& H = cost.hessianFactor();
  if (H.linearTerm().norm() > 0.0 || H.constantTerm() != 0.0) {
    throw std::runtime_error(
        "BuildQpCostObjectiveTerm: linear/constant QpCost terms are not "
        "supported yet.");
  }

  // Assemble the local SDP block matrix X_f in the Hessian factor's key order.
  std::vector<mf::Expression::t> blockRows;
  for (Key key_i : H.keys()) {
    std::vector<mf::Expression::t> rowBlocks;
    for (Key key_j : H.keys()) {
      rowBlocks.push_back(xijMap.at({key_i, key_j})->asExpr());
    }
    blockRows.push_back(
        mf::Expr::hstack(monty::new_array_ptr<mf::Expression::t>(rowBlocks)));
  }

  const auto X_f =
      mf::Expr::vstack(monty::new_array_ptr<mf::Expression::t>(blockRows));
  const Matrix Q_f = H.information();
  return mf::Expr::dot(convertToMosekDenseMatrix(Q_f), X_f);
}

mf::Expression::t BuildObjective(
    const QcqpProblem& problem,
    const LiftedVariableXijToSDPVariableViewMap& xijMap) {
  std::vector<mf::Expression::t> objectiveTerms;

  for (const auto& factor : problem.costs()) {
    if (!factor) {
      continue;
    }

    const auto* cost = dynamic_cast<const QpCost*>(factor.get());
    if (!cost) {
      throw std::runtime_error("BuildObjective: expected QpCost.");
    }

    objectiveTerms.push_back(BuildQpCostObjectiveTerm(*cost, xijMap));
  }

  if (objectiveTerms.empty()) {
    return mf::Expr::constTerm(0.0);
  }
  return mf::Expr::add(monty::new_array_ptr<mf::Expression::t>(objectiveTerms));
}

void AddQuadraticConstraint(
    const mf::Model::t& M, const QuadraticConstraint& constraint,
    const LiftedVariableXijToSDPVariableViewMap& xijMap) {
  const Key key = constraint.key();
  // Lower trace(X_i' A X_i) ~ b to the affine SDP constraint <A, Y_ii> ~ b.
  const auto Xii = xijMap.at({key, key})->asExpr();
  const auto lhs =
      mf::Expr::dot(convertToMosekDenseMatrix(constraint.A()), Xii);

  switch (constraint.sense()) {
    case QuadraticConstraint::Sense::Equal:
      M->constraint(lhs, mf::Domain::equalsTo(constraint.b()));
      break;
    case QuadraticConstraint::Sense::LessEqual:
      M->constraint(lhs, mf::Domain::lessThan(constraint.b()));
      break;
    case QuadraticConstraint::Sense::GreaterEqual:
      M->constraint(lhs, mf::Domain::greaterThan(constraint.b()));
      break;
  }
}

// TODO: This is extremely convoluted and we should change it. 
void AddLinearEqualityConstraint(
    const mf::Model::t& M, const LinearConstraint& constraint,
    const LiftedVariableXijToSDPVariableViewMap& xijMap) {
  const JacobianFactor& J = constraint.factor();
  if (constraint.sense() == LinearConstraint::Sense::Equal && J.size() == 1) {
    auto it = J.begin();
    const Key key = *it;
    const auto Xii = xijMap.at({key, key});
    const DenseIndex dim = J.getDim(it);

    auto first = monty::new_array_ptr<int, 1>({0, 0});
    auto last =
        monty::new_array_ptr<int, 1>({static_cast<int>(dim), 1});
    const auto xi = Xii->slice(first, last)->asExpr();
    const Matrix A = J.getA(it);
    const Vector b = J.getb();
    const auto lhs = mf::Expr::mul(convertToMosekDenseMatrix(A), xi);
    M->constraint(lhs, mf::Domain::equalsTo(convertToMosekDenseMatrix(b)));
  } else {
    throw std::runtime_error(
        "MonolithicSDP: only unary linear equality QCQP constraints are "
        "supported.");
  }
}

// The current D=1 QCQP uses one homogenization entry per key. These equalities
// keep all lifted copies of the fixed value one in the same Gram direction.
void AddHomogenizationConsistencyConstraints(
    const mf::Model::t& M, const KeyVector& orderedKeys,
    const LiftedVariableXijToSDPVariableViewMap& xijMap) {
  if (orderedKeys.empty()) {
    return;
  }

  const Key referenceKey = orderedKeys.front();
  for (size_t index = 1; index < orderedKeys.size(); ++index) {
    const Key key = orderedKeys[index];
    M->constraint(xijMap.at({referenceKey, key})->index(0, 0),
                  mf::Domain::equalsTo(1.0));
  }
}

}  // namespace

struct LiftedSDPProblem<MonolithicSDP, MosekSDPSolver>::Impl {
  mf::Model::t M;
  MosekSolveSummary lastSolveSummary;
  KeyVector orderedKeys;
  std::map<Key, DenseIndex> orderedKeyDims;
  std::map<Key, std::pair<DenseIndex, DenseIndex>> orderedKeyToYSlice;
  DenseIndex totalMonolithicDimension;
  LiftedVariableXijToSDPVariableViewMap liftedVariableXijToSDPVariableViewMap;
  std::vector<Vector> recoveredLiftedVectors;
  std::vector<double> recoveredVariableEVRs;

  ~Impl() {
    recoveredLiftedVectors.clear();
    recoveredVariableEVRs.clear();
    liftedVariableXijToSDPVariableViewMap.clear();
    DisposeMosekModel(M);
  }

  void collectOrderedKeysAndDims(const QcqpProblem& problem) {
    KeySet costKeys;
    orderedKeyDims = CollectQpCostKeyDims(problem, &costKeys);

    const KeySet eqKeys = problem.eConstraints().keys();
    const KeySet ineqKeys = problem.iConstraints().keys();

    KeySet constraintKeys = eqKeys;
    constraintKeys.merge(ineqKeys);

    // TODO: Does this always hold? 
    if (costKeys != constraintKeys) {
      throw std::runtime_error(
          "MonolithicSDP: QCQP constraint keys do not match objective cost keys.");
    }

    orderedKeys.assign(costKeys.begin(), costKeys.end());
  }

  void computeMonolithicLayout() {
    DenseIndex cumulativeIndex = 0;
    orderedKeyToYSlice.clear();
    for (Key key : orderedKeys) {
      const DenseIndex start = cumulativeIndex;
      const DenseIndex end = start + orderedKeyDims.at(key);
      orderedKeyToYSlice[key] = {start, end};
      cumulativeIndex = end;
    }
    totalMonolithicDimension = cumulativeIndex;
  }

  // # copied from gtsam-private
  void populateXijMap(const mf::Variable::t& Y) {
    liftedVariableXijToSDPVariableViewMap.clear();

    for (Key key_i : orderedKeys) {
      for (Key key_j : orderedKeys) {
        const auto [i_start, i_end] = orderedKeyToYSlice.at(key_i);
        const auto [j_start, j_end] = orderedKeyToYSlice.at(key_j);

        auto first = monty::new_array_ptr<int, 1>(
            {static_cast<int>(i_start), static_cast<int>(j_start)});
        auto last = monty::new_array_ptr<int, 1>(
            {static_cast<int>(i_end), static_cast<int>(j_end)});

        liftedVariableXijToSDPVariableViewMap.emplace(
            std::make_pair(key_i, key_j), Y->slice(first, last));
      }
    }
  }
};

LiftedSDPProblem<MonolithicSDP, MosekSDPSolver>::LiftedSDPProblem(
  const QcqpProblem& problem): impl_(std::make_unique<Impl>()) {

  impl_->collectOrderedKeysAndDims(problem);
  impl_->computeMonolithicLayout();

  // # copied from gtsam-private
  impl_->M = new mf::Model("MonolithicSDP_MosekSDPSolver");
  auto Y = impl_->M->variable(
      "Y",
      mf::Domain::inPSDCone(
          static_cast<int>(impl_->totalMonolithicDimension)));
  impl_->populateXijMap(Y);

  // QN: Should this be moved to the QCQP
  AddHomogenizationConsistencyConstraints(
      impl_->M, impl_->orderedKeys,
      impl_->liftedVariableXijToSDPVariableViewMap);

  const auto objective = BuildObjective(problem, impl_->liftedVariableXijToSDPVariableViewMap);
  impl_->M->objective(mf::ObjectiveSense::Minimize,
                      mf::Expr::mul(0.5, objective));
  
  // Process QCQP equality constraints: 
  for (const auto& factor : problem.eConstraints()) {
    if (!factor) {
      continue;
    }

    const auto* quadratic =
        dynamic_cast<const QuadraticEqualityConstraintFactor*>(factor.get());
    if (quadratic) {
      AddQuadraticConstraint(impl_->M, quadratic->quadraticConstraint(),
                             impl_->liftedVariableXijToSDPVariableViewMap);
      continue;
    }

    const auto* linear =
        dynamic_cast<const LinearEqualityConstraintFactor*>(factor.get());
    if (linear) {
      AddLinearEqualityConstraint(impl_->M, linear->linearConstraint(),
                                  impl_->liftedVariableXijToSDPVariableViewMap);
      continue;
    }

    throw std::runtime_error(
        "MonolithicSDP: expected quadratic or linear equality constraints.");
  }

  // Process QCQP Inequality constraints: 
  for (const auto& factor : problem.iConstraints()) {
    if (!factor) {
      continue;
    }

    const auto* quadratic =
        dynamic_cast<const QuadraticInequalityConstraintFactor*>(factor.get());
    if (quadratic) {
      AddQuadraticConstraint(impl_->M, quadratic->quadraticConstraint(),
                             impl_->liftedVariableXijToSDPVariableViewMap);
      continue;
    }

    if (dynamic_cast<const LinearInequalityConstraintFactor*>(factor.get())) {
      throw std::runtime_error(
          "MonolithicSDP: linear inequality constraints are not supported.");
    }

    throw std::runtime_error(
        "MonolithicSDP: expected quadratic inequality constraints.");
  }
}

LiftedSDPProblem<MonolithicSDP, MosekSDPSolver>::~LiftedSDPProblem() = default;

// # copied from gtsam-private
bool LiftedSDPProblem<MonolithicSDP, MosekSDPSolver>::solve(
    const std::map<std::string, double>& mosek_params) {
  impl_->lastSolveSummary = SolveMosekModel(impl_->M, mosek_params);
  return impl_->lastSolveSummary.solved;
}

// # copied from gtsam-private
double LiftedSDPProblem<MonolithicSDP, MosekSDPSolver>::objectiveValue() const {
  impl_->M->acceptedSolutionStatus(mf::AccSolutionStatus::Anything);
  return impl_->M->primalObjValue();
}

// # copied from gtsam-private
std::string LiftedSDPProblem<MonolithicSDP, MosekSDPSolver>::problemStatus()
    const {
  if (!impl_->lastSolveSummary.solved) {
    throw std::runtime_error("problemStatus: solve() has not been called.");
  }
  std::ostringstream out;
  out << impl_->lastSolveSummary.problemStatus;
  return out.str();
}

// # copied from gtsam-private
double LiftedSDPProblem<MonolithicSDP, MosekSDPSolver>::solveTimeSeconds()
    const {
  if (!impl_->lastSolveSummary.solved) {
    throw std::runtime_error("solveTimeSeconds: solve() has not been called.");
  }
  return impl_->lastSolveSummary.optimizerTimeSeconds;
}

void LiftedSDPProblem<MonolithicSDP, MosekSDPSolver>::recoverLiftedVectors() {
  impl_->M->acceptedSolutionStatus(mf::AccSolutionStatus::Anything);
  RecoverLiftedVectors(impl_->liftedVariableXijToSDPVariableViewMap,
                       impl_->orderedKeys, impl_->orderedKeyDims,
                       impl_->recoveredLiftedVectors,
                       impl_->recoveredVariableEVRs);
}

const std::vector<Vector>& LiftedSDPProblem<MonolithicSDP, MosekSDPSolver>::
    getRecoveredLiftedVectors() const {
  if (impl_->recoveredLiftedVectors.empty()) {
    throw std::runtime_error(
        "getRecoveredLiftedVectors: recoverLiftedVectors() must be called "
        "first.");
  }
  return impl_->recoveredLiftedVectors;
}

const std::vector<double>&
LiftedSDPProblem<MonolithicSDP, MosekSDPSolver>::getRecoveredVariableEVRs()
    const {
  if (impl_->recoveredVariableEVRs.empty()) {
    throw std::runtime_error(
        "getRecoveredVariableEVRs: recoverLiftedVectors() must be called "
        "first.");
  }
  return impl_->recoveredVariableEVRs;
}

const KeyVector&
LiftedSDPProblem<MonolithicSDP, MosekSDPSolver>::orderedKeys() const {
  return impl_->orderedKeys;
}

const std::map<Key, DenseIndex>&
LiftedSDPProblem<MonolithicSDP, MosekSDPSolver>::orderedKeyDims() const {
  return impl_->orderedKeyDims;
}
#endif

}  // namespace gtsam

/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    LiftedSDPProblem.cpp
 * @brief   Implementations of QCQP-backed lifted SDP formulations.
 * @author  Avinash Subramanian
 */

#include <gtsam/certifiable/LiftedSDPProblem.h>
#include <gtsam/certifiable/SDPNullspaceReduction.h>
#include <gtsam/symbolic/SymbolicFactorGraph.h>

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <set>
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

// Keep MOSEK-specific helpers private to this translation unit.
namespace {

// A view in original QCQP coordinates, optionally backed by a smaller cone.
struct SDPBlockView {
  mf::Variable::t variable;
  mf::Expression::t expression;
  Matrix rowBasis, columnBasis;

  explicit SDPBlockView(const mf::Variable::t& view)
      : variable(view), expression(view->asExpr()) {}
  SDPBlockView(const mf::Variable::t& cone, const Matrix& rows,
               const Matrix& columns);
  mf::Expression::t asExpr() const { return expression; }
  mf::Expression::t index(int row, int col) const {
    return expression->index(monty::new_array_ptr<int, 1>({row, col}));
  }
  Matrix matrix() const;
};

// Maps each pair of QCQP keys to its original-coordinate SDP block.
using LiftedVariableXijToSDPVariableViewMap =
    std::map<std::pair<Key, Key>, std::shared_ptr<SDPBlockView>>;

// Stores the solver information exposed by the public result accessors.
struct MosekSolveSummary {
  bool solved = false;
  mf::ProblemStatus problemStatus;
  double optimizerTimeSeconds;
};

// Return the accuracy settings used unless explicitly overridden by the caller.
std::map<std::string, double> DefaultMosekParams() {
  return {
      {"intpntCoTolRelGap", 1e-10},
      {"intpntCoTolDfeas", 1e-10},
      {"intpntCoTolPfeas", 1e-10},
      {"intpntCoTolInfeas", 1e-10},
  };
}

// Overlay caller-supplied solver parameters on the defaults.
std::map<std::string, double> MergeMosekParams(
    const std::map<std::string, double>& overrides) {
  auto merged = DefaultMosekParams();
  for (const auto& kv : overrides) {
    merged[kv.first] = kv.second;
  }
  return merged;
}

// Configure and solve a MOSEK model, retaining the public summary fields.
MosekSolveSummary SolveMosekModel(
    const mf::Model::t& M, const std::map<std::string, double>& mosekParams) {
  const auto mergedParams = MergeMosekParams(mosekParams);
  for (const auto& kv : mergedParams) {
    M->setSolverParam(kv.first, kv.second);
  }

  // Opt-in diagnostics preserve the model and solver parameters. Use a unique
  // prefix per solve; the task, iteration log, and raw solution stay together.
  const char* diagnosticPrefix = std::getenv("GTSAM_MOSEK_DIAGNOSTICS");
  std::shared_ptr<std::ofstream> diagnosticLog;
  if (diagnosticPrefix && *diagnosticPrefix) {
    diagnosticLog = std::make_shared<std::ofstream>(
        std::string(diagnosticPrefix) + ".log");
    if (!*diagnosticLog) {
      throw std::runtime_error("Cannot open MOSEK diagnostic log.");
    }
    M->setLogHandler([diagnosticLog](const std::string& message) {
      *diagnosticLog << message;
    });
    M->writeTask(std::string(diagnosticPrefix) + ".task.gz");
  }

  MosekSolveSummary summary;
  M->solve();
  summary.problemStatus = M->getProblemStatus();
  summary.optimizerTimeSeconds = M->getSolverDoubleInfo("optimizerTime");
  summary.solved = true;

  if (diagnosticLog) {
    *diagnosticLog << "\nGTSAM optimize response: "
                   << M->getSolverIntInfo("optimizeResponse") << '\n';
    const auto task = M->getTask();
    if (MSK_analyzesolution(task, MSK_STREAM_LOG, MSK_SOL_ITR) != MSK_RES_OK ||
        MSK_writejsonsol(task, (std::string(diagnosticPrefix) + ".solution.json")
                                  .c_str()) != MSK_RES_OK) {
      throw std::runtime_error("Cannot export MOSEK solution diagnostics.");
    }
    diagnosticLog->flush();
    M->setLogHandler(nullptr);
  }

  return summary;
}

// Collect the dimension of every key appearing in a quadratic cost.
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

// Establish a deterministic SDP block order and validate the QCQP key sets.
void CollectOrderedKeysAndDims(const QcqpProblem& problem,
                               KeyVector* orderedKeys,
                               std::map<Key, DenseIndex>* orderedKeyDims) {
  KeySet costKeys;
  *orderedKeyDims = CollectQpCostKeyDims(problem, &costKeys);

  const KeySet eqKeys = problem.eConstraints().keys();
  const KeySet ineqKeys = problem.iConstraints().keys();

  KeySet constraintKeys = eqKeys;
  constraintKeys.merge(ineqKeys);

  if (costKeys != constraintKeys) {
    throw std::runtime_error(
        "LiftedSDPProblem: QCQP constraint keys do not match objective cost "
        "keys.");
  }

  orderedKeys->assign(costKeys.begin(), costKeys.end());
}

// Build the symbolic sparsity graph induced by the QCQP objective factors.
SymbolicFactorGraph BuildQpCostSymbolicFactorGraph(const QcqpProblem& problem) {
  SymbolicFactorGraph sfg;

  for (const auto& factor : problem.costs()) {
    if (!factor) {
      continue;
    }

    const auto* cost = dynamic_cast<const QpCost*>(factor.get());
    if (!cost) {
      throw std::runtime_error(
          "BuildQpCostSymbolicFactorGraph: expected QpCost.");
    }

    sfg.push_back(SymbolicFactor(*cost));
  }

  if (sfg.empty()) {
    throw std::runtime_error(
        "BuildQpCostSymbolicFactorGraph: QCQP has no objective costs.");
  }

  return sfg;
}

// Eliminate the objective sparsity graph using the requested ordering.
SymbolicBayesTree BuildSymbolicBayesTree(const QcqpProblem& problem,
                                         ChordalOrderingType orderingType) {
  const SymbolicFactorGraph sfg = BuildQpCostSymbolicFactorGraph(problem);
  Ordering ordering;

  switch (orderingType) {
    case ChordalOrderingType::Metis:
#ifdef GTSAM_SUPPORT_NESTED_DISSECTION
      ordering = Ordering::Metis(sfg);
      break;
#else
      throw std::runtime_error(
          "BuildSymbolicBayesTree: METIS ordering requested but GTSAM was "
          "built without nested dissection support.");
#endif
    case ChordalOrderingType::Colamd:
      ordering = Ordering::Colamd(sfg);
      break;
  }

  auto bayesTree = sfg.eliminateMultifrontal(ordering);
  if (!bayesTree) {
    throw std::runtime_error(
        "BuildSymbolicBayesTree: symbolic elimination returned null.");
  }
  return *bayesTree;
}

// Release the native resources held by a Fusion model.
void DisposeMosekModel(const mf::Model::t& M) {
  if (M.get() != nullptr) {
    M->dispose();
  }
}

// Copy an Eigen matrix into the row-major buffer expected by Fusion.
std::shared_ptr<monty::ndarray<double, 1>> convertToMOSEKArray2D(
    const Matrix& mat) {
  const int rows = static_cast<int>(mat.rows());
  const int cols = static_cast<int>(mat.cols());
  auto buffer = monty::new_array_ptr<double, 1>(monty::shape(rows * cols));

  using RowMajorMat =
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
  Eigen::Map<RowMajorMat> view(buffer->raw(), rows, cols);
  view = mat;

  return buffer;
}

// Wrap an Eigen matrix as a dense Fusion matrix.
mf::Matrix::t convertToMosekDenseMatrix(const Matrix& mat) {
  return mf::Matrix::dense(static_cast<int>(mat.rows()),
                           static_cast<int>(mat.cols()),
                           convertToMOSEKArray2D(mat));
}

// Wrap an Eigen vector as a single-column dense Fusion matrix.
mf::Matrix::t convertToMosekDenseMatrix(const Vector& vec) {
  Matrix mat(vec.size(), 1);
  mat.col(0) = vec;
  return convertToMosekDenseMatrix(mat);
}

// Copy a column-major Fusion result buffer into an Eigen matrix.
Matrix ConvertFromMosekLevelColMajor(
    const std::shared_ptr<monty::ndarray<double, 1>>& level, int rows,
    int cols) {
  using ColMajorMat =
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>;
  Eigen::Map<const ColMajorMat> view(level->raw(), rows, cols);
  return Matrix(view);
}

// Express original QCQP rows through the post-processing substitution basis.
SDPBlockView::SDPBlockView(const mf::Variable::t& cone, const Matrix& rows,
                         const Matrix& columns)
    : variable(cone), rowBasis(rows), columnBasis(columns) {
  expression = mf::Expr::mul(
      mf::Expr::mul(convertToMosekDenseMatrix(rows), cone->asExpr()),
      convertToMosekDenseMatrix(Matrix(columns.transpose())));
}

// Lift the solved reduced cone back before running the existing recovery code.
Matrix SDPBlockView::matrix() const {
  Matrix result = ConvertFromMosekLevelColMajor(
      variable->level(), variable->getDim(0), variable->getDim(1));
  if (rowBasis.size()) result = rowBasis * result * columnBasis.transpose();
  return result;
}

// Extract and validate a square block in the original QCQP coordinates.
Matrix ExtractSolvedMatrixBlock(const std::shared_ptr<SDPBlockView>& blockView,
                                DenseIndex expectedDim) {
  const Matrix result = blockView->matrix();
  if (result.rows() != expectedDim || result.cols() != expectedDim) {
    throw std::runtime_error(
        "ExtractSolvedMatrixBlock: solved block size does not match the QCQP "
        "variable dimension.");
  }

  return result;
}

// Compute the dominant-to-second eigenvalue ratio used as a rank-one metric.
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

// Recover the D=1 QCQP vector represented by the first Gram column.
Vector RecoverQcqpVector(const Matrix& Xii) {
  if (Xii.rows() == 0 || Xii.cols() == 0 || std::abs(Xii(0, 0)) < 1e-9) {
    throw std::runtime_error(
        "RecoverQcqpVector: homogenization entry is near zero.");
  }
  return Xii.col(0);
}

// Recover one D=1 QCQP vector from every diagonal SDP block.
Values RecoverQcqpValues(const LiftedVariableXijToSDPVariableViewMap& XijMap,
                         const KeyVector& orderedKeys,
                         const std::map<Key, DenseIndex>& orderedKeyDims) {
  Values recoveredQcqpValues;
  for (Key key : orderedKeys) {
    const Matrix Xii =
        ExtractSolvedMatrixBlock(XijMap.at({key, key}), orderedKeyDims.at(key));
    recoveredQcqpValues.insert(key, Matrix(RecoverQcqpVector(Xii)));
  }
  return recoveredQcqpValues;
}

// Compute one rank-one eigenvalue ratio per diagonal SDP block.
std::vector<double> ComputeVariableEVRs(
    const LiftedVariableXijToSDPVariableViewMap& XijMap,
    const KeyVector& orderedKeys,
    const std::map<Key, DenseIndex>& orderedKeyDims) {
  std::vector<double> variableEVRs;
  variableEVRs.reserve(orderedKeys.size());
  for (Key key : orderedKeys) {
    const Matrix Xii =
        ExtractSolvedMatrixBlock(XijMap.at({key, key}), orderedKeyDims.at(key));
    variableEVRs.push_back(ComputeBlockRankOneRatio(Xii));
  }
  return variableEVRs;
}

// Form one lifted objective term in the Hessian factor's local key order.
mf::Expression::t BuildQpCostObjectiveTerm(
    const QpCost& cost, const LiftedVariableXijToSDPVariableViewMap& xijMap) {
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

// Sum the lifted objective terms contributed by all QCQP costs.
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

// Lower a unary quadratic QCQP constraint to an affine SDP constraint.
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

// Lower a supported unary linear equality to its first-column SDP constraint.
void AddLinearEqualityConstraint(
    const mf::Model::t& M, const LinearConstraint& constraint,
    const LiftedVariableXijToSDPVariableViewMap& xijMap) {
  const JacobianFactor& J = constraint.factor();
  if (constraint.sense() == LinearConstraint::Sense::Equal && J.size() == 1) {
    auto it = J.begin();
    const Key key = *it;
    const auto Xii = xijMap.at({key, key});
    const DenseIndex dim = J.getDim(it);
    const Matrix A = J.getA(it);
    const Vector b = J.getb();

    // Ax=b implies A*x*x'=b*x', hence A*X=b*x' after lifting X=x*x'.
    // Since x is the first column of X and x(0)=1, x'=X(0,:).
    // Enforcing only A*X(:,0)=b leaves unconstrained PSD slack in X.
    const auto xTranspose =
        Xii->asExpr()->slice(monty::new_array_ptr<int, 1>({0, 0}),
                            monty::new_array_ptr<int, 1>({1, static_cast<int>(dim)}));
    const auto lhs = mf::Expr::mul(convertToMosekDenseMatrix(A), Xii->asExpr());
    const auto rhs = mf::Expr::mul(convertToMosekDenseMatrix(b), xTranspose);
    M->constraint(mf::Expr::sub(lhs, rhs), mf::Domain::equalsTo(0.0));
  } else {
    throw std::runtime_error(
        "MonolithicSDP: only unary linear equality QCQP constraints are "
        "supported.");
  }
}

// Tie each monolithic block's homogenization entry to the reference block.
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

// Tie homogenization entries shared by adjacent chordal blocks.
void AddChordalHomogenizationConsistencyConstraints(
    const mf::Model::t& M, const QcqpProblem& problem,
    const LiftedVariableXijToSDPVariableViewMap& xijMap) {
  std::set<std::pair<Key, Key>> constrainedPairs;
  for (const auto& factor : problem.costs()) {
    if (!factor) {
      continue;
    }

    const auto* cost = dynamic_cast<const QpCost*>(factor.get());
    if (!cost) {
      throw std::runtime_error(
          "AddChordalHomogenizationConsistencyConstraints: expected QpCost.");
    }

    const KeyVector& keys = cost->keys();
    for (size_t i = 0; i < keys.size(); ++i) {
      for (size_t j = i + 1; j < keys.size(); ++j) {
        const std::pair<Key, Key> keyPair = std::minmax(keys[i], keys[j]);
        if (constrainedPairs.insert(keyPair).second) {
          M->constraint(xijMap.at(keyPair)->index(0, 0),
                        mf::Domain::equalsTo(1.0));
        }
      }
    }
  }
}

// Add all supported equality and inequality constraints to a Fusion model.
void AddQcqpConstraints(const mf::Model::t& M, const QcqpProblem& problem,
                        const LiftedVariableXijToSDPVariableViewMap& xijMap) {
  // Equality factors may be quadratic or linear.
  for (const auto& factor : problem.eConstraints()) {
    if (!factor) {
      continue;
    }

    const auto* quadratic =
        dynamic_cast<const QuadraticEqualityConstraintFactor*>(factor.get());
    if (quadratic) {
      AddQuadraticConstraint(M, quadratic->quadraticConstraint(), xijMap);
      continue;
    }

    const auto* linear =
        dynamic_cast<const LinearEqualityConstraintFactor*>(factor.get());
    if (linear) {
      AddLinearEqualityConstraint(M, linear->linearConstraint(), xijMap);
      continue;
    }

    throw std::runtime_error(
        "LiftedSDPProblem: expected quadratic or linear equality "
        "constraints.");
  }

  // Inequality factors currently support only quadratic constraints.
  for (const auto& factor : problem.iConstraints()) {
    if (!factor) {
      continue;
    }

    const auto* quadratic =
        dynamic_cast<const QuadraticInequalityConstraintFactor*>(factor.get());
    if (quadratic) {
      AddQuadraticConstraint(M, quadratic->quadraticConstraint(), xijMap);
      continue;
    }

    if (dynamic_cast<const LinearInequalityConstraintFactor*>(factor.get())) {
      throw std::runtime_error(
          "LiftedSDPProblem: linear inequality constraints are not "
          "supported.");
    }

    throw std::runtime_error(
        "LiftedSDPProblem: expected quadratic inequality constraints.");
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

  ~Impl() {
    liftedVariableXijToSDPVariableViewMap.clear();
    DisposeMosekModel(M);
  }

  // Compute the row and column range occupied by every key in the PSD matrix.
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

  // Cache a Fusion view for every block of the monolithic PSD matrix.
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
            std::make_pair(key_i, key_j),
            std::make_shared<SDPBlockView>(Y->slice(first, last)));
      }
    }
  }
};

struct LiftedSDPProblem<ChordalSDP, MosekSDPSolver>::Impl {
  mf::Model::t M;
  MosekSolveSummary lastSolveSummary;
  KeyVector orderedKeys;
  std::map<Key, DenseIndex> orderedKeyDims;
  SymbolicBayesTree bayesTree_;
  std::map<KeyVector, Matrix> cliqueBases;
  LiftedVariableXijToSDPVariableViewMap liftedVariableXijToSDPVariableViewMap;

  ~Impl() {
    liftedVariableXijToSDPVariableViewMap.clear();
    DisposeMosekModel(M);
  }

  // Build a stable, key-derived name for a clique's PSD variable.
  static std::string makeCliqueVariableName(const KeyVector& keys) {
    std::ostringstream out;
    out << "Y_C";
    for (Key key : keys) {
      out << "_" << key;
    }
    return out.str();
  }

  // Constrain duplicate clique views to agree on their shared entries.
  void addChordalOverlapEquality(const std::pair<Key, Key>& key,
                                 const std::shared_ptr<SDPBlockView>& owner,
                                 const std::shared_ptr<SDPBlockView>& duplicate) {
    if (key.first < key.second) {
      M->constraint(mf::Expr::sub(owner->asExpr(), duplicate->asExpr()),
                    mf::Domain::equalsTo(0.0));
      return;
    }

    if (key.first == key.second) {
      const DenseIndex dim = orderedKeyDims.at(key.first);
      for (DenseIndex r = 0; r < dim; ++r) {
        for (DenseIndex c = 0; c <= r; ++c) {
          M->constraint(
              mf::Expr::sub(
                  owner->index(static_cast<int>(r), static_cast<int>(c)),
                  duplicate->index(static_cast<int>(r), static_cast<int>(c))),
              mf::Domain::equalsTo(0.0));
        }
      }
      return;
    }
  }

  // Collect completed layouts for the separate numerical post-processing pass.
  static void collectCliques(const SymbolicBayesTree::sharedClique& clique,
                            std::vector<KeyVector>* layouts) {
    KeyVector keys = clique->conditional()->keys();
    std::sort(keys.begin(), keys.end());
    layouts->push_back(keys);
    for (const auto& child : clique->children) collectCliques(child, layouts);
  }

  // Register one original-coordinate block, regardless of its backing cone size.
  void registerBlock(const mf::Variable::t& cone, const Matrix* basis,
                     const std::pair<Key, Key>& key,
                     DenseIndex rowStart, DenseIndex rowEnd,
                     DenseIndex colStart, DenseIndex colEnd) {
    std::shared_ptr<SDPBlockView> view;
    if (basis) {
      view = std::make_shared<SDPBlockView>(
          cone, basis->middleRows(rowStart, rowEnd - rowStart),
          basis->middleRows(colStart, colEnd - colStart));
    } else {
      view = std::make_shared<SDPBlockView>(cone->slice(
          monty::new_array_ptr<int, 1>({int(rowStart), int(colStart)}),
          monty::new_array_ptr<int, 1>({int(rowEnd), int(colEnd)})));
    }
    auto [owner, inserted] = liftedVariableXijToSDPVariableViewMap.emplace(key, view);
    if (!inserted) addChordalOverlapEquality(key, owner->second, view);
  }

  // Allocate clique variables and register their block views recursively.
  void populateXijMapRecursive(const SymbolicBayesTree::sharedClique& clique) {
    if (!clique) {
      return;
    }

    KeyVector indices = clique->conditional()->keys();
    std::sort(indices.begin(), indices.end());

    DenseIndex cliqueDimension = 0;
    std::map<Key, std::pair<DenseIndex, DenseIndex>> keyToCliqueSlice;
    for (Key key : indices) {
      const DenseIndex start = cliqueDimension;
      const DenseIndex end = start + orderedKeyDims.at(key);
      keyToCliqueSlice[key] = {start, end};
      cliqueDimension = end;
    }

    if (!indices.empty()) {
      const auto found = cliqueBases.find(indices);
      const Matrix* basis = found == cliqueBases.end() ? nullptr : &found->second;
      const DenseIndex coneDimension = basis ? basis->cols() : cliqueDimension;
      auto cliqueY =
          M->variable(makeCliqueVariableName(indices),
                      mf::Domain::inPSDCone(static_cast<int>(coneDimension)));

      for (Key key_i : indices) {
        for (Key key_j : indices) {
          const auto [i_start, i_end] = keyToCliqueSlice.at(key_i);
          const auto [j_start, j_end] = keyToCliqueSlice.at(key_j);
          registerBlock(cliqueY, basis, {key_i, key_j},
                        i_start, i_end, j_start, j_end);
        }
      }
    }

    for (const auto& childClique : clique->children) {
      populateXijMapRecursive(childClique);
    }
  }

  // Populate block views for every root of the symbolic Bayes tree.
  void populateXijMap() {
    for (const auto& rootClique : bayesTree_.roots()) {
      populateXijMapRecursive(rootClique);
    }
  }
};

LiftedSDPProblem<MonolithicSDP, MosekSDPSolver>::LiftedSDPProblem(
    const QcqpProblem& problem)
    : impl_(std::make_unique<Impl>()) {
  CollectOrderedKeysAndDims(problem, &impl_->orderedKeys,
                            &impl_->orderedKeyDims);
  impl_->computeMonolithicLayout();

  // Represent the complete lifted matrix with one positive semidefinite cone.
  impl_->M = new mf::Model("MonolithicSDP_MosekSDPSolver");
  auto Y = impl_->M->variable(
      "Y",
      mf::Domain::inPSDCone(static_cast<int>(impl_->totalMonolithicDimension)));
  impl_->populateXijMap(Y);

  AddHomogenizationConsistencyConstraints(
      impl_->M, impl_->orderedKeys,
      impl_->liftedVariableXijToSDPVariableViewMap);

  const auto objective =
      BuildObjective(problem, impl_->liftedVariableXijToSDPVariableViewMap);
  impl_->M->objective(mf::ObjectiveSense::Minimize,
                      mf::Expr::mul(0.5, objective));

  AddQcqpConstraints(impl_->M, problem,
                     impl_->liftedVariableXijToSDPVariableViewMap);
}

LiftedSDPProblem<MonolithicSDP, MosekSDPSolver>::~LiftedSDPProblem() = default;

bool LiftedSDPProblem<MonolithicSDP, MosekSDPSolver>::solve(
    const std::map<std::string, double>& mosekParams) {
  impl_->lastSolveSummary = SolveMosekModel(impl_->M, mosekParams);
  return impl_->lastSolveSummary.solved;
}

double LiftedSDPProblem<MonolithicSDP, MosekSDPSolver>::objectiveValue() const {
  impl_->M->acceptedSolutionStatus(mf::AccSolutionStatus::Anything);
  return impl_->M->primalObjValue();
}

std::string LiftedSDPProblem<MonolithicSDP, MosekSDPSolver>::problemStatus()
    const {
  if (!impl_->lastSolveSummary.solved) {
    throw std::runtime_error("problemStatus: solve() has not been called.");
  }
  std::ostringstream out;
  out << impl_->lastSolveSummary.problemStatus;
  return out.str();
}

double LiftedSDPProblem<MonolithicSDP, MosekSDPSolver>::solveTimeSeconds()
    const {
  if (!impl_->lastSolveSummary.solved) {
    throw std::runtime_error("solveTimeSeconds: solve() has not been called.");
  }
  return impl_->lastSolveSummary.optimizerTimeSeconds;
}

Values LiftedSDPProblem<MonolithicSDP, MosekSDPSolver>::qcqpValues() const {
  if (!impl_->lastSolveSummary.solved) {
    throw std::runtime_error("qcqpValues: solve() has not been called.");
  }
  impl_->M->acceptedSolutionStatus(mf::AccSolutionStatus::Anything);
  return RecoverQcqpValues(impl_->liftedVariableXijToSDPVariableViewMap,
                           impl_->orderedKeys, impl_->orderedKeyDims);
}

std::vector<double>
LiftedSDPProblem<MonolithicSDP, MosekSDPSolver>::variableEVRs() const {
  if (!impl_->lastSolveSummary.solved) {
    throw std::runtime_error("variableEVRs: solve() has not been called.");
  }
  impl_->M->acceptedSolutionStatus(mf::AccSolutionStatus::Anything);
  return ComputeVariableEVRs(impl_->liftedVariableXijToSDPVariableViewMap,
                             impl_->orderedKeys, impl_->orderedKeyDims);
}

const KeyVector& LiftedSDPProblem<MonolithicSDP, MosekSDPSolver>::orderedKeys()
    const {
  return impl_->orderedKeys;
}

const std::map<Key, DenseIndex>&
LiftedSDPProblem<MonolithicSDP, MosekSDPSolver>::orderedKeyDims() const {
  return impl_->orderedKeyDims;
}

LiftedSDPProblem<ChordalSDP, MosekSDPSolver>::LiftedSDPProblem(
    const QcqpProblem& problem, ChordalOrderingType orderingType,
    bool eliminateKnownNullDirections)
    : impl_(std::make_unique<Impl>()) {
  CollectOrderedKeysAndDims(problem, &impl_->orderedKeys,
                            &impl_->orderedKeyDims);
  impl_->M = new mf::Model("ChordalSDP_MosekSDPSolver");
  impl_->bayesTree_ = BuildSymbolicBayesTree(problem, orderingType);

  // POST-PROCESSING PASS: reduce the numerical cone coordinates only.
  // The QCQP, clique membership, and original-coordinate constraints are intact.
  if (eliminateKnownNullDirections) {
    std::vector<KeyVector> layouts;
    for (const auto& root : impl_->bayesTree_.roots()) {
      Impl::collectCliques(root, &layouts);
    }
    impl_->cliqueBases = internal::EliminateKnownNullDirections(
        problem, impl_->orderedKeyDims, layouts);
  }

  // Use one positive semidefinite variable per symbolic clique.
  impl_->populateXijMap();

  AddChordalHomogenizationConsistencyConstraints(
      impl_->M, problem, impl_->liftedVariableXijToSDPVariableViewMap);

  const auto objective =
      BuildObjective(problem, impl_->liftedVariableXijToSDPVariableViewMap);
  impl_->M->objective(mf::ObjectiveSense::Minimize,
                      mf::Expr::mul(0.5, objective));

  AddQcqpConstraints(impl_->M, problem,
                     impl_->liftedVariableXijToSDPVariableViewMap);
}

LiftedSDPProblem<ChordalSDP, MosekSDPSolver>::~LiftedSDPProblem() = default;

bool LiftedSDPProblem<ChordalSDP, MosekSDPSolver>::solve(
    const std::map<std::string, double>& mosekParams) {
  impl_->lastSolveSummary = SolveMosekModel(impl_->M, mosekParams);
  return impl_->lastSolveSummary.solved;
}

double LiftedSDPProblem<ChordalSDP, MosekSDPSolver>::objectiveValue() const {
  impl_->M->acceptedSolutionStatus(mf::AccSolutionStatus::Anything);
  return impl_->M->primalObjValue();
}

std::string LiftedSDPProblem<ChordalSDP, MosekSDPSolver>::problemStatus()
    const {
  if (!impl_->lastSolveSummary.solved) {
    throw std::runtime_error("problemStatus: solve() has not been called.");
  }
  std::ostringstream out;
  out << impl_->lastSolveSummary.problemStatus;
  return out.str();
}

double LiftedSDPProblem<ChordalSDP, MosekSDPSolver>::solveTimeSeconds() const {
  if (!impl_->lastSolveSummary.solved) {
    throw std::runtime_error("solveTimeSeconds: solve() has not been called.");
  }
  return impl_->lastSolveSummary.optimizerTimeSeconds;
}

Values LiftedSDPProblem<ChordalSDP, MosekSDPSolver>::qcqpValues() const {
  if (!impl_->lastSolveSummary.solved) {
    throw std::runtime_error("qcqpValues: solve() has not been called.");
  }
  impl_->M->acceptedSolutionStatus(mf::AccSolutionStatus::Anything);
  return RecoverQcqpValues(impl_->liftedVariableXijToSDPVariableViewMap,
                           impl_->orderedKeys, impl_->orderedKeyDims);
}

std::vector<double> LiftedSDPProblem<ChordalSDP, MosekSDPSolver>::variableEVRs()
    const {
  if (!impl_->lastSolveSummary.solved) {
    throw std::runtime_error("variableEVRs: solve() has not been called.");
  }
  impl_->M->acceptedSolutionStatus(mf::AccSolutionStatus::Anything);
  return ComputeVariableEVRs(impl_->liftedVariableXijToSDPVariableViewMap,
                             impl_->orderedKeys, impl_->orderedKeyDims);
}

const KeyVector& LiftedSDPProblem<ChordalSDP, MosekSDPSolver>::orderedKeys()
    const {
  return impl_->orderedKeys;
}

const std::map<Key, DenseIndex>&
LiftedSDPProblem<ChordalSDP, MosekSDPSolver>::orderedKeyDims() const {
  return impl_->orderedKeyDims;
}

const SymbolicBayesTree&
LiftedSDPProblem<ChordalSDP, MosekSDPSolver>::bayesTree() const {
  return impl_->bayesTree_;
}
#endif

}  // namespace gtsam

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
#include <gtsam/symbolic/SymbolicFactorGraph.h>

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
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

// Maps each pair of QCQP keys to its block in an SDP variable.
using LiftedVariableXijToSDPVariableViewMap =
    std::map<std::pair<Key, Key>, mf::Variable::t>;

// Maps each QCQP key to the first-moment row y_k' of its PSD block.
using FirstMomentViewMap = std::map<Key, mf::Variable::t>;

// Assign each key a contiguous range after the constant entry at index 0.
// Every PSD block is the Shor lift [1 y'; y X] of the keys it holds.
std::map<Key, std::pair<int, int>> MakeBlockRanges(
    const KeyVector& keys, const std::map<Key, DenseIndex>& keyDims,
    int* dimension) {
  std::map<Key, std::pair<int, int>> ranges;
  *dimension = 1;
  for (Key key : keys) {
    const int start = *dimension;
    *dimension += static_cast<int>(keyDims.at(key));
    ranges.emplace(key, std::make_pair(start, *dimension));
  }
  return ranges;
}

// Return the view of rows [r0, r1) and columns [c0, c1) of a PSD variable.
mf::Variable::t SliceView(const mf::Variable::t& cone, int r0, int r1, int c0,
                          int c1) {
  return cone->slice(monty::new_array_ptr<int, 1>({r0, c0}),
                     monty::new_array_ptr<int, 1>({r1, c1}));
}

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

  if (!std::includes(costKeys.begin(), costKeys.end(), constraintKeys.begin(),
                     constraintKeys.end())) {
    throw std::runtime_error(
        "LiftedSDPProblem: every constrained QCQP key must appear in an "
        "objective cost.");
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

// Extract and validate a square SDP block from a solved Fusion variable.
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

  return ConvertFromMosekLevelColMajor(level, static_cast<int>(expectedDim),
                                       static_cast<int>(expectedDim));
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

// Read the solved first moments y_k from a 1-by-dim row view.
Vector ExtractSolvedFirstMoment(const mf::Variable::t& rowView,
                                DenseIndex expectedDim) {
  const auto level = rowView->level();
  if (static_cast<DenseIndex>(level->size(0)) != expectedDim) {
    throw std::runtime_error(
        "ExtractSolvedFirstMoment: solved row size does not match the lifted "
        "variable dimension.");
  }
  return Eigen::Map<const Vector>(level->raw(), expectedDim);
}

// Assemble the solved moment block [1 y_k'; y_k X_kk] of one key.
Matrix SolvedMomentBlock(const FirstMomentViewMap& firstMoments,
                         const LiftedVariableXijToSDPVariableViewMap& xijMap,
                         Key key, DenseIndex dim) {
  Matrix block(dim + 1, dim + 1);
  const Vector y = ExtractSolvedFirstMoment(firstMoments.at(key), dim);
  block(0, 0) = 1.0;
  block.block(0, 1, 1, dim) = y.transpose();
  block.block(1, 0, dim, 1) = y;
  block.bottomRightCorner(dim, dim) =
      ExtractSolvedMatrixBlock(xijMap.at({key, key}), dim);
  return block;
}

// Recover one D=1 QCQP vector per key from its first moments.
Values RecoverQcqpValues(const FirstMomentViewMap& firstMoments,
                         const KeyVector& orderedKeys,
                         const std::map<Key, DenseIndex>& keyDims) {
  Values recoveredQcqpValues;
  for (Key key : orderedKeys) {
    const Matrix value =
        ExtractSolvedFirstMoment(firstMoments.at(key), keyDims.at(key));
    recoveredQcqpValues.insert(key, value);
  }
  return recoveredQcqpValues;
}

// Compute one rank-one eigenvalue ratio per key from its moment block.
std::vector<double> ComputeVariableEVRs(
    const FirstMomentViewMap& firstMoments,
    const LiftedVariableXijToSDPVariableViewMap& xijMap,
    const KeyVector& orderedKeys, const std::map<Key, DenseIndex>& keyDims) {
  std::vector<double> variableEVRs;
  variableEVRs.reserve(orderedKeys.size());
  for (Key key : orderedKeys) {
    variableEVRs.push_back(ComputeBlockRankOneRatio(
        SolvedMomentBlock(firstMoments, xijMap, key, keyDims.at(key))));
  }
  return variableEVRs;
}

// Form the lifted objective term 0.5 <G, X_f> - g' y_f + 0.5 f of one cost.
mf::Expression::t BuildQpCostObjectiveTerm(
    const QpCost& cost, const FirstMomentViewMap& firstMoments,
    const LiftedVariableXijToSDPVariableViewMap& xijMap) {
  const HessianFactor& H = cost.hessianFactor();

  // Assemble the local SDP block matrix X_f in the Hessian factor's key order.
  std::vector<mf::Expression::t> blockRows, firstMomentRow;
  for (Key key_i : H.keys()) {
    std::vector<mf::Expression::t> rowBlocks;
    for (Key key_j : H.keys()) {
      rowBlocks.push_back(xijMap.at({key_i, key_j})->asExpr());
    }
    blockRows.push_back(
        mf::Expr::hstack(monty::new_array_ptr<mf::Expression::t>(rowBlocks)));
    firstMomentRow.push_back(firstMoments.at(key_i)->asExpr());
  }

  const auto X_f =
      mf::Expr::vstack(monty::new_array_ptr<mf::Expression::t>(blockRows));
  auto term = mf::Expr::mul(
      0.5, mf::Expr::dot(convertToMosekDenseMatrix(Matrix(H.information())),
                         X_f));
  const Vector g = H.linearTerm();
  if (!g.isZero(0.0)) {
    const auto y_f = mf::Expr::hstack(
        monty::new_array_ptr<mf::Expression::t>(firstMomentRow));
    term = mf::Expr::sub(
        term, mf::Expr::dot(convertToMosekDenseMatrix(Matrix(g.transpose())),
                            y_f));
  }
  if (H.constantTerm() != 0.0) {
    term = mf::Expr::add(term, mf::Expr::constTerm(0.5 * H.constantTerm()));
  }
  return term;
}

// Sum the lifted objective terms contributed by all QCQP costs.
mf::Expression::t BuildObjective(
    const QcqpProblem& problem, const FirstMomentViewMap& firstMoments,
    const LiftedVariableXijToSDPVariableViewMap& xijMap) {
  std::vector<mf::Expression::t> objectiveTerms;
  for (const auto& factor : problem.costs()) {
    if (!factor) continue;
    const auto* cost = dynamic_cast<const QpCost*>(factor.get());
    if (!cost) {
      throw std::runtime_error("BuildObjective: expected QpCost.");
    }
    objectiveTerms.push_back(
        BuildQpCostObjectiveTerm(*cost, firstMoments, xijMap));
  }

  if (objectiveTerms.empty()) {
    return mf::Expr::constTerm(0.0);
  }
  return mf::Expr::add(monty::new_array_ptr<mf::Expression::t>(objectiveTerms));
}

// Lower x'Ax + a'x ~ b to the affine SDP constraint <A, X> + a'y ~ b.
void AddQuadraticConstraint(
    const mf::Model::t& M, const QuadraticConstraint& constraint,
    const FirstMomentViewMap& firstMoments,
    const LiftedVariableXijToSDPVariableViewMap& xijMap) {
  const Key key = constraint.key();
  auto lhs = mf::Expr::dot(convertToMosekDenseMatrix(constraint.A()),
                           xijMap.at({key, key})->asExpr());
  if (constraint.hasLinearTerm()) {
    lhs = mf::Expr::add(
        lhs, mf::Expr::dot(convertToMosekDenseMatrix(
                               Matrix(constraint.a().transpose())),
                           firstMoments.at(key)->asExpr()));
  }

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

// Lower Ax = b on one key to both rows of [-b A] [1 y'; y X] = 0.
void AddLinearEqualityConstraint(
    const mf::Model::t& M, const LinearConstraint& equality,
    const FirstMomentViewMap& firstMoments,
    const LiftedVariableXijToSDPVariableViewMap& xijMap) {
  const JacobianFactor& J = equality.factor();
  if (J.size() != 1) {
    throw std::runtime_error(
        "LiftedSDPProblem: only unary linear equality QCQP constraints are "
        "supported.");
  }
  const Key key = *J.begin();
  const auto y = firstMoments.at(key)->asExpr();
  const auto X = xijMap.at({key, key})->asExpr();
  const auto A = convertToMosekDenseMatrix(Matrix(J.getA(J.begin())));
  const auto b = convertToMosekDenseMatrix(Vector(J.getb()));

  // Ax=b implies A*x*x'=b*x', hence A*X=b*y' after lifting. Enforcing only the
  // first moments Ay=b would leave unconstrained PSD slack in X.
  M->constraint(mf::Expr::mul(A, mf::Expr::transpose(y)),
                mf::Domain::equalsTo(b));
  M->constraint(mf::Expr::sub(mf::Expr::mul(A, X), mf::Expr::mul(b, y)),
                mf::Domain::equalsTo(0.0));
}

// Add all QCQP equality and inequality constraints to a Fusion model.
void AddQcqpConstraints(const mf::Model::t& M, const QcqpProblem& problem,
                        const FirstMomentViewMap& firstMoments,
                        const LiftedVariableXijToSDPVariableViewMap& xijMap) {
  // Equality factors may be quadratic or linear.
  for (const auto& factor : problem.eConstraints()) {
    if (!factor) continue;
    if (const auto* quadratic =
            dynamic_cast<const QuadraticEqualityConstraintFactor*>(
                factor.get())) {
      AddQuadraticConstraint(M, quadratic->quadraticConstraint(),
                             firstMoments, xijMap);
      continue;
    }
    if (const auto* linear =
            dynamic_cast<const LinearEqualityConstraintFactor*>(factor.get())) {
      AddLinearEqualityConstraint(M, linear->linearConstraint(), firstMoments,
                                  xijMap);
      continue;
    }
    throw std::runtime_error(
        "LiftedSDPProblem: expected quadratic or linear equality "
        "constraints.");
  }

  // Inequality factors currently support only quadratic constraints.
  for (const auto& factor : problem.iConstraints()) {
    if (!factor) continue;
    if (const auto* quadratic =
            dynamic_cast<const QuadraticInequalityConstraintFactor*>(
                factor.get())) {
      AddQuadraticConstraint(M, quadratic->quadraticConstraint(),
                             firstMoments, xijMap);
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

// Reject the removed per-key homogeneous-coordinate formulation.
void RequireSharedHomogeneousCoordinates(bool shareHomogeneousCoordinates) {
  if (!shareHomogeneousCoordinates) {
    throw std::invalid_argument(
        "LiftedSDPProblem: shareHomogeneousCoordinates=false is no longer "
        "supported; every PSD block carries one shared homogeneous "
        "coordinate.");
  }
}

}  // namespace

struct LiftedSDPProblem<MonolithicSDP, MosekSDPSolver>::Impl {
  mf::Model::t M;
  MosekSolveSummary lastSolveSummary;
  KeyVector orderedKeys;
  std::map<Key, DenseIndex> orderedKeyDims;
  FirstMomentViewMap firstMomentViews;
  LiftedVariableXijToSDPVariableViewMap liftedVariableXijToSDPVariableViewMap;

  ~Impl() {
    firstMomentViews.clear();
    liftedVariableXijToSDPVariableViewMap.clear();
    DisposeMosekModel(M);
  }

  // Cache first-moment rows and second-moment blocks of the monolithic matrix.
  void populateXijMap(const mf::Variable::t& Y,
                      const std::map<Key, std::pair<int, int>>& ranges) {
    liftedVariableXijToSDPVariableViewMap.clear();
    firstMomentViews.clear();

    for (Key key_i : orderedKeys) {
      const auto [i_start, i_end] = ranges.at(key_i);
      firstMomentViews.emplace(key_i, SliceView(Y, 0, 1, i_start, i_end));
      for (Key key_j : orderedKeys) {
        const auto [j_start, j_end] = ranges.at(key_j);
        liftedVariableXijToSDPVariableViewMap.emplace(
            std::make_pair(key_i, key_j),
            SliceView(Y, i_start, i_end, j_start, j_end));
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
  FirstMomentViewMap firstMomentViews;
  LiftedVariableXijToSDPVariableViewMap liftedVariableXijToSDPVariableViewMap;

  ~Impl() {
    firstMomentViews.clear();
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
                                 const mf::Variable::t& owner,
                                 const mf::Variable::t& duplicate) {
    if (key.first < key.second) {
      M->constraint(mf::Expr::sub(owner, duplicate), mf::Domain::equalsTo(0.0));
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

  // Allocate clique variables and register their block views recursively.
  void populateXijMapRecursive(const SymbolicBayesTree::sharedClique& clique) {
    if (!clique) {
      return;
    }

    KeyVector indices = clique->conditional()->keys();
    std::sort(indices.begin(), indices.end());

    int cliqueDimension;
    const auto ranges =
        MakeBlockRanges(indices, orderedKeyDims, &cliqueDimension);

    if (!indices.empty()) {
      auto cliqueY =
          M->variable(makeCliqueVariableName(indices),
                      mf::Domain::inPSDCone(static_cast<int>(cliqueDimension)));
      M->constraint(cliqueY->index(0, 0), mf::Domain::equalsTo(1.0));

      for (Key key_i : indices) {
        const auto [i_start, i_end] = ranges.at(key_i);
        auto firstMoment = SliceView(cliqueY, 0, 1, i_start, i_end);
        auto [firstIt, firstInserted] =
            firstMomentViews.emplace(key_i, firstMoment);
        if (!firstInserted) {
          M->constraint(mf::Expr::sub(firstIt->second, firstMoment),
                        mf::Domain::equalsTo(0.0));
        }

        for (Key key_j : indices) {
          const auto [j_start, j_end] = ranges.at(key_j);
          auto blockView = SliceView(cliqueY, i_start, i_end, j_start, j_end);

          const std::pair<Key, Key> key(key_i, key_j);
          auto [it, inserted] =
              liftedVariableXijToSDPVariableViewMap.emplace(key, blockView);
          if (!inserted) {
            addChordalOverlapEquality(key, it->second, blockView);
          }
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
    const QcqpProblem& problem, bool shareHomogeneousCoordinates)
    : impl_(std::make_unique<Impl>()) {
  RequireSharedHomogeneousCoordinates(shareHomogeneousCoordinates);
  CollectOrderedKeysAndDims(problem, &impl_->orderedKeys,
                            &impl_->orderedKeyDims);
  int coneDimension;
  const auto ranges = MakeBlockRanges(impl_->orderedKeys, impl_->orderedKeyDims,
                                      &coneDimension);

  // Represent the complete lifted matrix with one positive semidefinite cone.
  impl_->M = new mf::Model("MonolithicSDP_MosekSDPSolver");
  auto Y = impl_->M->variable("Y", mf::Domain::inPSDCone(coneDimension));
  impl_->populateXijMap(Y, ranges);

  impl_->M->constraint(Y->index(0, 0), mf::Domain::equalsTo(1.0));

  impl_->M->objective(
      mf::ObjectiveSense::Minimize,
      BuildObjective(problem, impl_->firstMomentViews,
                     impl_->liftedVariableXijToSDPVariableViewMap));

  AddQcqpConstraints(impl_->M, problem, impl_->firstMomentViews,
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
  return RecoverQcqpValues(impl_->firstMomentViews, impl_->orderedKeys,
                           impl_->orderedKeyDims);
}

std::vector<double>
LiftedSDPProblem<MonolithicSDP, MosekSDPSolver>::variableEVRs() const {
  if (!impl_->lastSolveSummary.solved) {
    throw std::runtime_error("variableEVRs: solve() has not been called.");
  }
  impl_->M->acceptedSolutionStatus(mf::AccSolutionStatus::Anything);
  return ComputeVariableEVRs(impl_->firstMomentViews,
                             impl_->liftedVariableXijToSDPVariableViewMap,
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
    bool shareHomogeneousCoordinates)
    : impl_(std::make_unique<Impl>()) {
  RequireSharedHomogeneousCoordinates(shareHomogeneousCoordinates);
  CollectOrderedKeysAndDims(problem, &impl_->orderedKeys,
                            &impl_->orderedKeyDims);
  impl_->M = new mf::Model("ChordalSDP_MosekSDPSolver");
  impl_->bayesTree_ = BuildSymbolicBayesTree(problem, orderingType);
  // Use one positive semidefinite variable per symbolic clique.
  impl_->populateXijMap();

  impl_->M->objective(
      mf::ObjectiveSense::Minimize,
      BuildObjective(problem, impl_->firstMomentViews,
                     impl_->liftedVariableXijToSDPVariableViewMap));

  AddQcqpConstraints(impl_->M, problem, impl_->firstMomentViews,
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
  return RecoverQcqpValues(impl_->firstMomentViews, impl_->orderedKeys,
                           impl_->orderedKeyDims);
}

std::vector<double> LiftedSDPProblem<ChordalSDP, MosekSDPSolver>::variableEVRs()
    const {
  if (!impl_->lastSolveSummary.solved) {
    throw std::runtime_error("variableEVRs: solve() has not been called.");
  }
  impl_->M->acceptedSolutionStatus(mf::AccSolutionStatus::Anything);
  return ComputeVariableEVRs(impl_->firstMomentViews,
                             impl_->liftedVariableXijToSDPVariableViewMap,
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

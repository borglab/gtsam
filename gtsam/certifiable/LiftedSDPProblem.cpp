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

// Recognize an exact, possibly scaled h^2=1 equality.
bool isUnitHomogeneousConstraint(const QuadraticConstraint& constraint) {
  const Matrix& A = constraint.A();
  if (!constraint.isEquality() || A.size() == 0 || constraint.b() == 0.0 ||
      !std::isfinite(constraint.b()) || A(0, 0) != constraint.b()) {
    return false;
  }
  for (DenseIndex column = 0; column < A.cols(); ++column) {
    for (DenseIndex row = 0; row < A.rows(); ++row) {
      if ((row != 0 || column != 0) && A(row, column) != 0.0) return false;
    }
  }
  return true;
}

// How one QCQP key is lifted. A key whose leading coordinate h is fixed by an
// exact h^2=1 equality drops that coordinate: every PSD block instead carries a
// single constant entry Y(0,0)=1, the Shor lift [1 y'; y X] of the remaining
// coordinates. Other keys are lifted as plain vectors. Since h=1 in every
// feasible point of the shared-coordinate relaxation, terms in h become linear
// and constant terms of the remaining coordinates.
struct LiftedKey {
  DenseIndex qcqpDim = 0;
  DenseIndex dim = 0;
  bool homogenized = false;
};
using LiftedKeyMap = std::map<Key, LiftedKey>;

// Classify every key by whether its leading coordinate is a unit homogenizer.
LiftedKeyMap MakeLiftedKeys(const QcqpProblem& problem,
                            const std::map<Key, DenseIndex>& qcqpDims) {
  KeySet homogenized;
  for (const auto& factor : problem.eConstraints()) {
    const auto* quadratic =
        dynamic_cast<const QuadraticEqualityConstraintFactor*>(factor.get());
    if (quadratic &&
        isUnitHomogeneousConstraint(quadratic->quadraticConstraint())) {
      homogenized.insert(quadratic->quadraticConstraint().key());
    }
  }

  LiftedKeyMap liftedKeys;
  for (const auto& [key, qcqpDim] : qcqpDims) {
    LiftedKey lifted;
    lifted.qcqpDim = qcqpDim;
    lifted.homogenized = homogenized.count(key) != 0;
    lifted.dim = qcqpDim - (lifted.homogenized ? 1 : 0);
    if (lifted.dim == 0) {
      throw std::runtime_error(
          "LiftedSDPProblem: a homogenized key needs at least one coordinate "
          "besides its homogenization entry.");
    }
    liftedKeys.emplace(key, lifted);
  }
  return liftedKeys;
}

// A QCQP cost 0.5 x'Gx - g'x + 0.5 f in the lifted coordinates of its keys.
struct LiftedCost {
  KeyVector keys;
  Matrix G;
  Vector g;
  double f = 0.0;
};

// A unary QCQP constraint x'Ax + a'x ~ b in the lifted coordinates of its key.
struct LiftedQuadraticConstraint {
  Key key;
  Matrix A;
  Vector a;
  double b = 0.0;
  QuadraticConstraint::Sense sense = QuadraticConstraint::Sense::Equal;
};

// A unary QCQP equality Ax = b in the lifted coordinates of its key.
struct LiftedLinearEquality {
  Key key;
  Matrix A;
  Vector b;
};

// Rewrite a QpCost by substituting h=1 for every homogenized key.
LiftedCost MakeLiftedCost(const QpCost& cost, const LiftedKeyMap& liftedKeys) {
  const HessianFactor& H = cost.hessianFactor();
  const Matrix Q = H.information();
  const Vector linear = H.linearTerm();

  std::vector<DenseIndex> physical, homogenization;
  DenseIndex offset = 0;
  for (Key key : H.keys()) {
    const LiftedKey& lifted = liftedKeys.at(key);
    const DenseIndex first = offset + (lifted.homogenized ? 1 : 0);
    if (lifted.homogenized) homogenization.push_back(offset);
    for (DenseIndex row = first; row < offset + lifted.qcqpDim; ++row) {
      physical.push_back(row);
    }
    offset += lifted.qcqpDim;
  }

  // With error 0.5 x'Qx - x'q + 0.5 c and h=1: the h columns of Q join the
  // linear term, and the h-h entries and h entries of q join the constant.
  LiftedCost lifted;
  lifted.keys = H.keys();
  lifted.G.resize(physical.size(), physical.size());
  lifted.g.resize(physical.size());
  for (size_t row = 0; row < physical.size(); ++row) {
    lifted.g(row) = linear(physical[row]);
    for (size_t column = 0; column < physical.size(); ++column) {
      lifted.G(row, column) = Q(physical[row], physical[column]);
    }
    for (DenseIndex h : homogenization) lifted.g(row) -= Q(physical[row], h);
  }
  lifted.f = H.constantTerm();
  for (DenseIndex h : homogenization) {
    lifted.f -= 2.0 * linear(h);
    for (DenseIndex other : homogenization) lifted.f += Q(h, other);
  }
  return lifted;
}

// Rewrite a quadratic constraint by substituting h=1 for a homogenized key.
LiftedQuadraticConstraint MakeLiftedQuadraticConstraint(
    const QuadraticConstraint& constraint, const LiftedKeyMap& liftedKeys) {
  const LiftedKey& liftedKey = liftedKeys.at(constraint.key());
  const Matrix& A = constraint.A();
  LiftedQuadraticConstraint lifted;
  lifted.key = constraint.key();
  lifted.sense = constraint.sense();
  if (liftedKey.homogenized) {
    const DenseIndex dim = liftedKey.dim;
    lifted.A = A.bottomRightCorner(dim, dim);
    lifted.a = 2.0 * A.block(0, 1, 1, dim).transpose();
    lifted.b = constraint.b() - A(0, 0);
  } else {
    lifted.A = A;
    lifted.a = Vector::Zero(A.rows());
    lifted.b = constraint.b();
  }
  return lifted;
}

// Rewrite a unary linear equality by substituting h=1 for a homogenized key.
// Rows that only involve h, such as the h=1 row of a fixed value, are dropped.
LiftedLinearEquality MakeLiftedLinearEquality(
    const LinearConstraint& constraint, const LiftedKeyMap& liftedKeys) {
  const JacobianFactor& J = constraint.factor();
  if (constraint.sense() != LinearConstraint::Sense::Equal || J.size() != 1) {
    throw std::runtime_error(
        "LiftedSDPProblem: only unary linear equality QCQP constraints are "
        "supported.");
  }
  const Key key = *J.begin();
  const LiftedKey& liftedKey = liftedKeys.at(key);
  const Matrix A = J.getA(J.begin());
  Matrix liftedA = A;
  Vector liftedB = J.getb();
  if (liftedKey.homogenized) {
    liftedA = A.rightCols(liftedKey.dim);
    liftedB -= A.col(0);
  }

  std::vector<DenseIndex> rows;
  for (DenseIndex row = 0; row < liftedA.rows(); ++row) {
    if (!liftedA.row(row).isZero(0.0)) {
      rows.push_back(row);
    } else if (std::abs(liftedB(row)) > 1e-12) {
      throw std::runtime_error(
          "LiftedSDPProblem: a linear equality is infeasible for h=1.");
    }
  }
  LiftedLinearEquality lifted;
  lifted.key = key;
  lifted.A.resize(rows.size(), liftedA.cols());
  lifted.b.resize(rows.size());
  for (size_t row = 0; row < rows.size(); ++row) {
    lifted.A.row(row) = liftedA.row(rows[row]);
    lifted.b(row) = liftedB(rows[row]);
  }
  return lifted;
}

// The QCQP rewritten in the lifted coordinates of every key.
struct LiftedQcqp {
  std::vector<LiftedCost> costs;
  std::vector<LiftedQuadraticConstraint> quadraticConstraints;
  std::vector<LiftedLinearEquality> linearEqualities;
};

// Constant quadratic constraints, such as h^2=1 itself, need no SDP row.
void AddLiftedQuadraticConstraint(LiftedQuadraticConstraint constraint,
                                  LiftedQcqp* lifted) {
  if (!constraint.A.isZero(0.0) || !constraint.a.isZero(0.0)) {
    lifted->quadraticConstraints.push_back(std::move(constraint));
    return;
  }
  const double b = constraint.b;
  const bool satisfied =
      constraint.sense == QuadraticConstraint::Sense::Equal ? b == 0.0
      : constraint.sense == QuadraticConstraint::Sense::LessEqual ? b >= 0.0
                                                                  : b <= 0.0;
  if (!satisfied) {
    throw std::runtime_error(
        "LiftedSDPProblem: a quadratic constraint is infeasible for h=1.");
  }
}

// Rewrite every cost and constraint of a QCQP in lifted coordinates.
LiftedQcqp MakeLiftedQcqp(const QcqpProblem& problem,
                          const LiftedKeyMap& liftedKeys) {
  LiftedQcqp lifted;
  for (const auto& factor : problem.costs()) {
    if (!factor) continue;
    const auto* cost = dynamic_cast<const QpCost*>(factor.get());
    if (!cost) {
      throw std::runtime_error("LiftedSDPProblem: expected QpCost.");
    }
    lifted.costs.push_back(MakeLiftedCost(*cost, liftedKeys));
  }

  // Equality factors may be quadratic or linear.
  for (const auto& factor : problem.eConstraints()) {
    if (!factor) continue;
    if (const auto* quadratic =
            dynamic_cast<const QuadraticEqualityConstraintFactor*>(
                factor.get())) {
      AddLiftedQuadraticConstraint(
          MakeLiftedQuadraticConstraint(quadratic->quadraticConstraint(),
                                        liftedKeys),
          &lifted);
      continue;
    }
    if (const auto* linear =
            dynamic_cast<const LinearEqualityConstraintFactor*>(factor.get())) {
      LiftedLinearEquality equality =
          MakeLiftedLinearEquality(linear->linearConstraint(), liftedKeys);
      if (equality.A.rows() > 0) {
        lifted.linearEqualities.push_back(std::move(equality));
      }
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
      AddLiftedQuadraticConstraint(
          MakeLiftedQuadraticConstraint(quadratic->quadraticConstraint(),
                                        liftedKeys),
          &lifted);
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
  return lifted;
}

// Assign each key a contiguous range after the constant entry at index 0.
std::map<Key, std::pair<int, int>> MakeBlockRanges(
    const KeyVector& keys, const LiftedKeyMap& liftedKeys, int* dimension) {
  std::map<Key, std::pair<int, int>> ranges;
  *dimension = 1;
  for (Key key : keys) {
    const int start = *dimension;
    *dimension += static_cast<int>(liftedKeys.at(key).dim);
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

// Recover one D=1 QCQP vector per key, restoring h=1 for homogenized keys.
Values RecoverQcqpValues(const FirstMomentViewMap& firstMoments,
                         const KeyVector& orderedKeys,
                         const LiftedKeyMap& liftedKeys) {
  Values recoveredQcqpValues;
  for (Key key : orderedKeys) {
    const LiftedKey& lifted = liftedKeys.at(key);
    const Vector y = ExtractSolvedFirstMoment(firstMoments.at(key), lifted.dim);
    Matrix value(lifted.qcqpDim, 1);
    if (lifted.homogenized) {
      value(0, 0) = 1.0;
      value.block(1, 0, lifted.dim, 1) = y;
    } else {
      value.col(0) = y;
    }
    recoveredQcqpValues.insert(key, value);
  }
  return recoveredQcqpValues;
}

// Compute one rank-one eigenvalue ratio per key from its moment block.
std::vector<double> ComputeVariableEVRs(
    const FirstMomentViewMap& firstMoments,
    const LiftedVariableXijToSDPVariableViewMap& xijMap,
    const KeyVector& orderedKeys, const LiftedKeyMap& liftedKeys) {
  std::vector<double> variableEVRs;
  variableEVRs.reserve(orderedKeys.size());
  for (Key key : orderedKeys) {
    variableEVRs.push_back(ComputeBlockRankOneRatio(SolvedMomentBlock(
        firstMoments, xijMap, key, liftedKeys.at(key).dim)));
  }
  return variableEVRs;
}

// Form the lifted objective term 0.5 <G, X_f> - g' y_f + 0.5 f of one cost.
mf::Expression::t BuildLiftedCostTerm(
    const LiftedCost& cost, const FirstMomentViewMap& firstMoments,
    const LiftedVariableXijToSDPVariableViewMap& xijMap) {
  // Assemble the local SDP block matrix X_f in the cost's key order.
  std::vector<mf::Expression::t> blockRows, firstMomentRow;
  for (Key key_i : cost.keys) {
    std::vector<mf::Expression::t> rowBlocks;
    for (Key key_j : cost.keys) {
      rowBlocks.push_back(xijMap.at({key_i, key_j})->asExpr());
    }
    blockRows.push_back(
        mf::Expr::hstack(monty::new_array_ptr<mf::Expression::t>(rowBlocks)));
    firstMomentRow.push_back(firstMoments.at(key_i)->asExpr());
  }

  const auto X_f =
      mf::Expr::vstack(monty::new_array_ptr<mf::Expression::t>(blockRows));
  auto term = mf::Expr::mul(
      0.5, mf::Expr::dot(convertToMosekDenseMatrix(cost.G), X_f));
  if (!cost.g.isZero(0.0)) {
    const auto y_f = mf::Expr::hstack(
        monty::new_array_ptr<mf::Expression::t>(firstMomentRow));
    term = mf::Expr::sub(
        term, mf::Expr::dot(convertToMosekDenseMatrix(
                                Matrix(cost.g.transpose())),
                            y_f));
  }
  if (cost.f != 0.0) {
    term = mf::Expr::add(term, mf::Expr::constTerm(0.5 * cost.f));
  }
  return term;
}

// Sum the lifted objective terms contributed by all QCQP costs.
mf::Expression::t BuildObjective(
    const LiftedQcqp& lifted, const FirstMomentViewMap& firstMoments,
    const LiftedVariableXijToSDPVariableViewMap& xijMap) {
  std::vector<mf::Expression::t> objectiveTerms;
  for (const auto& cost : lifted.costs) {
    objectiveTerms.push_back(BuildLiftedCostTerm(cost, firstMoments, xijMap));
  }

  if (objectiveTerms.empty()) {
    return mf::Expr::constTerm(0.0);
  }
  return mf::Expr::add(monty::new_array_ptr<mf::Expression::t>(objectiveTerms));
}

// Lower x'Ax + a'x ~ b to the affine SDP constraint <A, X> + a'y ~ b.
void AddQuadraticConstraint(
    const mf::Model::t& M, const LiftedQuadraticConstraint& constraint,
    const FirstMomentViewMap& firstMoments,
    const LiftedVariableXijToSDPVariableViewMap& xijMap) {
  const Key key = constraint.key;
  auto lhs = mf::Expr::dot(convertToMosekDenseMatrix(constraint.A),
                           xijMap.at({key, key})->asExpr());
  if (!constraint.a.isZero(0.0)) {
    lhs = mf::Expr::add(
        lhs, mf::Expr::dot(convertToMosekDenseMatrix(
                               Matrix(constraint.a.transpose())),
                           firstMoments.at(key)->asExpr()));
  }

  switch (constraint.sense) {
    case QuadraticConstraint::Sense::Equal:
      M->constraint(lhs, mf::Domain::equalsTo(constraint.b));
      break;
    case QuadraticConstraint::Sense::LessEqual:
      M->constraint(lhs, mf::Domain::lessThan(constraint.b));
      break;
    case QuadraticConstraint::Sense::GreaterEqual:
      M->constraint(lhs, mf::Domain::greaterThan(constraint.b));
      break;
  }
}

// Lower Ax = b on one key to both rows of [-b A] [1 y'; y X] = 0.
void AddLinearEqualityConstraint(
    const mf::Model::t& M, const LiftedLinearEquality& equality,
    const FirstMomentViewMap& firstMoments,
    const LiftedVariableXijToSDPVariableViewMap& xijMap) {
  const auto y = firstMoments.at(equality.key)->asExpr();
  const auto X = xijMap.at({equality.key, equality.key})->asExpr();
  const auto A = convertToMosekDenseMatrix(equality.A);
  const auto b = convertToMosekDenseMatrix(equality.b);

  // Ax=b implies A*x*x'=b*x', hence A*X=b*y' after lifting. Enforcing only the
  // first moments Ay=b would leave unconstrained PSD slack in X.
  M->constraint(mf::Expr::mul(A, mf::Expr::transpose(y)),
                mf::Domain::equalsTo(b));
  M->constraint(mf::Expr::sub(mf::Expr::mul(A, X), mf::Expr::mul(b, y)),
                mf::Domain::equalsTo(0.0));
}

// Add all lifted equality and inequality constraints to a Fusion model.
void AddQcqpConstraints(const mf::Model::t& M, const LiftedQcqp& lifted,
                        const FirstMomentViewMap& firstMoments,
                        const LiftedVariableXijToSDPVariableViewMap& xijMap) {
  for (const auto& constraint : lifted.quadraticConstraints) {
    AddQuadraticConstraint(M, constraint, firstMoments, xijMap);
  }
  for (const auto& equality : lifted.linearEqualities) {
    AddLinearEqualityConstraint(M, equality, firstMoments, xijMap);
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
  LiftedKeyMap liftedKeys;
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
  LiftedKeyMap liftedKeys;
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
      const DenseIndex dim = liftedKeys.at(key.first).dim;
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
    const auto ranges = MakeBlockRanges(indices, liftedKeys, &cliqueDimension);

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
  impl_->liftedKeys = MakeLiftedKeys(problem, impl_->orderedKeyDims);
  const LiftedQcqp lifted = MakeLiftedQcqp(problem, impl_->liftedKeys);
  int coneDimension;
  const auto ranges =
      MakeBlockRanges(impl_->orderedKeys, impl_->liftedKeys, &coneDimension);

  // Represent the complete lifted matrix with one positive semidefinite cone.
  impl_->M = new mf::Model("MonolithicSDP_MosekSDPSolver");
  auto Y = impl_->M->variable("Y", mf::Domain::inPSDCone(coneDimension));
  impl_->populateXijMap(Y, ranges);

  impl_->M->constraint(Y->index(0, 0), mf::Domain::equalsTo(1.0));

  impl_->M->objective(
      mf::ObjectiveSense::Minimize,
      BuildObjective(lifted, impl_->firstMomentViews,
                     impl_->liftedVariableXijToSDPVariableViewMap));

  AddQcqpConstraints(impl_->M, lifted, impl_->firstMomentViews,
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
                           impl_->liftedKeys);
}

std::vector<double>
LiftedSDPProblem<MonolithicSDP, MosekSDPSolver>::variableEVRs() const {
  if (!impl_->lastSolveSummary.solved) {
    throw std::runtime_error("variableEVRs: solve() has not been called.");
  }
  impl_->M->acceptedSolutionStatus(mf::AccSolutionStatus::Anything);
  return ComputeVariableEVRs(impl_->firstMomentViews,
                             impl_->liftedVariableXijToSDPVariableViewMap,
                             impl_->orderedKeys, impl_->liftedKeys);
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
  impl_->liftedKeys = MakeLiftedKeys(problem, impl_->orderedKeyDims);
  const LiftedQcqp lifted = MakeLiftedQcqp(problem, impl_->liftedKeys);
  impl_->M = new mf::Model("ChordalSDP_MosekSDPSolver");
  impl_->bayesTree_ = BuildSymbolicBayesTree(problem, orderingType);
  // Use one positive semidefinite variable per symbolic clique.
  impl_->populateXijMap();

  impl_->M->objective(
      mf::ObjectiveSense::Minimize,
      BuildObjective(lifted, impl_->firstMomentViews,
                     impl_->liftedVariableXijToSDPVariableViewMap));

  AddQcqpConstraints(impl_->M, lifted, impl_->firstMomentViews,
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
                           impl_->liftedKeys);
}

std::vector<double> LiftedSDPProblem<ChordalSDP, MosekSDPSolver>::variableEVRs()
    const {
  if (!impl_->lastSolveSummary.solved) {
    throw std::runtime_error("variableEVRs: solve() has not been called.");
  }
  impl_->M->acceptedSolutionStatus(mf::AccSolutionStatus::Anything);
  return ComputeVariableEVRs(impl_->firstMomentViews,
                             impl_->liftedVariableXijToSDPVariableViewMap,
                             impl_->orderedKeys, impl_->liftedKeys);
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

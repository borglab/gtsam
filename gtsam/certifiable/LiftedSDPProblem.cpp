#include <gtsam/certifiable/LiftedSDPProblem.h>

#include <iostream>
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

using LiftedVariableXijToSDPVariableViewMap = std::map<std::pair<Key, Key>, mf::Variable::t>;

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

void DisposeMosekModel(const mf::Model::t& M) {
  if (M.get() != nullptr) {
    M->dispose();
  }
}

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

mf::Matrix::t convertToMosekDenseMatrix(const Matrix& mat) {
  return mf::Matrix::dense(static_cast<int>(mat.rows()),
                           static_cast<int>(mat.cols()),
                           convertToMOSEKArray2D(mat));
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

}  // namespace

struct LiftedSDPProblem<MonolithicSDP, MosekSDPSolver>::Impl {
  mf::Model::t M;
  KeyVector orderedKeys;
  std::map<Key, DenseIndex> orderedKeyDims;
  std::map<Key, std::pair<DenseIndex, DenseIndex>> orderedKeyToYSlice;
  DenseIndex totalMonolithicDimension;
  LiftedVariableXijToSDPVariableViewMap liftedVariableXijToSDPVariableViewMap;

  ~Impl() {
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
      std::cout << DefaultKeyFormatter(key) << " Y slice [" << start << ", "
                << end << ")" << std::endl;
      cumulativeIndex = end;
    }
    totalMonolithicDimension = cumulativeIndex;
  }

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

  impl_->M = new mf::Model("MonolithicSDP_MosekSDPSolver");
  auto Y = impl_->M->variable(
      "Y",
      mf::Domain::inPSDCone(
          static_cast<int>(impl_->totalMonolithicDimension)));
  impl_->populateXijMap(Y);

  const auto objective = BuildObjective(problem, impl_->liftedVariableXijToSDPVariableViewMap);
  impl_->M->objective(mf::ObjectiveSense::Minimize,
                      mf::Expr::mul(0.5, objective));
}

LiftedSDPProblem<MonolithicSDP, MosekSDPSolver>::~LiftedSDPProblem() = default;

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

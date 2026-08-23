/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file CholmodSolver.cpp
 * @brief Optional reusable CHOLMOD solver for Gaussian factor graphs.
 */

#include <gtsam/linear/BatchJacobianFactor.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/SparseEigen.h>
#include <gtsam/linear/internal/BatchJacobianFactorElimination.h>
#include <gtsam/linear/internal/CholmodSolver.h>
#include <gtsam/linear/linearExceptions.h>

#include <Eigen/Sparse>
#include <algorithm>
#include <map>
#include <stdexcept>
#include <unordered_set>
#include <utility>
#include <vector>

#ifdef GTSAM_ENABLE_CHOLMOD
#include <cholmod.h>
#endif

namespace gtsam::internal {

#ifdef GTSAM_ENABLE_CHOLMOD
namespace {

struct SparseNormalSystem {
  SparseEigen hessian;
  Vector rhs;
  std::map<Key, size_t> dimensions;
};

struct VariableLayout {
  std::map<Key, size_t> dimensions;
  std::map<Key, DenseIndex> scalarOffsets;
  DenseIndex dimension = 0;
};

VariableLayout createVariableLayout(const GaussianFactorGraph& graph,
                                    const Ordering& ordering) {
  VariableLayout layout;
  layout.dimensions = graph.getKeyDimMap();
  if (ordering.size() != layout.dimensions.size()) {
    throw std::invalid_argument(
        "CHOLMOD ordering must contain every Gaussian variable exactly once");
  }
  std::unordered_set<Key> seen;
  for (Key key : ordering) {
    if (!layout.dimensions.count(key) || !seen.insert(key).second) {
      throw std::invalid_argument(
          "CHOLMOD ordering contains an unknown or duplicate key");
    }
    layout.scalarOffsets.emplace(key, layout.dimension);
    layout.dimension += static_cast<DenseIndex>(layout.dimensions.at(key));
  }
  return layout;
}

class TripletNormalAccumulator : public SparseNormalAccumulator {
 public:
  TripletNormalAccumulator(std::vector<Eigen::Triplet<double, int>>* entries,
                           Vector* rhs)
      : entries_(entries), rhs_(rhs) {}

  void addHessianBlock(DenseIndex rowOffset, DenseIndex columnOffset,
                       DenseIndex rows, DenseIndex columns,
                       const double* values) override {
    for (DenseIndex column = 0; column < columns; ++column) {
      for (DenseIndex row = 0; row < rows; ++row) {
        const DenseIndex globalRow = rowOffset + row;
        const DenseIndex globalColumn = columnOffset + column;
        if (rowOffset == columnOffset && globalRow > globalColumn) continue;
        const double value = values[row + column * rows];
        const DenseIndex upperRow = std::min(globalRow, globalColumn);
        const DenseIndex upperColumn = std::max(globalRow, globalColumn);
        entries_->emplace_back(static_cast<int>(upperRow),
                               static_cast<int>(upperColumn), value);
      }
    }
  }

  void addRhsBlock(DenseIndex offset, DenseIndex dimension,
                   const double* values) override {
    rhs_->segment(offset, dimension) +=
        Eigen::Map<const Vector>(values, dimension);
  }

 private:
  std::vector<Eigen::Triplet<double, int>>* entries_;
  Vector* rhs_;
};

void appendHessianFactor(const HessianFactor& factor,
                         const VariableLayout& layout,
                         TripletNormalAccumulator* accumulator) {
  for (size_t i = 0; i < factor.size(); ++i) {
    const Key keyI = factor.keys().at(i);
    const DenseIndex offsetI = layout.scalarOffsets.at(keyI);
    const DenseIndex dimensionI = factor.getDim(factor.begin() + i);
    accumulator->addRhsBlock(offsetI, dimensionI,
                             factor.linearTerm(factor.begin() + i).data());

    for (size_t j = i; j < factor.size(); ++j) {
      const Key keyJ = factor.keys().at(j);
      const DenseIndex offsetJ = layout.scalarOffsets.at(keyJ);
      const Matrix block = i == j
                               ? Matrix(factor.info().diagonalBlock(i))
                               : Matrix(factor.info().aboveDiagonalBlock(i, j));
      accumulator->addHessianBlock(offsetI, offsetJ, block.rows(), block.cols(),
                                   block.data());
    }
  }
}

void appendBatchFactor(const BatchJacobianFactorBase& factor,
                       const VariableLayout& layout,
                       TripletNormalAccumulator* accumulator) {
  std::vector<DenseIndex> scalarOffsets;
  scalarOffsets.reserve(factor.size());
  for (Key key : factor.keys()) {
    scalarOffsets.push_back(layout.scalarOffsets.at(key));
  }
  BatchJacobianFactorElimination::addSparseNormal(factor, scalarOffsets,
                                                  accumulator);
}

void appendFactor(const GaussianFactor& factor, const VariableLayout& layout,
                  TripletNormalAccumulator* accumulator) {
  if (const auto* hessian = dynamic_cast<const HessianFactor*>(&factor)) {
    appendHessianFactor(*hessian, layout, accumulator);
    return;
  }
  if (const auto* jacobian = dynamic_cast<const JacobianFactor*>(&factor)) {
    appendHessianFactor(HessianFactor(*jacobian), layout, accumulator);
    return;
  }
  if (const auto* batch =
          dynamic_cast<const BatchJacobianFactorBase*>(&factor)) {
    appendBatchFactor(*batch, layout, accumulator);
    return;
  }
  throw std::invalid_argument(
      "CHOLMOD supports JacobianFactor, BatchJacobianFactor, and HessianFactor "
      "inputs");
}

SparseEigen finalizeSparseHessian(
    DenseIndex dimension,
    const std::vector<Eigen::Triplet<double, int>>& entries) {
  SparseEigen hessian(dimension, dimension);
  hessian.setFromTriplets(entries.begin(), entries.end());
  hessian.makeCompressed();
  return hessian;
}

SparseNormalSystem buildSparseNormalSystem(const GaussianFactorGraph& graph,
                                           const Ordering& ordering) {
  if (hasConstraints(graph)) {
    throw std::invalid_argument(
        "CHOLMOD does not support constrained Gaussian factors");
  }
  const VariableLayout layout = createVariableLayout(graph, ordering);
  std::vector<Eigen::Triplet<double, int>> entries;
  Vector rhs = Vector::Zero(layout.dimension);
  TripletNormalAccumulator accumulator(&entries, &rhs);

  for (const GaussianFactor::shared_ptr& factor : graph) {
    if (factor) appendFactor(*factor, layout, &accumulator);
  }
  return {finalizeSparseHessian(layout.dimension, entries), std::move(rhs),
          layout.dimensions};
}

}  // namespace

struct CholmodSolver::Impl {
  cholmod_common common{};
  cholmod_factor* factor = nullptr;
  std::vector<int> outerPattern;
  std::vector<int> innerPattern;
  int dimension = 0;

  Impl() {
    cholmod_start(&common);
    common.print = 0;
    // The assembled normal system already expands the caller's key ordering
    // into scalar order. Keep it instead of silently using CHOLMOD's AMD.
    common.nmethods = 1;
    common.method[0].ordering = CHOLMOD_NATURAL;
    common.postorder = false;
  }

  ~Impl() {
    if (factor) cholmod_free_factor(&factor, &common);
    cholmod_finish(&common);
  }

  cholmod_sparse view(SparseEigen* matrix) {
    cholmod_sparse result{};
    result.nrow = static_cast<size_t>(matrix->rows());
    result.ncol = static_cast<size_t>(matrix->cols());
    result.nzmax = static_cast<size_t>(matrix->nonZeros());
    result.p = matrix->outerIndexPtr();
    result.i = matrix->innerIndexPtr();
    result.x = matrix->valuePtr();
    result.z = nullptr;
    result.nz = nullptr;
    result.stype = 1;
    result.itype = CHOLMOD_INT;
    result.xtype = CHOLMOD_REAL;
    result.dtype = CHOLMOD_DOUBLE;
    result.sorted = 1;
    result.packed = 1;
    return result;
  }

  void analyzeIfNeeded(SparseEigen* matrix) {
    const std::vector<int> outer(
        matrix->outerIndexPtr(),
        matrix->outerIndexPtr() + matrix->outerSize() + 1);
    const std::vector<int> inner(matrix->innerIndexPtr(),
                                 matrix->innerIndexPtr() + matrix->nonZeros());
    if (factor && dimension == matrix->rows() && outerPattern == outer &&
        innerPattern == inner) {
      return;
    }
    if (factor) cholmod_free_factor(&factor, &common);
    cholmod_sparse sparse = view(matrix);
    factor = cholmod_analyze(&sparse, &common);
    if (!factor) throw std::runtime_error("CHOLMOD symbolic analysis failed");
    dimension = matrix->rows();
    outerPattern = outer;
    innerPattern = inner;
  }

  Vector solve(SparseEigen* matrix, const Vector& rhs, Key nearbyKey) {
    analyzeIfNeeded(matrix);
    cholmod_sparse sparse = view(matrix);
    if (!cholmod_factorize(&sparse, factor, &common) ||
        common.status != CHOLMOD_OK) {
      throw IndeterminateSystemException(nearbyKey);
    }

    cholmod_dense denseRhs{};
    denseRhs.nrow = static_cast<size_t>(rhs.size());
    denseRhs.ncol = 1;
    denseRhs.nzmax = static_cast<size_t>(rhs.size());
    denseRhs.d = static_cast<size_t>(rhs.size());
    denseRhs.x = const_cast<double*>(rhs.data());
    denseRhs.z = nullptr;
    denseRhs.xtype = CHOLMOD_REAL;
    denseRhs.dtype = CHOLMOD_DOUBLE;
    cholmod_dense* rawSolution =
        cholmod_solve(CHOLMOD_A, factor, &denseRhs, &common);
    if (!rawSolution) throw std::runtime_error("CHOLMOD solve failed");
    Vector solution =
        Eigen::Map<Vector>(static_cast<double*>(rawSolution->x), rhs.size());
    cholmod_free_dense(&rawSolution, &common);
    return solution;
  }
};

#else

struct CholmodSolver::Impl {};

#endif

CholmodSolver::CholmodSolver() : impl_(std::make_unique<Impl>()) {}
CholmodSolver::~CholmodSolver() = default;

bool CholmodSolver::available() {
#ifdef GTSAM_ENABLE_CHOLMOD
  return true;
#else
  return false;
#endif
}

VectorValues CholmodSolver::solve(const GaussianFactorGraph& graph,
                                  const Ordering& ordering) {
#ifdef GTSAM_ENABLE_CHOLMOD
  SparseNormalSystem system = buildSparseNormalSystem(graph, ordering);
  if (ordering.empty()) return {};
  const Vector solution =
      impl_->solve(&system.hessian, system.rhs, ordering.front());
  VectorValues result;
  DenseIndex offset = 0;
  for (Key key : ordering) {
    const DenseIndex dimension =
        static_cast<DenseIndex>(system.dimensions.at(key));
    result.insert(key, solution.segment(offset, dimension));
    offset += dimension;
  }
  return result;
#else
  (void)graph;
  (void)ordering;
  throw std::runtime_error(
      "CHOLMOD was selected, but this GTSAM build has no CHOLMOD support; "
      "install SuiteSparse CHOLMOD and reconfigure GTSAM");
#endif
}

}  // namespace gtsam::internal

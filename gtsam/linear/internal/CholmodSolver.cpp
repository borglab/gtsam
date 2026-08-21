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

#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/SparseEigen.h>
#include <gtsam/linear/internal/CholmodSolver.h>
#include <gtsam/linear/linearExceptions.h>

#include <Eigen/Sparse>
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

SparseNormalSystem buildSparseNormalSystem(const GaussianFactorGraph& graph,
                                           const Ordering& ordering) {
  if (hasConstraints(graph)) {
    throw std::invalid_argument(
        "CHOLMOD does not support constrained Gaussian factors");
  }
  const std::map<Key, size_t> dimensions = graph.getKeyDimMap();
  if (ordering.size() != dimensions.size()) {
    throw std::invalid_argument(
        "CHOLMOD ordering must contain every Gaussian variable exactly once");
  }
  std::unordered_set<Key> seen;
  for (Key key : ordering) {
    if (!dimensions.count(key) || !seen.insert(key).second) {
      throw std::invalid_argument(
          "CHOLMOD ordering contains an unknown or duplicate key");
    }
  }

  std::map<Key, int> scalarOffsets;
  int dimension = 0;
  for (Key key : ordering) {
    scalarOffsets.emplace(key, dimension);
    dimension += static_cast<int>(dimensions.at(key));
  }

  std::vector<Eigen::Triplet<double, int>> hessianEntries;
  Vector rhs = Vector::Zero(dimension);
  for (const GaussianFactor::shared_ptr& factor : graph) {
    if (!factor) continue;

    const HessianFactor* hessian =
        dynamic_cast<const HessianFactor*>(factor.get());
    std::unique_ptr<HessianFactor> converted;
    if (!hessian) {
      if (const auto* jacobian =
              dynamic_cast<const JacobianFactor*>(factor.get())) {
        converted = std::make_unique<HessianFactor>(*jacobian);
        hessian = converted.get();
      } else {
        throw std::invalid_argument(
            "CHOLMOD supports JacobianFactor and HessianFactor inputs");
      }
    }

    for (size_t i = 0; i < hessian->size(); ++i) {
      const Key keyI = hessian->keys().at(i);
      const int offsetI = scalarOffsets.at(keyI);
      const int dimensionI = static_cast<int>(
          hessian->getDim(hessian->begin() + static_cast<DenseIndex>(i)));
      rhs.segment(offsetI, dimensionI) +=
          hessian->linearTerm(hessian->begin() + i).col(0);

      for (size_t j = i; j < hessian->size(); ++j) {
        const Key keyJ = hessian->keys().at(j);
        const int offsetJ = scalarOffsets.at(keyJ);
        const Matrix block =
            i == j ? Matrix(hessian->info().diagonalBlock(i))
                   : Matrix(hessian->info().aboveDiagonalBlock(i, j));
        for (DenseIndex row = 0; row < block.rows(); ++row) {
          for (DenseIndex column = 0; column < block.cols(); ++column) {
            if (i == j && row > column) continue;
            const int globalRow = offsetI + static_cast<int>(row);
            const int globalColumn = offsetJ + static_cast<int>(column);
            if (globalRow <= globalColumn) {
              hessianEntries.emplace_back(globalRow, globalColumn,
                                          block(row, column));
            } else {
              hessianEntries.emplace_back(globalColumn, globalRow,
                                          block(row, column));
            }
          }
        }
      }
    }
  }

  SparseEigen hessian(dimension, dimension);
  hessian.setFromTriplets(hessianEntries.begin(), hessianEntries.end());
  hessian.makeCompressed();
  return {std::move(hessian), std::move(rhs), dimensions};
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
    // sparseJacobian() already expands the caller's key ordering into scalar
    // column order. Keep that order instead of silently replacing it with
    // CHOLMOD's own AMD choice.
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

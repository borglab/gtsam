/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file SparseQpSolver.cpp
 * Sparse working-system algebra extracted from ActiveSetSolver.
 *
 * Original factors and reference-center messages remain consistent while
 * working rows and objective forcing change independently. GaussianISAM2
 * supplies factor updates and exact cached solves; all QP policy stays in the
 * constrained layer rather than the foundational linear solver.
 */

#include <gtsam/constrained/internal/SparseQpSolver.h>
#include <gtsam/linear/GaussianISAM2.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/linearAlgorithms-inst.h>

#include <algorithm>
#include <cmath>
#include <utility>

namespace gtsam {
namespace internal {

struct SparseQpSolver::Impl {
  const SparseQpData data;
  KeySet keys;
  Ordering ordering;
  GaussianFactorGraph::Eliminate eliminate = EliminatePreferCholesky;
  std::unique_ptr<GaussianISAM2> tree;
  std::vector<std::optional<FactorIndex>> inequalityFactors;
  VectorValues referenceCenter;

  explicit Impl(SparseQpData inputs) : data(std::move(inputs)) {
    // Preserve square-root costs through every clique, including separators.
    if (std::all_of(data.objective.begin(), data.objective.end(),
                    [](const GaussianFactor::shared_ptr& factor) {
                      return dynamic_cast<const JacobianFactor*>(factor.get());
                    }))
      eliminate = EliminateQR;
  }

  // Retain the original factor order: costs, proximal rows, equalities, then
  // active inequalities. This order also determines registered factor indices.
  GaussianFactorGraph workingGraph(const std::vector<bool>& active,
                                   const VectorValues& center) const {
    GaussianFactorGraph graph;
    graph.push_back(data.objective);
    if (data.regularization > 0.0) {
      const double weight = std::sqrt(data.regularization);
      for (const auto& [key, dimension] : data.keyDimensions)
        graph.emplace_shared<JacobianFactor>(
            key, weight * Matrix::Identity(dimension, dimension),
            weight * center.at(key));
    }
    graph.push_back(data.equalities);
    for (size_t row = 0; row < data.inequalities.size(); ++row)
      if (active.at(row)) graph.push_back(data.inequalities[row]);
    return graph;
  }

  // Connectivity changes preserve the ordering. A constraint-only variable
  // entering or leaving requires a new tree, using the same batch ordering.
  VectorValues solve(const std::vector<bool>& active, const VectorValues& center) {
    const GaussianFactorGraph graph = workingGraph(active, center);
    const KeySet graphKeys = graph.keys();
    if (graphKeys != keys) {
      tree.reset();
      ordering.clear();
      if (data.ordering) {
        for (Key key : *data.ordering)
          if (graphKeys.count(key)) ordering.push_back(key);
      } else {
        ordering = Ordering::Colamd(graph);
      }
      keys = graphKeys;
    }
    if (data.useIncremental) return solveIncremental(graph, active, center);
    // The batch comparison rebuilds once per face, then reuses that tree for
    // the same residual correction as the incremental path.
    GaussianISAM2 batch(ordering, eliminate);
    batch.update(graph);
    return solveRefined(batch, center, {});
  }

  // Register the complete graph once and remember the center encoded in its
  // original objective factors. Later updates never rewrite those factors.
  void initializeTree(const GaussianFactorGraph& graph,
                      const std::vector<bool>& active,
                      const VectorValues& center) {
    tree = std::make_unique<GaussianISAM2>(ordering, eliminate);
    const FactorIndices indices = tree->update(graph);
    inequalityFactors.assign(active.size(), std::nullopt);
    referenceCenter = center;
    size_t position = data.objective.size() + data.equalities.size() +
        (data.regularization > 0.0 ? data.keyDimensions.size() : 0);
    for (size_t row = 0; row < active.size(); ++row)
      if (active[row]) inequalityFactors[row] = indices.at(position++);
  }

  // Only entering and released inequalities modify the numerical factorization.
  void updateWorkingSet(const std::vector<bool>& active) {
    GaussianFactorGraph added;
    FactorIndices removed;
    std::vector<size_t> addedRows;
    for (size_t row = 0; row < active.size(); ++row) {
      if (active[row] && !inequalityFactors[row]) {
        added.push_back(data.inequalities[row]);
        addedRows.push_back(row);
      } else if (!active[row] && inequalityFactors[row]) {
        removed.push_back(*inequalityFactors[row]);
        inequalityFactors[row].reset();
      }
    }
    const FactorIndices indices = tree->update(added, removed);
    for (size_t index = 0; index < addedRows.size(); ++index)
      inequalityFactors[addedRows[index]] = indices.at(index);
  }

  // Form the original working objective gradient without converting native
  // Hessians to Jacobians or assembling a global normal matrix. Hard rows are
  // excluded: the cached constrained inverse projects onto their tangent space.
  VectorValues workingGradient(const VectorValues& candidate,
                               const VectorValues& center) const {
    VectorValues gradient;
    for (const auto& [key, value] : candidate) {
      Vector block = Vector::Zero(value.size());
      if (data.regularization > 0.0)
        block = data.regularization * (value - center.at(key));
      gradient.insert(key, std::move(block));
    }
    for (const auto& factor : data.objective)
      for (Key key : factor->keys())
        gradient.at(key) += factor->gradient(key, candidate);
    return gradient;
  }

  // Refine with homogeneous tangent corrections, not another affine solve.
  // Adding corrections to stored d before back-substitution can round them away
  // or reintroduce its original error along weak directions. Hard rows have
  // zero sigmas, so the cached inverse gives corrections tangent to their face.
  VectorValues solveRefined(const GaussianISAM2& factorization,
                            const VectorValues& center,
                            const VectorValues& forcing) const {
    VectorValues candidate = forcing.size() == 0
        ? factorization.optimize() : factorization.optimize(forcing);
    for (size_t iteration = 0; iteration < 4; ++iteration) {
      const VectorValues rhs = linearAlgorithms::transposeBayesTree(
          factorization, workingGradient(candidate, center));
      const VectorValues correction =
          linearAlgorithms::optimizeBayesTree(factorization, &rhs);
      VectorValues refined = candidate - correction;
      if (refined.equals(candidate, 0.0)) break;
      candidate = std::move(refined);
    }
    return candidate;
  }

  // A new center changes objective forcing only. Registered factors and orphan
  // messages stay at the reference center, so RHS-only solves never eliminate.
  VectorValues solveIncremental(const GaussianFactorGraph& graph,
                                const std::vector<bool>& active,
                                const VectorValues& center) {
    if (!tree) initializeTree(graph, active, center);
    else updateWorkingSet(active);
    VectorValues forcing;
    if (data.regularization > 0.0) {
      for (const auto& [key, value] : center) {
        Vector change = data.regularization * (value - referenceCenter.at(key));
        if (!change.isZero(0.0)) forcing.insert(key, std::move(change));
      }
    }
    return solveRefined(*tree, center, forcing);
  }
};

SparseQpSolver::SparseQpSolver(SparseQpData data)
    : impl_(std::make_unique<Impl>(std::move(data))) {}

SparseQpSolver::~SparseQpSolver() = default;

VectorValues SparseQpSolver::solve(const std::vector<bool>& active,
                                  const VectorValues& center) {
  return impl_->solve(active, center);
}

}  // namespace internal
}  // namespace gtsam

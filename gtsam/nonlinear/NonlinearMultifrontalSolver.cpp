/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file NonlinearMultifrontalSolver.cpp
 * @brief Implementation of nonlinear multifrontal solver.
 * @author Frank Dellaert
 * @date   January 2026
 */

#include <gtsam/base/types.h>
#include <gtsam/constrained/NonlinearEqualityConstraint.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/MultifrontalClique.h>
#include <gtsam/nonlinear/NonlinearMultifrontalSolver.h>
#include <gtsam/symbolic/IndexedJunctionTree.h>
#include <gtsam/symbolic/SymbolicJunctionTree.h>

namespace gtsam {

namespace {

std::map<Key, size_t> computeDimsFromValues(const Values& values) {
  return values.dims();
}

std::unordered_set<Key> collectFixedKeys(const NonlinearFactorGraph& graph) {
  std::unordered_set<Key> fixedKeys;
  for (const auto& factor : graph) {
    if (!factor || factor->keys().size() != 1) continue;
    if (auto constraint =
            std::dynamic_pointer_cast<NonlinearEqualityConstraint>(factor)) {
      if (constraint->isHardConstraint()) {
        fixedKeys.insert(factor->keys().front());
      } else {
        throw MultifrontalSolverNotSupported(
            "non-hard constraints are not supported");
      }
    }
  }
  return fixedKeys;
}

}  // namespace

/* ************************************************************************* */
NonlinearMultifrontalSolver::NonlinearMultifrontalSolver(
    const NonlinearFactorGraph& graph, const Values& values,
    const Ordering& ordering, MultifrontalSolver::Parameters params,
    DampingParams dampingParams)
    : MultifrontalSolver(
          NonlinearMultifrontalSolver::Precompute(graph, values, ordering),
          ordering, params),
      dampingParams_(dampingParams) {}

/* ************************************************************************* */
MultifrontalSolver::PrecomputedData NonlinearMultifrontalSolver::Precompute(
    const NonlinearFactorGraph& graph, const Values& values,
    const Ordering& ordering) {
  auto dims = computeDimsFromValues(values);
  auto fixedKeys = collectFixedKeys(graph);

  Ordering reducedOrdering;
  reducedOrdering.reserve(ordering.size());
  for (Key key : ordering) {
    if (!fixedKeys.count(key)) {
      reducedOrdering.push_back(key);
    }
  }

  IndexedJunctionTree indexedJunctionTree(graph, reducedOrdering, fixedKeys);
  
  std::vector<size_t> rowCounts;
  rowCounts.reserve(graph.size());
  for (const auto& factor : graph) {
    size_t dim = factor ? factor->dim() : 0;
    rowCounts.push_back(dim);
  }

  return MultifrontalSolver::PrecomputedData{
      std::move(dims), std::move(fixedKeys), std::move(indexedJunctionTree), std::move(rowCounts)};
}

/* ************************************************************************* */
void NonlinearMultifrontalSolver::load(const GaussianFactorGraph& graph) {
  MultifrontalSolver::load(graph);
  if (dampingParams_.diagonalDamping && dampingParams_.exactHessianDiagonal) {
    exactHessianDiagonal_ = graph.hessianDiagonal();
    hasExactHessianDiagonal_ = true;
  } else {
    hasExactHessianDiagonal_ = false;
    exactHessianDiagonal_ = VectorValues();
  }
}

/* ************************************************************************* */
void NonlinearMultifrontalSolver::eliminateInPlace(double lambda) {
  if (!loaded_) {
    throw std::runtime_error(
        "NonlinearMultifrontalSolver::eliminateInPlace: load() must be called "
        "before eliminating.");
  }

  eliminated_ = false;
  if (lambda <= 0.0) {
    MultifrontalSolver::eliminateInPlace();
  } else {
    runBottomUp(
        [this, lambda](MultifrontalClique& node) {
          node.eliminateInPlace(lambda, dampingParams_, exactHessianDiagonal_);
        },
        params_.eliminationParallelThreshold);
  }
  eliminated_ = true;
}

/* ************************************************************************* */
void NonlinearMultifrontalSolver::eliminateInPlace(
    const GaussianFactorGraph& graph, double lambda) {
  eliminated_ = false;
  if (lambda <= 0.0) {
    MultifrontalSolver::eliminateInPlace(graph);
    return;
  }

  // Calculate the exact Hessian diagonal if needed.
  if (dampingParams_.diagonalDamping && dampingParams_.exactHessianDiagonal) {
    exactHessianDiagonal_ = graph.hessianDiagonal();
    hasExactHessianDiagonal_ = true;
  } else {
    hasExactHessianDiagonal_ = false;
    exactHessianDiagonal_ = VectorValues();
  }

  // Run bottom-up elimination with damping.
  runBottomUp(
      [&graph, this, lambda](MultifrontalClique& node) {
        node.fillAb(graph);
        node.eliminateInPlace(lambda, dampingParams_, exactHessianDiagonal_);
      },
      params_.eliminationParallelThreshold);
  loaded_ = true;
  eliminated_ = true;
}

}  // namespace gtsam

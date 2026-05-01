/* ----------------------------------------------------------------------------
 *
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 *
 * -------------------------------------------------------------------------- */

/**
 * @file    IndexedJunctionTree.h
 * @brief   Build a symbolic junction tree that stores original factor indices.
 * @author  Tzvi Strauss
 */

#pragma once

#include <gtsam/base/types.h>
#include <gtsam/inference/Key.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/symbolic/SymbolicEliminationTree.h>
#include <gtsam/symbolic/SymbolicFactor.h>
#include <gtsam/symbolic/SymbolicFactorGraph.h>
#include <gtsam/symbolic/SymbolicJunctionTree.h>

#include <cstddef>
#include <unordered_set>

namespace gtsam {

namespace internal {
/// A SymbolicFactor that stores the index of the originating factor.
class IndexedSymbolicFactor : public SymbolicFactor {
 public:
  size_t index_;
  IndexedSymbolicFactor(const KeyVector& keys, size_t index)
      : SymbolicFactor(), index_(index) {
    keys_ = keys;
  }
};
}  // namespace internal

/**
 * A symbolic junction tree whose factors record the original factor indices from
 * a corresponding (non-symbolic) factor graph. This allows the junction tree
 * structure to be cached and reused for repeated eliminations when the factor
 * graph structure and ordering remain unchanged, avoiding the cost of rebuilding
 * the symbolic structure.
 *
 * The underlying implementation uses a `SymbolicJunctionTree` where each factor
 * is an `IndexedSymbolicFactor` that stores the original factor index.
 * During elimination, these indices are used to retrieve the actual numerical
 * factors from the original factor graph.
 *
 */
class GTSAM_EXPORT IndexedJunctionTree : public SymbolicJunctionTree {
 public:
  using SymbolicJunctionTree::SymbolicJunctionTree;

  /**
   * Construct an IndexedJunctionTree from any factor graph, ordering, and
   * optional set of fixed keys to filter out.
   *
   * @tparam GRAPH Factor graph type (e.g. GaussianFactorGraph, NonlinearFactorGraph)
   * @param graph The input factor graph
   * @param ordering The elimination ordering
   * @param fixedKeys Keys to filter out (e.g., from hard constraints)
   */
  template <typename GRAPH>
  IndexedJunctionTree(const GRAPH& graph, const Ordering& ordering,
                      const std::unordered_set<Key>& fixedKeys = {})
      : SymbolicJunctionTree(makeIndexedEliminationTree(graph, ordering, fixedKeys)) {}

 private:
  template <typename GRAPH>
  static SymbolicFactorGraph buildIndexedSymbolicFactorGraph(
      const GRAPH& graph, const std::unordered_set<Key>& fixedKeys) {
    SymbolicFactorGraph symbolicGraph;
    symbolicGraph.reserve(graph.size());
    for (size_t i = 0; i < graph.size(); ++i) {
      if (!graph.at(i)) continue;
      KeyVector keys;
      keys.reserve(graph[i]->size());
      for (Key key : graph[i]->keys()) {
        if (!fixedKeys.count(key)) keys.push_back(key);
      }
      // Skip factors that are fully constrained away.
      if (keys.empty()) continue;
      symbolicGraph.emplace_shared<internal::IndexedSymbolicFactor>(keys, i);
    }
    return symbolicGraph;
  }

  template <typename GRAPH>
  static SymbolicEliminationTree makeIndexedEliminationTree(
      const GRAPH& graph, const Ordering& ordering,
      const std::unordered_set<Key>& fixedKeys) {
    SymbolicFactorGraph symbolicGraph =
        buildIndexedSymbolicFactorGraph(graph, fixedKeys);
    return SymbolicEliminationTree(std::move(symbolicGraph), ordering);
  }
};
}  // namespace gtsam


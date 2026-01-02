/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file MultifrontalSolver.cpp
 * @brief Implementation of imperative-style multifrontal solver.
 * @author Frank Dellaert
 * @date   December 2025
 */

#include <gtsam/base/treeTraversal-inst.h>
#include <gtsam/base/types.h>
#include <gtsam/inference/Key.h>
#include <gtsam/linear/GaussianBayesTree.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/MultifrontalClique.h>
#include <gtsam/linear/MultifrontalSolver.h>
#include <gtsam/symbolic/SymbolicEliminationTree.h>
#include <gtsam/symbolic/SymbolicFactorGraph.h>
#include <gtsam/symbolic/SymbolicJunctionTree.h>

#include <algorithm>
#include <cassert>
#include <functional>
#include <iostream>
#include <ostream>
#include <stdexcept>
#include <string>

namespace gtsam {

namespace {

struct StructureStats {
  size_t count = 0;
  size_t maxFrontals = 0;
  size_t maxSeparators = 0;
  size_t maxTotal = 0;
  size_t maxChildren = 0;
  size_t sumFrontals = 0;
  size_t sumSeparators = 0;
  size_t sumTotal = 0;
  size_t sumChildren = 0;

  void update(size_t frontalDim, size_t separatorDim, size_t childCount) {
    const size_t totalDim = frontalDim + separatorDim;
    count += 1;
    maxFrontals = std::max(maxFrontals, frontalDim);
    maxSeparators = std::max(maxSeparators, separatorDim);
    maxTotal = std::max(maxTotal, totalDim);
    maxChildren = std::max(maxChildren, childCount);
    sumFrontals += frontalDim;
    sumSeparators += separatorDim;
    sumTotal += totalDim;
    sumChildren += childCount;
  }

  void report(const std::string& name, std::ostream* os) {
    auto avg = [this](size_t sum) {
      return count ? static_cast<double>(sum) / count : 0.0;
    };
    const double avgF = avg(sumFrontals);
    const double avgS = avg(sumSeparators);
    const double avgT = avg(sumTotal);
    const double avgC = avg(sumChildren);

    *os << name << "\n";
    *os << "  cliques:    " << count << "\n";
    *os << "  frontals:   max=" << maxFrontals << " avg=" << avgF << "\n";
    *os << "  separators: max=" << maxSeparators << " avg=" << avgS << "\n";
    *os << "  total dim:  max=" << maxTotal << " avg=" << avgT << "\n";
    *os << "  children:   max=" << maxChildren << " avg=" << avgC << "\n";
  }
};

// Compute variable dimensions from the GaussianFactorGraph
std::map<Key, size_t> computeDims(const GaussianFactorGraph& graph) {
  std::map<Key, size_t> dims;
  for (const auto& factor : graph) {
    if (!factor) continue;
    if (auto jacobianFactor =
            std::dynamic_pointer_cast<JacobianFactor>(factor)) {
      for (auto it = jacobianFactor->begin(); it != jacobianFactor->end();
           ++it) {
        dims[*it] = jacobianFactor->getDim(it);
      }
    } else if (auto hessianFactor =
                   std::dynamic_pointer_cast<HessianFactor>(factor)) {
      for (auto it = hessianFactor->begin(); it != hessianFactor->end(); ++it) {
        dims[*it] = hessianFactor->getDim(it);
      }
    }
  }
  return dims;
}

// Build SymbolicFactorGraph from GaussianFactorGraph
SymbolicFactorGraph buildSymbolicGraph(const GaussianFactorGraph& graph) {
  SymbolicFactorGraph symbolicGraph;
  symbolicGraph.reserve(graph.size());
  for (size_t i = 0; i < graph.size(); ++i) {
    if (!graph[i]) continue;
    symbolicGraph.emplace_shared<internal::IndexedSymbolicFactor>(*graph[i], i);
  }
  return symbolicGraph;
}

// Sum the dimensions of frontal variables in a symbolic cluster.
size_t frontalDimForSymbolicCluster(
    const SymbolicJunctionTree::sharedNode& cluster,
    const std::map<Key, size_t>& dims) {
  size_t dim = 0;
  for (Key key : cluster->orderedFrontalKeys) {
    auto it = dims.find(key);
    if (it != dims.end()) dim += it->second;
  }
  return dim;
}

KeySet separatorKeysForSymbolicCluster(
    const SymbolicJunctionTree::sharedNode& cluster,
    std::map<const SymbolicJunctionTree::Node*, KeySet>* cache) {
  if (!cluster) return KeySet();
  if (cache) {
    auto* raw = cluster.get();
    auto it = cache->find(raw);
    if (it != cache->end()) return it->second;
  }

  KeySet keys;
  for (const auto& factor : cluster->factors) {
    assert(factor);
    keys.insert(factor->begin(), factor->end());
  }
  for (const auto& child : cluster->children) {
    KeySet childSeparators = separatorKeysForSymbolicCluster(child, cache);
    keys.insert(childSeparators.begin(), childSeparators.end());
  }
  for (Key key : cluster->orderedFrontalKeys) {
    keys.erase(key);
  }

  if (cache) {
    auto result = cache->emplace(cluster.get(), std::move(keys));
    return result.first->second;
  }
  return keys;
}

size_t separatorDimForSymbolicCluster(
    const SymbolicJunctionTree::sharedNode& cluster,
    const std::map<Key, size_t>& dims,
    std::map<const SymbolicJunctionTree::Node*, KeySet>* cache) {
  size_t dim = 0;
  const KeySet separatorKeys = separatorKeysForSymbolicCluster(cluster, cache);
  for (Key key : separatorKeys) {
    auto it = dims.find(key);
    if (it != dims.end()) dim += it->second;
  }
  return dim;
}

size_t totalDimForSymbolicCluster(
    const SymbolicJunctionTree::sharedNode& cluster,
    const std::map<Key, size_t>& dims,
    std::map<const SymbolicJunctionTree::Node*, KeySet>* cache) {
  return frontalDimForSymbolicCluster(cluster, dims) +
         separatorDimForSymbolicCluster(cluster, dims, cache);
}

void accumulateSymbolicStats(
    const SymbolicJunctionTree::sharedNode& cluster,
    const std::map<Key, size_t>& dims,
    std::map<const SymbolicJunctionTree::Node*, KeySet>* cache,
    StructureStats* stats) {
  if (!cluster) return;
  const size_t frontalDim = frontalDimForSymbolicCluster(cluster, dims);
  const size_t separatorDim =
      separatorDimForSymbolicCluster(cluster, dims, cache);
  stats->update(frontalDim, separatorDim, cluster->children.size());
  for (const auto& child : cluster->children) {
    accumulateSymbolicStats(child, dims, cache, stats);
  }
}

// Bottom-up merge of child clusters whose merged total dimension stays below
// a threshold.
void mergeSmallClusters(const SymbolicJunctionTree::sharedNode& cluster,
                        const std::map<Key, size_t>& dims, size_t mergeDimCap) {
  if (!cluster) return;
  for (const auto& child : cluster->children) {
    mergeSmallClusters(child, dims, mergeDimCap);
  }
  if (cluster->children.empty()) return;

  const size_t parentTotalDim =
      totalDimForSymbolicCluster(cluster, dims, nullptr);
  std::vector<bool> merge(cluster->children.size(), false);
  bool any = false;
  for (size_t i = 0; i < cluster->children.size(); ++i) {
    const auto& child = cluster->children[i];
    if (!child) continue;
    const size_t childFrontalDim = frontalDimForSymbolicCluster(child, dims);
    if (childFrontalDim + parentTotalDim < mergeDimCap) {
      merge[i] = true;
      any = true;
    }
  }
  if (any) {
    cluster->mergeChildren(merge);
  }
}

void reportStructure(const SymbolicJunctionTree& junctionTree,
                     const std::map<Key, size_t>& dims, const std::string& name,
                     std::ostream* reportStream) {
  if (!reportStream) return;
  // Accumulate stats using cached separator keys.
  StructureStats stats;
  std::map<const SymbolicJunctionTree::Node*, KeySet> separatorCache;
  for (const auto& rootCluster : junctionTree.roots()) {
    accumulateSymbolicStats(rootCluster, dims, &separatorCache, &stats);
  }
  stats.report(name, reportStream);
}

KeyVector keyVectorFromKeySet(const KeySet& keys) {
  KeyVector result;
  result.reserve(keys.size());
  for (Key key : keys) result.push_back(key);
  return result;
}

std::vector<size_t> factorIndicesForSymbolicCluster(
    const SymbolicJunctionTree::sharedNode& cluster) {
  std::vector<size_t> indices;
  indices.reserve(cluster->factors.size());
  for (const auto& factor : cluster->factors) {
    assert(factor);
    auto indexed =
        std::static_pointer_cast<internal::IndexedSymbolicFactor>(factor);
    indices.push_back(indexed->index_);
  }
  return indices;
}

struct BuiltClique {
  MultifrontalSolver::CliquePtr clique;
  KeyVector separatorKeys;
};

// Build cliques from a symbolic junction tree and wire parent/child metadata.
struct CliqueBuilder {
  const std::map<Key, size_t>& dims;
  const GaussianFactorGraph& graph;
  VectorValues* solution;
  std::vector<MultifrontalSolver::CliquePtr>* cliques;
  std::map<const SymbolicJunctionTree::Node*, KeySet> separatorCache;

  BuiltClique build(const SymbolicJunctionTree::sharedNode& cluster,
                    std::weak_ptr<MultifrontalClique> parent) {
    if (!cluster) return {nullptr, KeyVector()};

    // Gather symbolic metadata for this clique.
    const KeyVector& frontals = cluster->orderedFrontalKeys;
    KeyVector separatorKeys = keyVectorFromKeySet(
        separatorKeysForSymbolicCluster(cluster, &separatorCache));

    // Create the clique node and cache static structure.
    auto clique = std::make_shared<MultifrontalClique>(
        factorIndicesForSymbolicCluster(cluster), parent, frontals,
        separatorKeys, dims, graph, solution);

    // Build children and collect separator keys.
    std::vector<MultifrontalClique::ChildInfo> childInfos;
    childInfos.reserve(cluster->children.size());
    for (const auto& childCluster : cluster->children) {
      BuiltClique child = build(childCluster, clique);
      if (child.clique) {
        childInfos.push_back(MultifrontalClique::ChildInfo{
            child.clique, std::move(child.separatorKeys)});
      }
    }

    // Finalize children metadata and register clique.
    clique->finalize(std::move(childInfos));
    cliques->push_back(clique);
    return {clique, std::move(separatorKeys)};
  }
};

}  // namespace

/* ************************************************************************* */
MultifrontalSolver::MultifrontalSolver(const GaussianFactorGraph& graph,
                                       const Ordering& ordering,
                                       size_t mergeDimCap,
                                       std::ostream* reportStream) {
  // Pre-compute variable dimensions
  dims_ = computeDims(graph);
  for (Key key : ordering) {
    solution_.insert(key, Vector::Zero(dims_.at(key)));
  }

  // Convert to SymbolicFactorGraph to build the elimination tree
  SymbolicFactorGraph symbolicGraph = buildSymbolicGraph(graph);

  // Build SymbolicEliminationTree and then SymbolicJunctionTree
  SymbolicEliminationTree eliminationTree(symbolicGraph, ordering);
  SymbolicJunctionTree junctionTree(eliminationTree);

  // Report the symbolic structure before any merge.
  reportStructure(junctionTree, dims_, "Symbolic cluster structure",
                  reportStream);

  // If applicable, merge small child cliques bottom-up.
  if (mergeDimCap > 0) {
    for (const auto& rootCluster : junctionTree.roots()) {
      mergeSmallClusters(rootCluster, dims_, mergeDimCap);
    }
    reportStructure(junctionTree, dims_, "Clique structure after merge",
                    reportStream);
  }

  // Build the actual MultifrontalClique structure.
  CliqueBuilder builder{dims_, graph, &solution_, &cliques_, {}};
  for (const auto& rootCluster : junctionTree.roots()) {
    if (rootCluster) {
      roots_.push_back(
          builder.build(rootCluster, std::weak_ptr<MultifrontalClique>())
              .clique);
    }
  }

  // Load initial numerical values after the structure is built.
  load(graph);
}

/* ************************************************************************* */
void MultifrontalSolver::load(const GaussianFactorGraph& graph) {
  for (auto& clique : cliques_) {
    clique->fillAb(graph);
  }
}

/* ************************************************************************* */
void MultifrontalSolver::eliminateInPlace() {
  // Parallel elimination uses PostOrderForestParallel, which will be
  // multi-threaded if GTSAM was compiled with TBB.
  struct EliminatePostVisitor {
    void operator()(const CliquePtr& node) const {
      if (node) node->eliminateInPlace();
    }
  };
  EliminatePostVisitor visitorPost;
  TbbOpenMPMixedScope threadLimiter;
  treeTraversal::PostOrderForestParallel(*this, visitorPost, 10);
}

/* ************************************************************************* */
GaussianBayesTree MultifrontalSolver::computeBayesTree() const {
  GaussianBayesTree bayesTree;
  using Clique = GaussianBayesTreeClique;
  using BayesCliquePtr = GaussianBayesTree::sharedClique;

  std::function<void(const CliquePtr&, const BayesCliquePtr&)> buildClique =
      [&](const CliquePtr& clique, const BayesCliquePtr& parent) {
        if (!clique) return;
        auto conditional = clique->conditional();
        auto bayesClique = std::make_shared<Clique>(conditional);
        bayesTree.addClique(bayesClique, parent);
        for (const auto& child : clique->children) {
          buildClique(child, bayesClique);
        }
      };

  for (const auto& root : roots_) {
    buildClique(root, BayesCliquePtr());
  }
  return bayesTree;
}

/* ************************************************************************* */
const VectorValues& MultifrontalSolver::updateSolution() const {
  for (const auto& clique : cliques_) {
    clique->updateSolution();
  }
  return solution_;
}

/* ************************************************************************* */
std::ostream& operator<<(std::ostream& os, const MultifrontalSolver& solver) {
  os << "MultifrontalSolver(roots=" << solver.roots_.size()
     << ", cliques=" << solver.cliques_.size()
     << ", dims=" << solver.dims_.size() << ")\n";

  std::function<void(const MultifrontalSolver::CliquePtr&, int)> dump =
      [&](const MultifrontalSolver::CliquePtr& clique, int depth) {
        if (!clique) return;
        os << std::string(depth * 2, ' ') << *clique << "\n";
        for (const auto& child : clique->children) {
          dump(child, depth + 1);
        }
      };

  for (const auto& root : solver.roots_) {
    dump(root, 0);
  }
  return os;
}

/* ************************************************************************* */
void MultifrontalSolver::print(const std::string& s,
                               const KeyFormatter& keyFormatter) const {
  if (!s.empty()) std::cout << s;
  std::cout << "MultifrontalSolver matrices (cliques=" << cliques_.size()
            << ")\n";
  for (const auto& clique : cliques_) {
    if (!clique) continue;
    clique->print("", keyFormatter);
  }
}

}  // namespace gtsam

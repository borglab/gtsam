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

#include <gtsam/base/FastMap.h>
#include <gtsam/config.h>
#include <gtsam/inference/Key.h>
#include <gtsam/linear/BatchJacobianFactor.h>
#include <gtsam/linear/GaussianBayesTree.h>
#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/MultifrontalClique.h>
#include <gtsam/linear/MultifrontalSolver.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/symbolic/IndexedJunctionTree.h>
#include <gtsam/symbolic/SymbolicJunctionTree.h>

#include <algorithm>
#include <cassert>
#include <functional>
#include <iostream>
#include <ostream>
#include <stdexcept>
#include <string>
#include <thread>

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

constexpr double kConstraintSigmaTol = 1e-12;
constexpr double kConstraintFeasibleTol = 1e-9;

struct PrecomputeScratch {
  std::map<Key, size_t> dims;
  std::unordered_set<Key> fixedKeys;
  std::vector<size_t> rowCounts;
};

PrecomputeScratch precomputeFromGraph(const GaussianFactorGraph& graph) {
  PrecomputeScratch out;
  out.rowCounts.resize(graph.size(), 0);
  for (size_t i = 0; i < graph.size(); ++i) {
    const auto& factor = graph[i];
    if (!factor) continue;

    auto jacobianFactor = std::dynamic_pointer_cast<JacobianFactor>(factor);
    if (!jacobianFactor) {
      auto batchFactor =
          std::dynamic_pointer_cast<BatchJacobianFactorBase>(factor);
      if (!batchFactor) {
        throw MultifrontalSolverNotSupported(
            "only JacobianFactor or BatchJacobianFactor inputs are supported");
      }
      if (auto model = batchFactor->get_model()) {
        if (model->isConstrained()) {
          throw MultifrontalSolverNotSupported(
              "constrained BatchJacobianFactor inputs are not supported");
        }
      }
      out.rowCounts[i] = batchFactor->rows();
      for (auto it = batchFactor->begin(); it != batchFactor->end(); ++it) {
        out.dims[*it] = static_cast<size_t>(batchFactor->getDim(it));
      }
      continue;
    }

    out.rowCounts[i] = jacobianFactor->rows();
    for (auto it = jacobianFactor->begin(); it != jacobianFactor->end(); ++it) {
      out.dims[*it] = jacobianFactor->getDim(it);
    }
    auto model = jacobianFactor->get_model();
    if (!model || !model->isConstrained()) continue;
    // Only accept fully constrained unary factors with zero residuals.
    if (jacobianFactor->size() != 1) {
      throw MultifrontalSolverNotSupported(
          "only unary constrained factors are supported");
    }
    const Vector& sigmas = model->sigmasRef();
    if (!(sigmas.array().abs() <= kConstraintSigmaTol).all()) {
      throw MultifrontalSolverNotSupported(
          "only fully constrained factors are supported");
    }
    if (jacobianFactor->getb().array().abs().maxCoeff() >
        kConstraintFeasibleTol) {
      throw MultifrontalSolverNotSupported(
          "constrained factor is not feasible");
    }
    out.fixedKeys.insert(*jacobianFactor->begin());
  }
  return out;
}

// Sum the dimensions of frontal variables in a symbolic cluster.
size_t frontalDimForSymbolicCluster(
    const SymbolicJunctionTree::sharedNode& cluster,
    const std::map<Key, size_t>& dims) {
  return internal::sumDims(dims, cluster->orderedFrontalKeys);
}

size_t separatorDimForSymbolicCluster(
    const SymbolicJunctionTree::sharedNode& cluster,
    const std::map<Key, size_t>& dims,
    SymbolicJunctionTree::Cluster::KeySetMap* cache) {
  if (!cluster) return 0;
  const KeySet separatorKeys = cluster->separatorKeys(cache);
  return internal::sumDims(dims, separatorKeys);
}

size_t totalDimForSymbolicCluster(
    const SymbolicJunctionTree::sharedNode& cluster,
    const std::map<Key, size_t>& dims,
    SymbolicJunctionTree::Cluster::KeySetMap* cache) {
  return frontalDimForSymbolicCluster(cluster, dims) +
         separatorDimForSymbolicCluster(cluster, dims, cache);
}

struct LeafInfo {
  SymbolicJunctionTree::sharedNode child;
  size_t totalDim = 0;
  size_t index = 0;
};

struct LeafGroup {
  std::vector<LeafInfo> leaves;
  size_t firstIndex = 0;
};

std::vector<LeafGroup> collectLeafGroups(
    const SymbolicJunctionTree::sharedNode& cluster,
    const std::map<Key, size_t>& dims,
    SymbolicJunctionTree::Cluster::KeySetMap* separatorCache) {
  // Group eligible leaf children by identical separators, keeping first-seen
  // order. One-factor leaves remain separate so their numerical load plan can
  // still select the fused direct-batch path.
  FastMap<KeySet, size_t> groupIndexBySeparator;
  std::vector<LeafGroup> groups;

  for (size_t i = 0; i < cluster->children.size(); ++i) {
    const auto& child = cluster->children[i];
    if (!child || !child->children.empty() || child->factors.size() == 1) {
      continue;
    }

    child->separatorKeys(separatorCache);
    const KeySet& separatorKeys = separatorCache->at(child.get());
    auto it = groupIndexBySeparator.find(separatorKeys);
    size_t groupIndex = 0;
    if (it == groupIndexBySeparator.end()) {
      groupIndex = groups.size();
      groupIndexBySeparator.emplace(separatorKeys, groupIndex);
      groups.push_back(LeafGroup{{}, i});
    } else {
      groupIndex = it->second;
    }

    const size_t totalDim =
        totalDimForSymbolicCluster(child, dims, separatorCache);
    groups[groupIndex].leaves.push_back({child, totalDim, i});
  }

  return groups;
}

struct LeafBatch {
  SymbolicJunctionTree::Cluster::Children leaves;
  size_t firstIndex = 0;
};

std::vector<LeafBatch> buildLeafBatches(const std::vector<LeafGroup>& groups,
                                        size_t leafMergeDimCap) {
  // Pack each group into batches capped by total dimension.
  std::vector<LeafBatch> batches;
  for (const auto& group : groups) {
    size_t currentTotalDim = 0;
    LeafBatch batch;
    batch.firstIndex = group.firstIndex;
    for (const auto& leaf : group.leaves) {
      if (!batch.leaves.empty() &&
          currentTotalDim + leaf.totalDim > leafMergeDimCap) {
        batches.push_back(batch);
        batch.leaves.clear();
        currentTotalDim = 0;
        batch.firstIndex = leaf.index;
      }
      if (batch.leaves.empty()) {
        batch.firstIndex = leaf.index;
      }
      batch.leaves.push_back(leaf.child);
      currentTotalDim += leaf.totalDim;
    }
    if (!batch.leaves.empty()) {
      batches.push_back(batch);
    }
  }

  std::sort(batches.begin(), batches.end(),
            [](const LeafBatch& a, const LeafBatch& b) {
              return a.firstIndex < b.firstIndex;
            });
  return batches;
}

struct MergedClusters {
  FastMap<const SymbolicJunctionTree::Cluster*, size_t> indexByChild;
  std::vector<SymbolicJunctionTree::sharedNode> clusters;
  size_t count = 0;
  size_t numSelectedLeaves = 0;
};

MergedClusters buildMergedClusters(const std::vector<LeafBatch>& clusters) {
  MergedClusters merged;
  merged.clusters.assign(clusters.size(), nullptr);

  for (size_t i = 0; i < clusters.size(); ++i) {
    const auto& batch = clusters[i];
    if (batch.leaves.size() <= 1) continue;

    merged.numSelectedLeaves += batch.leaves.size();
    // Map each leaf pointer to its batch index so we can skip and insert in
    // O(1) when scanning the original children list.
    for (const auto& leaf : batch.leaves) {
      if (leaf) merged.indexByChild.emplace(leaf.get(), i);
    }

    // Build the merged batch once so insertion is just pointer replacement.
    auto mergedCluster = std::make_shared<SymbolicJunctionTree::Cluster>();
    for (const auto& leaf : batch.leaves) {
      if (leaf) mergedCluster->merge(leaf);
    }
    // Cluster::merge prepends frontals, so restore the batch's original order.
    std::reverse(mergedCluster->orderedFrontalKeys.begin(),
                 mergedCluster->orderedFrontalKeys.end());
    merged.clusters[i] = mergedCluster;
    merged.count += 1;
  }

  return merged;
}

SymbolicJunctionTree::Cluster::Children buildMergedChildren(
    const SymbolicJunctionTree::Cluster::Children& oldChildren,
    const std::vector<LeafBatch>& batches, const MergedClusters& merged) {
  SymbolicJunctionTree::Cluster::Children newChildren;
  newChildren.reserve(oldChildren.size() - merged.numSelectedLeaves +
                      merged.count);
  std::vector<bool> inserted(batches.size(), false);

  for (size_t i = 0; i < oldChildren.size(); ++i) {
    const auto& child = oldChildren[i];
    if (!child) {
      newChildren.push_back(child);
      continue;
    }

    auto it = merged.indexByChild.find(child.get());
    if (it == merged.indexByChild.end()) {
      newChildren.push_back(child);
      continue;
    }

    const size_t batchIndex = it->second;
    // Insert the merged cluster exactly once, at the first original index where
    // that cluster appeared. All other leaves in the cluster are skipped.
    if (!inserted[batchIndex] && i == batches[batchIndex].firstIndex) {
      newChildren.push_back(merged.clusters[batchIndex]);
      inserted[batchIndex] = true;
    }
  }

  return newChildren;
}

bool mergeLeafBatches(const SymbolicJunctionTree::sharedNode& cluster,
                      const std::vector<LeafBatch>& batches) {
  if (batches.empty()) return false;

  // Merge in a single pass to avoid repeated scans of large child lists.
  const MergedClusters merged = buildMergedClusters(batches);
  if (merged.count == 0) return false;

  auto oldChildren = cluster->children;
  SymbolicJunctionTree::Cluster::Children newChildren =
      buildMergedChildren(oldChildren, batches, merged);
  cluster->children.swap(newChildren);
  return true;
}

void accumulateSymbolicStats(const SymbolicJunctionTree::sharedNode& cluster,
                             const std::map<Key, size_t>& dims,
                             SymbolicJunctionTree::Cluster::KeySetMap* cache,
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

/**
 * @brief Recursively groups eligible leaf children into super-leaves capped by
 * total dimension.
 *
 * This function traverses the symbolic junction tree rooted at the given
 * cluster node. For each cluster, it gathers direct leaf children (i.e.,
 * children with no further descendants) that contain more than one factor and
 * groups those with identical separators into new super-leaves until adding
 * another leaf would exceed the specified leafMergeDimCap.
 *
 * The merging process works as follows:
 * - Recursively process all children clusters first.
 * - Identify eligible direct leaf children and collect their total dimensions
 *   (separator plus frontal), separator keys, and original positions.
 * - Leave one-factor children separate so their numerical load plans can still
 *   select the fused direct-batch path.
 * - Group the remaining children by identical separator keys, preserving
 *   first-seen order for each separator.
 * - Build super-leaves by accumulating children under the dimension cap, then
 *   start a new super-leaf.
 * - Replace grouped children with the new super-leaves. Non-leaf and excluded
 *   children keep their relative order, and each super-leaf appears at the
 *   first original position represented by its group.
 *
 * @param cluster The current symbolic junction-tree node to process.
 * @param dims A map from variable keys to their dimensions.
 * @param leafMergeDimCap The maximum summed dimension for a merged leaf.
 */
void mergeLeafChildren(const SymbolicJunctionTree::sharedNode& cluster,
                       const std::map<Key, size_t>& dims,
                       size_t leafMergeDimCap) {
  if (!cluster || cluster->children.empty()) return;
  for (const auto& child : cluster->children) {
    mergeLeafChildren(child, dims, leafMergeDimCap);
  }

  SymbolicJunctionTree::Cluster::KeySetMap separatorCache;
  const std::vector<LeafGroup> groups =
      collectLeafGroups(cluster, dims, &separatorCache);
  const std::vector<LeafBatch> batches =
      buildLeafBatches(groups, leafMergeDimCap);
  if (!mergeLeafBatches(cluster, batches)) return;
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
    // The child's separator is already contained in the parent's total size.
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
  SymbolicJunctionTree::Cluster::KeySetMap separatorCache;
  for (const auto& rootCluster : junctionTree.roots()) {
    accumulateSymbolicStats(rootCluster, dims, &separatorCache, &stats);
  }
  stats.report(name, reportStream);
}

struct BuiltClique {
  MultifrontalSolver::CliquePtr clique;
  KeySet separatorKeys;
};

// Build cliques from a symbolic junction tree and wire parent/child metadata.
struct CliqueBuilder {
  const std::map<Key, size_t>& dims;
  const FastMap<Key, size_t>& orderingIndex;
  size_t firstPhaseSize;
  VectorValues* solution;
  std::vector<MultifrontalSolver::CliquePtr>* cliques;
  const std::unordered_set<Key>* fixedKeys;
  const MultifrontalParameters* params;
  const std::vector<size_t>* rowCounts;
  SymbolicJunctionTree::Cluster::KeySetMap separatorCache = {};

  BuiltClique build(const SymbolicJunctionTree::sharedNode& cluster,
                    std::weak_ptr<MultifrontalClique> parent) {
    if (!cluster) return {nullptr, KeySet()};

    // Gather symbolic metadata for this clique.
    KeyVector frontals = cluster->orderedFrontalKeys;
    // Symbolic cluster merges preserve membership but can leave frontals in
    // merge order. Restore the requested elimination order so a partial
    // solver's eliminated frontals form the prefix counted below.
    std::stable_sort(frontals.begin(), frontals.end(),
                     [this](Key a, Key b) {
                       return orderingIndex.at(a) < orderingIndex.at(b);
                     });
    size_t numEliminatedFrontals = 0;
    while (numEliminatedFrontals < frontals.size() &&
           orderingIndex.at(frontals[numEliminatedFrontals]) <
               firstPhaseSize) {
      ++numEliminatedFrontals;
    }
    const KeySet& separatorKeys = cluster->separatorKeys(&separatorCache);
    std::vector<size_t> factorIndices;
    factorIndices.reserve(cluster->factors.size());
    size_t vbmRows = 0;
    for (const auto& factor : cluster->factors) {
      assert(factor);
      auto indexed =
          std::static_pointer_cast<internal::IndexedSymbolicFactor>(factor);
      factorIndices.push_back(indexed->index_);
      vbmRows += rowCounts->at(indexed->index_);
    }

    // Create the clique node and cache static structure.
    auto clique = std::make_shared<MultifrontalClique>(
        std::move(factorIndices), parent, frontals, separatorKeys, dims,
        vbmRows, solution, fixedKeys, numEliminatedFrontals);

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
    clique->finalize(std::move(childInfos), *params);
    // Register after recursion so the flat clique list remains in post-order.
    cliques->push_back(clique);
    return {clique, std::move(separatorKeys)};
  }
};

size_t resolveThreadCount(size_t requested) {
  if (requested != 0) return requested;
  unsigned int hardwareThreads = std::thread::hardware_concurrency();
  if (hardwareThreads == 0) hardwareThreads = 1;
  size_t threads = (hardwareThreads * 3) / 4;
  return threads == 0 ? 1 : threads;
}

}  // namespace

/* ************************************************************************* */
MultifrontalSolver::MultifrontalSolver(const GaussianFactorGraph& graph,
                                       const Ordering& ordering,
                                       const Parameters& params)
    : MultifrontalSolver(Precompute(graph, ordering), ordering, ordering.size(),
                         params) {}

/* ************************************************************************* */
MultifrontalSolver::MultifrontalSolver(const GaussianFactorGraph& graph,
                                       const Ordering& ordering,
                                       size_t firstPhaseSize,
                                       const Parameters& params)
    : MultifrontalSolver(Precompute(graph, ordering), ordering, firstPhaseSize,
                         params) {}

/* ************************************************************************* */
MultifrontalSolver::MultifrontalSolver(PrecomputedData data,
                                       const Ordering& ordering,
                                       const Parameters& params)
    : MultifrontalSolver(std::move(data), ordering, ordering.size(), params) {}

/* ************************************************************************* */
MultifrontalSolver::MultifrontalSolver(PrecomputedData data,
                                       const Ordering& ordering,
                                       size_t firstPhaseSize,
                                       const Parameters& params)
    : ForestTraversal<MultifrontalSolver, MultifrontalClique>(
          resolveThreadCount(params.numThreads)),
      ordering_(ordering),
      firstPhaseSize_(firstPhaseSize),
      params_(params) {
  if (firstPhaseSize_ > ordering_.size()) {
    throw std::invalid_argument(
        "MultifrontalSolver: first phase exceeds the ordering size.");
  }
  dims_ = std::move(data.dims);
  fixedKeys_ = std::move(data.fixedKeys);

  // Seed the cached solution with zero vectors for all variables.
  for (Key key : ordering) {
    solution_.insert(key, Vector::Zero(dims_.at(key)));
  }
  for (Key key : fixedKeys_) {
    if (!solution_.exists(key)) {
      solution_.insert(key, Vector::Zero(dims_.at(key)));
    }
  }

  // Report the symbolic structure before any merge.
  reportStructure(data.indexedJunctionTree, dims_, "Symbolic cluster structure",
                  params_.reportStream);

  // Merge compatible leaf children under their separate dimension cap first.
  if (params_.leafMergeDimCap > 0) {
    for (const auto& rootCluster : data.indexedJunctionTree.roots()) {
      mergeLeafChildren(rootCluster, dims_, params_.leafMergeDimCap);
    }
    reportStructure(data.indexedJunctionTree, dims_,
                    "Clique structure after leaf merge", params_.reportStream);
  }

  // If applicable, merge small child cliques bottom-up.
  if (params_.mergeDimCap > 0) {
    for (const auto& rootCluster : data.indexedJunctionTree.roots()) {
      mergeSmallClusters(rootCluster, dims_, params_.mergeDimCap);
    }
    reportStructure(data.indexedJunctionTree, dims_,
                    "Clique structure after merge", params_.reportStream);
  }

  // Build the actual MultifrontalClique structure.
  FastMap<Key, size_t> orderingIndex;
  for (size_t i = 0; i < ordering_.size(); ++i) {
    orderingIndex.emplace(ordering_[i], i);
  }
  CliqueBuilder builder{dims_,       orderingIndex, firstPhaseSize_,
                        &solution_,  &cliques_,  &fixedKeys_,
                        &params_,    &data.rowCounts};
  for (const auto& rootCluster : data.indexedJunctionTree.roots()) {
    if (rootCluster) {
      roots_.push_back(
          builder.build(rootCluster, std::weak_ptr<MultifrontalClique>())
              .clique);
    }
  }

  // Caller is responsible for loading numerical values via load().
}

/* ************************************************************************* */
MultifrontalSolver::PrecomputedData MultifrontalSolver::Precompute(
    const GaussianFactorGraph& graph, const Ordering& ordering) {
  PrecomputeScratch scratch = precomputeFromGraph(graph);

  Ordering reducedOrdering;
  reducedOrdering.reserve(ordering.size());
  // Exact unary constraints remove their keys from symbolic elimination.
  for (Key key : ordering) {
    if (!scratch.fixedKeys.count(key)) {
      reducedOrdering.push_back(key);
    }
  }

  IndexedJunctionTree indexedJunctionTree =
      graph.buildIndexedJunctionTree(reducedOrdering, scratch.fixedKeys);

  return MultifrontalSolver::PrecomputedData{
      std::move(scratch.dims), std::move(scratch.fixedKeys),
      std::move(indexedJunctionTree), std::move(scratch.rowCounts)};
}

/* ************************************************************************* */
void MultifrontalSolver::load(const GaussianFactorGraph& graph) {
  for (auto& clique : cliques_) {
    clique->fillAb(graph);
  }
  loaded_ = true;
  eliminated_ = false;
  partiallyEliminated_ = false;
  hasDeltaError_ = false;
}

/* ************************************************************************* */
void MultifrontalSolver::eliminateInPlace() {
  if (firstPhaseSize_ != ordering_.size()) {
    throw std::runtime_error(
        "MultifrontalSolver::eliminateInPlace: solver is configured for "
        "partial elimination.");
  }
  if (!loaded_) {
    throw std::runtime_error(
        "MultifrontalSolver::eliminateInPlace: load() must be called before "
        "eliminating.");
  }
  eliminated_ = false;
  hasDeltaError_ = false;
  runBottomUp([](MultifrontalClique& node) { node.eliminateInPlace(); },
              params_.eliminationParallelThreshold,
              params_.leafAggregationProblemSize);
  eliminated_ = true;
}

/* ************************************************************************* */
void MultifrontalSolver::eliminateInPlace(const GaussianFactorGraph& graph) {
  if (firstPhaseSize_ != ordering_.size()) {
    throw std::runtime_error(
        "MultifrontalSolver::eliminateInPlace: solver is configured for "
        "partial elimination.");
  }
  // Combine load + eliminate in one post-order traversal to improve locality.
  runBottomUp(
      [&graph](MultifrontalClique& node) {
        node.fillAb(graph);
        node.eliminateInPlace();
      },
      params_.eliminationParallelThreshold,
      params_.leafAggregationProblemSize);
  loaded_ = true;
  eliminated_ = true;
  hasDeltaError_ = false;
}

/* ************************************************************************* */
void MultifrontalSolver::eliminatePartialInPlace() {
  if (!loaded_) {
    throw std::runtime_error(
        "MultifrontalSolver::eliminatePartialInPlace: load() must be called "
        "before eliminating.");
  }
  if (firstPhaseSize_ >= ordering_.size()) {
    throw std::runtime_error(
        "MultifrontalSolver::eliminatePartialInPlace: solver is not "
        "configured with a partial ordering.");
  }
  runBottomUp(
      [](MultifrontalClique& node) {
        node.prepareForElimination();
        if (node.numFrontals() > 0) node.factorize();
      },
      params_.eliminationParallelThreshold,
      params_.leafAggregationProblemSize);
  eliminated_ = false;
  partiallyEliminated_ = true;
  hasDeltaError_ = false;
}

/* ************************************************************************* */
void MultifrontalSolver::eliminatePartialInPlace(
    const GaussianFactorGraph& graph) {
  if (firstPhaseSize_ >= ordering_.size()) {
    throw std::runtime_error(
        "MultifrontalSolver::eliminatePartialInPlace: solver is not "
        "configured with a partial ordering.");
  }
  runBottomUp(
      [&graph](MultifrontalClique& node) {
        node.fillAb(graph);
        node.prepareForElimination();
        if (node.numFrontals() > 0) node.factorize();
      },
      params_.eliminationParallelThreshold,
      params_.leafAggregationProblemSize);
  loaded_ = true;
  eliminated_ = false;
  partiallyEliminated_ = true;
  hasDeltaError_ = false;
}

/* ************************************************************************* */
GaussianFactorGraph MultifrontalSolver::remainingFactorGraph() const {
  if (!partiallyEliminated_) {
    throw std::runtime_error(
        "MultifrontalSolver::remainingFactorGraph requires partial "
        "elimination.");
  }
  GaussianFactorGraph remaining;
  for (const auto& clique : cliques_) {
    if (clique && !clique->fullyEliminated()) {
      remaining.push_back(clique->remainingFactor());
    }
  }
  return remaining;
}

/* ************************************************************************* */
GaussianBayesTree MultifrontalSolver::computeBayesTree() const {
  assert(loaded_ && eliminated_);
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

  if (!fixedKeys_.empty()) {
    // Reintroduce fixed variables as independent constrained conditionals.
    for (Key key : fixedKeys_) {
      auto dimIt = dims_.find(key);
      if (dimIt == dims_.end()) {
        throw std::runtime_error(
            "MultifrontalSolver: fixed key has unknown dimension.");
      }
      const size_t dim = dimIt->second;
      Vector d = Vector::Zero(dim);
      Matrix R = Matrix::Identity(dim, dim);
      auto model = noiseModel::Constrained::All(dim);
      auto conditional =
          std::make_shared<GaussianConditional>(key, d, R, model);
      auto bayesClique = std::make_shared<Clique>(conditional);
      bayesTree.addClique(bayesClique, BayesCliquePtr());
    }
  }
  return bayesTree;
}

/* ************************************************************************* */
const VectorValues& MultifrontalSolver::updateSolution() {
  assert(loaded_ && eliminated_);

  // Back-substitute the solution with a top-down pass through the cliques.
  runTopDown([](MultifrontalClique& node) { node.updateSolution(); },
             params_.solutionParallelThreshold);

  // Enforce constrained keys as zero in the final solution.
  for (Key key : fixedKeys_) {
    solution_.at(key).setZero();
  }

  // Aggregate per-clique linearized errors from the latest solution.
  lastOldError_ = 0.0;
  lastNewError_ = 0.0;
  for (const auto& clique : cliques_) {
    if (!clique) continue;
    lastOldError_ += clique->lastOldError();
    lastNewError_ += clique->lastNewError();
  }

  // Add the constant term once from the root cliques.
  double constantError = 0.0;
  for (const auto& root : roots_) {
    if (!root) continue;
    constantError += root->constantTermError();
  }
  lastOldError_ += constantError;
  lastNewError_ += constantError;

  // Mark delta error as valid for subsequent deltaError() calls.
  hasDeltaError_ = true;
  return solution_;
}

/* ************************************************************************* */
const VectorValues& MultifrontalSolver::updateSolution(
    const VectorValues& retainedSolution) {
  if (!partiallyEliminated_) {
    throw std::runtime_error(
        "MultifrontalSolver::updateSolution(retained) requires partial "
        "elimination.");
  }

  // Seed retained variables before back-substituting the eliminated prefix.
  for (size_t i = firstPhaseSize_; i < ordering_.size(); ++i) {
    const Key key = ordering_[i];
    if (fixedKeys_.count(key)) {
      solution_.at(key).setZero();
    } else {
      solution_.at(key) = retainedSolution.at(key);
    }
  }
  runTopDown([](MultifrontalClique& node) { node.updateSolution(); },
             params_.solutionParallelThreshold);
  hasDeltaError_ = false;
  return solution_;
}

double MultifrontalSolver::deltaError(double* oldError,
                                      double* newError) const {
  if (!hasDeltaError_) {
    throw std::runtime_error(
        "MultifrontalSolver::deltaError requires updateSolution().");
  }
  if (oldError) {
    *oldError = lastOldError_;
  }
  if (newError) {
    *newError = lastNewError_;
  }
  return lastOldError_ - lastNewError_;
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

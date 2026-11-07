/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file GaussianISAM2.cpp
 * Fixed-order Gaussian updates through affected-region elimination and cached
 * Bayes-tree solves for changes to the objective information vector.
 * @author Frank Dellaert
 */

#include <gtsam/linear/GaussianISAM2.h>

#include <gtsam/inference/BayesTree-inst.h>
#include <gtsam/inference/BayesTreeCliqueBase-inst.h>
#include <gtsam/inference/JunctionTree-inst.h>
#include <gtsam/linear/GaussianBayesTree.h>
#include <gtsam/linear/GaussianEliminationTree.h>
#include <gtsam/linear/linearAlgorithms-inst.h>

#include <algorithm>
#include <stdexcept>

namespace gtsam {

template class BayesTreeCliqueBase<GaussianISAM2Clique, GaussianFactorGraph>;
template class BayesTree<GaussianISAM2Clique>;

namespace {

// The generic tree constructors are protected. These local types expose only
// construction; the existing junction-tree machinery does all elimination.
struct IncrementalTree : BayesTree<GaussianISAM2Clique> {};

class IncrementalJunctionTree
    : public JunctionTree<IncrementalTree, GaussianFactorGraph> {
 public:
  explicit IncrementalJunctionTree(const GaussianEliminationTree& tree)
      : JunctionTree<IncrementalTree, GaussianFactorGraph>(tree) {
    // Reuse the Gaussian elimination tree with the incremental clique type.
  }
};

GaussianFactorGraph selectRemovedFactors(
    const GaussianFactorGraph& factors, const FactorIndices& indices) {
  // Check removal indices before the factor store or Bayes tree is modified.
  GaussianFactorGraph removed;
  FactorIndexSet unique;

  for (FactorIndex index : indices) {
    if (index >= factors.size() || !factors[index] || !unique.insert(index).second)
      throw std::invalid_argument("GaussianISAM2: invalid removal index.");
    removed.push_back(factors[index]);
  }
  return removed;
}

void checkNewFactor(const GaussianFactor& factor,
                    FastMap<Key, DenseIndex>* dimensions,
                    FastMap<Key, ptrdiff_t>* referenceChanges) {
  // Establish dimensions once and count each key's surviving factor references.
  KeySet unique;

  for (auto variable = factor.begin(); variable != factor.end(); ++variable) {
    auto known = dimensions->find(*variable);
    const DenseIndex dimension = factor.getDim(variable);
    if (known == dimensions->end() || !unique.insert(*variable).second ||
        dimension <= 0 || (known->second != 0 && known->second != dimension))
      throw std::invalid_argument("GaussianISAM2: incompatible factor variables.");
    known->second = dimension;
    ++(*referenceChanges)[*variable];
  }
}

}  // namespace

void GaussianISAM2Clique::setEliminationResult(
    const GaussianFactorGraph::EliminationResult& eliminationResult) {
  // Preserve the separator contribution for later reuse of this subtree.
  conditional_ = eliminationResult.first;
  cachedFactor_ = eliminationResult.second;
}

GaussianISAM2::GaussianISAM2(
    const Ordering& ordering, const GaussianFactorGraph::Eliminate& eliminate)
    : ordering_(ordering), eliminate_(eliminate) {
  // Require a callable elimination rule and a unique, fixed variable set.
  if (!eliminate_)
    throw std::invalid_argument("GaussianISAM2: empty elimination function.");

  for (Key key : ordering_) {
    if (!dimensions_.emplace(key, 0).second)
      throw std::invalid_argument("GaussianISAM2: repeated ordering key.");
  }
}

FastMap<Key, DenseIndex> GaussianISAM2::validateUpdate(
    const GaussianFactorGraph& newFactors,
    const GaussianFactorGraph& removedFactors) const {
  // Validate additions against the fixed dimensions before changing any state.
  auto dimensions = dimensions_;
  FastMap<Key, ptrdiff_t> referenceChanges;

  for (const auto& factor : newFactors) {
    if (!factor)
      throw std::invalid_argument("GaussianISAM2: null factor.");
    checkNewFactor(*factor, &dimensions, &referenceChanges);
  }

  // Every ordering key must retain at least one factor after the update.
  for (const auto& factor : removedFactors)
    for (Key key : factor->keys()) --referenceChanges[key];
  if (!initialized_ && referenceChanges.size() != ordering_.size())
    throw std::invalid_argument("GaussianISAM2: initial graph must cover ordering.");

  for (const auto& [key, change] : referenceChanges) {
    const auto entry = variableIndex_.find(key);
    const ptrdiff_t previous = entry == variableIndex_.end()
                                   ? 0
                                   : static_cast<ptrdiff_t>(entry->second.size());
    if (previous + change <= 0)
      throw std::invalid_argument("GaussianISAM2: update removes a variable.");
  }
  return dimensions;
}

FactorIndices GaussianISAM2::update(const GaussianFactorGraph& newFactors,
                                  const FactorIndices& removeFactorIndices) {
  // Finish argument checks before mutating factor storage or the Bayes tree.
  if (newFactors.empty() && removeFactorIndices.empty()) return {};

  const auto removed = selectRemovedFactors(factors_, removeFactorIndices);
  auto dimensions = validateUpdate(newFactors, removed);
  KeySet touchedKeys = newFactors.keys();
  const KeySet removedKeys = removed.keys();
  touchedKeys.insert(removedKeys.begin(), removedKeys.end());

  // Keep append-only factor indices and variable incidence lists in sync.
  variableIndex_.remove(removeFactorIndices.begin(), removeFactorIndices.end(),
                        removed);
  for (FactorIndex index : removeFactorIndices) factors_.remove(index);
  const FactorIndices added = factors_.add_factors(newFactors);
  variableIndex_.augment(newFactors, added);
  dimensions_ = std::move(dimensions);

  // Build the complete tree once, then rebuild only affected ancestor paths.
  if (!initialized_) {
    eliminateRegion(factors_, ordering_);
    initialized_ = true;
  } else if (!touchedKeys.empty()) {
    updateTop(touchedKeys);
  }
  return added;
}

GaussianFactorGraph GaussianISAM2::affectedFactors(
    const KeySet& affectedKeys) const {
  // Find incident factors first, then exclude those owned by orphan subtrees.
  FactorIndexSet indices;
  for (Key key : affectedKeys) {
    const auto& incident = variableIndex_[key];
    indices.insert(incident.begin(), incident.end());
  }

  GaussianFactorGraph affected;
  for (FactorIndex index : indices) {
    const auto& factor = factors_[index];
    if (std::all_of(factor->begin(), factor->end(), [&](Key key) {
          return affectedKeys.count(key) != 0;
        }))
      affected.push_back(factor);
  }
  return affected;
}

void GaussianISAM2::updateTop(const KeySet& touchedKeys) {
  // Detach affected ancestor paths while retaining untouched subtree objects.
  GaussianBayesNet removed;
  Cliques orphans;
  removeTop(KeyVector(touchedKeys.begin(), touchedKeys.end()), &removed, &orphans);

  const KeySet affectedKeys = removed.keys();
  GaussianFactorGraph factors = affectedFactors(affectedKeys);

  // Supply cached separator factors and wrappers that reattach the same orphans.
  for (const auto& orphan : orphans) {
    if (orphan->cachedFactor()) factors.push_back(orphan->cachedFactor());
    factors.emplace_shared<BayesTreeOrphanWrapper<GaussianISAM2Clique>>(orphan);
  }

  // Preserve the caller's relative ordering within the affected region.
  Ordering affectedOrdering;
  for (Key key : ordering_)
    if (affectedKeys.count(key)) affectedOrdering.push_back(key);

  eliminateRegion(factors, affectedOrdering);
}

void GaussianISAM2::eliminateRegion(const GaussianFactorGraph& factors,
                                  const Ordering& ordering) {
  // Reuse multifrontal elimination, including standard orphan reattachment.
  if (ordering.empty()) return;

  const GaussianEliminationTree tree(factors, ordering);
  const auto result = IncrementalJunctionTree(tree).eliminate(eliminate_).first;

  // Merge the rebuilt roots and frontal lookup entries into the retained tree.
  roots_.insert(roots_.end(), result->roots().begin(), result->roots().end());
  nodes_.insert(result->nodes().begin(), result->nodes().end());
}

VectorValues GaussianISAM2::optimize() const {
  // Exact back-substitution requires an initial tree for nonempty orderings.
  if (!initialized_ && !ordering_.empty())
    throw std::logic_error("GaussianISAM2: an initial update is required.");

  return internal::linearAlgorithms::optimizeBayesTree(*this);
}

VectorValues GaussianISAM2::optimize(
    const VectorValues& linearTermDelta) const {
  // Validate this query without modifying registered factors or cached cliques.
  if (!initialized_ && !ordering_.empty())
    throw std::logic_error("GaussianISAM2: an initial update is required.");

  // Missing forcing blocks mean zero changes in the original variable coordinates.
  VectorValues complete;
  for (const auto& [key, dimension] : dimensions_)
    complete.insert(key, Vector::Zero(dimension));

  for (const auto& [key, value] : linearTermDelta) {
    auto destination = complete.find(key);
    if (destination == complete.end() ||
        destination->second.size() != value.size() || !value.allFinite())
      throw std::invalid_argument("GaussianISAM2: invalid objective forcing.");
    destination->second = value;
  }

  // With raw conditional operator R and diagonal conditional sigmas D, the
  // constrained inverse is R^-1 D^2 R^-T. Hard rows have D=0: their affine
  // equations retain stored d while the objective forcing changes soft rows.
  const VectorValues rhs =
      internal::linearAlgorithms::transposeBayesTree(*this, std::move(complete));
  return internal::linearAlgorithms::optimizeBayesTree(*this, &rhs, true);
}

}  // namespace gtsam

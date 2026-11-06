/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file GaussianISAM2.h
 * @brief Incremental Gaussian elimination with factor removal.
 */

#pragma once

#include <gtsam/inference/BayesTree.h>
#include <gtsam/inference/BayesTreeCliqueBase.h>
#include <gtsam/inference/VariableIndex.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>

namespace gtsam {

/** A Gaussian clique retaining its outgoing separator factor for updates. */
class GTSAM_EXPORT GaussianISAM2Clique
    : public BayesTreeCliqueBase<GaussianISAM2Clique, GaussianFactorGraph> {
 public:
  using Base = BayesTreeCliqueBase<GaussianISAM2Clique, GaussianFactorGraph>;
  using shared_ptr = std::shared_ptr<GaussianISAM2Clique>;
  using weak_ptr = std::weak_ptr<GaussianISAM2Clique>;

  /** Store both the conditional and the remaining separator factor. */
  void setEliminationResult(
      const GaussianFactorGraph::EliminationResult& eliminationResult);

  /// Return the cached contribution of this subtree to its separator.
  const GaussianFactor::shared_ptr& cachedFactor() const { return cachedFactor_; }

 private:
  GaussianFactor::shared_ptr cachedFactor_;
};

/**
 * Incrementally eliminate a Gaussian graph with a fixed variable ordering.
 *
 * The first update supplies the complete graph. Subsequent updates add and
 * remove factors, retaining unaffected Bayes-tree subtrees. Every updated graph
 * must contain exactly the constructor's keys, with unchanged dimensions.
 * Factors are shared with the caller and must remain immutable while registered.
 * Use a new instance if the variable set changes.
 *
 * Invalid inputs are rejected before modification. If numerical elimination
 * throws, discard this instance; updates do not provide rollback or recovery.
 */
class GTSAM_EXPORT GaussianISAM2 : public BayesTree<GaussianISAM2Clique> {
 public:
  using Base = BayesTree<GaussianISAM2Clique>;

  /** Fix the ordering and elimination rule for the lifetime of this solver. */
  explicit GaussianISAM2(
      const Ordering& ordering,
      const GaussianFactorGraph::Eliminate& eliminate =
          GaussianFactorGraph::EliminationTraitsType::DefaultEliminate);

  /**
   * Add and remove factors, returning the indices assigned to newFactors.
   * Indices append to the factor store and remain valid until removal. A
   * replacement can remove a factor and add its replacement in the same call.
   * Null factors, invalid or repeated removal indices, and changes to the key
   * set or variable dimensions throw std::invalid_argument before modification.
   * An update with no additions or removals performs no elimination.
   */
  FactorIndices update(const GaussianFactorGraph& newFactors,
                       const FactorIndices& removeFactorIndices = {});

  /**
   * Back-substitute through the entire tree to obtain the exact solution.
   * A nonempty ordering requires a successful initial update first.
   */
  VectorValues optimize() const;

 private:
  Ordering ordering_;
  GaussianFactorGraph::Eliminate eliminate_;
  GaussianFactorGraph factors_;
  VariableIndex variableIndex_;
  FastMap<Key, DenseIndex> dimensions_;
  bool initialized_ = false;

  /// @name Internal helpers
  /// @{

  /** Validate an update and return the dimensions established by its factors. */
  FastMap<Key, DenseIndex> validateUpdate(
      const GaussianFactorGraph& newFactors,
      const GaussianFactorGraph& removedFactors) const;

  /** Collect original factors wholly contained in the affected variables. */
  GaussianFactorGraph affectedFactors(const KeySet& affectedKeys) const;

  /** Detach, reconstruct, and re-eliminate the affected part of the tree. */
  void updateTop(const KeySet& touchedKeys);

  /** Eliminate a region and merge its roots and frontal index into this tree. */
  void eliminateRegion(const GaussianFactorGraph& factors,
                       const Ordering& ordering);

  /// @}
};

}  // namespace gtsam

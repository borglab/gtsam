/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file GaussianISAM2.h
 * Fixed-order Gaussian elimination with factor updates and cached RHS solves.
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/inference/BayesTree.h>
#include <gtsam/inference/BayesTreeCliqueBase.h>
#include <gtsam/inference/VariableIndex.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>

namespace gtsam {

/// A Gaussian clique retaining its outgoing separator factor for updates.
class GTSAM_EXPORT GaussianISAM2Clique
    : public BayesTreeCliqueBase<GaussianISAM2Clique, GaussianFactorGraph> {
 public:
  using Base = BayesTreeCliqueBase<GaussianISAM2Clique, GaussianFactorGraph>;
  using shared_ptr = std::shared_ptr<GaussianISAM2Clique>;
  using weak_ptr = std::weak_ptr<GaussianISAM2Clique>;

  /**
   * Store the eliminated conditional and its outgoing separator factor so an
   * unaffected subtree can contribute to a later incremental update.
   */
  void setEliminationResult(
      const GaussianFactorGraph::EliminationResult& eliminationResult);

  /// Return the cached contribution of this subtree to its separator.
  const GaussianFactor::shared_ptr& cachedFactor() const { return cachedFactor_; }

 private:
  /// This subtree's outgoing contribution to its separator variables.
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

  /**
   * Fix the ordering and elimination rule for the lifetime of this solver.
   * Duplicate ordering keys or an empty elimination callback throw
   * std::invalid_argument. The first successful update establishes dimensions.
   */
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
   * Numerical elimination failures propagate; discard the instance afterward.
   */
  FactorIndices update(const GaussianFactorGraph& newFactors,
                       const FactorIndices& removeFactorIndices = {});

  /**
   * Back-substitute through the entire tree to obtain the exact solution.
   * A nonempty ordering requires a successful initial update first; otherwise
   * this method throws std::logic_error.
   */
  VectorValues optimize() const;

  /**
   * Minimize the registered objective minus linearTermDelta.dot(x), subject to
   * the same exact constraints, using only cached triangular solves.
   *
   * linearTermDelta is an additive change to the objective's information
   * vector, in the original variable coordinates. It is relative to the
   * registered factors, not the preceding call. Missing keys mean zero;
   * unknown keys, wrong dimensions, or nonfinite entries throw
   * std::invalid_argument. A nonempty ordering requires a successful initial
   * update first; otherwise this method throws std::logic_error.
   * Exact affine constraint right-hand sides remain unchanged.
   *
   * This query performs no numerical elimination and modifies neither original
   * factors nor cached conditionals/separator factors. Subsequent factor updates
   * therefore still reconstruct the registered reference graph. The caller can
   * apply any desired forcing again after arbitrary additions/removals.
   */
  VectorValues optimize(const VectorValues& linearTermDelta) const;

 private:
  /// Caller-supplied variable order, retained for every affected-region solve.
  Ordering ordering_;
  /// Numerical elimination callback used only when factors change.
  GaussianFactorGraph::Eliminate eliminate_;
  /// Original shared factors; removed entries remain empty to preserve indices.
  GaussianFactorGraph factors_;
  /// Incidence lists for finding original factors in an affected region.
  VariableIndex variableIndex_;
  /// Fixed key dimensions, established by the initial graph.
  FastMap<Key, DenseIndex> dimensions_;
  /// Whether an initial factorization has completed successfully.
  bool initialized_ = false;

  /// @name Internal helpers
  /// @{

  /// Validate an update and return the dimensions established by its factors.
  FastMap<Key, DenseIndex> validateUpdate(
      const GaussianFactorGraph& newFactors,
      const GaussianFactorGraph& removedFactors) const;

  /// Collect original factors wholly contained in the affected variables.
  GaussianFactorGraph affectedFactors(const KeySet& affectedKeys) const;

  /// Detach, reconstruct, and re-eliminate the affected part of the tree.
  void updateTop(const KeySet& touchedKeys);

  /// Eliminate a region and merge its roots and frontal index into this tree.
  void eliminateRegion(const GaussianFactorGraph& factors,
                       const Ordering& ordering);

  /// @}
};

}  // namespace gtsam

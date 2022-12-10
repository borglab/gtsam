/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file HybridEliminationTree.h
 * @date Mar 11, 2022
 * @author Fan Jiang
 */

#pragma once

#include <gtsam/hybrid/HybridBayesNet.h>
#include <gtsam/hybrid/HybridGaussianFactorGraph.h>
#include <gtsam/inference/EliminationTree.h>

namespace gtsam {

/**
 * Elimination Tree type for Hybrid Factor Graphs.
 *
 * @ingroup hybrid
 */
class GTSAM_EXPORT HybridEliminationTree
    : public EliminationTree<HybridBayesNet, HybridGaussianFactorGraph> {
 private:
  friend class ::EliminationTreeTester;

 public:
  typedef EliminationTree<HybridBayesNet, HybridGaussianFactorGraph>
      Base;                                    ///< Base class
  typedef HybridEliminationTree This;          ///< This class
  typedef boost::shared_ptr<This> shared_ptr;  ///< Shared pointer to this class

  /// @name Constructors
  /// @{

  /**
   * Build the elimination tree of a factor graph using pre-computed column
   * structure.
   * @param factorGraph The factor graph for which to build the elimination tree
   * @param structure The set of factors involving each variable.  If this is
   * not precomputed, you can call the Create(const FactorGraph<DERIVEDFACTOR>&)
   * named constructor instead.
   * @return The elimination tree
   */
  HybridEliminationTree(const HybridGaussianFactorGraph& factorGraph,
                        const VariableIndex& structure, const Ordering& order);

  /** Build the elimination tree of a factor graph.  Note that this has to
   * compute the column structure as a VariableIndex, so if you already have
   * this precomputed, use the other constructor instead.
   * @param factorGraph The factor graph for which to build the elimination tree
   */
  HybridEliminationTree(const HybridGaussianFactorGraph& factorGraph,
                        const Ordering& order);

  /// @}

  /** Test whether the tree is equal to another */
  bool equals(const This& other, double tol = 1e-9) const;
};

}  // namespace gtsam

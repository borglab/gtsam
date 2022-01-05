/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file HybridEliminationTree.h
 * @date Mar 29, 2013
 * @author Frank Dellaert
 * @author Richard Roberts
 */

#pragma once

#include <gtsam/hybrid/HybridBayesNet.h>
#include <gtsam/hybrid/HybridFactorGraph.h>
#include <gtsam/inference/EliminationTree.h>

namespace gtsam {

class GTSAM_EXPORT HybridEliminationTree
    : public EliminationTree<HybridBayesNet, HybridFactorGraph> {
 public:
  typedef EliminationTree<HybridBayesNet, HybridFactorGraph>
      Base;                                    ///< Base class
  typedef HybridEliminationTree This;          ///< This class
  typedef boost::shared_ptr<This> shared_ptr;  ///< Shared pointer to this class

  /// Build the elimination tree of factor graph given ordering.
  HybridEliminationTree(const HybridFactorGraph& factorGraph,
                        const VariableIndex& structure, const Ordering& order);

  /// Build the elimination tree of a hybrid factor graph.
  HybridEliminationTree(const HybridFactorGraph& factorGraph,
                        const Ordering& order);

  /// Test whether the tree is equal to another
  bool equals(const This& other, double tol = 1e-9) const;

 private:
  friend class ::EliminationTreeTester;
};

}  // namespace gtsam

/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file HybridJunctionTree.h
 * @date Mar 11, 2022
 * @author Fan Jiang
 */

#pragma once

#include <gtsam/hybrid/HybridBayesTree.h>
#include <gtsam/hybrid/HybridGaussianFactorGraph.h>
#include <gtsam/inference/JunctionTree.h>

namespace gtsam {

// Forward declarations
class HybridEliminationTree;

/**
 * An EliminatableClusterTree, i.e., a set of variable clusters with factors,
 * arranged in a tree, with the additional property that it represents the
 * clique tree associated with a Bayes net.
 *
 * In GTSAM a junction tree is an intermediate data structure in multifrontal
 * variable elimination.  Each node is a cluster of factors, along with a
 * clique of variables that are eliminated all at once. In detail, every node k
 * represents a clique (maximal fully connected subset) of an associated chordal
 * graph, such as a chordal Bayes net resulting from elimination.
 *
 * The difference with the BayesTree is that a JunctionTree stores factors,
 * whereas a BayesTree stores conditionals, that are the product of eliminating
 * the factors in the corresponding JunctionTree cliques.
 *
 * The tree structure and elimination method are exactly analogous to the
 * EliminationTree, except that in the JunctionTree, at each node multiple
 * variables are eliminated at a time.
 *
 * \ingroup Multifrontal
 * \ingroup hybrid
 * \nosubgrouping
 */
class GTSAM_EXPORT HybridJunctionTree
    : public JunctionTree<HybridBayesTree, HybridGaussianFactorGraph> {
  /// Record the elimination tree for use in hybrid elimination.
  HybridEliminationTree etree_;
  /// Store the provided variable index.
  VariableIndex variable_index_;

 public:
  typedef JunctionTree<HybridBayesTree, HybridGaussianFactorGraph>
      Base;                                    ///< Base class
  typedef HybridJunctionTree This;             ///< This class
  typedef boost::shared_ptr<This> shared_ptr;  ///< Shared pointer to this class

  /**
   * Build the elimination tree of a factor graph using precomputed column
   * structure.
   * @param factorGraph The factor graph for which to build the elimination tree
   * @param structure The set of factors involving each variable.  If this is
   * not precomputed, you can call the Create(const FactorGraph<DERIVEDFACTOR>&)
   * named constructor instead.
   * @return The elimination tree
   */
  HybridJunctionTree(const HybridEliminationTree& eliminationTree);

 protected:
  /**
   * @brief Eliminate all the continuous variables from the factor graph.
   *
   * @param function The hybrid elimination function.
   * @param graph The factor graph to eliminate.
   * @param continuous_ordering The ordering of continuous variables.
   * @return std::pair<boost::shared_ptr<HybridBayesTree>,
   * boost::shared_ptr<HybridGaussianFactorGraph>>
   */
  std::pair<boost::shared_ptr<HybridBayesTree>,
            boost::shared_ptr<HybridGaussianFactorGraph>>
  eliminateContinuous(const Eliminate& function,
                      const HybridGaussianFactorGraph& graph,
                      const Ordering& continuous_ordering) const;

  /**
   * @brief Eliminate all the discrete variables in the hybrid factor graph.
   *
   * @param function The hybrid elimination function.
   * @param continuousBayesTree The bayes tree corresponding to
   * the eliminated continuous variables.
   * @param discreteGraph Factor graph of factors containing
   * only discrete variables.
   * @param discrete_ordering The elimination ordering for
   * the discrete variables.
   * @return std::pair<boost::shared_ptr<HybridBayesTree>,
   * boost::shared_ptr<HybridGaussianFactorGraph>>
   */
  std::pair<boost::shared_ptr<HybridBayesTree>,
            boost::shared_ptr<HybridGaussianFactorGraph>>
  eliminateDiscrete(const Eliminate& function,
                    const HybridBayesTree::shared_ptr& continuousBayesTree,
                    const HybridGaussianFactorGraph::shared_ptr& discreteGraph,
                    const Ordering& discrete_ordering) const;

  /**
   * @brief Compute the unnormalized probability P'(X | M, Z)
   * for the factor graph in each leaf of the discrete tree.
   * The discrete decision tree formed as a result is added to the
   * `discreteGraph` for discrete elimination.
   *
   * @param continuousBayesTree The bayes tree corresponding to
   * the eliminated continuous variables.
   * @param discreteGraph Factor graph consisting of factors
   * on discrete variables only.
   * @return boost::shared_ptr<HybridGaussianFactorGraph>
   */
  boost::shared_ptr<HybridGaussianFactorGraph> addProbPrimes(
      const HybridBayesTree::shared_ptr& continuousBayesTree,
      const HybridGaussianFactorGraph::shared_ptr& discreteGraph) const;

 public:
  /**
   * @brief Override the eliminate method so we can
   * perform hybrid elimination correctly.
   *
   * @param function The hybrid elimination function.
   * @return std::pair<boost::shared_ptr<HybridBayesTree>,
   * boost::shared_ptr<HybridGaussianFactorGraph>>
   */
  std::pair<boost::shared_ptr<HybridBayesTree>,
            boost::shared_ptr<HybridGaussianFactorGraph>>
  eliminate(const Eliminate& function) const;
};

}  // namespace gtsam

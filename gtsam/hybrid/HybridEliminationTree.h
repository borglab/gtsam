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
 * The elimination tree helps in elimination by specifying the parents to which
 * the "combined factor" after elimination should be assigned to.
 *
 * In the case of hybrid elimination, we use the elimination tree to store the
 * intermediate results.
 * Concretely, we wish to estimate the unnormalized probability P'(X | M, Z) for
 * each mode assignment M.
 * P'(X | M, Z) can be computed by estimating X* which maximizes the continuous
 * factor graph given the discrete modes, and then computing the unnormalized
 * probability as P(X=X* | M, Z).
 *
 * This is done by eliminating the (continuous) factor graph present at each
 * leaf of the discrete tree formed for the discrete sequence and using the
 * inferred X* to compute the `probPrime`.
 *
 * @ingroup hybrid
 */
class GTSAM_EXPORT HybridEliminationTree
    : public EliminationTree<HybridBayesNet, HybridGaussianFactorGraph> {
 private:
  friend class ::EliminationTreeTester;

  Ordering continuous_ordering_, discrete_ordering_;
  /// Used to store the original factor graph to eliminate
  HybridGaussianFactorGraph graph_;
  /// Store the provided variable index.
  VariableIndex variable_index_;

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

 protected:
  /**
   * @brief Helper method to eliminate continuous variables.
   *
   * If no continuous variables exist, return an empty bayes net
   * and the original graph.
   *
   * @param function Elimination function for hybrid elimination.
   * @return std::pair<boost::shared_ptr<HybridBayesNet>,
   * boost::shared_ptr<HybridGaussianFactorGraph> >
   */
  std::pair<boost::shared_ptr<HybridBayesNet>,
            boost::shared_ptr<HybridGaussianFactorGraph>>
  eliminateContinuous(Eliminate function) const;

  /**
   * @brief Compute the unnormalized probability P'(X | M, Z)
   * for the factor graph in each leaf of the discrete tree.
   * The discrete decision tree formed as a result is added to the
   * `discreteGraph` for discrete elimination.
   *
   * @param continuousBayesNet The bayes nets corresponding to
   * the eliminated continuous variables.
   * @param discreteGraph Factor graph consisting of factors
   * on discrete variables only.
   * @return boost::shared_ptr<HybridGaussianFactorGraph>
   */
  boost::shared_ptr<HybridGaussianFactorGraph> addProbPrimes(
      const HybridBayesNet::shared_ptr& continuousBayesNet,
      const HybridGaussianFactorGraph::shared_ptr& discreteGraph) const;

  /**
   * @brief Helper method to eliminate the discrete variables after the
   * continuous variables have been eliminated.
   *
   * If there are no discrete variables, return an empty bayes net and the
   * discreteGraph which is passed in.
   *
   * @param function Hybrid elimination function
   * @param discreteGraph The factor graph with the factor ϕ(X | M, Z).
   * @return std::pair<boost::shared_ptr<HybridBayesNet>,
   * boost::shared_ptr<HybridGaussianFactorGraph> >
   */
  std::pair<boost::shared_ptr<HybridBayesNet>,
            boost::shared_ptr<HybridGaussianFactorGraph>>
  eliminateDiscrete(
      Eliminate function,
      const HybridGaussianFactorGraph::shared_ptr& discreteGraph) const;

 public:
  /**
   * @brief Override the EliminationTree eliminate method
   * so we can perform hybrid elimination correctly.
   *
   * @param function Hybrid elimination function
   * @return std::pair<boost::shared_ptr<HybridBayesNet>,
   * boost::shared_ptr<HybridGaussianFactorGraph> >
   */
  std::pair<boost::shared_ptr<HybridBayesNet>,
            boost::shared_ptr<HybridGaussianFactorGraph>>
  eliminate(Eliminate function) const;

  Ordering continuousOrdering() const { return continuous_ordering_; }
  Ordering discreteOrdering() const { return discrete_ordering_; }

  /// Store the provided variable index.
  VariableIndex variableIndex() const { return variable_index_; }
  HybridGaussianFactorGraph graph() const { return graph_; }
};

}  // namespace gtsam

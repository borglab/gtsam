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
 * Elimination Tree type for Hybrid
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

  /**
   * @brief Helper method to eliminate continuous variables.
   *
   * If no continuous variables exist, return an empty bayes net
   * and the original graph.
   *
   * @param function Elimination function for hybrid elimination.
   * @return std::pair<boost::shared_ptr<BayesNetType>,
   * boost::shared_ptr<FactorGraphType> >
   */
  std::pair<boost::shared_ptr<BayesNetType>, boost::shared_ptr<FactorGraphType>>
  eliminateContinuous(Eliminate function) const {
    if (continuous_ordering_.size() > 0) {
      This continuous_etree(graph_, variable_index_, continuous_ordering_);
      return continuous_etree.Base::eliminate(function);

    } else {
      BayesNetType::shared_ptr bayesNet = boost::make_shared<BayesNetType>();
      FactorGraphType::shared_ptr discreteGraph =
          boost::make_shared<FactorGraphType>(graph_);
      return std::make_pair(bayesNet, discreteGraph);
    }
  }

  /**
   * @brief Helper method to eliminate the discrete variables after the
   * continuous variables have been eliminated.
   *
   * If there are no discrete variables, return an empty bayes net and the
   * discreteGraph which is passed in.
   *
   * @param function Elimination function
   * @param discreteGraph The factor graph with the factor ϕ(X | M, Z).
   * @return std::pair<boost::shared_ptr<BayesNetType>,
   * boost::shared_ptr<FactorGraphType> >
   */
  std::pair<boost::shared_ptr<BayesNetType>, boost::shared_ptr<FactorGraphType>>
  eliminateDiscrete(Eliminate function,
                    const FactorGraphType::shared_ptr& discreteGraph) const {
    BayesNetType::shared_ptr discreteBayesNet;
    FactorGraphType::shared_ptr finalGraph;
    if (discrete_ordering_.size() > 0) {
      This discrete_etree(*discreteGraph, VariableIndex(*discreteGraph),
                          discrete_ordering_);

      std::tie(discreteBayesNet, finalGraph) =
          discrete_etree.Base::eliminate(function);

    } else {
      discreteBayesNet = boost::make_shared<BayesNetType>();
      finalGraph = discreteGraph;
    }

    return std::make_pair(discreteBayesNet, finalGraph);
  }

  /**
   * @brief Override the EliminationTree eliminate method
   * so we can perform hybrid elimination correctly.
   *
   * @param function
   * @return std::pair<boost::shared_ptr<BayesNetType>,
   * boost::shared_ptr<FactorGraphType> >
   */
  std::pair<boost::shared_ptr<BayesNetType>, boost::shared_ptr<FactorGraphType>>
  eliminate(Eliminate function) const {
    // Perform continuous elimination
    BayesNetType::shared_ptr bayesNet;
    FactorGraphType::shared_ptr discreteGraph;
    std::tie(bayesNet, discreteGraph) = this->eliminateContinuous(function);

    // If we have eliminated continuous variables
    // and have discrete variables to eliminate,
    // then compute P(X | M, Z)
    if (continuous_ordering_.size() > 0 && discrete_ordering_.size() > 0) {
      // Get the last continuous conditional
      // which will have all the discrete keys
      HybridConditional::shared_ptr last_conditional =
          bayesNet->at(bayesNet->size() - 1);
      DiscreteKeys discrete_keys = last_conditional->discreteKeys();

      // DecisionTree for P'(X|M, Z) for all mode sequences M
      const AlgebraicDecisionTree<Key> probPrimeTree =
          graph_.continuousProbPrimes(discrete_keys, bayesNet);

      // Add the model selection factor P(M|Z)
      discreteGraph->add(DecisionTreeFactor(discrete_keys, probPrimeTree));
    }

    // Perform discrete elimination
    BayesNetType::shared_ptr discreteBayesNet;
    FactorGraphType::shared_ptr finalGraph;
    std::tie(discreteBayesNet, finalGraph) =
        eliminateDiscrete(function, discreteGraph);

    // Add the discrete conditionals to the hybrid conditionals
    bayesNet->add(*discreteBayesNet);

    return std::make_pair(bayesNet, finalGraph);
  }
};

}  // namespace gtsam

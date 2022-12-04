/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file HybridEliminationTree.cpp
 * @date Mar 11, 2022
 * @author Fan Jiang
 */

#include <gtsam/hybrid/HybridEliminationTree.h>
#include <gtsam/inference/EliminationTree-inst.h>

namespace gtsam {

// Instantiate base class
template class EliminationTree<HybridBayesNet, HybridGaussianFactorGraph>;

/* ************************************************************************* */
HybridEliminationTree::HybridEliminationTree(
    const HybridGaussianFactorGraph& factorGraph,
    const VariableIndex& structure, const Ordering& order)
    : Base(factorGraph, structure, order),
      graph_(factorGraph),
      variable_index_(structure) {
  // Segregate the continuous and the discrete keys
  std::tie(continuous_ordering_, discrete_ordering_) =
      graph_.separateContinuousDiscreteOrdering(order);
}

/* ************************************************************************* */
HybridEliminationTree::HybridEliminationTree(
    const HybridGaussianFactorGraph& factorGraph, const Ordering& order)
    : Base(factorGraph, order),
      graph_(factorGraph),
      variable_index_(VariableIndex(factorGraph)) {}

/* ************************************************************************* */
bool HybridEliminationTree::equals(const This& other, double tol) const {
  return Base::equals(other, tol);
}

/* ************************************************************************* */
std::pair<boost::shared_ptr<HybridBayesNet>,
          boost::shared_ptr<HybridGaussianFactorGraph>>
HybridEliminationTree::eliminateContinuous(Eliminate function) const {
  if (continuous_ordering_.size() > 0) {
    This continuous_etree(graph_, variable_index_, continuous_ordering_);
    return continuous_etree.Base::eliminate(function);

  } else {
    HybridBayesNet::shared_ptr bayesNet = boost::make_shared<HybridBayesNet>();
    HybridGaussianFactorGraph::shared_ptr discreteGraph =
        boost::make_shared<HybridGaussianFactorGraph>(graph_);
    return std::make_pair(bayesNet, discreteGraph);
  }
}

/* ************************************************************************* */
boost::shared_ptr<HybridGaussianFactorGraph>
HybridEliminationTree::addProbPrimes(
    const HybridBayesNet::shared_ptr& continuousBayesNet,
    const HybridGaussianFactorGraph::shared_ptr& discreteGraph) const {
  if (continuous_ordering_.size() > 0 && discrete_ordering_.size() > 0) {
    // Get the last continuous conditional
    // which will have all the discrete keys
    HybridConditional::shared_ptr last_conditional =
        continuousBayesNet->at(continuousBayesNet->size() - 1);
    DiscreteKeys discrete_keys = last_conditional->discreteKeys();

    // DecisionTree for P'(X|M, Z) for all mode sequences M
    const AlgebraicDecisionTree<Key> probPrimeTree =
        graph_.continuousProbPrimes(discrete_keys, continuousBayesNet);

    // Add the model selection factor P(M|Z)
    discreteGraph->add(DecisionTreeFactor(discrete_keys, probPrimeTree));
  }
  return discreteGraph;
}

/* ************************************************************************* */
std::pair<boost::shared_ptr<HybridBayesNet>,
          boost::shared_ptr<HybridGaussianFactorGraph>>
HybridEliminationTree::eliminateDiscrete(
    Eliminate function,
    const HybridGaussianFactorGraph::shared_ptr& discreteGraph) const {
  HybridBayesNet::shared_ptr discreteBayesNet;
  HybridGaussianFactorGraph::shared_ptr finalGraph;
  if (discrete_ordering_.size() > 0) {
    This discrete_etree(*discreteGraph, VariableIndex(*discreteGraph),
                        discrete_ordering_);

    std::tie(discreteBayesNet, finalGraph) =
        discrete_etree.Base::eliminate(function);

  } else {
    discreteBayesNet = boost::make_shared<HybridBayesNet>();
    finalGraph = discreteGraph;
  }

  return std::make_pair(discreteBayesNet, finalGraph);
}

/* ************************************************************************* */
std::pair<boost::shared_ptr<HybridBayesNet>,
          boost::shared_ptr<HybridGaussianFactorGraph>>
HybridEliminationTree::eliminate(Eliminate function) const {
  // Perform continuous elimination
  HybridBayesNet::shared_ptr bayesNet;
  HybridGaussianFactorGraph::shared_ptr discreteGraph;
  std::tie(bayesNet, discreteGraph) = this->eliminateContinuous(function);

  // If we have eliminated continuous variables
  // and have discrete variables to eliminate,
  // then compute P(X | M, Z)
  HybridGaussianFactorGraph::shared_ptr updatedDiscreteGraph =
      addProbPrimes(bayesNet, discreteGraph);

  // Perform discrete elimination
  HybridBayesNet::shared_ptr discreteBayesNet;
  HybridGaussianFactorGraph::shared_ptr finalGraph;
  std::tie(discreteBayesNet, finalGraph) =
      eliminateDiscrete(function, updatedDiscreteGraph);

  // Add the discrete conditionals to the hybrid conditionals
  bayesNet->add(*discreteBayesNet);

  return std::make_pair(bayesNet, finalGraph);
}

}  // namespace gtsam

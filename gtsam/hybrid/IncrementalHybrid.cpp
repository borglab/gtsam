/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    IncrementalHybrid.cpp
 * @brief   An incremental solver for hybrid factor graphs
 * @author  Fan Jiang
 * @author  Frank Dellaert
 * @date    December 2021
 */

#include <gtsam/hybrid/IncrementalHybrid.h>

#include <algorithm>
#include <unordered_set>

namespace gtsam {

/* ************************************************************************* */
void IncrementalHybrid::update(GaussianHybridFactorGraph graph,
                               const Ordering &ordering,
                               boost::optional<size_t> maxNrLeaves) {
  // Add the necessary conditionals from the previous timestep(s).
  std::tie(graph, hybridBayesNet_) =
      addConditionals(graph, hybridBayesNet_, ordering);

  gttic_(Elimination);
  // Eliminate partially.
  HybridBayesNet bayesNetFragment;
  auto result = graph.eliminatePartialSequential(ordering);
  bayesNetFragment = *result.first;
  remainingFactorGraph_ = *result.second;

  gttoc_(Elimination);

  /// Prune
  // Check if discreteGraph is not empty.
  // Possibly empty if no discrete variables.
  if (maxNrLeaves && !remainingFactorGraph_.discreteGraph().empty()) {
    // First we prune the discrete probability tree (tree with doubles at
    // leaves). This method computes the threshold based on maxNrLeaves and sets
    // all leaves below the threshold to 0.0.
    auto discreteFactor = boost::dynamic_pointer_cast<DecisionTreeFactor>(
        remainingFactorGraph_.discreteGraph().at(0));
    DecisionTreeFactor prunedDiscreteFactor =
        discreteFactor->prune(*maxNrLeaves);

    // Assign the thresholded tree so we it is accessible from
    // remainingFactorGraph. Imperative :-(
    discreteFactor->root_ = prunedDiscreteFactor.root_;

    // `pruneBayesNet` sets the leaves with 0 in discreteFactor to nullptr in
    // all the conditionals with the same keys in bayesNetFragment.
    HybridBayesNet prunedBayesNetFragment =
        bayesNetFragment.prune(discreteFactor);
    // Set the bayes net fragment to the pruned version
    bayesNetFragment = prunedBayesNetFragment;
  }

  // Add the partial bayes net to the posterior bayes net.
  hybridBayesNet_.push_back<HybridBayesNet>(bayesNetFragment);

  tictoc_print_();
}

/* ************************************************************************* */
std::pair<GaussianHybridFactorGraph, HybridBayesNet>
IncrementalHybrid::addConditionals(
    const GaussianHybridFactorGraph &originalGraph,
    const HybridBayesNet &originalHybridBayesNet,
    const Ordering &ordering) const {
  GaussianHybridFactorGraph graph(originalGraph);
  HybridBayesNet hybridBayesNet(originalHybridBayesNet);

  // If we are not at the first iteration, means we have conditionals to add.
  if (!hybridBayesNet.empty()) {
    // We add all relevant conditional mixtures on the last continuous variable
    // in the previous `hybridBayesNet` to the graph
    std::unordered_set<Key> allKeys(ordering.begin(), ordering.end());

    // Conditionals to remove from the bayes net
    // since the conditional will be updated.
    std::vector<AbstractConditional::shared_ptr> conditionals_to_erase;

    // TODO(Varun) Using a for-range loop doesn't work since some of the
    // conditionals are invalid pointers
    for (size_t i = 0; i < hybridBayesNet.size(); i++) {
      auto conditional = hybridBayesNet.at(i);

      for (auto &key : conditional->frontals()) {
        if (allKeys.find(key) != allKeys.end()) {
          if (auto gf =
                  boost::dynamic_pointer_cast<GaussianMixture>(conditional)) {
            graph.push_back(gf);
            conditionals_to_erase.push_back(conditional);

          } else if (auto df = boost::dynamic_pointer_cast<DiscreteConditional>(
                         conditional)) {
            graph.push_back(df);
            conditionals_to_erase.push_back(conditional);
          }

          break;
        }
      }
    }
    // Remove conditionals at the end so we don't affect the order in the
    // original bayes net.
    for (auto &&conditional : conditionals_to_erase) {
      auto it = find(hybridBayesNet.begin(), hybridBayesNet.end(), conditional);
      hybridBayesNet.erase(it);
    }
  }
  return {graph, hybridBayesNet};
}

/* ************************************************************************* */
GaussianMixture::shared_ptr IncrementalHybrid::gaussianMixture(
    size_t index) const {
  return boost::dynamic_pointer_cast<GaussianMixture>(
      hybridBayesNet_.at(index));
}

/* ************************************************************************* */
const DiscreteFactorGraph &IncrementalHybrid::remainingDiscreteGraph() const {
  return remainingFactorGraph_.discreteGraph();
}

/* ************************************************************************* */
const HybridBayesNet &IncrementalHybrid::hybridBayesNet() const {
  return hybridBayesNet_;
}

/* ************************************************************************* */
const GaussianHybridFactorGraph &IncrementalHybrid::remainingFactorGraph()
    const {
  return remainingFactorGraph_;
}

}  // namespace gtsam

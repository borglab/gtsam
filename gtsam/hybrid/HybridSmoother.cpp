/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    HybridSmoother.cpp
 * @brief   An incremental smoother for hybrid factor graphs
 * @author  Varun Agrawal
 * @date    October 2022
 */

#include <gtsam/hybrid/HybridSmoother.h>

#include <algorithm>
#include <unordered_set>

namespace gtsam {

/* ************************************************************************* */
void HybridSmoother::update(HybridGaussianFactorGraph graph,
                            const Ordering &ordering,
                            boost::optional<size_t> maxNrLeaves) {
  // Add the necessary conditionals from the previous timestep(s).
  std::tie(graph, hybridBayesNet_) =
      addConditionals(graph, hybridBayesNet_, ordering);

  // Eliminate.
  auto bayesNetFragment = graph.eliminateSequential(ordering);

  /// Prune
  if (maxNrLeaves) {
    // `pruneBayesNet` sets the leaves with 0 in discreteFactor to nullptr in
    // all the conditionals with the same keys in bayesNetFragment.
    HybridBayesNet prunedBayesNetFragment =
        bayesNetFragment->prune(*maxNrLeaves);
    // Set the bayes net fragment to the pruned version
    bayesNetFragment =
        boost::make_shared<HybridBayesNet>(prunedBayesNetFragment);
  }

  // Add the partial bayes net to the posterior bayes net.
  hybridBayesNet_.push_back<HybridBayesNet>(*bayesNetFragment);
}

/* ************************************************************************* */
std::pair<HybridGaussianFactorGraph, HybridBayesNet>
HybridSmoother::addConditionals(const HybridGaussianFactorGraph &originalGraph,
                                const HybridBayesNet &originalHybridBayesNet,
                                const Ordering &ordering) const {
  HybridGaussianFactorGraph graph(originalGraph);
  HybridBayesNet hybridBayesNet(originalHybridBayesNet);

  // If we are not at the first iteration, means we have conditionals to add.
  if (!hybridBayesNet.empty()) {
    // We add all relevant conditional mixtures on the last continuous variable
    // in the previous `hybridBayesNet` to the graph

    // Conditionals to remove from the bayes net
    // since the conditional will be updated.
    std::vector<HybridConditional::shared_ptr> conditionals_to_erase;

    // New conditionals to add to the graph
    gtsam::HybridBayesNet newConditionals;

    // NOTE(Varun) Using a for-range loop doesn't work since some of the
    // conditionals are invalid pointers
    for (size_t i = 0; i < hybridBayesNet.size(); i++) {
      auto conditional = hybridBayesNet.at(i);

      for (auto &key : conditional->frontals()) {
        if (std::find(ordering.begin(), ordering.end(), key) !=
            ordering.end()) {
          newConditionals.push_back(conditional);
          conditionals_to_erase.push_back(conditional);

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

    graph.push_back(newConditionals);
    // newConditionals.print("\n\n\nNew Conditionals to add back");
  }
  return {graph, hybridBayesNet};
}

/* ************************************************************************* */
GaussianMixture::shared_ptr HybridSmoother::gaussianMixture(
    size_t index) const {
  return hybridBayesNet_.atMixture(index);
}

/* ************************************************************************* */
const HybridBayesNet &HybridSmoother::hybridBayesNet() const {
  return hybridBayesNet_;
}

}  // namespace gtsam

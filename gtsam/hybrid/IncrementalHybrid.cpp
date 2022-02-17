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
  // if we are not at the first iteration
  if (!hybridBayesNet_.empty()) {
    // We add all relevant conditional mixtures on the last continuous variable
    // in the previous `hybridBayesNet` to the graph
    std::unordered_set<Key> allVars(ordering.begin(), ordering.end());
    for (auto &&conditional : hybridBayesNet_) {
      for (auto &key : conditional->frontals()) {
        if (allVars.find(key) != allVars.end()) {
          if (auto gf =
                  boost::dynamic_pointer_cast<GaussianMixture>(conditional)) {
            graph.push_back(gf);
          } else if (auto df = boost::dynamic_pointer_cast<DiscreteConditional>(
                         conditional)) {
            graph.push_back(df);
          }
          break;
        }
      }
    }
  }

  // Eliminate partially.
  HybridBayesNet::shared_ptr bayesNetFragment;
  std::tie(bayesNetFragment, remainingFactorGraph_) =
      graph.eliminatePartialSequential(ordering);

  // Add the partial bayes net to the posterior bayes net.
  hybridBayesNet_.push_back<HybridBayesNet>(*bayesNetFragment);

  // Prune
  if (maxNrLeaves) {
    const auto N = *maxNrLeaves;

    const auto lastDensity =
        boost::dynamic_pointer_cast<GaussianMixture>(hybridBayesNet_.back());

    auto discreteFactor = boost::dynamic_pointer_cast<DecisionTreeFactor>(
        remainingFactorGraph_->discreteGraph().at(0));

    // Let's assume that the structure of the last discrete density will be the
    // same as the last continuous
    std::vector<double> probabilities;
    // TODO(fan): The number of probabilities can be lower than the actual
    // number of choices
    discreteFactor->visit(
        [&](const double &prob) { probabilities.emplace_back(prob); });

    if (probabilities.size() < N) return;

    std::nth_element(probabilities.begin(), probabilities.begin() + N,
                     probabilities.end(), std::greater<double>{});

    auto thresholdValue = probabilities[N - 1];

    // Now threshold
    auto threshold = [thresholdValue](const double &value) {
      return value < thresholdValue ? 0.0 : value;
    };
    DecisionTree<Key, double> thresholded(*discreteFactor, threshold);

    // Create a new factor with pruned tree
    // DecisionTreeFactor newFactor(discreteFactor->discreteKeys(),
    // thresholded);
    discreteFactor->root_ = thresholded.root_;

    std::vector<std::pair<DiscreteValues, double>> assignments =
        discreteFactor->enumerate();

    // Loop over all assignments and create a vector of GaussianConditionals
    std::vector<GaussianFactor::shared_ptr> prunedConditionals;
    for (auto &&av : assignments) {
      const DiscreteValues &assignment = av.first;
      const double value = av.second;

      if (value == 0.0) {
        prunedConditionals.emplace_back(nullptr);
      } else {
        prunedConditionals.emplace_back(lastDensity->operator()(assignment));
      }
    }

    GaussianMixture::Factors prunedConditionalsTree(lastDensity->discreteKeys(),
                                                    prunedConditionals);

    hybridBayesNet_.atGaussian(hybridBayesNet_.size() - 1)->factors_ =
        prunedConditionalsTree;
  }
}

/* ************************************************************************* */
GaussianMixture::shared_ptr IncrementalHybrid::gaussianMixture(
    size_t index) const {
  return boost::dynamic_pointer_cast<GaussianMixture>(
      hybridBayesNet_.at(index));
}

/* ************************************************************************* */
const DiscreteFactorGraph &IncrementalHybrid::remainingDiscreteGraph() const {
  return remainingFactorGraph_->discreteGraph();
}

/* ************************************************************************* */
const HybridBayesNet &IncrementalHybrid::hybridBayesNet() const {
  return hybridBayesNet_;
}

/* ************************************************************************* */
GaussianHybridFactorGraph::shared_ptr IncrementalHybrid::remainingFactorGraph()
    const {
  return remainingFactorGraph_;
}

}  // namespace gtsam

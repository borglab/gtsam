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

    // Conditionals to remove from the bayes net
    // since the conditional will be updated.
    std::vector<AbstractConditional::shared_ptr> conditionals_to_erase;

    // TODO(Varun) Using a for-range loop doesn't work since some of the
    // conditionals are invalid pointers
    for (size_t i = 0; i < hybridBayesNet_.size(); i++) {
      auto conditional = hybridBayesNet_.at(i);

      for (auto &key : conditional->frontals()) {
        if (allVars.find(key) != allVars.end()) {
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
      auto it =
          find(hybridBayesNet_.begin(), hybridBayesNet_.end(), conditional);
      hybridBayesNet_.erase(it);
    }
  }

  gttic_(Elimination);
  // Eliminate partially.
  HybridBayesNet::shared_ptr bayesNetFragment;
  auto result = graph.eliminatePartialSequential(ordering);
  bayesNetFragment = result.first;
  remainingFactorGraph_ = *result.second;

  gttoc_(Elimination);

  // Prune
  if (maxNrLeaves) {
    DecisionTreeFactor::shared_ptr discreteFactor = prune(*maxNrLeaves);

    if (!discreteFactor) return;

    std::vector<std::pair<DiscreteValues, double>> assignments =
        discreteFactor->enumerate();

    // Go through all the conditionals in the
    // bayesNetFragment and prune them as well.
    for (auto &&conditional : *bayesNetFragment) {
      auto gaussianMixture =
          boost::dynamic_pointer_cast<GaussianMixture>(conditional);

      // Loop over all assignments and create a vector of GaussianConditionals
      std::vector<GaussianFactor::shared_ptr> prunedConditionals;
      for (auto &&av : assignments) {
        const DiscreteValues &assignment = av.first;
        const double value = av.second;

        if (value == -1.0) {
          prunedConditionals.emplace_back(nullptr);
        } else {
          prunedConditionals.emplace_back(
              gaussianMixture->operator()(assignment));
        }
      }

      GaussianMixture::Factors prunedConditionalsTree(
          gaussianMixture->discreteKeys(), prunedConditionals);
      gaussianMixture->factors_ = prunedConditionalsTree;
    }
  }

  // Add the partial bayes net to the posterior bayes net.
  hybridBayesNet_.push_back<HybridBayesNet>(*bayesNetFragment);

  tictoc_print_();
}

DecisionTreeFactor::shared_ptr IncrementalHybrid::prune(size_t maxNrLeaves) {
  std::cout << "------------------ Perform pruning!!!!" << std::endl;
  std::cout << maxNrLeaves << "  ||  "
            << remainingFactorGraph_.discreteGraph().empty() << std::endl;
  const auto N = maxNrLeaves;

  // Check if discreteGraph is empty. Possible if no discrete variables.
  if (remainingFactorGraph_.discreteGraph().empty()) return nullptr;

  auto discreteFactor = boost::dynamic_pointer_cast<DecisionTreeFactor>(
      remainingFactorGraph_.discreteGraph().at(0));

  // Let's assume that the structure of the last discrete density will be the
  // same as the last continuous
  std::vector<double> probabilities;
  // number of choices
  discreteFactor->visit(
      [&](const double &prob) { probabilities.emplace_back(prob); });

  std::cout << "probabilities.size(): " << probabilities.size() << std::endl;
  // The number of probabilities can be lower than max_leaves
  if (probabilities.size() <= N) return discreteFactor;

  std::sort(probabilities.begin(), probabilities.end(), std::greater<double>{});

  double threshold = probabilities[N - 1];

  // Now threshold the decision tree
  size_t total = 0;
  auto thresholdFunc = [threshold, &total, N](const double &value) {
    if (value < threshold || total >= N) {
      return -1.0;
    } else {
      total += 1;
      return value;
    }
  };
  DecisionTree<Key, double> thresholded(*discreteFactor, thresholdFunc);

  // Now we prune the DecisionTree
  DecisionTree<Key, double> pruned(thresholded.prune(
      thresholded.root_, [](const double &x) { return x == -1; }));

  std::cout << "Initial number of leaves: " << discreteFactor->nrLeaves()
            << std::endl;
  std::cout << "Leaves after pruning: " << pruned.nrLeaves() << std::endl;

  // Assign the thresholded tree
  // discreteFactor->root_ = thresholded.root_;
  // discreteFactor->root_ = pruned.root_;
  discreteFactor = boost::make_shared<DecisionTreeFactor>(
      discreteFactor->discreteKeys(), pruned);

  return discreteFactor;
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

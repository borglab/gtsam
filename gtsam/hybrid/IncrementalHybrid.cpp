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

    // If valid pruned discrete factor, then propagate to gaussian mixtures
    if (discreteFactor) {
      // To Prune, we visitWith every leaf in the GaussianMixture. For each
      // leaf, we apply an operation, where using the assignment, we can
      // check the discrete decision tree for an exception and if yes, then just
      // set the leaf to a nullptr. We can later check the GaussianMixture for
      // just nullptrs.

      discreteFactor->print();

      // Go through all the conditionals in the
      // bayesNetFragment and prune them as well.
      for (auto &&conditional : *bayesNetFragment) {
        auto gaussianMixture =
            boost::dynamic_pointer_cast<GaussianMixture>(conditional);

        // Loop over all assignments and create a set of GaussianConditionals
        auto pruner = [&](const Assignment<Key> &choices,
                          const GaussianFactor::shared_ptr &gf) {
          // typecast so we can use this below
          DiscreteValues values(choices);

          try {
            if ((*discreteFactor)(values) == -1.0) {
              (*gaussianMixture)(values) = nullptr;
            }
          } catch (std::exception &e) {
            // assignment not present so we continue
          }
        };

        if (auto gf =
                boost::dynamic_pointer_cast<GaussianMixture>(conditional)) {
          gf->factors_.visitWith(pruner);
        }
      }
    }
  }

  // Add the partial bayes net to the posterior bayes net.
  hybridBayesNet_.push_back<HybridBayesNet>(*bayesNetFragment);

  tictoc_print_();
}

DecisionTreeFactor::shared_ptr IncrementalHybrid::prune(size_t maxNrLeaves) {
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

  std::cout << "Initial number of leaves: " << discreteFactor->nrLeaves()
            << std::endl;

  // Assign the thresholded tree. Imperative :-(
  discreteFactor->root_ = thresholded.root_;

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

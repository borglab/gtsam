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
#include <unordered_set>
#include <algorithm>

void gtsam::IncrementalHybrid::update(gtsam::GaussianHybridFactorGraph graph,
                                      const gtsam::Ordering &ordering,
                                      boost::optional<size_t> maxNrLeaves) {
  // if we are not at the first iteration
  if (hybridBayesNet_) {
    // We add all relevant densities in the previous `hybridBayesNet` to the graph
    std::unordered_set<Key> allVars(ordering.begin(), ordering.end());
    for (auto &density : *hybridBayesNet_) {
      for (auto &key : density->frontals()) {
        if (allVars.find(key) != allVars.end()) {
          graph.push_back(density);
          break;
        }
      }
    }
  }

  // Eliminate partially.
  std::tie(hybridBayesNet_, remainingFactorGraph_) =
      graph.eliminatePartialSequential(ordering);

  // Prune
  if (maxNrLeaves) {
    const auto N = *maxNrLeaves;
    const auto lastKey = *ordering.rbegin();

    const auto lastDensity =
        std::find_if(hybridBayesNet_->begin(),
                     hybridBayesNet_->end(),
                     [&lastKey](const GaussianMixture::shared_ptr &p) {
                       if (std::find(p->frontals().begin(),
                                     p->frontals().end(),
                                     lastKey)
                           != p->frontals().end()) {
                         return true;
                       } else {
                         return false;
                       }
                     });

    auto discreteFactor_m1 = boost::dynamic_pointer_cast<DecisionTreeFactor>(
        remainingFactorGraph_->discreteGraph().at(0));
    if (lastDensity != hybridBayesNet_->end()) {
      // Let's assume that the structure of the last discrete density will be the same as the last continuous
      // TODO(fan): make a `zip` instead
      using ProbPair = std::pair<double, Assignment<Key>>;
      std::vector<ProbPair> probabilities;
      size_t ord = 0;
      discreteFactor_m1->visitWith([&](const Assignment<Key> &values, const double &prob) {
        probabilities.emplace_back(prob, values);
        ord += 1;
      });
      std::nth_element(probabilities.begin(),
                       probabilities.begin() + N,
                       probabilities.end(),
                       std::greater<ProbPair>{});

      (*lastDensity)->factors().visit([&](const GaussianFactor::shared_ptr &p) {
        // TODO(fan): how to remove stuff??
      });
    } else {
      throw std::out_of_range("We cannot find the last eliminated density");
    }
  }
}

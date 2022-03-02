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
#include <gtsam/base/utilities.h>

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
      std::cout << "Removing "; conditional->print("");
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

  // Add the partial bayes net to the posterior bayes net.
  hybridBayesNet_.push_back<HybridBayesNet>(*bayesNetFragment);

  // Prune
  if (maxNrLeaves) {
    const auto N = *maxNrLeaves;

    // Check if discreteGraph is empty. Possible if no discrete variables.
    if (remainingFactorGraph_.discreteGraph().empty()) return;

    const auto lastDensity =
        boost::dynamic_pointer_cast<GaussianMixture>(hybridBayesNet_.back());

    auto discreteFactor = boost::dynamic_pointer_cast<DecisionTreeFactor>(
        remainingFactorGraph_.discreteGraph().at(0));

    std::cout << "Initial number of leaves: " << discreteFactor->nrLeaves() << std::endl;

    // Let's assume that the structure of the last discrete density will be the
    // same as the last continuous
    std::vector<double> probabilities;
    // TODO(fan): The number of probabilities can be lower than the actual
    // number of choices
    discreteFactor->visit(
        [&](const double &prob) { probabilities.emplace_back(prob); });

    if (probabilities.size() <= N) return;

    std::nth_element(probabilities.begin(), probabilities.begin() + N,
                     probabilities.end(), std::greater<double>{});

    double threshold = probabilities[N];

    // Now threshold the decision tree
    size_t total = 0;
    auto thresholdFunc = [threshold, &total, N](const double &value) {
      if (value < threshold || total > N) {
        return 0.0;
      } else {
        total += 1;
        return value;
      }
    };
    DecisionTree<Key, double> thresholded(*discreteFactor, thresholdFunc);
    size_t nrPrunedLeaves = 0;
    thresholded.visit([&nrPrunedLeaves](const double &d) {
      if (d > 0) nrPrunedLeaves += 1;
    });
    std::cout << "Leaves after pruning: " << nrPrunedLeaves << std::endl;

    // Create a new factor with pruned tree
    // DecisionTreeFactor newFactor(discreteFactor->discreteKeys(),
    // thresholded);
    discreteFactor->root_ = thresholded.root_;

    std::vector<std::pair<DiscreteValues, double>> assignments =
        discreteFactor->enumerate();

    // Loop over all assignments and create a vector of GaussianConditionals
    std::vector<GaussianMixture::shared_ptr> lastClique;
    lastClique.push_back(lastDensity);
    lastClique.push_back(hybridBayesNet_.atGaussian(hybridBayesNet_.size()-2));
    for (auto &p : lastClique) {
      std::vector<GaussianFactor::shared_ptr> prunedConditionals;
      for (auto &&av : assignments) {
        const DiscreteValues &assignment = av.first;
        const double value = av.second;

        if (value == 0.0) {
          prunedConditionals.emplace_back(nullptr);
        } else {
          prunedConditionals.emplace_back(p->operator()(assignment));
        }
      }

      GaussianMixture::Factors prunedConditionalsTree(p->discreteKeys(),
                                                      prunedConditionals);

      p->factors_ =
          prunedConditionalsTree;

      p->factors_.print("", DefaultKeyFormatter, [](GaussianFactor::shared_ptr p){
        RedirectCout rd;
        if (p) p->print();
        return rd.str();
      });
    }
  }
  tictoc_print_();
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

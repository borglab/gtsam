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
#include <unordered_map>

// #define DEBUG_SMOOTHER
namespace gtsam {

/* ************************************************************************* */
void HybridSmoother::reInitialize(HybridBayesNet&& hybridBayesNet) {
  hybridBayesNet_ = std::move(hybridBayesNet);
}

/* ************************************************************************* */
void HybridSmoother::reInitialize(HybridBayesNet& hybridBayesNet) {
  this->reInitialize(std::move(hybridBayesNet));
}

/* ************************************************************************* */
Ordering HybridSmoother::getOrdering(const HybridGaussianFactorGraph& factors,
                                     const KeySet& lastKeysToEliminate) {
  // Get all the discrete keys from the factors
  KeySet allDiscrete = factors.discreteKeySet();

  // Create KeyVector with continuous keys followed by discrete keys.
  KeyVector lastKeys;

  // Insert continuous keys first.
  for (auto& k : lastKeysToEliminate) {
    if (!allDiscrete.exists(k)) {
      lastKeys.push_back(k);
    }
  }

  // Insert discrete keys at the end
  std::copy(allDiscrete.begin(), allDiscrete.end(),
            std::back_inserter(lastKeys));

  // Get an ordering where the new keys are eliminated last
  Ordering ordering = Ordering::ColamdConstrainedLast(
      factors, KeyVector(lastKeys.begin(), lastKeys.end()), true);

  return ordering;
}

/* ************************************************************************* */
Ordering HybridSmoother::maybeComputeOrdering(
    const HybridGaussianFactorGraph& updatedGraph,
    const std::optional<Ordering> givenOrdering) {
  Ordering ordering;
  // If no ordering provided, then we compute one
  if (!givenOrdering.has_value()) {
    // Get the keys from the new factors
    KeySet continuousKeysToInclude;  // Scheme 1: empty, 15sec/2000, 64sec/3000
                                     // (69s without TF)
    // continuousKeysToInclude = newFactors.keys();  // Scheme 2: all,
    // 8sec/2000, 160sec/3000 continuousKeysToInclude = updatedGraph.keys();  //
    // Scheme 3: all, stopped after 80sec/2000

    // Since updatedGraph now has all the connected conditionals,
    // we can get the correct ordering.
    ordering = this->getOrdering(updatedGraph, continuousKeysToInclude);
  } else {
    ordering = *givenOrdering;
  }

  return ordering;
}

/* ************************************************************************* */
HybridGaussianFactorGraph HybridSmoother::removeFixedValues(
    const HybridGaussianFactorGraph& graph,
    const HybridGaussianFactorGraph& newFactors) {
  // Initialize graph
  HybridGaussianFactorGraph updatedGraph(graph);

  for (DiscreteKey dkey : newFactors.discreteKeys()) {
    Key key = dkey.first;
    if (fixedValues_.find(key) != fixedValues_.end()) {
      // Add corresponding discrete factor to reintroduce the information
      std::vector<double> probabilities(
          dkey.second, (1 - *marginalThreshold_) / dkey.second);
      probabilities[fixedValues_[key]] = *marginalThreshold_;
      DiscreteConditional dc(dkey, DiscreteKeys{}, probabilities);
      updatedGraph.push_back(dc);

      // Remove fixed value
      fixedValues_.erase(key);
    }
  }

  return updatedGraph;
}

/* ************************************************************************* */
std::optional<HybridBayesNet> HybridSmoother::eliminateWithoutNoInformation(
    const HybridGaussianFactorGraph& graph) const {
  // We can only choose a single Gaussian graph if every mode is decided.
  for (Key key : graph.discreteKeySet()) {
    if (fixedValues_.count(key) == 0) return {};
  }

  // Check if any hybrid factor is fixed to a component without keys.
  bool dropsComponent = false;
  for (const auto& factor : graph) {
    if (auto hgf = std::dynamic_pointer_cast<HybridGaussianFactor>(factor)) {
      auto component = (*hgf)(fixedValues_).first;
      if (component && component->keys().empty()) {
        dropsComponent = true;
        break;
      }
    }
  }
  if (!dropsComponent) return {};

  // Eliminate the chosen Gaussian graph without the keyless components.
  GaussianFactorGraph chosen;
  for (const auto& factor : graph.choose(fixedValues_)) {
    if (factor && !factor->keys().empty()) chosen.push_back(factor);
  }

  const GaussianBayesNet::shared_ptr gaussianBayesNet =
      chosen.eliminateSequential();
  HybridBayesNet fragment;
  for (const auto& conditional : *gaussianBayesNet) {
    fragment.push_back(std::make_shared<HybridConditional>(conditional));
  }
  return fragment;
}

/* ************************************************************************* */
void HybridSmoother::update(const HybridNonlinearFactorGraph& newFactors,
                            const Values& initial,
                            std::optional<size_t> maxNrLeaves,
                            const std::optional<Ordering> given_ordering) {
  HybridGaussianFactorGraph linearizedFactors = *newFactors.linearize(initial);

  // Record the new nonlinear factors and
  // linearization point for relinearization
  allFactors_.push_back(newFactors);
  linearizationPoint_.insert_or_assign(initial);

  const KeySet originalNewFactorKeys = newFactors.keys();
#ifdef DEBUG_SMOOTHER
  std::cout << "hybridBayesNet_ size before: " << hybridBayesNet_.size()
            << std::endl;
  std::cout << "newFactors size: " << newFactors.size() << std::endl;
#endif
  HybridGaussianFactorGraph updatedGraph;
  // Add the necessary conditionals from the previous timestep(s).
  std::tie(updatedGraph, hybridBayesNet_) =
      addConditionals(linearizedFactors, hybridBayesNet_);
#ifdef DEBUG_SMOOTHER
  // print size of newFactors, updatedGraph, hybridBayesNet_
  std::cout << "updatedGraph size: " << updatedGraph.size() << std::endl;
  std::cout << "hybridBayesNet_ size after: " << hybridBayesNet_.size()
            << std::endl;
  std::cout << "total size: " << updatedGraph.size() + hybridBayesNet_.size()
            << std::endl;
#endif

  if (marginalThreshold_) {
    // Remove fixed values for discrete keys which are introduced in newFactors
    updatedGraph = removeFixedValues(updatedGraph, newFactors);
  }

  Ordering ordering = this->maybeComputeOrdering(updatedGraph, given_ordering);

#if GTSAM_HYBRID_TIMING
  gttic_(HybridSmootherEliminate);
#endif
  // Eliminate.
  HybridBayesNet bayesNetFragment = *updatedGraph.eliminateSequential(ordering);
#if GTSAM_HYBRID_TIMING
  gttoc_(HybridSmootherEliminate);
#endif

#ifdef DEBUG_SMOOTHER_DETAIL
  for (auto conditional : bayesNetFragment) {
    auto e = std::dynamic_pointer_cast<HybridConditional::BaseConditional>(
        conditional);
    GTSAM_PRINT(*e);
  }
#endif

#ifdef DEBUG_SMOOTHER
  // Print discrete keys in the bayesNetFragment:
  std::cout << "Discrete keys in bayesNetFragment: ";
  for (auto& key : HybridFactorGraph(bayesNetFragment).discreteKeySet()) {
    std::cout << DefaultKeyFormatter(key) << " ";
  }
#endif

  /// Prune
  if (maxNrLeaves) {
#if GTSAM_HYBRID_TIMING
    gttic_(HybridSmootherPrune);
#endif
    // `pruneBayesNet` sets the leaves with 0 in discreteFactor to nullptr in
    // all the conditionals with the same keys in bayesNetFragment.
    DiscreteValues newlyFixedValues;
    bayesNetFragment = bayesNetFragment.prune(*maxNrLeaves, marginalThreshold_,
                                              &newlyFixedValues);
    // Insert or assign so it updates existing fixed values
    // instead of replacing them.
    fixedValues_.insert_or_assign(newlyFixedValues);
#if GTSAM_HYBRID_TIMING
    gttoc_(HybridSmootherPrune);
#endif
  }

  // Remove the fill-in from components that were fixed to no information.
  if (auto fragment = eliminateWithoutNoInformation(updatedGraph)) {
    bayesNetFragment = std::move(*fragment);
  }

#ifdef DEBUG_SMOOTHER
  // Print discrete keys in the bayesNetFragment:
  std::cout << "\nAfter pruning: ";
  for (auto& key : HybridFactorGraph(bayesNetFragment).discreteKeySet()) {
    std::cout << DefaultKeyFormatter(key) << " ";
  }
  std::cout << std::endl << std::endl;
#endif

#ifdef DEBUG_SMOOTHER_DETAIL
  for (auto conditional : bayesNetFragment) {
    auto c = std::dynamic_pointer_cast<HybridConditional::BaseConditional>(
        conditional);
    GTSAM_PRINT(*c);
  }
#endif

  // Add the partial bayes net to the posterior bayes net.
  hybridBayesNet_.add(bayesNetFragment);
}

/* ************************************************************************* */
std::pair<HybridGaussianFactorGraph, HybridBayesNet>
HybridSmoother::addConditionals(const HybridGaussianFactorGraph& newFactors,
                                const HybridBayesNet& hybridBayesNet) const {
  HybridGaussianFactorGraph graph(newFactors);

  // If hybridBayesNet is empty, there are no conditionals to add.
  if (hybridBayesNet.empty()) {
    return {graph, hybridBayesNet};
  }

  // Map each frontal key to the index of the conditional that eliminates it.
  std::unordered_map<Key, size_t> frontalToIndex;
  for (size_t i = 0; i < hybridBayesNet.size(); i++) {
    for (Key key : hybridBayesNet.at(i)->frontals()) {
      frontalToIndex[key] = i;
    }
  }

  // Starting from the keys of the new factors, walk up through the parents and
  // mark every conditional that has to be re-eliminated. Unlike a single
  // forward pass, this does not assume the Bayes net is in elimination order.
  std::vector<bool> involved(hybridBayesNet.size(), false);
  const KeySet newKeys = newFactors.keys();
  KeyVector stack(newKeys.begin(), newKeys.end());
  while (!stack.empty()) {
    const Key key = stack.back();
    stack.pop_back();
    auto it = frontalToIndex.find(key);
    if (it == frontalToIndex.end() || involved[it->second]) continue;
    involved[it->second] = true;
    for (Key parentKey : hybridBayesNet.at(it->second)->parents()) {
      stack.push_back(parentKey);
    }
  }

  // Split the Bayes net in a single pass: involved conditionals go into the
  // factor graph and the rest remain in the Bayes net.
  HybridBayesNet updatedHybridBayesNet;
  for (size_t i = 0; i < hybridBayesNet.size(); i++) {
    if (involved[i]) {
      graph.push_back(hybridBayesNet.at(i));
    } else {
      updatedHybridBayesNet.push_back(hybridBayesNet.at(i));
    }
  }

  return {graph, updatedHybridBayesNet};
}

/* ************************************************************************* */
HybridGaussianConditional::shared_ptr HybridSmoother::gaussianMixture(
    size_t index) const {
  return hybridBayesNet_.at(index)->asHybrid();
}

/* ************************************************************************* */
const HybridBayesNet& HybridSmoother::hybridBayesNet() const {
  return hybridBayesNet_;
}

/* ************************************************************************* */
HybridValues HybridSmoother::optimize() const {
  // Solve for the MPE
  DiscreteValues mpe = hybridBayesNet_.mpe();

  // Add fixed values to the MPE.
  mpe.insert(fixedValues_);

  // Given the MPE, compute the optimal continuous values.
  GaussianBayesNet gbn = hybridBayesNet_.choose(mpe);
  const VectorValues continuous = gbn.optimize();
  if (std::find(gbn.begin(), gbn.end(), nullptr) != gbn.end()) {
    throw std::runtime_error("At least one nullptr factor in hybridBayesNet_");
  }
  return HybridValues(continuous, mpe);
}

/* ************************************************************************* */
void HybridSmoother::relinearize(const std::optional<Ordering> givenOrdering) {
  allFactors_ = allFactors_.restrict(fixedValues_);
  HybridGaussianFactorGraph::shared_ptr linearized =
      allFactors_.linearize(linearizationPoint_);

  // Compute ordering if not given
  Ordering ordering = this->maybeComputeOrdering(*linearized, givenOrdering);

  HybridBayesNet::shared_ptr bayesNet =
      linearized->eliminateSequential(ordering);
  HybridValues delta = bayesNet->optimize();
  linearizationPoint_ = linearizationPoint_.retract(delta.continuous());

  reInitialize(*bayesNet);
}

/* ************************************************************************* */
Values HybridSmoother::linearizationPoint() const {
  return linearizationPoint_;
}

/* ************************************************************************* */
HybridNonlinearFactorGraph HybridSmoother::allFactors() const {
  return allFactors_;
}

/* ************************************************************************* */
double HybridSmoother::error(const HybridValues& x) const {
  return hybridBayesNet_.error(x);
}

/* ************************************************************************* */
double HybridSmoother::error(const VectorValues& x) const {
  DiscreteValues discreteAssignment = hybridBayesNet_.mpe();
  discreteAssignment.insert(fixedValues_);
  return hybridBayesNet_.error(HybridValues{x, discreteAssignment});
}

}  // namespace gtsam

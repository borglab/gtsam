/* ----------------------------------------------------------------------------
 * Copyright 2021 The Ambitious Folks of the MRG
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   HybridFactorGraph.cpp
 * @brief  Custom hybrid factor graph for discrete + continuous factors
 * @author Kevin Doherty, kdoherty@mit.edu
 * @date   December 2021
 */

#include <gtsam/hybrid/DCGaussianMixtureFactor.h>
#include <gtsam/hybrid/HybridEliminationTree.h>
#include <gtsam/hybrid/HybridFactorGraph.h>
#include <gtsam/inference/EliminateableFactorGraph-inst.h>
#include <gtsam/linear/HessianFactor.h>

#include <boost/make_shared.hpp>

namespace gtsam {

// Instantiate base classes
// template class FactorGraph<Factor>;
template class EliminateableFactorGraph<HybridFactorGraph>;

void HybridFactorGraph::print(const std::string& str,
                              const gtsam::KeyFormatter& keyFormatter) const {
  std::string prefix = str.empty() ? str : str + ".";
  std::cout << prefix << "size: " << size() << std::endl;
  nonlinearGraph_.print(prefix + "NonlinearFactorGraph", keyFormatter);
  discreteGraph_.print(prefix + "DiscreteFactorGraph", keyFormatter);
  dcGraph_.print(prefix + "DCFactorGraph", keyFormatter);
  gaussianGraph_.print(prefix + "GaussianGraph", keyFormatter);
}

HybridFactorGraph HybridFactorGraph::linearize(
    const Values& continuousValues) const {
  // linearize the continuous factors
  auto gaussianFactorGraph = nonlinearGraph_.linearize(continuousValues);

  // linearize the DCFactors
  DCFactorGraph linearized_DC_factors;
  for (auto&& dcFactor : dcGraph_) {
    // If dcFactor is a DCGaussianMixtureFactor, we don't linearize.
    if (boost::dynamic_pointer_cast<DCGaussianMixtureFactor>(dcFactor)) {
      linearized_DC_factors.push_back(dcFactor);
    } else {
      auto linearizedDCFactor = dcFactor->linearize(continuousValues);
      linearized_DC_factors.push_back(linearizedDCFactor);
    }
  }

  // Add the original factors from the gaussian factor graph
  for (auto&& gaussianFactor : gaussianGraph()) {
    gaussianFactorGraph->push_back(gaussianFactor);
  }

  // Construct new linearized HybridFactorGraph
  return HybridFactorGraph({}, discreteGraph_, linearized_DC_factors,
                           *gaussianFactorGraph);
}

bool HybridFactorGraph::equals(const HybridFactorGraph& other,
                               double tol) const {
  return Base::equals(other, tol) &&
         nonlinearGraph_.equals(other.nonlinearGraph_, tol) &&
         discreteGraph_.equals(other.discreteGraph_, tol) &&
         dcGraph_.equals(other.dcGraph_, tol) &&
         gaussianGraph_.equals(other.gaussianGraph_, tol);
}

void HybridFactorGraph::clear() {
  nonlinearGraph_.resize(0);
  discreteGraph_.resize(0);
  dcGraph_.resize(0);
  gaussianGraph_.resize(0);
}

/// Define adding a GaussianFactor to a sum.
using Sum = DCGaussianMixtureFactor::Sum;
static Sum& operator+=(Sum& sum, const GaussianFactor::shared_ptr& factor) {
  using Y = GaussianFactorGraph;
  auto add = [&factor](const Y& graph) {
    auto result = graph;
    result.push_back(factor);
    return result;
  };
  sum = sum.apply(add);
  return sum;
}

Sum HybridFactorGraph::sum() const {
  if (nrNonlinearFactors()) {
    throw std::runtime_error(
        "HybridFactorGraph::sum cannot handle NonlinearFactors.");
  }

  if (nrDiscreteFactors()) {
    throw std::runtime_error(
        "HybridFactorGraph::sum cannot handle DiscreteFactors.");
  }

  // "sum" all factors, gathering into GaussianFactorGraph
  DCGaussianMixtureFactor::Sum sum;
  for (auto&& dcFactor : dcGraph()) {
    if (auto mixtureFactor =
            boost::dynamic_pointer_cast<DCGaussianMixtureFactor>(dcFactor)) {
      sum += *mixtureFactor;
    } else {
      throw std::runtime_error(
          "HybridFactorGraph::sum can only handleDCGaussianMixtureFactors.");
    }
  }

  // Add the original factors from the gaussian factor graph
  for (auto&& gaussianFactor : gaussianGraph()) {
    sum += gaussianFactor;
  }
  return sum;
}

std::ostream& operator<<(std::ostream& os,
                         const GaussianFactorGraph::EliminationResult& er) {
  os << "ER" << std::endl;
  return os;
}

// The function type that does a single elimination step on a variable.
std::pair<GaussianMixture::shared_ptr, boost::shared_ptr<Factor>>
EliminateHybrid(const HybridFactorGraph& factors, const Ordering& ordering) {
  // Create a new decision tree with all factors gathered at leaves.
  auto sum = factors.sum();

  // Now we need to eliminate each one using conventional Cholesky:
  // We can use this by creating a *new* decision tree:
  using GFG = GaussianFactorGraph;
  using Pair = GaussianFactorGraph::EliminationResult;

  KeyVector keys;
  KeyVector separatorKeys;
  auto eliminate = [&](const GFG& graph) {
    auto result = EliminatePreferCholesky(graph, ordering);
    if (keys.size() == 0) keys = result.first->keys();
    if (separatorKeys.size() == 0) separatorKeys = result.second->keys();
    return result;
  };
  DecisionTree<Key, Pair> eliminationResults(sum, eliminate);

  // Grab the conditionals and create the GaussianMixture
  const DiscreteKeys discreteKeys;  // TODO
  auto first = [](const Pair& result) { return result.first; };
  GaussianMixture::Conditionals conditionals(eliminationResults, first);
  auto conditional =
      boost::make_shared<GaussianMixture>(keys, discreteKeys, conditionals);

  // Create a resulting DCGaussianMixture on the separator.
  const DiscreteKeys separatorDiscreteKeys;  // TODO
  auto second = [](const Pair& result) { return result.second; };
  DCGaussianMixtureFactor::Factors separatorFactors(eliminationResults, second);
  auto factor = boost::make_shared<DCGaussianMixtureFactor>(
      separatorKeys, separatorDiscreteKeys, separatorFactors);

  // Return the result as a pair.
  return {conditional, factor};
}

}  // namespace gtsam

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

#include <gtsam/discrete/DiscreteEliminationTree.h>
#include <gtsam/discrete/DiscreteJunctionTree.h>
#include <gtsam/hybrid/DCGaussianMixtureFactor.h>
#include <gtsam/hybrid/HybridEliminationTree.h>
#include <gtsam/hybrid/HybridFactorGraph.h>
#include <gtsam/inference/EliminateableFactorGraph-inst.h>
#include <gtsam/linear/GaussianEliminationTree.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/GaussianJunctionTree.h>
#include <gtsam/linear/HessianFactor.h>

#include <boost/make_shared.hpp>

using namespace std;

namespace gtsam {

// Instantiate base classes
// template class FactorGraph<Factor>;
template class EliminateableFactorGraph<HybridFactorGraph>;

void HybridFactorGraph::print(const string& str,
                              const gtsam::KeyFormatter& keyFormatter) const {
  string prefix = str.empty() ? str : str + ".";
  cout << prefix << "size: " << size() << endl;
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
  // Base::equals(other, tol) && TODO(fan): this does not work because Factor::equals should be virtual!!!
  return nonlinearGraph_.equals(other.nonlinearGraph_, tol) &&
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

DiscreteKeys HybridFactorGraph::discreteKeys() const {
  DiscreteKeys result;
  // Discrete keys from the discrete graph.
  result = discreteGraph_.discreteKeys();
  // Discrete keys from the DC factor graph.
  auto dcKeys = dcGraph_.discreteKeys();
  for (auto&& key : dcKeys) {
    // Only insert unique keys
    if (std::find(result.begin(), result.end(), key) == result.end()) {
      result.push_back(key);
    }
  }
  return result;
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
  // "sum" all factors, gathering into GaussianFactorGraph
  DCGaussianMixtureFactor::Sum sum;
  for (auto&& dcFactor : dcGraph()) {
    if (auto mixtureFactor =
            boost::dynamic_pointer_cast<DCGaussianMixtureFactor>(dcFactor)) {
      sum += *mixtureFactor;
    } else {
      throw runtime_error(
          "HybridFactorGraph::sum can only handleDCGaussianMixtureFactors.");
    }
  }

  // Add the original factors from the gaussian factor graph
  for (auto&& gaussianFactor : gaussianGraph()) {
    sum += gaussianFactor;
  }
  return sum;
}

DecisionTreeFactor::shared_ptr HybridFactorGraph::toDecisionTreeFactor() const {
  // Get the decision tree mapping an assignment to a GaussianFactorGraph
  Sum sum = this->sum();

  // Get the decision tree with each leaf as the error for that assignment
  auto gfgError = [&](const GaussianFactorGraph& graph) {
    VectorValues values = graph.optimize();
    return graph.probPrime(values);
  };
  DecisionTree<Key, double> gfgdt(sum, gfgError);

  return boost::make_shared<DecisionTreeFactor>(discreteKeys(), gfgdt);
}

ostream& operator<<(ostream& os,
                    const GaussianFactorGraph::EliminationResult& er) {
  os << "ER" << endl;
  return os;
}

// The function type that does a single elimination step on a variable.
pair<GaussianMixture::shared_ptr, boost::shared_ptr<Factor>> EliminateHybrid(
    const HybridFactorGraph& factors, const Ordering& ordering) {
  // STEP 1: SUM
  // Create a new decision tree with all factors gathered at leaves.
  auto sum = factors.sum();

  // STEP 1: ELIMINATE
  // Eliminate each sum using conventional Cholesky:
  // We can use this by creating a *new* decision tree:

  // Each pair is a GaussianConditional and the factor generated after
  // elimination.
  using Pair = GaussianFactorGraph::EliminationResult;

  KeyVector keysOfEliminated; // Not the ordering
  KeyVector keysOfSeparator;  // TODO(frank): Is this just (keys - ordering)?
  auto eliminate = [&](const GaussianFactorGraph& graph) {
    auto result = EliminatePreferCholesky(graph, ordering);
    if (keysOfEliminated.empty()) keysOfEliminated = result.first->keys(); // Initialize the keysOfEliminated to be the keysOfEliminated of the GaussianConditional
    if (keysOfSeparator.empty()) keysOfSeparator = result.second->keys();
    return result;
  };
  DecisionTree<Key, Pair> eliminationResults(sum, eliminate);

  // STEP 3: Create result
  auto pair = unzip(eliminationResults);
  const GaussianMixture::Conditionals& conditionals = pair.first;
  const DCGaussianMixtureFactor::Factors& separatorFactors = pair.second;

  // Create the GaussianMixture from the conditionals
  const size_t nrFrontals = ordering.size();
  const DiscreteKeys discreteKeys = factors.discreteKeys();
  auto conditional =
      boost::make_shared<GaussianMixture>(nrFrontals, keysOfEliminated, discreteKeys, conditionals);

  // If there are no more continuous parents, then we should create here a
  // DiscreteFactor, with the error for each discrete choice.
  if (keysOfSeparator.empty()) {
    VectorValues empty_values;
    auto factorError = [&](const GaussianFactor::shared_ptr& factor) {
      return exp(-factor->error(empty_values));
    };
    DecisionTree<Key, double> fdt(separatorFactors, factorError);
    auto discreteFactor =
        boost::make_shared<DecisionTreeFactor>(factors.discreteKeys(), fdt);

    return {conditional, discreteFactor};

  } else {
    // Create a resulting DCGaussianMixture on the separator.
    auto factor = boost::make_shared<DCGaussianMixtureFactor>(
        keysOfSeparator, discreteKeys, separatorFactors);
    return {conditional, factor};
  }
}

}  // namespace gtsam

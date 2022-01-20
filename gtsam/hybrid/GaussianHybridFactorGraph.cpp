/* ----------------------------------------------------------------------------
 * Copyright 2021 The Ambitious Folks of the MRG
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   GaussianHybridFactorGraph.cpp
 * @brief  Custom hybrid factor graph for discrete + continuous factors
 * @author Kevin Doherty, kdoherty@mit.edu
 * @date   December 2021
 */

#include <gtsam/discrete/DiscreteEliminationTree.h>
#include <gtsam/discrete/DiscreteJunctionTree.h>
#include <gtsam/hybrid/DCGaussianMixtureFactor.h>
#include <gtsam/hybrid/GaussianHybridFactorGraph.h>
#include <gtsam/hybrid/HybridEliminationTree.h>
#include <gtsam/inference/EliminateableFactorGraph-inst.h>
#include <gtsam/linear/GaussianEliminationTree.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/GaussianJunctionTree.h>
#include <gtsam/linear/HessianFactor.h>

#include <boost/make_shared.hpp>

using namespace std;

namespace gtsam {

// Instantiate base classes
template class EliminateableFactorGraph<GaussianHybridFactorGraph>;

void GaussianHybridFactorGraph::print(
    const string& str, const gtsam::KeyFormatter& keyFormatter) const {
  Base::print(str, keyFormatter);
  gaussianGraph_.print("GaussianGraph", keyFormatter);
}

bool GaussianHybridFactorGraph::equals(const GaussianHybridFactorGraph& other,
                                       double tol) const {
  return Base::equals(other, tol) &&
         gaussianGraph_.equals(other.gaussianGraph_, tol);
}

void GaussianHybridFactorGraph::clear() {
  Base::clear();
  gaussianGraph_.resize(0);
}

DiscreteKeys GaussianHybridFactorGraph::discreteKeys() const {
  DiscreteKeys result;
  for (auto&& factor : discreteGraph_) {
    if (auto p = boost::dynamic_pointer_cast<DecisionTreeFactor>(factor)) {
      for (auto&& key : factor->keys()) {
        result.emplace_back(key, p->cardinality(key));
      }
    }
  }
  for (auto&& key : dcGraph_.discreteKeys()) {
    result.push_back(key);
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

Sum GaussianHybridFactorGraph::sum() const {
  // "sum" all factors, gathering into GaussianFactorGraph
  DCGaussianMixtureFactor::Sum sum;
  for (auto&& dcFactor : dcGraph()) {
    if (auto mixtureFactor =
            boost::dynamic_pointer_cast<DCGaussianMixtureFactor>(dcFactor)) {
      sum += *mixtureFactor;
    } else {
      throw runtime_error(
          "GaussianHybridFactorGraph::sum can only "
          "handleDCGaussianMixtureFactors.");
    }
  }

  // Add the original factors from the gaussian factor graph
  for (auto&& gaussianFactor : gaussianGraph()) {
    sum += gaussianFactor;
  }
  return sum;
}

ostream& operator<<(ostream& os,
                    const GaussianFactorGraph::EliminationResult& er) {
  os << "ER" << endl;
  return os;
}

// The function type that does a single elimination step on a variable.
pair<GaussianMixture::shared_ptr, boost::shared_ptr<Factor>> EliminateHybrid(
    const GaussianHybridFactorGraph& factors, const Ordering& ordering) {
  // STEP 1: SUM
  // Create a new decision tree with all factors gathered at leaves.
  Sum sum = factors.sum();

  // STEP 1: ELIMINATE
  // Eliminate each sum using conventional Cholesky:
  // We can use this by creating a *new* decision tree:
  using Pair = GaussianFactorGraph::EliminationResult;

  KeyVector keys;
  KeyVector separatorKeys;  // Do with optional?
  auto eliminate = [&](const GaussianFactorGraph& graph) {
    auto result = EliminatePreferCholesky(graph, ordering);
    if (keys.size() == 0) keys = result.first->keys();
    if (separatorKeys.size() == 0) separatorKeys = result.second->keys();
    return result;
  };
  DecisionTree<Key, Pair> eliminationResults(sum, eliminate);

  // STEP 3: Create result
  auto pair = unzip(eliminationResults);
  const GaussianMixture::Conditionals& conditionals = pair.first;
  const DCGaussianMixtureFactor::Factors& separatorFactors = pair.second;

  const DiscreteKeys discreteKeys = factors.discreteKeys();

  // Create the GaussianMixture from the conditionals
  auto conditional =
      boost::make_shared<GaussianMixture>(keys, discreteKeys, conditionals);

  // If there are no more continuous parents, then we should create here a
  // DiscreteFactor, with the error for each discrete choice.
  if (separatorKeys.size() == 0) {
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
        separatorKeys, discreteKeys, separatorFactors);
    return {conditional, factor};
  }
}

DecisionTreeFactor::shared_ptr GaussianHybridFactorGraph::toDecisionTreeFactor()
    const {
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

}  // namespace gtsam

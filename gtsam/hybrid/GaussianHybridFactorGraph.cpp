/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   GaussianHybridFactorGraph.cpp
 * @brief  Custom hybrid factor graph for discrete + continuous factors
 * @author Varun Agrawal
 * @date   January 2022
 */

#include <gtsam/base/utilities.h>

#include <gtsam/discrete/DiscreteEliminationTree.h>
#include <gtsam/discrete/DiscreteJunctionTree.h>
#include <gtsam/hybrid/DCGaussianMixtureFactor.h>
#include <gtsam/hybrid/GaussianHybridFactorGraph.h>
#include <gtsam/hybrid/HybridEliminationTree.h>
#include <gtsam/inference/EliminateableFactorGraph-inst.h>
#include <gtsam/linear/GaussianEliminationTree.h>
#include <gtsam/linear/GaussianJunctionTree.h>

// Needed for DecisionTree
#include <boost/make_shared.hpp>

using namespace std;

namespace gtsam {

// Instantiate base classes
template class EliminateableFactorGraph<GaussianHybridFactorGraph>;

void GaussianHybridFactorGraph::print(
    const string& str, const gtsam::KeyFormatter& keyFormatter) const {
  std::cout << (str.empty() ? str : str + " ") << std::endl;
  Base::print("", keyFormatter);
  factorGraph_.print("GaussianGraph", keyFormatter);
}

/// Define adding a GaussianFactor to a sum.
using Sum = DCGaussianMixtureFactor::Sum;

// This is the version for Gaussian Factors
static Sum& operator+=(Sum& sum, const GaussianFactor::shared_ptr& factor) {
  using Y = GaussianFactorGraph;
  // If the decision tree is not intiialized, then intialize it.
  if (sum.empty()) {
    GaussianFactorGraph result;
    result.push_back(factor);
    sum = Sum(result);

  } else {
    auto add = [&factor](const Y& graph) {
      auto result = graph;
      result.push_back(factor);
      return result;
    };
    sum = sum.apply(add);
  }
  return sum;
}

Sum GaussianHybridFactorGraph::sum() const {
  gttic_(Sum);
  // "sum" all factors, gathering into GaussianFactorGraph
  DCGaussianMixtureFactor::Sum sum;
  for (auto&& dcFactor : dcGraph()) {
    if (auto mixtureFactor =
            boost::dynamic_pointer_cast<DCGaussianMixtureFactor>(dcFactor)) {
      // This is the += version for mixture factors, different from above!
      sum += *mixtureFactor;
    } else {
      throw runtime_error(
          "GaussianHybridFactorGraph::sum can only handle "
          "DCGaussianMixtureFactors.");
    }
  }

  // Add the original factors from the gaussian factor graph
  for (auto&& gaussianFactor : gaussianGraph()) {
    sum += gaussianFactor;
  }
  return sum;
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

ostream& operator<<(ostream& os,
                    const GaussianFactorGraph::EliminationResult& er) {
  os << "ER" << endl;
  return os;
}

// The function type that does a single elimination step on a variable.
pair<AbstractConditional::shared_ptr, boost::shared_ptr<Factor>>
EliminateHybrid(const GaussianHybridFactorGraph& factors,
                const Ordering& ordering) {

  ordering.print("\nEliminating:");
  // STEP 1: SUM
  // Create a new decision tree with all factors gathered at leaves.
  Sum sum = factors.sum();

  std::cout << "\033[1;32m";
  sum.print("", DefaultKeyFormatter, [](GaussianFactorGraph gfg){
    RedirectCout rd;
    gfg.keys().print("");
    return rd.str();
  });
  std::cout << "\033[0m\n";

  // zero out all sums with null ptrs
  auto zeroOut = [](const GaussianFactorGraph& gfg) {
    bool hasNull =
        std::any_of(gfg.begin(), gfg.end(),
                    [](const GaussianFactor::shared_ptr& ptr) { return !ptr; });

    return hasNull ? GaussianFactorGraph() : gfg;
  };

  // TODO(fan): Now let's assume that all continuous will be eliminated first!
  // Here sum is null if remaining are all discrete
  if (sum.empty()) {
    gttic_(DFG);
    // Not sure if this is the correct thing, but anyway!
    DiscreteFactorGraph dfg;
    dfg.push_back(factors.discreteGraph());

    auto dbn = EliminateForMPE(dfg, ordering);
    auto& df = dbn.first;
    auto& newFactor = dbn.second;
    return {df, newFactor};
  }

  sum = Sum(sum, zeroOut);

  // STEP 1: ELIMINATE
  // Eliminate each sum using conventional Cholesky:
  // We can use this by creating a *new* decision tree:

  // Each pair is a GaussianConditional and the factor generated after
  // elimination.
  using EliminationPair = GaussianFactorGraph::EliminationResult;

  KeyVector keysOfEliminated;  // Not the ordering
  KeyVector keysOfSeparator;   // TODO(frank): Is this just (keys - ordering)?
  auto eliminate = [&](const GaussianFactorGraph& graph)
      -> GaussianFactorGraph::EliminationResult {
      gttic_(Eliminate);
    if (graph.empty()) return {nullptr, nullptr};
    auto result = EliminatePreferCholesky(graph, ordering);
    if (keysOfEliminated.empty())
      keysOfEliminated =
          result.first->keys();  // Initialize the keysOfEliminated to be the
    // keysOfEliminated of the GaussianConditional
    if (keysOfSeparator.empty()) keysOfSeparator = result.second->keys();
    return result;
  };

  auto valueFormatter = [&](const GaussianFactorGraph &v) {
    auto printCapture = [&](const GaussianFactorGraph &p) {
      RedirectCout rd;
      p.print("", DefaultKeyFormatter);
      std::string s = rd.str();
      return s;
    };

    std::string format_template = "Gaussian factor graph with %d factors:\n%s\n";
    return (boost::format(format_template) % v.size() % printCapture(v)).str();
  };
//  sum.print(">>>>>>>>>>>>>\n", DefaultKeyFormatter, valueFormatter);

  gttic_(EliminationResult);
  std::cout << ">>>>>>> nrLeaves in `sum`: " << sum.nrLeaves() << std::endl;
  std::cout << "\033[1;31m";
  sum.print("", DefaultKeyFormatter, [](GaussianFactorGraph gfg){
    RedirectCout rd;
    gfg.keys().print("");
    return rd.str();
  });
  std::cout << "\033[0m\n";
  DecisionTree<Key, EliminationPair> eliminationResults(sum, eliminate);
  // std::cout << "Elimination done!!!!!!!\n\n" << std::endl;
  gttoc_(EliminationResult);

  gttic_(Leftover);
  // STEP 3: Create result
  auto pair = unzip(eliminationResults);

  const GaussianMixture::Conditionals& conditionals = pair.first;
  const DCGaussianMixtureFactor::Factors& separatorFactors = pair.second;

  // Create the GaussianMixture from the conditionals
  const size_t nrFrontals = ordering.size();
  const DiscreteKeys discreteKeys = factors.discreteKeys();
  auto conditional = boost::make_shared<GaussianMixture>(
      nrFrontals, keysOfEliminated, discreteKeys, conditionals);

  // If there are no more continuous parents, then we should create here a
  // DiscreteFactor, with the error for each discrete choice.
  if (keysOfSeparator.empty()) {
    VectorValues empty_values;
    auto factorError = [&](const GaussianFactor::shared_ptr& factor) {
      if (!factor) return 0.0;
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

/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   HybridGaussianFactorGraph.cpp
 * @brief  Hybrid factor graph that uses type erasure
 * @author Fan Jiang
 * @author Varun Agrawal
 * @author Frank Dellaert
 * @date   Mar 11, 2022
 */

#include <gtsam/base/utilities.h>
#include <gtsam/discrete/Assignment.h>
#include <gtsam/discrete/DiscreteEliminationTree.h>
#include <gtsam/discrete/DiscreteFactorGraph.h>
#include <gtsam/discrete/DiscreteJunctionTree.h>
#include <gtsam/hybrid/GaussianMixture.h>
#include <gtsam/hybrid/GaussianMixtureFactor.h>
#include <gtsam/hybrid/HybridConditional.h>
#include <gtsam/hybrid/HybridEliminationTree.h>
#include <gtsam/hybrid/HybridFactor.h>
#include <gtsam/hybrid/HybridGaussianFactorGraph.h>
#include <gtsam/hybrid/HybridJunctionTree.h>
#include <gtsam/inference/EliminateableFactorGraph-inst.h>
#include <gtsam/inference/Key.h>
#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/linear/GaussianEliminationTree.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/GaussianJunctionTree.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/JacobianFactor.h>

#include <algorithm>
#include <cstddef>
#include <iostream>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

// #define HYBRID_TIMING

namespace gtsam {

/// Specialize EliminateableFactorGraph for HybridGaussianFactorGraph:
template class EliminateableFactorGraph<HybridGaussianFactorGraph>;

using OrphanWrapper = BayesTreeOrphanWrapper<HybridBayesTree::Clique>;

using std::dynamic_pointer_cast;

/* ************************************************************************ */
// Throw a runtime exception for method specified in string s, and factor f:
static void throwRuntimeError(const std::string &s,
                              const std::shared_ptr<Factor> &f) {
  auto &fr = *f;
  throw std::runtime_error(s + " not implemented for factor type " +
                           demangle(typeid(fr).name()) + ".");
}

/* ************************************************************************ */
const Ordering HybridOrdering(const HybridGaussianFactorGraph &graph) {
  KeySet discrete_keys = graph.discreteKeySet();
  const VariableIndex index(graph);
  return Ordering::ColamdConstrainedLast(
      index, KeyVector(discrete_keys.begin(), discrete_keys.end()), true);
}

/* ************************************************************************ */
static GaussianFactorGraphTree addGaussian(
    const GaussianFactorGraphTree &gfgTree,
    const GaussianFactor::shared_ptr &factor) {
  // If the decision tree is not initialized, then initialize it.
  if (gfgTree.empty()) {
    GaussianFactorGraph result{factor};
    return GaussianFactorGraphTree(result);
  } else {
    auto add = [&factor](const GaussianFactorGraph &graph) {
      auto result = graph;
      result.push_back(factor);
      return result;
    };
    return gfgTree.apply(add);
  }
}

/* ************************************************************************ */
// TODO(dellaert): it's probably more efficient to first collect the discrete
// keys, and then loop over all assignments to populate a vector.
GaussianFactorGraphTree HybridGaussianFactorGraph::assembleGraphTree() const {
  gttic(assembleGraphTree);

  GaussianFactorGraphTree result;

  for (auto &f : factors_) {
    // TODO(dellaert): just use a virtual method defined in HybridFactor.
    if (auto gf = dynamic_pointer_cast<GaussianFactor>(f)) {
      result = addGaussian(result, gf);
    } else if (auto gmf = dynamic_pointer_cast<GaussianMixtureFactor>(f)) {
      result = gmf->add(result);
    } else if (auto gm = dynamic_pointer_cast<GaussianMixture>(f)) {
      result = gm->add(result);
    } else if (auto hc = dynamic_pointer_cast<HybridConditional>(f)) {
      if (auto gm = hc->asMixture()) {
        result = gm->add(result);
      } else if (auto g = hc->asGaussian()) {
        result = addGaussian(result, g);
      } else {
        // Has to be discrete.
        // TODO(dellaert): in C++20, we can use std::visit.
        continue;
      }
    } else if (dynamic_pointer_cast<DecisionTreeFactor>(f)) {
      // Don't do anything for discrete-only factors
      // since we want to eliminate continuous values only.
      continue;
    } else {
      // TODO(dellaert): there was an unattributed comment here: We need to
      // handle the case where the object is actually an BayesTreeOrphanWrapper!
      throwRuntimeError("gtsam::assembleGraphTree", f);
    }
  }

  gttoc(assembleGraphTree);

  return result;
}

/* ************************************************************************ */
static std::pair<HybridConditional::shared_ptr, std::shared_ptr<Factor>>
continuousElimination(const HybridGaussianFactorGraph &factors,
                      const Ordering &frontalKeys) {
  GaussianFactorGraph gfg;
  for (auto &f : factors) {
    if (auto gf = dynamic_pointer_cast<GaussianFactor>(f)) {
      gfg.push_back(gf);
    } else if (auto orphan = dynamic_pointer_cast<OrphanWrapper>(f)) {
      // Ignore orphaned clique.
      // TODO(dellaert): is this correct? If so explain here.
    } else if (auto hc = dynamic_pointer_cast<HybridConditional>(f)) {
      auto gc = hc->asGaussian();
      if (!gc) throwRuntimeError("continuousElimination", gc);
      gfg.push_back(gc);
    } else {
      throwRuntimeError("continuousElimination", f);
    }
  }

  auto result = EliminatePreferCholesky(gfg, frontalKeys);
  return {std::make_shared<HybridConditional>(result.first), result.second};
}

/* ************************************************************************ */
static std::pair<HybridConditional::shared_ptr, std::shared_ptr<Factor>>
discreteElimination(const HybridGaussianFactorGraph &factors,
                    const Ordering &frontalKeys) {
  DiscreteFactorGraph dfg;

  for (auto &f : factors) {
    if (auto dtf = dynamic_pointer_cast<DecisionTreeFactor>(f)) {
      dfg.push_back(dtf);
    } else if (auto orphan = dynamic_pointer_cast<OrphanWrapper>(f)) {
      // Ignore orphaned clique.
      // TODO(dellaert): is this correct? If so explain here.
    } else if (auto hc = dynamic_pointer_cast<HybridConditional>(f)) {
      auto dc = hc->asDiscrete();
      if (!dc) throwRuntimeError("continuousElimination", dc);
      dfg.push_back(hc->asDiscrete());
    } else {
      throwRuntimeError("continuousElimination", f);
    }
  }

  // NOTE: This does sum-product. For max-product, use EliminateForMPE.
  auto result = EliminateDiscrete(dfg, frontalKeys);

  return {std::make_shared<HybridConditional>(result.first), result.second};
}

/* ************************************************************************ */
// If any GaussianFactorGraph in the decision tree contains a nullptr, convert
// that leaf to an empty GaussianFactorGraph. Needed since the DecisionTree will
// otherwise create a GFG with a single (null) factor, which doesn't register as null.
GaussianFactorGraphTree removeEmpty(const GaussianFactorGraphTree &sum) {
  auto emptyGaussian = [](const GaussianFactorGraph &graph) {
    bool hasNull =
        std::any_of(graph.begin(), graph.end(),
                    [](const GaussianFactor::shared_ptr &ptr) { return !ptr; });
    return hasNull ? GaussianFactorGraph() : graph;
  };
  return GaussianFactorGraphTree(sum, emptyGaussian);
}

/* ************************************************************************ */
static std::pair<HybridConditional::shared_ptr, std::shared_ptr<Factor>>
hybridElimination(const HybridGaussianFactorGraph &factors,
                  const Ordering &frontalKeys,
                  const KeyVector &continuousSeparator,
                  const std::set<DiscreteKey> &discreteSeparatorSet) {
  // NOTE: since we use the special JunctionTree,
  // only possibility is continuous conditioned on discrete.
  DiscreteKeys discreteSeparator(discreteSeparatorSet.begin(),
                                 discreteSeparatorSet.end());

  // Collect all the factors to create a set of Gaussian factor graphs in a
  // decision tree indexed by all discrete keys involved.
  GaussianFactorGraphTree factorGraphTree = factors.assembleGraphTree();

  // Convert factor graphs with a nullptr to an empty factor graph.
  // This is done after assembly since it is non-trivial to keep track of which
  // FG has a nullptr as we're looping over the factors.
  factorGraphTree = removeEmpty(factorGraphTree);

  using Result = std::pair<std::shared_ptr<GaussianConditional>,
                           GaussianMixtureFactor::sharedFactor>;

  // This is the elimination method on the leaf nodes
  auto eliminate = [&](const GaussianFactorGraph &graph) -> Result {
    if (graph.empty()) {
      return {nullptr, nullptr};
    }

#ifdef HYBRID_TIMING
    gttic_(hybrid_eliminate);
#endif

    auto result = EliminatePreferCholesky(graph, frontalKeys);

#ifdef HYBRID_TIMING
    gttoc_(hybrid_eliminate);
#endif

    return result;
  };

  // Perform elimination!
  DecisionTree<Key, Result> eliminationResults(factorGraphTree, eliminate);

#ifdef HYBRID_TIMING
  tictoc_print_();
#endif

  // Separate out decision tree into conditionals and remaining factors.
  const auto [conditionals, newFactors] = unzip(eliminationResults);

  // Create the GaussianMixture from the conditionals
  auto gaussianMixture = std::make_shared<GaussianMixture>(
      frontalKeys, continuousSeparator, discreteSeparator, conditionals);

  if (continuousSeparator.empty()) {
    // If there are no more continuous parents, then we create a
    // DiscreteFactor here, with the error for each discrete choice.

    // Integrate the probability mass in the last continuous conditional using
    // the unnormalized probability q(μ;m) = exp(-error(μ;m)) at the mean.
    //   discrete_probability = exp(-error(μ;m)) * sqrt(det(2π Σ_m))
    auto probability = [&](const Result &pair) -> double {
      static const VectorValues kEmpty;
      // If the factor is not null, it has no keys, just contains the residual.
      const auto &factor = pair.second;
      if (!factor) return 1.0;  // TODO(dellaert): not loving this.
      return exp(-factor->error(kEmpty)) / pair.first->normalizationConstant();
    };

    DecisionTree<Key, double> probabilities(eliminationResults, probability);
    return {
        std::make_shared<HybridConditional>(gaussianMixture),
        std::make_shared<DecisionTreeFactor>(discreteSeparator, probabilities)};
  } else {
    // Otherwise, we create a resulting GaussianMixtureFactor on the separator,
    // taking care to correct for conditional constant.

    // Correct for the normalization constant used up by the conditional
    auto correct = [&](const Result &pair) {
      const auto &factor = pair.second;
      if (!factor) return;
      auto hf = std::dynamic_pointer_cast<HessianFactor>(factor);
      if (!hf) throw std::runtime_error("Expected HessianFactor!");
      hf->constantTerm() += 2.0 * pair.first->logNormalizationConstant();
    };
    eliminationResults.visit(correct);

    const auto mixtureFactor = std::make_shared<GaussianMixtureFactor>(
        continuousSeparator, discreteSeparator, newFactors);

    return {std::make_shared<HybridConditional>(gaussianMixture),
            mixtureFactor};
  }
}

/* ************************************************************************
 * Function to eliminate variables **under the following assumptions**:
 * 1. When the ordering is fully continuous, and the graph only contains
 * continuous and hybrid factors
 * 2. When the ordering is fully discrete, and the graph only contains discrete
 * factors
 *
 * Any usage outside of this is considered incorrect.
 *
 * \warning This function is not meant to be used with arbitrary hybrid factor
 * graphs. For example, if there exists continuous parents, and one tries to
 * eliminate a discrete variable (as specified in the ordering), the result will
 * be INCORRECT and there will be NO error raised.
 */
std::pair<HybridConditional::shared_ptr, std::shared_ptr<Factor>>  //
EliminateHybrid(const HybridGaussianFactorGraph &factors,
                const Ordering &frontalKeys) {
  // NOTE: Because we are in the Conditional Gaussian regime there are only
  // a few cases:
  // 1. continuous variable, make a Gaussian Mixture if there are hybrid
  // factors;
  // 2. continuous variable, we make a Gaussian Factor if there are no hybrid
  // factors;
  // 3. discrete variable, no continuous factor is allowed
  // (escapes Conditional Gaussian regime), if discrete only we do the discrete
  // elimination.

  // However it is not that simple. During elimination it is possible that the
  // multifrontal needs to eliminate an ordering that contains both Gaussian and
  // hybrid variables, for example x1, c1.
  // In this scenario, we will have a density P(x1, c1) that is a Conditional
  // Linear Gaussian P(x1|c1)P(c1) (see Murphy02).

  // The issue here is that, how can we know which variable is discrete if we
  // unify Values? Obviously we can tell using the factors, but is that fast?

  // In the case of multifrontal, we will need to use a constrained ordering
  // so that the discrete parts will be guaranteed to be eliminated last!
  // Because of all these reasons, we carefully consider how to
  // implement the hybrid factors so that we do not get poor performance.

  // The first thing is how to represent the GaussianMixture.
  // A very possible scenario is that the incoming factors will have different
  // levels of discrete keys. For example, imagine we are going to eliminate the
  // fragment: $\phi(x1,c1,c2)$, $\phi(x1,c2,c3)$, which is perfectly valid.
  // Now we will need to know how to retrieve the corresponding continuous
  // densities for the assignment (c1,c2,c3) (OR (c2,c3,c1), note there is NO
  // defined order!). We also need to consider when there is pruning. Two
  // mixture factors could have different pruning patterns - one could have
  // (c1=0,c2=1) pruned, and another could have (c2=0,c3=1) pruned, and this
  // creates a big problem in how to identify the intersection of non-pruned
  // branches.

  // Our approach is first building the collection of all discrete keys. After
  // that we enumerate the space of all key combinations *lazily* so that the
  // exploration branch terminates whenever an assignment yields NULL in any of
  // the hybrid factors.

  // When the number of assignments is large we may encounter stack overflows.
  // However this is also the case with iSAM2, so no pressure :)

  // PREPROCESS: Identify the nature of the current elimination

  // TODO(dellaert): just check the factors:
  // 1. if all factors are discrete, then we can do discrete elimination:
  // 2. if all factors are continuous, then we can do continuous elimination:
  // 3. if not, we do hybrid elimination:

  // First, identify the separator keys, i.e. all keys that are not frontal.
  KeySet separatorKeys;
  for (auto &&factor : factors) {
    separatorKeys.insert(factor->begin(), factor->end());
  }
  // remove frontals from separator
  for (auto &k : frontalKeys) {
    separatorKeys.erase(k);
  }

  // Build a map from keys to DiscreteKeys
  auto mapFromKeyToDiscreteKey = factors.discreteKeyMap();

  // Fill in discrete frontals and continuous frontals.
  std::set<DiscreteKey> discreteFrontals;
  KeySet continuousFrontals;
  for (auto &k : frontalKeys) {
    if (mapFromKeyToDiscreteKey.find(k) != mapFromKeyToDiscreteKey.end()) {
      discreteFrontals.insert(mapFromKeyToDiscreteKey.at(k));
    } else {
      continuousFrontals.insert(k);
    }
  }

  // Fill in discrete discrete separator keys and continuous separator keys.
  std::set<DiscreteKey> discreteSeparatorSet;
  KeyVector continuousSeparator;
  for (auto &k : separatorKeys) {
    if (mapFromKeyToDiscreteKey.find(k) != mapFromKeyToDiscreteKey.end()) {
      discreteSeparatorSet.insert(mapFromKeyToDiscreteKey.at(k));
    } else {
      continuousSeparator.push_back(k);
    }
  }

  // Check if we have any continuous keys:
  const bool discrete_only =
      continuousFrontals.empty() && continuousSeparator.empty();

  // NOTE: We should really defer the product here because of pruning

  if (discrete_only) {
    // Case 1: we are only dealing with discrete
    return discreteElimination(factors, frontalKeys);
  } else if (mapFromKeyToDiscreteKey.empty()) {
    // Case 2: we are only dealing with continuous
    return continuousElimination(factors, frontalKeys);
  } else {
    // Case 3: We are now in the hybrid land!
    return hybridElimination(factors, frontalKeys, continuousSeparator,
                             discreteSeparatorSet);
  }
}

/* ************************************************************************ */
AlgebraicDecisionTree<Key> HybridGaussianFactorGraph::error(
    const VectorValues &continuousValues) const {
  AlgebraicDecisionTree<Key> error_tree(0.0);

  // Iterate over each factor.
  for (auto &f : factors_) {
    // TODO(dellaert): just use a virtual method defined in HybridFactor.
    AlgebraicDecisionTree<Key> factor_error;

    if (auto gaussianMixture = dynamic_pointer_cast<GaussianMixtureFactor>(f)) {
      // Compute factor error and add it.
      error_tree = error_tree + gaussianMixture->error(continuousValues);
    } else if (auto gaussian = dynamic_pointer_cast<GaussianFactor>(f)) {
      // If continuous only, get the (double) error
      // and add it to the error_tree
      double error = gaussian->error(continuousValues);
      // Add the gaussian factor error to every leaf of the error tree.
      error_tree = error_tree.apply(
          [error](double leaf_value) { return leaf_value + error; });
    } else if (dynamic_pointer_cast<DecisionTreeFactor>(f)) {
      // If factor at `idx` is discrete-only, we skip.
      continue;
    } else {
      throwRuntimeError("HybridGaussianFactorGraph::error(VV)", f);
    }
  }

  return error_tree;
}

/* ************************************************************************ */
double HybridGaussianFactorGraph::probPrime(const HybridValues &values) const {
  double error = this->error(values);
  // NOTE: The 0.5 term is handled by each factor
  return std::exp(-error);
}

/* ************************************************************************ */
AlgebraicDecisionTree<Key> HybridGaussianFactorGraph::probPrime(
    const VectorValues &continuousValues) const {
  AlgebraicDecisionTree<Key> error_tree = this->error(continuousValues);
  AlgebraicDecisionTree<Key> prob_tree = error_tree.apply([](double error) {
    // NOTE: The 0.5 term is handled by each factor
    return exp(-error);
  });
  return prob_tree;
}

}  // namespace gtsam

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
#include <gtsam/hybrid/HybridDiscreteFactor.h>
#include <gtsam/hybrid/HybridEliminationTree.h>
#include <gtsam/hybrid/HybridFactor.h>
#include <gtsam/hybrid/HybridGaussianFactor.h>
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
#include <unordered_map>
#include <utility>
#include <vector>

namespace gtsam {

template class EliminateableFactorGraph<HybridGaussianFactorGraph>;

/* ************************************************************************ */
static GaussianMixtureFactor::Sum &addGaussian(
    GaussianMixtureFactor::Sum &sum, const GaussianFactor::shared_ptr &factor) {
  using Y = GaussianFactorGraph;
  // If the decision tree is not intiialized, then intialize it.
  if (sum.empty()) {
    GaussianFactorGraph result;
    result.push_back(factor);
    sum = GaussianMixtureFactor::Sum(result);

  } else {
    auto add = [&factor](const Y &graph) {
      auto result = graph;
      result.push_back(factor);
      return result;
    };
    sum = sum.apply(add);
  }
  return sum;
}

/* ************************************************************************ */
GaussianMixtureFactor::Sum sumFrontals(
    const HybridGaussianFactorGraph &factors) {
  // sum out frontals, this is the factor on the separator
  gttic(sum);

  GaussianMixtureFactor::Sum sum;
  std::vector<GaussianFactor::shared_ptr> deferredFactors;

  for (auto &f : factors) {
    if (f->isHybrid()) {
      if (auto cgmf = boost::dynamic_pointer_cast<GaussianMixtureFactor>(f)) {
        sum = cgmf->add(sum);
      }

      if (auto gm = boost::dynamic_pointer_cast<HybridConditional>(f)) {
        sum = gm->asMixture()->add(sum);
      }

    } else if (f->isContinuous()) {
      deferredFactors.push_back(
          boost::dynamic_pointer_cast<HybridGaussianFactor>(f)->inner());
    } else {
      // We need to handle the case where the object is actually an
      // BayesTreeOrphanWrapper!
      auto orphan = boost::dynamic_pointer_cast<
          BayesTreeOrphanWrapper<HybridBayesTree::Clique>>(f);
      if (!orphan) {
        auto &fr = *f;
        throw std::invalid_argument(
            std::string("factor is discrete in continuous elimination") +
            typeid(fr).name());
      }
    }
  }

  for (auto &f : deferredFactors) {
    sum = addGaussian(sum, f);
  }

  gttoc(sum);

  return sum;
}

/* ************************************************************************ */
std::pair<HybridConditional::shared_ptr, HybridFactor::shared_ptr>
continuousElimination(const HybridGaussianFactorGraph &factors,
                      const Ordering &frontalKeys) {
  GaussianFactorGraph gfg;
  for (auto &fp : factors) {
    if (auto ptr = boost::dynamic_pointer_cast<HybridGaussianFactor>(fp)) {
      gfg.push_back(ptr->inner());
    } else if (auto p =
                   boost::static_pointer_cast<HybridConditional>(fp)->inner()) {
      gfg.push_back(boost::static_pointer_cast<GaussianConditional>(p));
    } else {
      // It is an orphan wrapped conditional
    }
  }

  auto result = EliminatePreferCholesky(gfg, frontalKeys);
  return {boost::make_shared<HybridConditional>(result.first),
          boost::make_shared<HybridGaussianFactor>(result.second)};
}

/* ************************************************************************ */
std::pair<HybridConditional::shared_ptr, HybridFactor::shared_ptr>
discreteElimination(const HybridGaussianFactorGraph &factors,
                    const Ordering &frontalKeys) {
  DiscreteFactorGraph dfg;
  for (auto &fp : factors) {
    if (auto ptr = boost::dynamic_pointer_cast<HybridDiscreteFactor>(fp)) {
      dfg.push_back(ptr->inner());
    } else if (auto p =
                   boost::static_pointer_cast<HybridConditional>(fp)->inner()) {
      dfg.push_back(boost::static_pointer_cast<DiscreteConditional>(p));
    } else {
      // It is an orphan wrapper
    }
  }

  auto result = EliminateDiscrete(dfg, frontalKeys);

  return {boost::make_shared<HybridConditional>(result.first),
          boost::make_shared<HybridDiscreteFactor>(result.second)};
}

/* ************************************************************************ */
std::pair<HybridConditional::shared_ptr, HybridFactor::shared_ptr>
hybridElimination(const HybridGaussianFactorGraph &factors,
                  const Ordering &frontalKeys,
                  const KeySet &continuousSeparator,
                  const std::set<DiscreteKey> &discreteSeparatorSet) {
  // NOTE: since we use the special JunctionTree,
  // only possiblity is continuous conditioned on discrete.
  DiscreteKeys discreteSeparator(discreteSeparatorSet.begin(),
                                 discreteSeparatorSet.end());

  // sum out frontals, this is the factor on the separator
  GaussianMixtureFactor::Sum sum = sumFrontals(factors);

  using EliminationPair = GaussianFactorGraph::EliminationResult;

  KeyVector keysOfEliminated;  // Not the ordering
  KeyVector keysOfSeparator;   // TODO(frank): Is this just (keys - ordering)?

  // This is the elimination method on the leaf nodes
  auto eliminate = [&](const GaussianFactorGraph &graph)
      -> GaussianFactorGraph::EliminationResult {
    if (graph.empty()) {
      return {nullptr, nullptr};
    }
    auto result = EliminatePreferCholesky(graph, frontalKeys);
    if (keysOfEliminated.empty()) {
      keysOfEliminated =
          result.first->keys();  // Initialize the keysOfEliminated to be the
    }
    // keysOfEliminated of the GaussianConditional
    if (keysOfSeparator.empty()) {
      keysOfSeparator = result.second->keys();
    }
    return result;
  };

  // Perform elimination!
  DecisionTree<Key, EliminationPair> eliminationResults(sum, eliminate);

  // Separate out decision tree into conditionals and remaining factors.
  auto pair = unzip(eliminationResults);

  const GaussianMixtureFactor::Factors &separatorFactors = pair.second;

  // Create the GaussianMixture from the conditionals
  auto conditional = boost::make_shared<GaussianMixture>(
      frontalKeys, keysOfSeparator, discreteSeparator, pair.first);

  // If there are no more continuous parents, then we should create here a
  // DiscreteFactor, with the error for each discrete choice.
  if (keysOfSeparator.empty()) {
    VectorValues empty_values;
    auto factorError = [&](const GaussianFactor::shared_ptr &factor) {
      if (!factor) return 0.0;  // TODO(fan): does this make sense?
      return exp(-factor->error(empty_values));
    };
    DecisionTree<Key, double> fdt(separatorFactors, factorError);
    auto discreteFactor =
        boost::make_shared<DecisionTreeFactor>(discreteSeparator, fdt);

    return {boost::make_shared<HybridConditional>(conditional),
            boost::make_shared<HybridDiscreteFactor>(discreteFactor)};

  } else {
    // Create a resulting DCGaussianMixture on the separator.
    auto factor = boost::make_shared<GaussianMixtureFactor>(
        KeyVector(continuousSeparator.begin(), continuousSeparator.end()),
        discreteSeparator, separatorFactors);
    return {boost::make_shared<HybridConditional>(conditional), factor};
  }
}
/* ************************************************************************ */
std::pair<HybridConditional::shared_ptr, HybridFactor::shared_ptr>  //
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
  std::unordered_map<Key, DiscreteKey> mapFromKeyToDiscreteKey;
  std::set<DiscreteKey> discreteSeparatorSet;
  std::set<DiscreteKey> discreteFrontals;

  KeySet separatorKeys;
  KeySet allContinuousKeys;
  KeySet continuousFrontals;
  KeySet continuousSeparator;

  // This initializes separatorKeys and mapFromKeyToDiscreteKey
  for (auto &&factor : factors) {
    separatorKeys.insert(factor->begin(), factor->end());
    if (!factor->isContinuous()) {
      for (auto &k : factor->discreteKeys()) {
        mapFromKeyToDiscreteKey[k.first] = k;
      }
    }
  }

  // remove frontals from separator
  for (auto &k : frontalKeys) {
    separatorKeys.erase(k);
  }

  // Fill in discrete frontals and continuous frontals for the end result
  for (auto &k : frontalKeys) {
    if (mapFromKeyToDiscreteKey.find(k) != mapFromKeyToDiscreteKey.end()) {
      discreteFrontals.insert(mapFromKeyToDiscreteKey.at(k));
    } else {
      continuousFrontals.insert(k);
      allContinuousKeys.insert(k);
    }
  }

  // Fill in discrete frontals and continuous frontals for the end result
  for (auto &k : separatorKeys) {
    if (mapFromKeyToDiscreteKey.find(k) != mapFromKeyToDiscreteKey.end()) {
      discreteSeparatorSet.insert(mapFromKeyToDiscreteKey.at(k));
    } else {
      continuousSeparator.insert(k);
      allContinuousKeys.insert(k);
    }
  }

  // NOTE: We should really defer the product here because of pruning

  // Case 1: we are only dealing with continuous
  if (mapFromKeyToDiscreteKey.empty() && !allContinuousKeys.empty()) {
    return continuousElimination(factors, frontalKeys);
  }

  // Case 2: we are only dealing with discrete
  if (allContinuousKeys.empty()) {
    return discreteElimination(factors, frontalKeys);
  }

  // Case 3: We are now in the hybrid land!
  return hybridElimination(factors, frontalKeys, continuousSeparator,
                           discreteSeparatorSet);
}

/* ************************************************************************ */
void HybridGaussianFactorGraph::add(JacobianFactor &&factor) {
  FactorGraph::add(boost::make_shared<HybridGaussianFactor>(std::move(factor)));
}

/* ************************************************************************ */
void HybridGaussianFactorGraph::add(JacobianFactor::shared_ptr factor) {
  FactorGraph::add(boost::make_shared<HybridGaussianFactor>(factor));
}

/* ************************************************************************ */
void HybridGaussianFactorGraph::add(DecisionTreeFactor &&factor) {
  FactorGraph::add(boost::make_shared<HybridDiscreteFactor>(std::move(factor)));
}

/* ************************************************************************ */
void HybridGaussianFactorGraph::add(DecisionTreeFactor::shared_ptr factor) {
  FactorGraph::add(boost::make_shared<HybridDiscreteFactor>(factor));
}

}  // namespace gtsam

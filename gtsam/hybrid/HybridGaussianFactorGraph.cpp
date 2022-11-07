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

// #define HYBRID_TIMING

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
      if (auto gf = boost::dynamic_pointer_cast<HybridGaussianFactor>(f)) {
        deferredFactors.push_back(gf->inner());
      }
      if (auto cg = boost::dynamic_pointer_cast<HybridConditional>(f)) {
        deferredFactors.push_back(cg->asGaussian());
      }

    } else if (f->isDiscrete()) {
      // Don't do anything for discrete-only factors
      // since we want to eliminate continuous values only.
      continue;

    } else {
      // We need to handle the case where the object is actually an
      // BayesTreeOrphanWrapper!
      auto orphan = boost::dynamic_pointer_cast<
          BayesTreeOrphanWrapper<HybridBayesTree::Clique>>(f);
      if (!orphan) {
        auto &fr = *f;
        throw std::invalid_argument(
            std::string("factor is discrete in continuous elimination ") +
            demangle(typeid(fr).name()));
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
    } else if (auto ptr = boost::static_pointer_cast<HybridConditional>(fp)) {
      gfg.push_back(
          boost::static_pointer_cast<GaussianConditional>(ptr->inner()));
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

  for (auto &factor : factors) {
    if (auto p = boost::dynamic_pointer_cast<HybridDiscreteFactor>(factor)) {
      dfg.push_back(p->inner());
    } else if (auto p = boost::static_pointer_cast<HybridConditional>(factor)) {
      auto discrete_conditional =
          boost::static_pointer_cast<DiscreteConditional>(p->inner());
      dfg.push_back(discrete_conditional);
    } else {
      // It is an orphan wrapper
    }
  }

  auto result = EliminateForMPE(dfg, frontalKeys);

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

  // If a tree leaf contains nullptr,
  // convert that leaf to an empty GaussianFactorGraph.
  // Needed since the DecisionTree will otherwise create
  // a GFG with a single (null) factor.
  auto emptyGaussian = [](const GaussianFactorGraph &gfg) {
    bool hasNull =
        std::any_of(gfg.begin(), gfg.end(),
                    [](const GaussianFactor::shared_ptr &ptr) { return !ptr; });

    return hasNull ? GaussianFactorGraph() : gfg;
  };
  sum = GaussianMixtureFactor::Sum(sum, emptyGaussian);

  using EliminationPair = GaussianFactorGraph::EliminationResult;

  KeyVector keysOfEliminated;  // Not the ordering
  KeyVector keysOfSeparator;   // TODO(frank): Is this just (keys - ordering)?

  // This is the elimination method on the leaf nodes
  auto eliminate = [&](const GaussianFactorGraph &graph)
      -> GaussianFactorGraph::EliminationResult {
    if (graph.empty()) {
      return {nullptr, nullptr};
    }

#ifdef HYBRID_TIMING
    gttic_(hybrid_eliminate);
#endif

    std::pair<boost::shared_ptr<GaussianConditional>,
              boost::shared_ptr<GaussianFactor>>
        result = EliminatePreferCholesky(graph, frontalKeys);

    // Initialize the keysOfEliminated to be the keys of the
    // eliminated GaussianConditional
    keysOfEliminated = result.first->keys();
    keysOfSeparator = result.second->keys();

#ifdef HYBRID_TIMING
    gttoc_(hybrid_eliminate);
#endif

    return result;
  };

  // Perform elimination!
  DecisionTree<Key, EliminationPair> eliminationResults(sum, eliminate);

#ifdef HYBRID_TIMING
  tictoc_print_();
  tictoc_reset_();
#endif

  // Separate out decision tree into conditionals and remaining factors.
  auto pair = unzip(eliminationResults);

  const GaussianMixtureFactor::Factors &separatorFactors = pair.second;

  // Create the GaussianMixture from the conditionals
  auto conditional = boost::make_shared<GaussianMixture>(
      frontalKeys, keysOfSeparator, discreteSeparator, pair.first);

  // If there are no more continuous parents, then we should create here a
  // DiscreteFactor, with the error for each discrete choice.
  if (keysOfSeparator.empty()) {
    // TODO(Varun) Use the math from the iMHS_Math-1-indexed document
    VectorValues empty_values;
    auto factorProb = [&](const GaussianFactor::shared_ptr &factor) {
      if (!factor) {
        return 0.0;  // If nullptr, return 0.0 probability
      } else {
        return 1.0;
      }
    };
    DecisionTree<Key, double> fdt(separatorFactors, factorProb);

    auto discreteFactor =
        boost::make_shared<DecisionTreeFactor>(discreteSeparator, fdt);

    return {boost::make_shared<HybridConditional>(conditional),
            boost::make_shared<HybridDiscreteFactor>(discreteFactor)};

  } else {
    // Create a resulting GaussianMixtureFactor on the separator.
    auto factor = boost::make_shared<GaussianMixtureFactor>(
        KeyVector(continuousSeparator.begin(), continuousSeparator.end()),
        discreteSeparator, separatorFactors);
    return {boost::make_shared<HybridConditional>(conditional), factor};
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

#ifdef HYBRID_TIMING
  tictoc_reset_();
#endif
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

/* ************************************************************************ */
const Ordering HybridGaussianFactorGraph::getHybridOrdering() const {
  KeySet discrete_keys = discreteKeys();
  for (auto &factor : factors_) {
    for (const DiscreteKey &k : factor->discreteKeys()) {
      discrete_keys.insert(k.first);
    }
  }

  const VariableIndex index(factors_);
  Ordering ordering = Ordering::ColamdConstrainedLast(
      index, KeyVector(discrete_keys.begin(), discrete_keys.end()), true);
  return ordering;
}

/* ************************************************************************ */
AlgebraicDecisionTree<Key> HybridGaussianFactorGraph::error(
    const VectorValues &continuousValues) const {
  AlgebraicDecisionTree<Key> error_tree(0.0);

  for (size_t idx = 0; idx < size(); idx++) {
    AlgebraicDecisionTree<Key> factor_error;

    if (factors_.at(idx)->isHybrid()) {
      // If factor is hybrid, select based on assignment.
      GaussianMixtureFactor::shared_ptr gaussianMixture =
          boost::static_pointer_cast<GaussianMixtureFactor>(factors_.at(idx));
      factor_error = gaussianMixture->error(continuousValues);

      if (idx == 0) {
        error_tree = factor_error;
      } else {
        error_tree = error_tree + factor_error;
      }

    } else if (factors_.at(idx)->isContinuous()) {
      // If continuous only, get the (double) error
      // and add it to the error_tree
      auto hybridGaussianFactor =
          boost::static_pointer_cast<HybridGaussianFactor>(factors_.at(idx));
      GaussianFactor::shared_ptr gaussian = hybridGaussianFactor->inner();

      double error = gaussian->error(continuousValues);
      error_tree = error_tree.apply(
          [error](double leaf_value) { return leaf_value + error; });

    } else if (factors_.at(idx)->isDiscrete()) {
      // If factor at `idx` is discrete-only, we skip.
      continue;
    }
  }

  return error_tree;
}

/* ************************************************************************ */
AlgebraicDecisionTree<Key> HybridGaussianFactorGraph::probPrime(
    const VectorValues &continuousValues) const {
  AlgebraicDecisionTree<Key> error_tree = this->error(continuousValues);
  AlgebraicDecisionTree<Key> prob_tree =
      error_tree.apply([](double error) { return exp(-error); });
  return prob_tree;
}

/* ************************************************************************ */
boost::shared_ptr<HybridGaussianFactorGraph::BayesNetType>
HybridGaussianFactorGraph::eliminateHybridSequential() const {
  Ordering continuous_ordering(this->continuousKeys()),
      discrete_ordering(this->discreteKeys());

  // Eliminate continuous
  HybridBayesNet::shared_ptr bayesNet;
  HybridGaussianFactorGraph::shared_ptr discreteGraph;
  std::tie(bayesNet, discreteGraph) =
      BaseEliminateable::eliminatePartialSequential(
          continuous_ordering, EliminationTraitsType::DefaultEliminate);

  // Get the last continuous conditional which will have all the discrete keys
  auto last_conditional = bayesNet->at(bayesNet->size() - 1);
  // Get all the discrete assignments
  DiscreteKeys discrete_keys = last_conditional->discreteKeys();
  const std::vector<DiscreteValues> assignments =
      DiscreteValues::CartesianProduct(discrete_keys);

  // Reverse discrete keys order for correct tree construction
  std::reverse(discrete_keys.begin(), discrete_keys.end());

  // Create a decision tree of all the different VectorValues
  std::vector<VectorValues::shared_ptr> vector_values;
  for (const DiscreteValues &assignment : assignments) {
    VectorValues values = bayesNet->optimize(assignment);
    vector_values.push_back(boost::make_shared<VectorValues>(values));
  }
  DecisionTree<Key, VectorValues::shared_ptr> delta_tree(discrete_keys,
                                                         vector_values);

  // Get the probPrime tree with the correct leaf probabilities
  std::vector<double> probPrimes;
  for (const DiscreteValues &assignment : assignments) {
    double error = 0.0;
    VectorValues delta = *delta_tree(assignment);
    for (auto factor : *this) {
      if (factor->isHybrid()) {
        auto f = boost::static_pointer_cast<GaussianMixtureFactor>(factor);
        error += f->error(delta, assignment);

      } else if (factor->isContinuous()) {
        auto f = boost::static_pointer_cast<HybridGaussianFactor>(factor);
        error += f->inner()->error(delta);
      }
    }
    probPrimes.push_back(exp(-error));
  }
  AlgebraicDecisionTree<Key> probPrimeTree(discrete_keys, probPrimes);
  discreteGraph->add(DecisionTreeFactor(discrete_keys, probPrimeTree));

  // Perform discrete elimination
  HybridBayesNet::shared_ptr discreteBayesNet =
      discreteGraph->eliminateSequential(
          discrete_ordering, EliminationTraitsType::DefaultEliminate);
  bayesNet->add(*discreteBayesNet);

  return bayesNet;
}

}  // namespace gtsam
